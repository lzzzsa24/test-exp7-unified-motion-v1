#include "line_recovery.h"
#include "line_search_model.h"
#include "drive_base.h"
#include "wheel_encoder.h"

#define HISTORY_SIZE              16U
#define HISTORY_PERIOD_MS         20U
#define HISTORY_MIN_AGE_MS       160U
#define HISTORY_MAX_AGE_MS       600U
#define DIRECTION_GUARD_MS        70U
#define SENSOR_CONFIRM_MS        20U
#define CAPTURE_STATIONARY_MS     80U
#define EPISODE_TIMEOUT_MS      8000U
#define MOVE_TIMEOUT_MS          600U
#define PROBE_TIMEOUT_MS         700U
#define EDGE_TIMEOUT_MS         2800U
#define POSITION_TOLERANCE        12U
#define MAX_BACKTRACKS             2U
#define MAX_PROBES                 3U
#define BACKTRACK_CPS           1870L
/* Wheel travel limits; none is a chassis displacement/angle measurement. */
#define MM_COUNTS(mm) ((int32_t)(((int64_t)(mm) * 1000000LL * \
    LINE_SEARCH_COUNTS_PER_REV) / (3141593LL * VEHICLE_WHEEL_DIAMETER_MM)))
#define BACKTRACK_COUNTS MM_COUNTS(45)
#define PROBE_COUNTS     MM_COUNTS(140)
#define EDGE_COUNTS      MM_COUNTS(420)
#define ROLLBACK_LIMIT   MM_COUNTS(450)
#define COARSE_MARGIN   MM_COUNTS(8)
#define MOTION_MINIMUM  MM_COUNTS(5)

typedef struct { int32_t wheel[4]; uint32_t time; } Checkpoint;
typedef enum { REC_IDLE, REC_BRAKE_LOSS, REC_BACKTRACK, REC_WAIT_PROBE,
               REC_PROBE, REC_BRAKE_ROLLBACK, REC_ROLLBACK,
               REC_BRAKE_CAPTURE, REC_BRAKE_BACKTRACK_DONE, REC_BRAKE_RETURN_DONE,
               REC_CONFIRM_CAPTURE, REC_CAPTURED,
               REC_FAILED } RecoveryPhase;

static Checkpoint history[HISTORY_SIZE];
static uint8_t history_head, history_count, episode_active;
static uint8_t backtracks, probes, center_candidate;
static int8_t side, outer_candidate;
static uint32_t episode_start, phase_start, center_since, outer_since;
static int32_t probe_origin[4];
static int32_t move_origin[4], move_cps[4], move_limit[4];
static uint32_t move_start, move_timeout;
static RecoveryPhase phase, capture_from;
static LineRecoveryStopReason stop_reason;

static int32_t absolute(int32_t v) { return v < 0 ? -v : v; }
static void counts_now(int32_t values[4])
{
  WheelEncoderCounts c;
  WheelEncoder_GetCounts(&c);
  values[0] = c.motor1; values[1] = c.motor2;
  values[2] = c.motor3; values[3] = c.motor4;
}
static void checkpoint(uint32_t now)
{
  counts_now(history[history_head].wheel);
  history[history_head].time = now;
  history_head = (uint8_t)((history_head + 1U) % HISTORY_SIZE);
  if (history_count < HISTORY_SIZE) ++history_count;
}
static uint8_t center_stable(uint8_t visible, uint32_t now)
{
  if (!visible) { center_candidate = 0U; return 0U; }
  if (!center_candidate) { center_candidate = 1U; center_since = now; }
  return now - center_since >= SENSOR_CONFIRM_MS;
}
LineRecoveryStopReason LineRecovery_GetStopReason(void) { return stop_reason; }
void LineRecovery_Stop(LineRecoveryStopReason reason)
{
  DriveBase_Stop(DRIVE_STOP_COAST);
  stop_reason = reason;
  phase = REC_FAILED;
}
uint8_t LineRecovery_Expired(uint32_t now)
{
  return episode_active && (DriveBase_GetFaultMask() != 0U ||
                            now - episode_start >= EPISODE_TIMEOUT_MS);
}
void LineRecovery_Reset(void)
{
  if (episode_active) DriveBase_Stop(DRIVE_STOP_COAST);
  history_head = history_count = episode_active = 0U;
  backtracks = probes = center_candidate = 0U;
  outer_candidate = side = 0;
  phase = REC_IDLE;
  stop_reason = LINE_REC_STOP_NONE;
}
void LineRecovery_Commit(void)
{
  /* Accept only after the caller's low-speed forward capture interval. */
  history_head = history_count = episode_active = 0U;
  phase = REC_IDLE;
  stop_reason = LINE_REC_STOP_NONE;
}
void LineRecovery_Record(const LineTrackingReading *r, uint32_t now)
{
  DriveBaseTelemetry telemetry;
  uint8_t i;
  uint8_t last;
  DriveBase_GetTelemetry(&telemetry);
  for (i = 0; i < 4U; ++i)
  {
    if (telemetry.requested_cps[i] < 0)
    {
      history_head = history_count = 0U;
      return;
    }
  }
  if (!(r->x1_black || r->x3_black) || (r->x2_black && r->x4_black)) return;
  last = (uint8_t)((history_head + HISTORY_SIZE - 1U) % HISTORY_SIZE);
  if (!history_count || now - history[last].time >= HISTORY_PERIOD_MS)
    checkpoint(now);
}
void LineRecovery_Begin(int8_t preferred_side, uint32_t now)
{
  if (!episode_active)
  {
    episode_active = 1U;
    episode_start = now;
    backtracks = probes = 0U;
  }
  side = preferred_side > 0 ? 1 : -1;
  center_candidate = 0U;
  outer_candidate = 0;
  DriveBase_Stop(DRIVE_STOP_BRAKE);
  phase = REC_BRAKE_LOSS;
  phase_start = now;
}

/* Return 1 for started, 2 for already near target, 0 for rejected. */
static uint8_t move_to(const int32_t target[4], uint8_t clip_backtrack, uint32_t now)
{
  int32_t current[4], minimum = ROLLBACK_LIMIT, maximum = 0;
  uint8_t i;
  counts_now(current);
  for (i = 0; i < 4U; ++i)
  {
    int32_t direction = clip_backtrack ? -1 : (i < 2U ? -side : side);
    int32_t remaining = (target[i] - current[i]) * direction;
    if (remaining < minimum) minimum = remaining;
    if (absolute(remaining) > maximum) maximum = absolute(remaining);
    move_cps[i] = direction * (clip_backtrack ? BACKTRACK_CPS : LINE_SEARCH_TARGET_CPS);
    move_origin[i] = current[i];
    move_limit[i] = remaining - COARSE_MARGIN;
  }
  if (!clip_backtrack && maximum > ROLLBACK_LIMIT) return 0U;
  /* Stop the whole chassis before any wheel reaches/passes its old count.
     Do not pulse or reverse individual wheels to chase a precise endpoint. */
  if (minimum <= COARSE_MARGIN * 2)
    return !clip_backtrack && maximum > COARSE_MARGIN * 2 ? 0U : 2U;
  if (clip_backtrack)
  {
    int32_t distance = minimum - COARSE_MARGIN;
    if (distance > BACKTRACK_COUNTS) distance = BACKTRACK_COUNTS;
    for (i = 0; i < 4U; ++i) move_limit[i] = distance;
  }
  move_start = now;
  move_timeout = clip_backtrack ? MOVE_TIMEOUT_MS : EDGE_TIMEOUT_MS;
  return 1U;
}
static uint8_t backtrack(uint32_t now)
{
  int32_t current[4];
  int selected = -1, fresh = -1;
  uint8_t age_index, i;
  if (backtracks >= MAX_BACKTRACKS) return 2U;
  counts_now(current);
  for (age_index = 0; age_index < history_count; ++age_index)
  {
    uint8_t index = (uint8_t)((history_head + HISTORY_SIZE - 1U - age_index) % HISTORY_SIZE);
    uint32_t age = now - history[index].time;
    uint8_t forward_path = 1U;
    if (age > HISTORY_MAX_AGE_MS) break;
    /* Do not pretend an earlier counter-rotation was a known forward path. */
    for (i = 0; i < 4U; ++i)
      if (current[i] - history[index].wheel[i] < -(int32_t)POSITION_TOLERANCE)
        forward_path = 0U;
    if (!forward_path) continue;
    if (fresh < 0) fresh = index;
    if (age >= HISTORY_MIN_AGE_MS) { selected = index; break; }
  }
  if (selected < 0) selected = fresh;
  if (selected < 0) return 2U;
  ++backtracks;
  return move_to(history[selected].wheel, 1U, now);
}
static void wait_probe(uint32_t now)
{
  DriveBase_Stop(DRIVE_STOP_COAST);
  center_candidate = 0U;
  outer_candidate = 0;
  phase = REC_WAIT_PROBE;
  phase_start = now;
}
static void capture(RecoveryPhase from, uint32_t now)
{
  capture_from = from;
  DriveBase_Stop(DRIVE_STOP_BRAKE);
  phase = REC_BRAKE_CAPTURE;
  phase_start = now;
  center_candidate = 0U;
  outer_candidate = 0;
}
static void rollback_brake(uint32_t now)
{
  DriveBase_Stop(DRIVE_STOP_BRAKE);
  phase = REC_BRAKE_ROLLBACK;
  phase_start = now;
  center_candidate = 0U;
}

LineRecoveryResult LineRecovery_Step(const LineTrackingReading *r,
                                     LineTrackingCommand *command, uint32_t now)
{
  DriveBaseTelemetry telemetry;
  uint8_t visible = (r->x1_black || r->x3_black) && !(r->x2_black && r->x4_black);
  int8_t outer = r->x2_black && !r->x4_black ? -1 :
               (r->x4_black && !r->x2_black ? 1 : 0);
  uint8_t started;
  command->valid = 0U;
  command->left_cps = command->right_cps = 0;
  command->action = side < 0 ? LINE_ACTION_SEARCH_LEFT : LINE_ACTION_SEARCH_RIGHT;
  DriveBase_Task(now);
  if (DriveBase_GetFaultMask()) LineRecovery_Stop(LINE_REC_STOP_DRIVE_FAULT);
  else if (phase != REC_FAILED && LineRecovery_Expired(now))
    LineRecovery_Stop(LINE_REC_STOP_SEARCH_TIMEOUT);
  DriveBase_GetTelemetry(&telemetry);
  if (phase == REC_FAILED) return LINE_RECOVERY_FAILED;
  if (phase == REC_CAPTURED) return LINE_RECOVERY_CAPTURED;

  if (phase == REC_BRAKE_LOSS || phase == REC_BRAKE_ROLLBACK || phase == REC_BRAKE_CAPTURE ||
      phase == REC_BRAKE_BACKTRACK_DONE || phase == REC_BRAKE_RETURN_DONE)
  {
    RecoveryPhase finished = phase;
    if (telemetry.mode == DRIVE_BASE_BRAKING) return LINE_RECOVERY_BUSY;
    /* Braking retains ownership until its pulse completes. */
    DriveBase_Stop(DRIVE_STOP_COAST);
    if (finished == REC_BRAKE_CAPTURE)
    {
      phase = REC_CONFIRM_CAPTURE;
      phase_start = now;
      center_candidate = 0U;
    }
    else if (finished == REC_BRAKE_ROLLBACK)
    {
      started = move_to(probe_origin, 0U, now);
      if (!started) LineRecovery_Stop(LINE_REC_STOP_RETURN_REJECTED);
      else if (started == 2U) { side = (int8_t)-side; wait_probe(now); }
      else phase = REC_ROLLBACK;
    }
    else if (finished == REC_BRAKE_BACKTRACK_DONE || finished == REC_BRAKE_RETURN_DONE)
    {
      if (finished == REC_BRAKE_RETURN_DONE) side = (int8_t)-side;
      wait_probe(now);
    }
    else if (visible || outer)
    {
      /* A transient white sample must not force a reverse manoeuvre after
         the line has already returned during the initial brake. */
      capture_from = REC_BRAKE_LOSS;
      phase = REC_CONFIRM_CAPTURE;
      phase_start = now;
      center_candidate = 0U;
    }
    else
    {
      started = backtrack(now);
      if (!started) LineRecovery_Stop(LINE_REC_STOP_RETURN_REJECTED);
      else if (started == 2U) wait_probe(now);
      else { phase = REC_BACKTRACK; center_candidate = 0U; }
    }
  }
  else if (phase == REC_BACKTRACK || phase == REC_ROLLBACK)
  {
    int32_t current[4], minimum = ROLLBACK_LIMIT;
    uint8_t i, completed = 0U;
    counts_now(current);
    for (i = 0; i < 4U; ++i)
    {
      int32_t travel = (current[i] - move_origin[i]) * (move_cps[i] > 0 ? 1 : -1);
      if (travel >= move_limit[i]) completed = 1U;
      if (travel < minimum) minimum = travel;
    }
    if (visible || outer) capture(phase, now);
    else if (completed || now - move_start >= move_timeout)
    {
      if (minimum < MOTION_MINIMUM) LineRecovery_Stop(LINE_REC_STOP_NO_MOTION);
      else if (phase == REC_ROLLBACK && !completed) LineRecovery_Stop(LINE_REC_STOP_RETURN_TIMEOUT);
      else
      {
        RecoveryPhase finished = phase;
        DriveBase_Stop(DRIVE_STOP_BRAKE);
        phase = finished == REC_BACKTRACK ? REC_BRAKE_BACKTRACK_DONE : REC_BRAKE_RETURN_DONE;
      }
    }
  }
  else if (phase == REC_CONFIRM_CAPTURE)
  {
    if (center_stable(visible, now))
    {
      checkpoint(now);
      phase = REC_CAPTURED;
      return LINE_RECOVERY_CAPTURED;
    }
    if (outer != outer_candidate) { outer_candidate = outer; outer_since = now; }
    if (!visible && outer && now - outer_since >= SENSOR_CONFIRM_MS)
    {
      /* An outer hit is guidance, never permission to resume forward travel. */
      side = outer;
      wait_probe(now);
    }
    else if (now - phase_start >= CAPTURE_STATIONARY_MS)
    {
      if (capture_from == REC_PROBE || capture_from == REC_ROLLBACK) rollback_brake(now);
      else wait_probe(now);
    }
  }
  else if (phase == REC_WAIT_PROBE)
  {
    if (center_stable(visible, now))
    {
      checkpoint(now);
      phase = REC_CAPTURED;
      return LINE_RECOVERY_CAPTURED;
    }
    if (outer != outer_candidate) { outer_candidate = outer; outer_since = now; }
    if (outer && now - outer_since >= SENSOR_CONFIRM_MS) side = outer;
    if (now - phase_start >= DIRECTION_GUARD_MS && !visible)
    {
      if (probes >= MAX_PROBES) LineRecovery_Stop(LINE_REC_STOP_SCAN_LIMIT);
      else
      {
        /* Each wider probe starts after a coarse group return, not precise
           wheel-by-wheel endpoint correction. Sensor guidance can choose side. */
        counts_now(probe_origin);
        ++probes;
        phase = REC_PROBE;
        phase_start = now;
        outer_candidate = 0;
      }
    }
  }
  else if (phase == REC_PROBE)
  {
    int32_t current[4], maximum = 0, minimum = ROLLBACK_LIMIT;
    uint8_t i;
    counts_now(current);
    for (i = 0; i < 4U; ++i)
    {
      int32_t travel = (current[i] - probe_origin[i]) * (i < 2U ? side : -side);
      if (absolute(current[i] - probe_origin[i]) > ROLLBACK_LIMIT)
      { LineRecovery_Stop(LINE_REC_STOP_ENCODER_RANGE); break; }
      if (travel > maximum) maximum = travel;
      if (travel < minimum) minimum = travel;
    }
    /* Stop on the first middle hit, then debounce while stationary. Waiting
       for confirmation while moving can carry the sensor across a thin line. */
    if (phase == REC_FAILED) return LINE_RECOVERY_FAILED;
    if (visible) capture(REC_PROBE, now);
    else
    {
      if (outer != outer_candidate) { outer_candidate = outer; outer_since = now; }
      if (outer != 0 && outer != side && now - outer_since >= SENSOR_CONFIRM_MS)
        capture(REC_PROBE, now);
      else if (maximum >= (outer == side ? EDGE_COUNTS : PROBE_COUNTS * probes) ||
          now - phase_start >= (outer == side ? EDGE_TIMEOUT_MS : PROBE_TIMEOUT_MS * probes))
      {
        if (minimum < MOTION_MINIMUM) LineRecovery_Stop(LINE_REC_STOP_NO_MOTION);
        else rollback_brake(now);
      }
    }
  }
  if (phase == REC_PROBE)
  {
    int32_t left = side < 0 ? -LINE_SEARCH_TARGET_CPS : LINE_SEARCH_TARGET_CPS;
    DriveBase_PrepareLineTurnAssist(left, -left);
    DriveBase_SetWheelCps(left, left, -left, -left);
  }
  else if (phase == REC_BACKTRACK || phase == REC_ROLLBACK)
  {
    if (phase == REC_ROLLBACK) DriveBase_PrepareLineTurnAssist(move_cps[0], move_cps[2]);
    DriveBase_SetWheelCps(move_cps[0], move_cps[1], move_cps[2], move_cps[3]);
  }
  return phase == REC_FAILED ? LINE_RECOVERY_FAILED : LINE_RECOVERY_BUSY;
}
