#include "line_recovery.h"
#include "line_search_model.h"
#include "drive_base.h"
#include "buzzer_phrase_40077493715.h"

#define SENSOR_CONFIRM_MS        20U
#define CAPTURE_STATIONARY_MS     80U

typedef enum { REC_IDLE, REC_BRAKE_SEARCH, REC_SEARCH, REC_BRAKE_CAPTURE,
               REC_CONFIRM_CAPTURE, REC_CAPTURED, REC_FAULT } RecoveryPhase;
static RecoveryPhase phase;
static int8_t side;
static uint8_t center_candidate, audio_owned, audio_requested;
static uint32_t phase_start, center_since;
static LineRecoveryStopReason stop_reason;

static void stop_audio(void)
{
  if (audio_owned) BuzzerPhrase400_Stop();
  audio_owned = 0U;
  audio_requested = 0U;
}
static void search_audio(uint32_t now)
{
  BuzzerPhrase400_Task(now);
  if (!BuzzerPhrase400_IsPlaying())
  {
    audio_owned = BuzzerPhrase400_Start(1U);
  }
}
LineRecoveryStopReason LineRecovery_GetStopReason(void) { return stop_reason; }
void LineRecovery_Stop(LineRecoveryStopReason reason)
{
  DriveBase_Stop(DRIVE_STOP_COAST);
  stop_audio();
  stop_reason = reason;
  phase = REC_FAULT;
}
void LineRecovery_Reset(void)
{
  if (phase != REC_IDLE) DriveBase_Stop(DRIVE_STOP_COAST);
  stop_audio();
  phase = REC_IDLE;
  stop_reason = LINE_REC_STOP_NONE;
  center_candidate = 0U;
}
void LineRecovery_Commit(void)
{
  stop_audio();
  phase = REC_IDLE;
  stop_reason = LINE_REC_STOP_NONE;
}
void LineRecovery_Begin(int8_t preferred_side, uint32_t now)
{
  side = preferred_side > 0 ? 1 : -1;
  center_candidate = 0U;
  stop_reason = LINE_REC_STOP_NONE;
  DriveBase_Stop(DRIVE_STOP_BRAKE);
  phase = REC_BRAKE_SEARCH;
  phase_start = now;
  if (DriveBase_GetFaultMask()) LineRecovery_Stop(LINE_REC_STOP_DRIVE_FAULT);
  else
  {
    /* Claim this phrase only for active line recovery. Normal manual audio
       remains untouched by reset/commit when recovery did not own it. */
    audio_owned = BuzzerPhrase400_Start(1U);
    audio_requested = 1U;
  }
}
void LineRecovery_BeginCorner(int8_t preferred_side, uint32_t now)
{
  side = preferred_side > 0 ? 1 : -1;
  center_candidate = 0U;
  stop_reason = LINE_REC_STOP_NONE;
  phase = REC_SEARCH;
  phase_start = now;
  audio_requested = 0U;
  if (DriveBase_GetFaultMask()) LineRecovery_Stop(LINE_REC_STOP_DRIVE_FAULT);
}
LineRecoveryResult LineRecovery_Step(const LineTrackingReading *r,
                                     LineTrackingCommand *command, uint32_t now)
{
  DriveBaseTelemetry telemetry;
  /* An outer+inner pair is still an edge, not a completed turn. */
  uint8_t visible = (r->x1_black || r->x3_black) && !r->x2_black && !r->x4_black;
  command->valid = 0U;
  command->left_cps = command->right_cps = 0;
  command->action = side < 0 ? LINE_ACTION_SEARCH_LEFT : LINE_ACTION_SEARCH_RIGHT;
  DriveBase_Task(now);
  if (DriveBase_GetFaultMask()) LineRecovery_Stop(LINE_REC_STOP_DRIVE_FAULT);
  if (phase == REC_FAULT) return LINE_RECOVERY_FAILED;
  if (phase == REC_CAPTURED) return LINE_RECOVERY_CAPTURED;
  if (!(r->x1_black || r->x2_black || r->x3_black || r->x4_black)) audio_requested = 1U;
  if (audio_requested) search_audio(now);
  DriveBase_GetTelemetry(&telemetry);

  if (phase == REC_BRAKE_SEARCH || phase == REC_BRAKE_CAPTURE)
  {
    RecoveryPhase finished = phase;
    if (telemetry.mode == DRIVE_BASE_BRAKING) return LINE_RECOVERY_BUSY;
    DriveBase_Stop(DRIVE_STOP_COAST);
    phase = finished == REC_BRAKE_CAPTURE || visible ? REC_CONFIRM_CAPTURE : REC_SEARCH;
    phase_start = now;
    center_candidate = 0U;
  }
  else if (phase == REC_SEARCH)
  {
    /* Confirm while still turning, so a one-frame hit never drops torque. */
    if (!visible) center_candidate = 0U;
    else
    {
      if (!center_candidate) { center_candidate = 1U; center_since = now; }
      if (now - center_since >= SENSOR_CONFIRM_MS)
      {
        DriveBase_Stop(DRIVE_STOP_BRAKE);
        phase = REC_BRAKE_CAPTURE;
        center_candidate = 0U;
      }
    }
  }
  else if (phase == REC_CONFIRM_CAPTURE)
  {
    if (!visible) center_candidate = 0U;
    else
    {
      if (!center_candidate) { center_candidate = 1U; center_since = now; }
      if (now - center_since >= SENSOR_CONFIRM_MS)
      {
        stop_audio();
        phase = REC_CAPTURED;
        return LINE_RECOVERY_CAPTURED;
      }
    }
    /* A false hit resumes rotation. This timer never latches a stop. */
    if (now - phase_start >= CAPTURE_STATIONARY_MS)
    {
      phase = REC_SEARCH;
      center_candidate = 0U;
    }
  }
  if (phase == REC_SEARCH)
  {
    int32_t left = side < 0 ? -LINE_SEARCH_TARGET_CPS : LINE_SEARCH_TARGET_CPS;
    DriveBase_PrepareLineTurnAssist(left, -left);
    DriveBase_SetWheelCps(left, left, -left, -left);
  }
  return LINE_RECOVERY_BUSY;
}
