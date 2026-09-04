#include "line_obstacle_bypass.h"

#include "drive_base.h"
#include "encoder_linear.h"
#include "encoder_turn.h"
#include "main.h"
#include "motion_advanced.h"

/* Fault bits 0..3 are reserved for M1..M4 encoder/motor faults. */
#define BYPASS_FAULT_CONTROLLER            0x10U
#define BYPASS_FAULT_NET_TURN_LIMIT         0x20U
#define BYPASS_FAULT_INFRARED_INVALID       0x40U
#define BYPASS_FAULT_INPUT_INVALID          0x80U
#define BYPASS_ADC_MAX                      4095U

typedef enum
{
  BYPASS_RELATION_UNKNOWN = 0U,
  BYPASS_RELATION_TOO_CLOSE,
  BYPASS_RELATION_IN_BAND,
  BYPASS_RELATION_TOO_FAR
} BypassIrRelation;

typedef enum
{
  BYPASS_INTENT_NONE = 0U,
  BYPASS_INTENT_BACKUP,
  BYPASS_INTENT_ACQUIRE_FLANK,
  BYPASS_INTENT_FOLLOW_FLANK,
  BYPASS_INTENT_CLEAR_PROBE,
  BYPASS_INTENT_RETURN_TO_LINE
} BypassMotionIntent;

typedef enum
{
  BYPASS_GUIDED_TURN_NONE = 0U,
  BYPASS_GUIDED_TURN_UNTIL_SAFE,
  BYPASS_GUIDED_TURN_UNTIL_FLANK
} BypassGuidedTurn;

static LineObstacleBypassConfig bypass_config;
static LineObstacleBypassState bypass_state;
static BypassMotionIntent active_drive_intent;
static BypassMotionIntent after_turn_drive_intent;
static int8_t bypass_direction;
static uint8_t fault_mask;
static uint32_t phase_deadline_ms;
static int32_t net_turn_mdeg;
static uint32_t entry_speed_cps;
static uint8_t emergency_brake_active;

static uint8_t original_line_cleared;
static uint8_t line_clear_count;
static uint8_t line_confirm_count;
static uint8_t flank_acquired;
static uint8_t acquire_escape_committed;
static uint8_t return_aligned;
static uint8_t return_alignment_pending;
static uint8_t parallel_alignment_pending;
static uint8_t clear_probe_steps;
static uint32_t acquire_travel_mm;
static uint32_t flank_travel_mm;
static uint32_t return_travel_mm;
static uint32_t segment_accounted_mm;
static uint8_t latest_line_mask;
static int32_t return_target_mdeg;

static uint16_t inside_ir_adc;
static uint16_t inside_ir_lower;
static uint16_t inside_ir_upper;
static BypassIrRelation raw_relation;
static BypassIrRelation stable_relation;
static BypassIrRelation relation_candidate;
static uint8_t relation_candidate_count;
static uint32_t next_relation_sample_ms;

static int32_t active_turn_mdeg;
static uint8_t turn_steps_remaining;
static uint16_t after_turn_distance_mm;
static BypassGuidedTurn guided_turn_mode;

static uint8_t start_linear_motion(BypassMotionIntent intent,
                                   int32_t distance_mm,
                                   uint16_t cps);
static uint8_t start_turn_sequence(int32_t angle_mdeg,
                                   uint8_t steps,
                                   BypassMotionIntent next_intent,
                                   uint16_t next_distance_mm);
static uint8_t start_guided_turn(int32_t angle_mdeg,
                                 BypassGuidedTurn guided_mode,
                                 BypassMotionIntent next_intent,
                                 uint16_t next_distance_mm);
static void begin_parallel_escape(void);
static void start_parallel_alignment(void);
static void begin_return_to_line(void);
static void start_return_alignment(void);

static int32_t abs_i32(int32_t value)
{
  return value < 0L ? -value : value;
}

static uint8_t tick_reached(uint32_t now, uint32_t deadline)
{
  return (int32_t)(now - deadline) >= 0L ? 1U : 0U;
}

static void reset_relation_filter(void)
{
  stable_relation = BYPASS_RELATION_UNKNOWN;
  relation_candidate = BYPASS_RELATION_UNKNOWN;
  relation_candidate_count = 0U;
  next_relation_sample_ms = HAL_GetTick();
}

static void stop_motion_controllers(void)
{
  EncoderLinear_Stop();
  EncoderTurn_Stop();
  advanced_stop();
  guided_turn_mode = BYPASS_GUIDED_TURN_NONE;
}

static void enter_fault(uint8_t mask)
{
  EncoderLinear_Stop();
  EncoderTurn_Stop();
  DriveBase_Stop(DRIVE_STOP_BRAKE);
  guided_turn_mode = BYPASS_GUIDED_TURN_NONE;
  fault_mask = mask != 0U ? mask : BYPASS_FAULT_CONTROLLER;
  bypass_state = LINE_BYPASS_FAULT;
}

static void finish_done(void)
{
  stop_motion_controllers();
  bypass_state = LINE_BYPASS_DONE;
}

static int32_t outward_turn_mdeg(void)
{
  /* EncoderTurn uses positive=left. Bypass direction uses positive=right. */
  return bypass_direction > 0 ?
      -bypass_config.turn_step_mdeg : bypass_config.turn_step_mdeg;
}

static void update_inside_ir(const LineObstacleBypassInput *input)
{
  uint16_t threshold;
  uint16_t hysteresis;
  uint16_t band;
  uint32_t target;
  uint32_t upper;

  if (bypass_direction < 0)
  {
    inside_ir_adc = input->right_ir_adc;
    threshold = input->right_ir_threshold;
    hysteresis = input->right_ir_hysteresis;
  }
  else
  {
    inside_ir_adc = input->left_ir_adc;
    threshold = input->left_ir_threshold;
    hysteresis = input->left_ir_hysteresis;
  }

  /* Lower ADC means a closer obstacle. Center the tracking band on the
     calibrated release boundary (threshold + hysteresis), keeping the body
     farther from the obstacle than the original threshold-centered band. */
  band = (uint16_t)(hysteresis / 2U);
  if (band < bypass_config.minimum_band_adc)
  {
    band = bypass_config.minimum_band_adc;
  }
  target = (uint32_t)threshold + hysteresis;
  if (target > BYPASS_ADC_MAX)
  {
    target = BYPASS_ADC_MAX;
  }
  inside_ir_lower = target > band ?
      (uint16_t)(target - band) : 0U;
  upper = target + band;
  inside_ir_upper = upper < BYPASS_ADC_MAX ?
      (uint16_t)upper : BYPASS_ADC_MAX;

  if (inside_ir_adc < inside_ir_lower)
  {
    raw_relation = BYPASS_RELATION_TOO_CLOSE;
  }
  else if (inside_ir_adc > inside_ir_upper)
  {
    raw_relation = BYPASS_RELATION_TOO_FAR;
  }
  else
  {
    raw_relation = BYPASS_RELATION_IN_BAND;
  }
}

static void sample_relation(uint32_t now)
{
  uint8_t required = bypass_config.sensor_filter_samples;

  if (bypass_config.sensor_filter_interval_ms != 0U &&
      tick_reached(now, next_relation_sample_ms) == 0U)
  {
    return;
  }
  next_relation_sample_ms = now +
                            bypass_config.sensor_filter_interval_ms;
  if (required == 0U)
  {
    required = 1U;
  }
  if (raw_relation != relation_candidate)
  {
    relation_candidate = raw_relation;
    relation_candidate_count = 1U;
  }
  else if (relation_candidate_count < required)
  {
    ++relation_candidate_count;
  }
  if (relation_candidate_count >= required)
  {
    stable_relation = relation_candidate;
  }
}

static void account_drive_progress(void)
{
  uint32_t progress;
  uint32_t delta;

  if (active_drive_intent != BYPASS_INTENT_ACQUIRE_FLANK &&
      active_drive_intent != BYPASS_INTENT_RETURN_TO_LINE &&
      active_drive_intent != BYPASS_INTENT_FOLLOW_FLANK)
  {
    return;
  }

  progress = EncoderLinear_GetProgressMm();
  if (progress <= segment_accounted_mm)
  {
    return;
  }
  delta = progress - segment_accounted_mm;
  segment_accounted_mm = progress;
  if (active_drive_intent == BYPASS_INTENT_ACQUIRE_FLANK)
  {
    if (UINT32_MAX - acquire_travel_mm < delta)
    {
      acquire_travel_mm = UINT32_MAX;
    }
    else
    {
      acquire_travel_mm += delta;
    }
  }
  else if (active_drive_intent == BYPASS_INTENT_RETURN_TO_LINE)
  {
    if (UINT32_MAX - return_travel_mm < delta)
    {
      return_travel_mm = UINT32_MAX;
    }
    else
    {
      return_travel_mm += delta;
    }
  }
  else
  {
    if (UINT32_MAX - flank_travel_mm < delta)
    {
      flank_travel_mm = UINT32_MAX;
    }
    else
    {
      flank_travel_mm += delta;
    }
  }
}

static BypassIrRelation relation_for_decision(void)
{
  return stable_relation != BYPASS_RELATION_UNKNOWN ?
      stable_relation : raw_relation;
}

static uint8_t state_allows_line_reacquire(void)
{
  return (bypass_state == LINE_BYPASS_TURNING ||
          bypass_state == LINE_BYPASS_DRIVING ||
          bypass_state == LINE_BYPASS_EVALUATING) ? 1U : 0U;
}

static uint8_t line_reacquire_is_armed(void)
{
  return (flank_acquired != 0U ||
          (acquire_escape_committed != 0U &&
           flank_travel_mm >=
               bypass_config.blind_parallel_travel_mm)) ? 1U : 0U;
}

static void update_line_history(uint8_t line_mask)
{
  uint8_t clear_required = bypass_config.line_clear_samples;
  uint8_t confirm_required = bypass_config.line_confirm_samples;

  latest_line_mask = line_mask;
  if (clear_required == 0U)
  {
    clear_required = 1U;
  }
  if (confirm_required == 0U)
  {
    confirm_required = 1U;
  }

  if (line_mask == 0U)
  {
    if (line_clear_count < clear_required)
    {
      ++line_clear_count;
    }
    if (line_clear_count >= clear_required)
    {
      original_line_cleared = 1U;
    }
    line_confirm_count = 0U;
    return;
  }

  line_clear_count = 0U;
  if (original_line_cleared != 0U &&
      line_reacquire_is_armed() != 0U &&
      state_allows_line_reacquire() != 0U)
  {
    if (line_confirm_count < confirm_required)
    {
      ++line_confirm_count;
    }
  }
  else
  {
    line_confirm_count = 0U;
  }
}

static uint8_t line_reacquired(void)
{
  uint8_t required = bypass_config.line_confirm_samples;

  if (required == 0U)
  {
    required = 1U;
  }
  return (original_line_cleared != 0U &&
          line_reacquire_is_armed() != 0U &&
          line_confirm_count >= required) ? 1U : 0U;
}

static uint8_t acquire_turn_limit_reached(void)
{
  int32_t limit = bypass_config.acquire_turn_limit_mdeg;

  if (limit <= 0L)
  {
    limit = bypass_config.turn_step_mdeg;
  }
  return abs_i32(net_turn_mdeg) >= abs_i32(limit) ? 1U : 0U;
}

static void continue_acquire_without_circling(void)
{
  acquire_escape_committed = 1U;
  if (acquire_travel_mm >= bypass_config.acquire_max_travel_mm)
  {
    /* A missing flank reading must not send the car straight back across the
       same area in front of the obstacle.  First cancel the outward heading,
       then advance parallel to the original line before returning. */
    begin_parallel_escape();
  }
  else
  {
    (void)start_linear_motion(BYPASS_INTENT_ACQUIRE_FLANK,
                              (int32_t)bypass_config.forward_step_mm,
                              bypass_config.forward_cps);
  }
}

static uint8_t start_linear_motion(BypassMotionIntent intent,
                                   int32_t distance_mm,
                                   uint16_t cps)
{
  EncoderTurn_Stop();
  EncoderLinear_Stop();
  guided_turn_mode = BYPASS_GUIDED_TURN_NONE;
  if (EncoderLinear_Start(distance_mm, (int32_t)cps) == 0U)
  {
    enter_fault(BYPASS_FAULT_CONTROLLER);
    return 0U;
  }
  active_drive_intent = intent;
  segment_accounted_mm = 0U;
  reset_relation_filter();
  bypass_state = distance_mm < 0L ?
      LINE_BYPASS_REVERSING : LINE_BYPASS_DRIVING;
  return 1U;
}

static uint8_t start_next_turn_step(void)
{
  if (EncoderTurn_Start(active_turn_mdeg, 0L,
                        (int32_t)bypass_config.turn_cps) == 0U)
  {
    enter_fault(BYPASS_FAULT_CONTROLLER);
    return 0U;
  }
  bypass_state = LINE_BYPASS_TURNING;
  return 1U;
}

static uint8_t start_turn_sequence(int32_t angle_mdeg,
                                   uint8_t steps,
                                   BypassMotionIntent next_intent,
                                   uint16_t next_distance_mm)
{
  if (angle_mdeg == 0L || steps == 0U)
  {
    enter_fault(BYPASS_FAULT_CONTROLLER);
    return 0U;
  }

  EncoderLinear_Stop();
  EncoderTurn_Stop();
  active_turn_mdeg = angle_mdeg;
  turn_steps_remaining = steps;
  after_turn_drive_intent = next_intent;
  after_turn_distance_mm = next_distance_mm;
  active_drive_intent = next_intent;
  guided_turn_mode = BYPASS_GUIDED_TURN_NONE;
  reset_relation_filter();
  return start_next_turn_step();
}

static uint8_t start_guided_turn(int32_t angle_mdeg,
                                 BypassGuidedTurn guided_mode,
                                 BypassMotionIntent next_intent,
                                 uint16_t next_distance_mm)
{
  if (guided_mode == BYPASS_GUIDED_TURN_NONE)
  {
    return 0U;
  }
  if (start_turn_sequence(angle_mdeg, 1U, next_intent,
                          next_distance_mm) == 0U)
  {
    return 0U;
  }
  guided_turn_mode = guided_mode;
  return 1U;
}

static void begin_parallel_escape(void)
{
  acquire_escape_committed = 1U;
  flank_travel_mm = 0U;
  clear_probe_steps = 0U;
  return_alignment_pending = 0U;
  start_parallel_alignment();
}

static int32_t bounded_alignment_turn(int32_t correction_mdeg)
{
  int32_t step_mdeg = abs_i32(bypass_config.continuous_turn_limit_mdeg);

  if (step_mdeg == 0L)
  {
    step_mdeg = abs_i32(bypass_config.turn_step_mdeg);
  }
  if (step_mdeg == 0L)
  {
    step_mdeg = 45000L;
  }
  if (correction_mdeg > step_mdeg)
  {
    return step_mdeg;
  }
  if (correction_mdeg < -step_mdeg)
  {
    return -step_mdeg;
  }
  return correction_mdeg;
}

static void start_parallel_alignment(void)
{
  int32_t correction_mdeg = -net_turn_mdeg;
  int32_t tolerance_mdeg =
      abs_i32(bypass_config.return_heading_tolerance_mdeg);

  if (tolerance_mdeg == 0L)
  {
    tolerance_mdeg = 1000L;
  }
  /* Cancel the accumulated outward turn with one encoder-closed continuous
     segment (bounded by continuous_turn_limit_mdeg). */
  if (abs_i32(correction_mdeg) <= tolerance_mdeg)
  {
    parallel_alignment_pending = 0U;
    (void)start_linear_motion(BYPASS_INTENT_FOLLOW_FLANK,
                              (int32_t)bypass_config.forward_step_mm,
                              bypass_config.forward_cps);
  }
  else
  {
    parallel_alignment_pending = 1U;
    (void)start_turn_sequence(bounded_alignment_turn(correction_mdeg), 1U,
                              BYPASS_INTENT_FOLLOW_FLANK,
                              bypass_config.post_turn_step_mm);
  }
}

static void start_outward_follow(void)
{
  int32_t angle_mdeg = outward_turn_mdeg();
  int32_t limit_mdeg = abs_i32(bypass_config.continuous_turn_limit_mdeg);

  /* Let a close-side correction run continuously for up to two former
     15-degree steps, but stop it as soon as the filtered IR leaves TOO_CLOSE. */
  angle_mdeg *= 2L;
  if (limit_mdeg > 0L && abs_i32(angle_mdeg) > limit_mdeg)
  {
    angle_mdeg = angle_mdeg > 0L ? limit_mdeg : -limit_mdeg;
  }
  (void)start_guided_turn(angle_mdeg,
                          BYPASS_GUIDED_TURN_UNTIL_SAFE,
                          BYPASS_INTENT_FOLLOW_FLANK,
                          bypass_config.forward_step_mm);
}

static void start_outward_acquire(void)
{
  int32_t outward_step = outward_turn_mdeg();
  int32_t outward_sign = outward_step > 0L ? 1L : -1L;
  int32_t turn_limit = abs_i32(bypass_config.acquire_turn_limit_mdeg);
  int32_t continuous_limit =
      abs_i32(bypass_config.continuous_turn_limit_mdeg);
  int32_t outward_progress = net_turn_mdeg * outward_sign;
  int32_t remaining;
  BypassIrRelation relation = relation_for_decision();

  if (turn_limit == 0L)
  {
    turn_limit = abs_i32(outward_step);
  }
  if (continuous_limit == 0L)
  {
    continuous_limit = turn_limit;
  }

  /* The search-angle limit must never suppress an outward safety correction.
     When already too close, turn continuously for up to 30 degrees and let
     the filtered IR boundary request an earlier stop. */
  if (relation == BYPASS_RELATION_TOO_CLOSE)
  {
    remaining = abs_i32(outward_step) * 2L;
    if (remaining > continuous_limit)
    {
      remaining = continuous_limit;
    }
    (void)start_guided_turn(outward_sign * remaining,
                            BYPASS_GUIDED_TURN_UNTIL_SAFE,
                            BYPASS_INTENT_ACQUIRE_FLANK,
                            bypass_config.forward_step_mm);
    return;
  }

  if (outward_progress < 0L)
  {
    outward_progress = 0L;
  }
  remaining = turn_limit - outward_progress;
  if (remaining <= 0L)
  {
    continue_acquire_without_circling();
    return;
  }
  if (remaining > continuous_limit)
  {
    remaining = continuous_limit;
  }

  /* If the side is already close, stop when it becomes safe.  If the 45-degree
     sensor has not seen the flank yet, keep sweeping until it first sees the
     flank or the continuous angle limit is reached. */
  (void)start_guided_turn(outward_sign * remaining,
                          BYPASS_GUIDED_TURN_UNTIL_FLANK,
                          BYPASS_INTENT_ACQUIRE_FLANK,
                          bypass_config.forward_step_mm);
}

static int32_t configured_return_target(void)
{
  int32_t magnitude = abs_i32(bypass_config.return_heading_mdeg);

  if (magnitude == 0L)
  {
    magnitude = abs_i32(bypass_config.acquire_turn_limit_mdeg);
  }
  if (magnitude == 0L)
  {
    magnitude = abs_i32(bypass_config.turn_step_mdeg);
  }
  /* Positive EncoderTurn angle is left.  After bypassing to the right the
     return heading must point left across the original line, and vice versa. */
  return bypass_direction > 0 ? magnitude : -magnitude;
}

static void start_return_alignment(void)
{
  int32_t correction_mdeg = return_target_mdeg - net_turn_mdeg;
  int32_t tolerance_mdeg =
      abs_i32(bypass_config.return_heading_tolerance_mdeg);

  if (tolerance_mdeg == 0L)
  {
    tolerance_mdeg = 1000L;
  }
  if (abs_i32(correction_mdeg) <= tolerance_mdeg)
  {
    return_aligned = 1U;
    return_alignment_pending = 0U;
    (void)start_linear_motion(BYPASS_INTENT_RETURN_TO_LINE,
                              (int32_t)bypass_config.return_step_mm,
                              bypass_config.return_cps);
    return;
  }

  return_aligned = 0U;
  return_alignment_pending = 1U;
  (void)start_turn_sequence(bounded_alignment_turn(correction_mdeg), 1U,
                            BYPASS_INTENT_RETURN_TO_LINE,
                            bypass_config.post_turn_step_mm);
}

static void begin_return_to_line(void)
{
  return_target_mdeg = configured_return_target();
  return_travel_mm = 0U;
  return_aligned = 0U;
  return_alignment_pending = 0U;
  start_return_alignment();
}

static void start_return_safety_correction(void)
{
  int32_t angle_mdeg = outward_turn_mdeg() * 2L;
  int32_t limit_mdeg = abs_i32(bypass_config.continuous_turn_limit_mdeg);

  /* Move one step away from a close rear corner, then recompute the absolute
     return heading.  Do not change into flank-follow mode and do not keep
     steering throughout the whole return path. */
  return_aligned = 0U;
  return_alignment_pending = 0U;
  if (limit_mdeg > 0L && abs_i32(angle_mdeg) > limit_mdeg)
  {
    angle_mdeg = angle_mdeg > 0L ? limit_mdeg : -limit_mdeg;
  }
  (void)start_guided_turn(angle_mdeg,
                          BYPASS_GUIDED_TURN_UNTIL_SAFE,
                          BYPASS_INTENT_RETURN_TO_LINE,
                          bypass_config.post_turn_step_mm);
}

static void start_clear_probe(void)
{
  (void)start_linear_motion(BYPASS_INTENT_CLEAR_PROBE,
                            (int32_t)bypass_config.clear_probe_mm,
                            bypass_config.clear_probe_cps);
}

static void begin_direction_guard(void)
{
  EncoderLinear_Stop();
  advanced_stop();
  active_drive_intent = BYPASS_INTENT_NONE;
  bypass_state = LINE_BYPASS_DIRECTION_GUARD;
  phase_deadline_ms = HAL_GetTick() +
                      bypass_config.direction_guard_ms;
  reset_relation_filter();
}

static void begin_evaluation(void)
{
  EncoderLinear_Stop();
  advanced_stop();
  bypass_state = LINE_BYPASS_EVALUATING;
}

static void handle_evaluation(void)
{
  BypassIrRelation relation = relation_for_decision();

  if (line_reacquired() != 0U)
  {
    finish_done();
    return;
  }

  if (parallel_alignment_pending != 0U &&
      active_drive_intent == BYPASS_INTENT_FOLLOW_FLANK)
  {
    /* Every alignment turn has already been followed by a short safe drive.
       Recompute from the achieved net angle only after that translation. */
    parallel_alignment_pending = 0U;
    if (relation == BYPASS_RELATION_TOO_CLOSE)
    {
      clear_probe_steps = 0U;
      start_outward_follow();
    }
    else
    {
      start_parallel_alignment();
    }
    return;
  }

  switch (active_drive_intent)
  {
    case BYPASS_INTENT_ACQUIRE_FLANK:
      if (relation == BYPASS_RELATION_TOO_CLOSE)
      {
        clear_probe_steps = 0U;
        start_outward_acquire();
      }
      else if (relation == BYPASS_RELATION_IN_BAND)
      {
        flank_travel_mm = 0U;
        flank_acquired = 1U;
        clear_probe_steps = 0U;
        (void)start_linear_motion(BYPASS_INTENT_FOLLOW_FLANK,
                                  (int32_t)bypass_config.forward_step_mm,
                                  bypass_config.forward_cps);
      }
      else
      {
        /* Keep the selected bypass side locked, but do not keep turning in a
           circle when the 45-degree sensor never enters its narrow band.
           After about 45 degrees, hold heading and search forward. */
        if (acquire_turn_limit_reached() == 0U)
        {
          start_outward_acquire();
        }
        else
        {
          continue_acquire_without_circling();
        }
      }
      break;

    case BYPASS_INTENT_FOLLOW_FLANK:
      if (relation == BYPASS_RELATION_TOO_CLOSE)
      {
        clear_probe_steps = 0U;
        start_outward_follow();
      }
      else if (relation == BYPASS_RELATION_TOO_FAR)
      {
        uint32_t required_travel = flank_acquired != 0U ?
            bypass_config.minimum_flank_travel_mm :
            bypass_config.blind_parallel_travel_mm;

        clear_probe_steps = 0U;
        if (flank_travel_mm < required_travel)
        {
          /* A short loss near the front corner is not the rear edge.  If the
             side sensor was never acquired, use the longer geometry fallback
             so the car advances beyond the obstacle instead of returning to
             the black line behind it. */
          (void)start_linear_motion(BYPASS_INTENT_FOLLOW_FLANK,
                                    (int32_t)bypass_config.forward_step_mm,
                                    bypass_config.forward_cps);
        }
        else
        {
          start_clear_probe();
        }
      }
      else
      {
        clear_probe_steps = 0U;
        (void)start_linear_motion(BYPASS_INTENT_FOLLOW_FLANK,
                                  (int32_t)bypass_config.forward_step_mm,
                                  bypass_config.forward_cps);
      }
      break;

    case BYPASS_INTENT_CLEAR_PROBE:
      if (relation == BYPASS_RELATION_TOO_FAR)
      {
        if (clear_probe_steps < bypass_config.clear_confirm_steps)
        {
          ++clear_probe_steps;
        }
        if (clear_probe_steps >= bypass_config.clear_confirm_steps)
        {
          begin_return_to_line();
        }
        else
        {
          start_clear_probe();
        }
      }
      else
      {
        /* A single lost reading was not the rear edge. Resume flank tracking. */
        clear_probe_steps = 0U;
        flank_acquired = 1U;
        if (relation == BYPASS_RELATION_TOO_CLOSE)
        {
          start_outward_follow();
        }
        else
        {
          (void)start_linear_motion(BYPASS_INTENT_FOLLOW_FLANK,
                                    (int32_t)bypass_config.forward_step_mm,
                                    bypass_config.forward_cps);
        }
      }
      break;

    case BYPASS_INTENT_RETURN_TO_LINE:
      if (relation == BYPASS_RELATION_TOO_CLOSE)
      {
        /* A genuinely close square corner still interrupts the return.
           An in-band reading is ignored here because the 45-degree sensor
           can legitimately see the already-passed rear face. */
        clear_probe_steps = 0U;
        start_return_safety_correction();
      }
      else if (return_aligned == 0U)
      {
        start_return_alignment();
      }
      else
      {
        /* Once aligned, keep the same diagonal heading and search in straight
           encoder-limited segments.  Repeating a turn before every segment
           creates the circular path seen in the ground test. */
        (void)start_linear_motion(BYPASS_INTENT_RETURN_TO_LINE,
                                  (int32_t)bypass_config.return_step_mm,
                                  bypass_config.return_cps);
      }
      break;

    default:
      enter_fault(BYPASS_FAULT_CONTROLLER);
      break;
  }
}

void LineObstacleBypass_GetDefaultConfig(LineObstacleBypassConfig *config)
{
  if (config == 0)
  {
    return;
  }

  config->reverse_max_mm = 40U;
  config->forward_step_mm = 40U;
  config->clear_probe_mm = 20U;
  config->return_step_mm = 20U;
  config->post_turn_step_mm = 20U;
  config->stop_time_ms = 120U;
  config->emergency_stop_time_ms = 30U;
  config->direction_guard_ms = 80U;
  config->minimum_band_adc = 20U;
  config->minimum_flank_travel_mm = 120U;
  config->acquire_max_travel_mm = 120U;
  config->blind_parallel_travel_mm = 180U;
  config->sensor_filter_interval_ms = 20U;
  config->reverse_cps = 1400U;
  config->emergency_reverse_cps = 2600U;
  config->forward_cps = 2000U;
  config->clear_probe_cps = 1600U;
  config->return_cps = 1700U;
  config->turn_cps = 1800U;
  config->emergency_speed_cps = 3500U;
  config->turn_step_mdeg = 15000L;
  config->continuous_turn_limit_mdeg = 45000L;
  config->acquire_turn_limit_mdeg = 45000L;
  config->return_heading_mdeg = 45000L;
  config->return_heading_tolerance_mdeg = 8000L;
  config->maximum_net_turn_mdeg = 360000L;
  config->clear_confirm_steps = 2U;
  config->sensor_filter_samples = 3U;
  config->line_clear_samples = 3U;
  config->line_confirm_samples = 3U;
}

void LineObstacleBypass_Init(const LineObstacleBypassConfig *config)
{
  if (config != 0)
  {
    bypass_config = *config;
  }
  else
  {
    LineObstacleBypass_GetDefaultConfig(&bypass_config);
  }

  bypass_state = LINE_BYPASS_IDLE;
  active_drive_intent = BYPASS_INTENT_NONE;
  after_turn_drive_intent = BYPASS_INTENT_NONE;
  bypass_direction = 1;
  fault_mask = 0U;
  phase_deadline_ms = 0U;
  net_turn_mdeg = 0L;
  entry_speed_cps = 0U;
  emergency_brake_active = 0U;
  original_line_cleared = 0U;
  line_clear_count = 0U;
  line_confirm_count = 0U;
  flank_acquired = 0U;
  acquire_escape_committed = 0U;
  return_aligned = 0U;
  return_alignment_pending = 0U;
  parallel_alignment_pending = 0U;
  clear_probe_steps = 0U;
  acquire_travel_mm = 0U;
  flank_travel_mm = 0U;
  return_travel_mm = 0U;
  segment_accounted_mm = 0U;
  latest_line_mask = 0U;
  return_target_mdeg = 0L;
  inside_ir_adc = 0U;
  inside_ir_lower = 0U;
  inside_ir_upper = 0U;
  raw_relation = BYPASS_RELATION_UNKNOWN;
  active_turn_mdeg = 0L;
  turn_steps_remaining = 0U;
  after_turn_distance_mm = 0U;
  guided_turn_mode = BYPASS_GUIDED_TURN_NONE;
  reset_relation_filter();
}

uint8_t LineObstacleBypass_Start(int8_t direction)
{
  return LineObstacleBypass_StartWithSpeed(direction, 0U);
}

uint8_t LineObstacleBypass_StartWithSpeed(int8_t direction,
                                          uint32_t measured_entry_speed_cps)
{
  if (direction == 0)
  {
    return 0U;
  }

  /* Do not issue an unconditional coast-stop here: that would cancel the
     unified drive layer's non-blocking emergency brake just after detection. */
  EncoderLinear_Stop();
  EncoderTurn_Stop();
  guided_turn_mode = BYPASS_GUIDED_TURN_NONE;
  bypass_direction = direction > 0 ? 1 : -1;
  fault_mask = 0U;
  net_turn_mdeg = 0L;
  entry_speed_cps = measured_entry_speed_cps;
  emergency_brake_active =
      (bypass_config.emergency_speed_cps > 0U &&
       entry_speed_cps >= bypass_config.emergency_speed_cps) ? 1U : 0U;
  DriveBase_Stop(emergency_brake_active != 0U ?
                 DRIVE_STOP_BRAKE : DRIVE_STOP_COAST);
  original_line_cleared = 0U;
  line_clear_count = 0U;
  line_confirm_count = 0U;
  flank_acquired = 0U;
  acquire_escape_committed = 0U;
  return_aligned = 0U;
  return_alignment_pending = 0U;
  parallel_alignment_pending = 0U;
  clear_probe_steps = 0U;
  acquire_travel_mm = 0U;
  flank_travel_mm = 0U;
  return_travel_mm = 0U;
  segment_accounted_mm = 0U;
  latest_line_mask = 0U;
  return_target_mdeg = 0L;
  active_drive_intent = BYPASS_INTENT_NONE;
  after_turn_drive_intent = BYPASS_INTENT_NONE;
  active_turn_mdeg = 0L;
  turn_steps_remaining = 0U;
  after_turn_distance_mm = 0U;
  guided_turn_mode = BYPASS_GUIDED_TURN_NONE;
  reset_relation_filter();
  bypass_state = LINE_BYPASS_STOPPING;
  phase_deadline_ms = HAL_GetTick() +
      (emergency_brake_active != 0U ?
       bypass_config.emergency_stop_time_ms : bypass_config.stop_time_ms);
  return 1U;
}

void LineObstacleBypass_Task(const LineObstacleBypassInput *input)
{
  uint32_t now = HAL_GetTick();

  if (bypass_state == LINE_BYPASS_IDLE ||
      bypass_state == LINE_BYPASS_DONE ||
      bypass_state == LINE_BYPASS_FAULT)
  {
    if (bypass_state == LINE_BYPASS_DONE)
    {
      advanced_stop();
    }
    return;
  }
  if (input == 0)
  {
    enter_fault(BYPASS_FAULT_INPUT_INVALID);
    return;
  }

  update_inside_ir(input);
  sample_relation(now);
  update_line_history(input->line_mask);
  if (input->infrared_valid == 0U)
  {
    /* Side collision cannot be made safe without a valid inside IR channel. */
    enter_fault(BYPASS_FAULT_INFRARED_INVALID);
    return;
  }
  if (line_reacquired() != 0U)
  {
    finish_done();
    return;
  }

  switch (bypass_state)
  {
    case LINE_BYPASS_STOPPING:
      if (tick_reached(now, phase_deadline_ms) != 0U)
      {
        DriveBaseTelemetry drive_telemetry;
        uint16_t reverse_cps = emergency_brake_active != 0U ?
            bypass_config.emergency_reverse_cps : bypass_config.reverse_cps;

        DriveBase_GetTelemetry(&drive_telemetry);
        if (drive_telemetry.mode == DRIVE_BASE_BRAKING)
        {
          break;
        }
        if (reverse_cps == 0U)
        {
          reverse_cps = bypass_config.reverse_cps;
        }
        (void)start_linear_motion(BYPASS_INTENT_BACKUP,
                                  -(int32_t)bypass_config.reverse_max_mm,
                                  reverse_cps);
      }
      break;

    case LINE_BYPASS_REVERSING:
      EncoderLinear_Task();
      if (EncoderLinear_GetState() == ENCODER_LINEAR_FAULT)
      {
        enter_fault(EncoderLinear_GetFaultMask());
      }
      else if (stable_relation == BYPASS_RELATION_TOO_FAR)
      {
        /* Reversing already crossed the safe side-distance boundary. */
        begin_direction_guard();
      }
      else if (EncoderLinear_GetState() == ENCODER_LINEAR_DONE)
      {
        begin_direction_guard();
      }
      break;

    case LINE_BYPASS_DIRECTION_GUARD:
      advanced_stop();
      if (tick_reached(now, phase_deadline_ms) != 0U)
      {
        start_outward_acquire();
      }
      break;

    case LINE_BYPASS_TURNING:
      if ((guided_turn_mode == BYPASS_GUIDED_TURN_UNTIL_SAFE &&
           (stable_relation == BYPASS_RELATION_IN_BAND ||
            stable_relation == BYPASS_RELATION_TOO_FAR)) ||
          (guided_turn_mode == BYPASS_GUIDED_TURN_UNTIL_FLANK &&
           (stable_relation == BYPASS_RELATION_IN_BAND ||
            stable_relation == BYPASS_RELATION_TOO_CLOSE)))
      {
        /* Stop a continuous sensor-guided turn at the first stable boundary;
           EncoderTurn keeps counting through coast before reporting DONE. */
        (void)EncoderTurn_RequestStop();
        guided_turn_mode = BYPASS_GUIDED_TURN_NONE;
      }
      EncoderTurn_Task();
      if (EncoderTurn_GetState() == ENCODER_TURN_FAULT)
      {
        enter_fault(EncoderTurn_GetFaultMask());
      }
      else if (EncoderTurn_GetState() == ENCODER_TURN_DONE)
      {
        int32_t achieved_turn_mdeg =
            EncoderTurn_GetAchievedAngleMdeg();

        EncoderTurn_Stop();
        guided_turn_mode = BYPASS_GUIDED_TURN_NONE;
        if (achieved_turn_mdeg == 0L)
        {
          enter_fault(BYPASS_FAULT_CONTROLLER);
          break;
        }
        /* Count what the wheels actually completed.  Opposite turns retain
           their sign and therefore cancel in the net-angle guard. */
        net_turn_mdeg += achieved_turn_mdeg;
        if (abs_i32(net_turn_mdeg) >=
            bypass_config.maximum_net_turn_mdeg)
        {
          enter_fault(BYPASS_FAULT_NET_TURN_LIMIT);
        }
        else
        {
          if (turn_steps_remaining > 0U)
          {
            --turn_steps_remaining;
          }
          if (turn_steps_remaining > 0U)
          {
            (void)start_next_turn_step();
          }
          else if (after_turn_distance_mm > 0U)
          {
            BypassIrRelation relation = relation_for_decision();

            if (relation == BYPASS_RELATION_TOO_CLOSE)
            {
              /* The mandatory translation is allowed only while the selected
                 inside IR remains outside the close boundary. */
              after_turn_distance_mm = 0U;
              begin_evaluation();
            }
            else
            {
              uint16_t distance_mm = after_turn_distance_mm;
              uint16_t cps = bypass_config.forward_cps;

              after_turn_distance_mm = 0U;
              if (after_turn_drive_intent == BYPASS_INTENT_CLEAR_PROBE)
              {
                cps = bypass_config.clear_probe_cps;
              }
              else if (after_turn_drive_intent ==
                       BYPASS_INTENT_RETURN_TO_LINE)
              {
                cps = bypass_config.return_cps;
              }
              (void)start_linear_motion(after_turn_drive_intent,
                                        (int32_t)distance_mm,
                                        cps);
            }
          }
          else if (parallel_alignment_pending != 0U)
          {
            parallel_alignment_pending = 0U;
            start_parallel_alignment();
          }
          else if (return_alignment_pending != 0U)
          {
            return_alignment_pending = 0U;
            start_return_alignment();
          }
          else
          {
            bypass_state = LINE_BYPASS_EVALUATING;
          }
        }
      }
      break;

    case LINE_BYPASS_DRIVING:
      EncoderLinear_Task();
      account_drive_progress();
      if (stable_relation == BYPASS_RELATION_TOO_CLOSE ||
          stable_relation == BYPASS_RELATION_IN_BAND)
      {
        if (active_drive_intent != BYPASS_INTENT_ACQUIRE_FLANK &&
            active_drive_intent != BYPASS_INTENT_RETURN_TO_LINE)
        {
          if (flank_acquired == 0U)
          {
            /* Measure the minimum side-follow distance from the first stable
               side observation, not from the earlier blind offset leg. */
            flank_travel_mm = 0U;
          }
          flank_acquired = 1U;
        }
      }
      if (line_reacquired() != 0U)
      {
        finish_done();
      }
      else if (EncoderLinear_GetState() == ENCODER_LINEAR_FAULT)
      {
        enter_fault(EncoderLinear_GetFaultMask());
      }
      else if ((active_drive_intent == BYPASS_INTENT_ACQUIRE_FLANK &&
                stable_relation == BYPASS_RELATION_TOO_CLOSE) ||
               (active_drive_intent == BYPASS_INTENT_FOLLOW_FLANK &&
                stable_relation == BYPASS_RELATION_TOO_CLOSE) ||
               (active_drive_intent == BYPASS_INTENT_CLEAR_PROBE &&
                stable_relation != BYPASS_RELATION_UNKNOWN &&
                stable_relation != BYPASS_RELATION_TOO_FAR) ||
               (active_drive_intent == BYPASS_INTENT_RETURN_TO_LINE &&
                stable_relation == BYPASS_RELATION_TOO_CLOSE))
      {
        /* A close boundary remains an immediate safety interrupt. A far
           reading in flank-follow mode is evaluated only after the full
           40 mm step, preventing timid start-stop corrections. */
        begin_evaluation();
      }
      else if (EncoderLinear_GetState() == ENCODER_LINEAR_DONE)
      {
        begin_evaluation();
      }
      break;

    case LINE_BYPASS_EVALUATING:
      handle_evaluation();
      break;

    default:
      enter_fault(BYPASS_FAULT_CONTROLLER);
      break;
  }
}

void LineObstacleBypass_Stop(void)
{
  stop_motion_controllers();
  bypass_state = LINE_BYPASS_IDLE;
  active_drive_intent = BYPASS_INTENT_NONE;
  after_turn_drive_intent = BYPASS_INTENT_NONE;
  fault_mask = 0U;
  entry_speed_cps = 0U;
  emergency_brake_active = 0U;
  guided_turn_mode = BYPASS_GUIDED_TURN_NONE;
  reset_relation_filter();
}

LineObstacleBypassState LineObstacleBypass_GetState(void)
{
  return bypass_state;
}

uint8_t LineObstacleBypass_GetFaultMask(void)
{
  return fault_mask;
}

void LineObstacleBypass_GetTelemetry(LineObstacleBypassTelemetry *telemetry)
{
  if (telemetry == 0)
  {
    return;
  }

  telemetry->state = bypass_state;
  telemetry->bypass_direction = bypass_direction;
  telemetry->motion_intent = (uint8_t)active_drive_intent;
  telemetry->fault_mask = fault_mask;
  telemetry->line_mask = latest_line_mask;
  telemetry->original_line_cleared = original_line_cleared;
  telemetry->flank_acquired = flank_acquired;
  telemetry->acquire_escape_committed = acquire_escape_committed;
  telemetry->return_aligned = return_aligned;
  telemetry->clear_probe_steps = clear_probe_steps;
  telemetry->acquire_travel_mm = acquire_travel_mm;
  telemetry->flank_travel_mm = flank_travel_mm;
  telemetry->return_travel_mm = return_travel_mm;
  telemetry->entry_speed_cps = entry_speed_cps;
  telemetry->inside_ir_adc = inside_ir_adc;
  telemetry->inside_ir_lower = inside_ir_lower;
  telemetry->inside_ir_upper = inside_ir_upper;
  telemetry->segment_progress_mm = EncoderLinear_GetProgressMm();
  telemetry->net_turn_mdeg = net_turn_mdeg;
  telemetry->return_target_mdeg = return_target_mdeg;
  telemetry->emergency_brake_active = emergency_brake_active;
  telemetry->guided_turn_active =
      guided_turn_mode != BYPASS_GUIDED_TURN_NONE ? 1U : 0U;
}
