#include "drive_base.h"

#include "battery_monitor.h"
#include "main.h"
#include "motorPWM.h"
#include "wheel_encoder.h"
#include "line_turn_load.h"

#define DRIVE_CONTROL_PERIOD_MS                 20U
#define DRIVE_TARGET_RAMP_CPS_PER_PERIOD       700L
#define DRIVE_MAX_CPS                         9000L
#define DRIVE_CONTINUOUS_MIN_CPS              1412L
#define DRIVE_MAX_CORRECTION_PWM               500L
#define DRIVE_INTEGRAL_LIMIT               2000000L
#define DRIVE_STALL_TIMEOUT_MS                 1600U
#define DRIVE_DIRECTION_GUARD_MS                120U
#define DRIVE_DIRECTION_FAULT_SAMPLES             3U
#define DRIVE_ILLEGAL_FAULT_PER_SAMPLE             6U
#define DRIVE_ILLEGAL_FAULT_SAMPLES               3U
#define DRIVE_POSITION_DECEL_COUNTS              360L
#define DRIVE_POSITION_MIN_COMMAND_CPS            350L
#define DRIVE_POSITION_PULSE_ZONE_COUNTS          180L
#define DRIVE_POSITION_SYNC_GAIN_CPS              900L
#define DRIVE_POSITION_SYNC_LIMIT_CPS             500L
#define DRIVE_POSITION_SYNC_HOLD_PERMILLE          45L
#define DRIVE_POSITION_SYNC_FAULT_PERMILLE        550L
#define DRIVE_POSITION_SYNC_FAULT_MS              600U
#define DRIVE_POSITION_DEFAULT_TOLERANCE            12U
#define DRIVE_POSITION_SETTLE_MS                    90U
#define DRIVE_POSITION_COMPLETE_MAX_CPS            100L
#define DRIVE_PULSE_INITIAL_WIDTH_MS                 3U
#define DRIVE_PULSE_MIN_WIDTH_MS                     1U
#define DRIVE_PULSE_MAX_WIDTH_MS                    20U
#define DRIVE_PULSE_SETTLE_MS                       36U
#define DRIVE_PULSE_STABLE_MS                       12U
#define DRIVE_PULSE_START_STAGGER_MS                 25U
#define DRIVE_PULSE_MIN_COUNTS                       4L
#define DRIVE_PULSE_MAX_COUNTS                      24L
#define DRIVE_PULSE_NO_RESPONSE_LIMIT                20U
#define DRIVE_PULSE_WRONG_DIRECTION_LIMIT             3U
#define DRIVE_PULSE_MIN_PWM                       2200
#define DRIVE_PULSE_MAX_PWM                       3400
#define DRIVE_RECOVERY_BASE_DELAY_MS               220U
#define DRIVE_RECOVERY_STAGGER_MS                    40U
#define DRIVE_RECOVERY_BOOST_MS                      80U
#define DRIVE_BRAKE_GUARD_MS                         30U
#define DRIVE_BRAKE_PULSE_MS                         22U
#define DRIVE_BRAKE_PWM                            2400
#define DRIVE_CALIBRATION_VOLTAGE_MV               7800U
#define DRIVE_VOLTAGE_COMP_MIN_PERMILLE             850U
#define DRIVE_VOLTAGE_COMP_MAX_PERMILLE            1150U

#define DRIVE_CPS_AT_2200                          1412L
#define DRIVE_CPS_AT_2400                          2357L
#define DRIVE_CPS_AT_3000                          5273L
#define DRIVE_CPS_AT_MAX                           8250L

typedef enum
{
  DRIVE_BRAKE_NONE = 0U,
  DRIVE_BRAKE_GUARD,
  DRIVE_BRAKE_PULSE
} DriveBrakePhase;

static DriveBaseMode drive_mode;
static DrivePositionState position_state;
static DriveBrakePhase brake_phase;
static uint8_t fault_mask;
static uint8_t direction_fault_mask;
static uint8_t encoder_signal_fault_mask;
static uint8_t sync_fault;
static uint8_t position_completion_pending;
static uint8_t sync_fault_pending;
static uint32_t last_control_ms;
static uint32_t position_timeout_deadline_ms;
static uint32_t position_settle_deadline_ms;
static uint32_t sync_fault_since_ms;
static uint32_t brake_deadline_ms;
static int32_t requested_cps[DRIVE_BASE_WHEEL_COUNT];
static int32_t controlled_cps[DRIVE_BASE_WHEEL_COUNT];
static int32_t measured_cps[DRIVE_BASE_WHEEL_COUNT];
static int16_t output_pwm[DRIVE_BASE_WHEEL_COUNT];
static int32_t integral_error[DRIVE_BASE_WHEEL_COUNT];
static int8_t previous_command_sign[DRIVE_BASE_WHEEL_COUNT];
static int8_t brake_direction[DRIVE_BASE_WHEEL_COUNT];
static uint8_t direction_mismatch_streak[DRIVE_BASE_WHEEL_COUNT];
static uint8_t illegal_transition_streak[DRIVE_BASE_WHEEL_COUNT];
static uint32_t direction_guard_until_ms[DRIVE_BASE_WHEEL_COUNT];
static uint32_t no_motion_ms[DRIVE_BASE_WHEEL_COUNT];
static uint32_t recovery_boost_remaining_ms[DRIVE_BASE_WHEEL_COUNT];
static uint8_t recovery_boost_used[DRIVE_BASE_WHEEL_COUNT];
static uint32_t direction_mismatch_count[DRIVE_BASE_WHEEL_COUNT];
static uint32_t last_valid_motion_ms[DRIVE_BASE_WHEEL_COUNT];
static uint32_t previous_illegal_count[DRIVE_BASE_WHEEL_COUNT];
static WheelEncoderCounts previous_counts;
static WheelEncoderCounts position_start_counts;
static int32_t position_target_counts[DRIVE_BASE_WHEEL_COUNT];
static int32_t position_maximum_cps[DRIVE_BASE_WHEEL_COUNT];
static int32_t position_moved_counts[DRIVE_BASE_WHEEL_COUNT];
static int32_t position_remaining_counts[DRIVE_BASE_WHEEL_COUNT];
static int16_t normalized_progress_permille[DRIVE_BASE_WHEEL_COUNT];
static int16_t maximum_progress_spread_permille;
static int16_t position_mean_progress_permille;
static uint16_t position_tolerance_counts;
static DriveStopMode position_completion_stop_mode;
static uint8_t position_low_mode[DRIVE_BASE_WHEEL_COUNT];
static uint8_t pulse_active[DRIVE_BASE_WHEEL_COUNT];
static int8_t pulse_direction[DRIVE_BASE_WHEEL_COUNT];
static uint8_t pulse_width_ms[DRIVE_BASE_WHEEL_COUNT];
static int16_t pulse_pwm[DRIVE_BASE_WHEEL_COUNT];
static int32_t pulse_target_counts[DRIVE_BASE_WHEEL_COUNT];
static int32_t pulse_start_count[DRIVE_BASE_WHEEL_COUNT];
static int32_t pulse_last_observed_count[DRIVE_BASE_WHEEL_COUNT];
static uint32_t pulse_deadline_ms[DRIVE_BASE_WHEEL_COUNT];
static uint32_t pulse_restart_not_before_ms[DRIVE_BASE_WHEEL_COUNT];
static uint32_t pulse_stable_since_ms[DRIVE_BASE_WHEEL_COUNT];
static uint8_t pulse_response_pending[DRIVE_BASE_WHEEL_COUNT];
static int32_t pulse_last_response_counts[DRIVE_BASE_WHEEL_COUNT];
static uint8_t pulse_no_response_streak[DRIVE_BASE_WHEEL_COUNT];
static uint8_t pulse_wrong_direction_streak[DRIVE_BASE_WHEEL_COUNT];
static uint16_t battery_mv;
static uint16_t voltage_compensation_permille;
static uint8_t line_assist_pending, line_assist_active;
static uint32_t line_assist_prepared_ms, line_assist_accepted_ms;
static int32_t line_assist_left, line_assist_right;
static LineTurnLoadState line_load[DRIVE_BASE_WHEEL_COUNT];

static void reset_line_assist(void)
{
  uint8_t motor;
  line_assist_pending = line_assist_active = 0U;
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    line_load[motor].slow_ms = 0U;
    line_load[motor].extra_pwm = 0;
  }
}

/* Direction-specific tables are intentionally separate even where their
   initial values match. Reverse operation has not yet received an equivalent
   loaded calibration, so future calibration must not silently alter forward
   behavior. */
static const int16_t minimum_continuous_pwm[2][DRIVE_BASE_WHEEL_COUNT] =
{
  {2200, 2200, 2200, 2200}, /* reverse */
  {2200, 2200, 2200, 2200}  /* forward */
};
static const int16_t recovery_boost_pwm[2][DRIVE_BASE_WHEEL_COUNT] =
{
  {3350, 3350, 3250, 3350},
  {3350, 3350, 3250, 3350}
};
static const int16_t initial_position_pulse_pwm[2][DRIVE_BASE_WHEEL_COUNT] =
{
  {2600, 2600, 2550, 2600},
  {2600, 2600, 2550, 2600}
};

static int32_t abs_i32(int32_t value)
{
  return value < 0L ? -value : value;
}

static int8_t sign_i32(int32_t value)
{
  return value > 0L ? 1 : (value < 0L ? -1 : 0);
}

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

static uint8_t tick_reached(uint32_t now, uint32_t deadline)
{
  return (int32_t)(now - deadline) >= 0L ? 1U : 0U;
}

static int32_t interpolate(int32_t value,
                           int32_t x0,
                           int32_t y0,
                           int32_t x1,
                           int32_t y1)
{
  if (x1 == x0) return y0;
  return y0 + ((value - x0) * (y1 - y0) + (x1 - x0) / 2L) /
              (x1 - x0);
}

static int32_t count_for_motor(const WheelEncoderCounts *counts,
                               uint8_t motor)
{
  switch (motor)
  {
    case 0U: return counts->motor1;
    case 1U: return counts->motor2;
    case 2U: return counts->motor3;
    case 3U: return counts->motor4;
    default: return 0L;
  }
}

static uint8_t all_targets_zero(void)
{
  uint8_t motor;

  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    if (requested_cps[motor] != 0L) return 0U;
  }
  return 1U;
}

static void apply_motor_output(uint8_t motor, int16_t signed_pwm)
{
  int16_t magnitude = (int16_t)abs_i32((int32_t)signed_pwm);

  if (magnitude > (int16_t)MOTOR_PWM_PERIOD)
    magnitude = (int16_t)MOTOR_PWM_PERIOD;
  output_pwm[motor] = signed_pwm;
  switch (motor)
  {
    case 0U:
      if (signed_pwm > 0) pwm_motor1_forward(magnitude);
      else if (signed_pwm < 0) pwm_motor1_backward(magnitude);
      else pwm_motor1_forward(0);
      break;
    case 1U:
      if (signed_pwm > 0) pwm_motor2_forward(magnitude);
      else if (signed_pwm < 0) pwm_motor2_backward(magnitude);
      else pwm_motor2_forward(0);
      break;
    case 2U:
      if (signed_pwm > 0) pwm_motor3_forward(magnitude);
      else if (signed_pwm < 0) pwm_motor3_backward(magnitude);
      else pwm_motor3_forward(0);
      break;
    case 3U:
      if (signed_pwm > 0) pwm_motor4_forward(magnitude);
      else if (signed_pwm < 0) pwm_motor4_backward(magnitude);
      else pwm_motor4_forward(0);
      break;
    default:
      break;
  }
}

static void coast_all(void)
{
  uint8_t motor;

  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    apply_motor_output(motor, 0);
  }
}

static void reset_controller_state(uint32_t now)
{
  uint8_t motor;

  WheelEncoder_GetCounts(&previous_counts);
  last_control_ms = now;
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    integral_error[motor] = 0L;
    previous_command_sign[motor] = 0;
    direction_mismatch_streak[motor] = 0U;
    illegal_transition_streak[motor] = 0U;
    direction_guard_until_ms[motor] = now + DRIVE_DIRECTION_GUARD_MS;
    no_motion_ms[motor] = 0U;
    recovery_boost_remaining_ms[motor] = 0U;
    recovery_boost_used[motor] = 0U;
    pulse_active[motor] = 0U;
    position_low_mode[motor] = 0U;
    pulse_direction[motor] = 0;
    pulse_response_pending[motor] = 0U;
    pulse_last_response_counts[motor] = 0L;
    pulse_no_response_streak[motor] = 0U;
    pulse_wrong_direction_streak[motor] = 0U;
    /* Low-speed position pulses are intentionally staggered.  A spacing below
       the 20 ms control period made all four wheels eligible in the same pass,
       defeating the current-limiting intent of the stagger. */
    pulse_restart_not_before_ms[motor] =
        now + (uint32_t)motor * DRIVE_PULSE_START_STAGGER_MS;
    pulse_stable_since_ms[motor] = now;
  }
}

static int32_t cps_for_pwm_magnitude(int32_t pwm)
{
  pwm = clamp_i32(pwm, 0L, MOTOR_PWM_PERIOD);
  if (pwm == 0L) return 0L;
  if (pwm <= 2200L) return DRIVE_CPS_AT_2200;
  if (pwm <= 2400L)
  {
    return interpolate(pwm, 2200L, DRIVE_CPS_AT_2200,
                       2400L, DRIVE_CPS_AT_2400);
  }
  if (pwm <= 3000L)
  {
    return interpolate(pwm, 2400L, DRIVE_CPS_AT_2400,
                       3000L, DRIVE_CPS_AT_3000);
  }
  return interpolate(pwm, 3000L, DRIVE_CPS_AT_3000,
                     MOTOR_PWM_PERIOD, DRIVE_CPS_AT_MAX);
}

int32_t DriveBase_EquivalentCpsFromPwm(int16_t signed_pwm)
{
  int32_t magnitude = abs_i32((int32_t)signed_pwm);
  int32_t cps;

  if (magnitude == 0L) return 0L;
  cps = cps_for_pwm_magnitude(magnitude);
  return signed_pwm > 0 ? cps : -cps;
}

static int32_t pwm_for_cps_magnitude(int32_t cps)
{
  int32_t pwm;

  cps = abs_i32(cps);
  if (cps == 0L) return 0L;
  if (cps <= DRIVE_CPS_AT_2200)
  {
    pwm = 2200L;
  }
  else if (cps <= DRIVE_CPS_AT_2400)
  {
    pwm = interpolate(cps, DRIVE_CPS_AT_2200, 2200L,
                      DRIVE_CPS_AT_2400, 2400L);
  }
  else if (cps <= DRIVE_CPS_AT_3000)
  {
    pwm = interpolate(cps, DRIVE_CPS_AT_2400, 2400L,
                      DRIVE_CPS_AT_3000, 3000L);
  }
  else
  {
    pwm = interpolate(cps, DRIVE_CPS_AT_3000, 3000L,
                      DRIVE_CPS_AT_MAX, MOTOR_PWM_PERIOD);
  }
  return clamp_i32(pwm, 1L, MOTOR_PWM_PERIOD);
}

static void update_voltage_compensation(void)
{
  BatteryMonitorStatus battery;
  uint32_t compensation;

  BatteryMonitor_Get(&battery);
  battery_mv = battery.valid != 0U ? battery.millivolts : 0U;
  if (battery.valid == 0U || battery.millivolts < 6000U ||
      battery.millivolts > 9000U)
  {
    voltage_compensation_permille = 1000U;
    return;
  }

  compensation = ((uint32_t)DRIVE_CALIBRATION_VOLTAGE_MV * 1000U +
                  battery.millivolts / 2U) / battery.millivolts;
  if (compensation < DRIVE_VOLTAGE_COMP_MIN_PERMILLE)
    compensation = DRIVE_VOLTAGE_COMP_MIN_PERMILLE;
  if (compensation > DRIVE_VOLTAGE_COMP_MAX_PERMILLE)
    compensation = DRIVE_VOLTAGE_COMP_MAX_PERMILLE;
  voltage_compensation_permille = (uint16_t)compensation;
}

static int32_t ramp_toward(int32_t current, int32_t target)
{
  int32_t step = DRIVE_TARGET_RAMP_CPS_PER_PERIOD;

  target = clamp_i32(target, -DRIVE_MAX_CPS, DRIVE_MAX_CPS);
  if ((current > 0L && target < 0L) || (current < 0L && target > 0L))
  {
    target = 0L;
  }
  if (target > current + step) return current + step;
  if (target < current - step) return current - step;
  return target;
}

static int16_t speed_control_output(uint8_t motor,
                                    int32_t target_cps,
                                    int32_t delta_counts,
                                    uint32_t elapsed_ms)
{
  int32_t feedforward;
  int32_t target_delta_milli;
  int32_t actual_delta_milli;
  int32_t error;
  int32_t candidate_integral;
  int32_t correction;
  int32_t signed_output;
  int32_t minimum_output;
  int16_t line_extra;
  int8_t direction = sign_i32(target_cps);
  uint8_t direction_index = direction > 0 ? 1U : 0U;

  line_extra = LineTurnLoad_Update(&line_load[motor],
      (uint8_t)(drive_mode == DRIVE_BASE_SPEED && line_assist_active &&
                HAL_GetTick() - line_assist_accepted_ms < 60U &&
                target_cps == requested_cps[motor]),
      target_cps, measured_cps[motor], elapsed_ms);

  if (target_cps == 0L || elapsed_ms == 0U)
  {
    integral_error[motor] = 0L;
    recovery_boost_remaining_ms[motor] = 0U;
    recovery_boost_used[motor] = 0U;
    return 0;
  }

  if (recovery_boost_remaining_ms[motor] > 0U)
  {
    recovery_boost_remaining_ms[motor] =
        recovery_boost_remaining_ms[motor] > elapsed_ms ?
        recovery_boost_remaining_ms[motor] - elapsed_ms : 0U;
    return (int16_t)((int32_t)direction *
                     recovery_boost_pwm[direction_index][motor]);
  }

  feedforward = pwm_for_cps_magnitude(target_cps);
  feedforward = (feedforward * voltage_compensation_permille + 500L) / 1000L;
  feedforward = clamp_i32(feedforward, 1L, MOTOR_PWM_PERIOD);
  signed_output = (int32_t)direction * feedforward;
  target_delta_milli = target_cps * (int32_t)elapsed_ms;
  actual_delta_milli = delta_counts * 1000L;
  error = target_delta_milli - actual_delta_milli;
  candidate_integral = clamp_i32(integral_error[motor] + error,
                                 -DRIVE_INTEGRAL_LIMIT,
                                 DRIVE_INTEGRAL_LIMIT);
  correction = error / 250L + candidate_integral / 5000L;
  correction = clamp_i32(correction,
                         -DRIVE_MAX_CORRECTION_PWM,
                         DRIVE_MAX_CORRECTION_PWM);
  signed_output += correction;
  /* Keep the target, legacy PI gains and output rails. Only the lagging wheel
     receives bounded extra effort; saturation still participates in anti-windup. */
  signed_output += (int32_t)direction * line_extra;
  minimum_output = minimum_continuous_pwm[direction_index][motor];

  if (direction > 0)
  {
    if (signed_output > (int32_t)MOTOR_PWM_PERIOD && error > 0L)
    {
      /* Anti-windup: do not integrate an error that pushes farther into the
         positive output rail. */
    }
    else if (signed_output < minimum_output && error < 0L)
    {
      /* Same rule at the lower continuous-speed rail. */
    }
    else
    {
      integral_error[motor] = candidate_integral;
    }
    signed_output = clamp_i32(signed_output,
                              minimum_output,
                              MOTOR_PWM_PERIOD);
  }
  else
  {
    if (signed_output < -(int32_t)MOTOR_PWM_PERIOD && error < 0L)
    {
      /* Negative saturation. */
    }
    else if (signed_output > -minimum_output && error > 0L)
    {
      /* Negative lower rail. */
    }
    else
    {
      integral_error[motor] = candidate_integral;
    }
    signed_output = clamp_i32(signed_output,
                              -(int32_t)MOTOR_PWM_PERIOD,
                              -minimum_output);
  }
  return (int16_t)signed_output;
}

static void clear_motion_targets(void)
{
  uint8_t motor;

  reset_line_assist();

  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    requested_cps[motor] = 0L;
    controlled_cps[motor] = 0L;
    integral_error[motor] = 0L;
    no_motion_ms[motor] = 0U;
    recovery_boost_remaining_ms[motor] = 0U;
    recovery_boost_used[motor] = 0U;
    pulse_active[motor] = 0U;
    position_low_mode[motor] = 0U;
  }
}

static void finish_brake(uint32_t now)
{
  coast_all();
  brake_phase = DRIVE_BRAKE_NONE;
  drive_mode = fault_mask != 0U ? DRIVE_BASE_FAULT : DRIVE_BASE_STOPPED;
  if (position_completion_pending != 0U)
  {
    position_settle_deadline_ms = now + DRIVE_POSITION_SETTLE_MS;
  }
}

static void begin_brake(DriveStopMode mode,
                        uint8_t position_pending,
                        uint32_t now)
{
  uint8_t motor;
  uint8_t any_motion = 0U;

  if (brake_phase != DRIVE_BRAKE_NONE)
  {
    if (position_pending != 0U) position_completion_pending = 1U;
    return;
  }

  position_completion_pending = position_pending;
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    int8_t direction = sign_i32(controlled_cps[motor]);

    if (direction == 0) direction = sign_i32(output_pwm[motor]);
    if (direction == 0) direction = sign_i32(measured_cps[motor]);
    brake_direction[motor] = (int8_t)-direction;
    if (direction != 0 && abs_i32(measured_cps[motor]) >= 250L)
      any_motion = 1U;
  }
  clear_motion_targets();
  coast_all();

  if (mode == DRIVE_STOP_BRAKE && any_motion != 0U)
  {
    drive_mode = DRIVE_BASE_BRAKING;
    brake_phase = DRIVE_BRAKE_GUARD;
    brake_deadline_ms = now + DRIVE_BRAKE_GUARD_MS;
  }
  else
  {
    finish_brake(now);
  }
}

static void enter_fault(uint8_t mask, uint32_t now)
{
  fault_mask |= mask;
  position_state = DRIVE_POSITION_FAULT;
  position_completion_pending = 0U;
  begin_brake(DRIVE_STOP_BRAKE, 0U, now);
}

static void service_brake(uint32_t now)
{
  uint8_t motor;

  if (brake_phase == DRIVE_BRAKE_NONE) return;
  if (brake_phase == DRIVE_BRAKE_GUARD)
  {
    if (tick_reached(now, brake_deadline_ms) == 0U) return;
    for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
    {
      apply_motor_output(motor,
          (int16_t)((int32_t)brake_direction[motor] * DRIVE_BRAKE_PWM));
    }
    brake_phase = DRIVE_BRAKE_PULSE;
    brake_deadline_ms = now + DRIVE_BRAKE_PULSE_MS;
    return;
  }
  if (tick_reached(now, brake_deadline_ms) != 0U)
  {
    finish_brake(now);
  }
}

static void update_encoder_fault_inputs(const WheelEncoderCounts *current,
                                        const int32_t delta[4],
                                        uint32_t elapsed_ms,
                                        uint32_t now)
{
  WheelEncoderDiagnostics diagnostics;
  uint8_t motor;

  WheelEncoder_GetDiagnostics(&diagnostics);
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    uint32_t illegal_delta = diagnostics.illegal_transition_count[motor] -
                             previous_illegal_count[motor];
    int32_t target = controlled_cps[motor];
    int32_t forward_delta = delta[motor] * (int32_t)sign_i32(target);

    previous_illegal_count[motor] =
        diagnostics.illegal_transition_count[motor];
    if (illegal_delta >= DRIVE_ILLEGAL_FAULT_PER_SAMPLE)
    {
      if (illegal_transition_streak[motor] < 255U)
        ++illegal_transition_streak[motor];
    }
    else
    {
      illegal_transition_streak[motor] = 0U;
    }
    if (illegal_transition_streak[motor] >= DRIVE_ILLEGAL_FAULT_SAMPLES)
    {
      encoder_signal_fault_mask |= (uint8_t)(1U << motor);
    }

    if (drive_mode == DRIVE_BASE_POSITION &&
        position_low_mode[motor] != 0U)
    {
      /* A low-speed wheel is deliberately unpowered while its pulse settles
         or while it waits for lagging wheels.  Coast/rebound during that
         interval is not a commanded stall or wrong-direction event; pulse
         response is checked explicitly by finish_position_pulse(). */
      direction_mismatch_streak[motor] = 0U;
      no_motion_ms[motor] = 0U;
      continue;
    }

    if (target == 0L || abs_i32(target) < DRIVE_POSITION_MIN_COMMAND_CPS)
    {
      direction_mismatch_streak[motor] = 0U;
      no_motion_ms[motor] = 0U;
      continue;
    }

    if (forward_delta > 0L)
    {
      last_valid_motion_ms[motor] = now;
      no_motion_ms[motor] = 0U;
      recovery_boost_used[motor] = 0U;
      direction_mismatch_streak[motor] = 0U;
    }
    else if (delta[motor] == 0L)
    {
      no_motion_ms[motor] += elapsed_ms;
      if (abs_i32(target) >= DRIVE_CONTINUOUS_MIN_CPS &&
          no_motion_ms[motor] >= DRIVE_RECOVERY_BASE_DELAY_MS +
                                  (uint32_t)motor *
                                  DRIVE_RECOVERY_STAGGER_MS &&
          recovery_boost_remaining_ms[motor] == 0U &&
          recovery_boost_used[motor] == 0U)
      {
        recovery_boost_remaining_ms[motor] = DRIVE_RECOVERY_BOOST_MS;
        recovery_boost_used[motor] = 1U;
        integral_error[motor] = 0L;
      }
    }
    else if (tick_reached(now, direction_guard_until_ms[motor]) != 0U)
    {
      if (direction_mismatch_streak[motor] < 255U)
        ++direction_mismatch_streak[motor];
      ++direction_mismatch_count[motor];
      if (direction_mismatch_streak[motor] >=
          DRIVE_DIRECTION_FAULT_SAMPLES)
      {
        direction_fault_mask |= (uint8_t)(1U << motor);
      }
    }

    if (no_motion_ms[motor] >= DRIVE_STALL_TIMEOUT_MS)
    {
      fault_mask |= (uint8_t)(1U << motor);
    }
  }

  (void)current;
  if (direction_fault_mask != 0U)
  {
    fault_mask |= DRIVE_FAULT_DIRECTION;
    fault_mask |= direction_fault_mask;
  }
  if (encoder_signal_fault_mask != 0U)
  {
    fault_mask |= DRIVE_FAULT_ENCODER_SIGNAL;
    fault_mask |= encoder_signal_fault_mask;
  }
}

static void update_command_direction_guard(uint8_t motor,
                                           int32_t command,
                                           uint32_t now)
{
  int8_t next_sign = sign_i32(command);

  if (next_sign != previous_command_sign[motor])
  {
    previous_command_sign[motor] = next_sign;
    direction_guard_until_ms[motor] = now + DRIVE_DIRECTION_GUARD_MS;
    direction_mismatch_streak[motor] = 0U;
    integral_error[motor] = 0L;
    no_motion_ms[motor] = 0U;
    recovery_boost_used[motor] = 0U;
  }
}

static void update_position_geometry(const WheelEncoderCounts *current)
{
  int32_t minimum_progress = 32767L;
  int32_t maximum_progress = -32768L;
  int64_t progress_sum = 0LL;
  uint8_t active = 0U;
  uint8_t motor;

  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    int32_t target = position_target_counts[motor];
    int32_t moved = count_for_motor(current, motor) -
                    count_for_motor(&position_start_counts, motor);
    int32_t progress = 1000L;

    position_moved_counts[motor] = moved;
    position_remaining_counts[motor] = target - moved;
    if (target != 0L)
    {
      progress = (int32_t)(((int64_t)moved * 1000LL) / target);
      progress = clamp_i32(progress, -1000L, 2000L);
      ++active;
      progress_sum += progress;
      if (progress < minimum_progress) minimum_progress = progress;
      if (progress > maximum_progress) maximum_progress = progress;
    }
    normalized_progress_permille[motor] = (int16_t)progress;
  }

  if (active == 0U)
  {
    maximum_progress_spread_permille = 0;
  }
  else
  {
    (void)progress_sum;
    maximum_progress_spread_permille =
        (int16_t)(maximum_progress - minimum_progress);
  }
}

static int32_t position_command_for_motor(uint8_t motor,
                                          int32_t mean_progress,
                                          uint8_t finish_together)
{
  int32_t error = position_remaining_counts[motor];
  int32_t maximum = abs_i32(position_maximum_cps[motor]);
  int32_t remaining = abs_i32(error);
  int32_t magnitude;
  int32_t sync_correction;

  if (position_target_counts[motor] == 0L ||
      remaining <= (int32_t)position_tolerance_counts)
  {
    return 0L;
  }
  if (maximum < DRIVE_POSITION_MIN_COMMAND_CPS)
    maximum = DRIVE_POSITION_MIN_COMMAND_CPS;

  magnitude = maximum;
  if (remaining < DRIVE_POSITION_DECEL_COUNTS)
  {
    magnitude = DRIVE_POSITION_MIN_COMMAND_CPS +
        ((maximum - DRIVE_POSITION_MIN_COMMAND_CPS) * remaining) /
        DRIVE_POSITION_DECEL_COUNTS;
  }
  sync_correction = ((mean_progress -
      normalized_progress_permille[motor]) *
      DRIVE_POSITION_SYNC_GAIN_CPS) / 1000L;
  sync_correction = clamp_i32(sync_correction,
                              -DRIVE_POSITION_SYNC_LIMIT_CPS,
                              DRIVE_POSITION_SYNC_LIMIT_CPS);
  magnitude += sync_correction;
  magnitude = clamp_i32(magnitude,
                        DRIVE_POSITION_MIN_COMMAND_CPS,
                        maximum + DRIVE_POSITION_SYNC_LIMIT_CPS);
  if (finish_together != 0U && magnitude >= DRIVE_CONTINUOUS_MIN_CPS)
  {
    /* If one wheel has entered tolerance, the remaining wheels may only make
       bounded position-feedback pulses. This avoids three stopped wheels and
       one wheel continuing with high continuous torque. */
    magnitude = DRIVE_CONTINUOUS_MIN_CPS - 1L;
  }
  return (int32_t)sign_i32(error) * magnitude;
}

static void finish_position_pulse(uint8_t motor,
                                  int32_t response_counts,
                                  uint32_t now)
{
  apply_motor_output(motor, 0);
  pulse_active[motor] = 0U;
  pulse_restart_not_before_ms[motor] = now + DRIVE_PULSE_SETTLE_MS;
  pulse_stable_since_ms[motor] = now;
  pulse_last_observed_count[motor] = pulse_start_count[motor] +
      (int32_t)pulse_direction[motor] * response_counts;
  pulse_response_pending[motor] = 1U;
}

static uint8_t evaluate_position_pulse_response(uint8_t motor,
                                                int32_t response_counts,
                                                uint32_t now)
{
  int32_t desired = pulse_target_counts[motor];
  uint8_t motor_bit = (uint8_t)(1U << motor);

  pulse_response_pending[motor] = 0U;
  pulse_last_response_counts[motor] = response_counts > 0L ?
      response_counts : 0L;

  if (response_counts > 0L)
  {
    pulse_no_response_streak[motor] = 0U;
    pulse_wrong_direction_streak[motor] = 0U;
    last_valid_motion_ms[motor] = now;
  }
  else
  {
    if (pulse_no_response_streak[motor] < 255U)
      ++pulse_no_response_streak[motor];
    if (response_counts <= -DRIVE_PULSE_MIN_COUNTS)
    {
      if (pulse_wrong_direction_streak[motor] < 255U)
        ++pulse_wrong_direction_streak[motor];
      ++direction_mismatch_count[motor];
    }
    else
    {
      pulse_wrong_direction_streak[motor] = 0U;
    }

    if (pulse_wrong_direction_streak[motor] >=
        DRIVE_PULSE_WRONG_DIRECTION_LIMIT)
    {
      direction_fault_mask |= motor_bit;
      enter_fault((uint8_t)(DRIVE_FAULT_DIRECTION | motor_bit), now);
      return 0U;
    }
    if (pulse_no_response_streak[motor] >=
        DRIVE_PULSE_NO_RESPONSE_LIMIT)
    {
      enter_fault(motor_bit, now);
      return 0U;
    }
  }

  if (response_counts <= 0L)
  {
    if (pulse_width_ms[motor] < DRIVE_PULSE_MAX_WIDTH_MS)
      ++pulse_width_ms[motor];
    if (pulse_pwm[motor] <= DRIVE_PULSE_MAX_PWM - 100)
      pulse_pwm[motor] += 100;
  }
  else if (response_counts > desired + desired / 2L + 2L)
  {
    int32_t scaled_width =
        ((int32_t)pulse_width_ms[motor] * desired +
         response_counts - 1L) / response_counts;

    scaled_width = clamp_i32(scaled_width,
                             DRIVE_PULSE_MIN_WIDTH_MS,
                             DRIVE_PULSE_MAX_WIDTH_MS);
    if (scaled_width >= pulse_width_ms[motor] &&
        pulse_width_ms[motor] > DRIVE_PULSE_MIN_WIDTH_MS)
    {
      scaled_width = (int32_t)pulse_width_ms[motor] - 1L;
    }
    pulse_width_ms[motor] = (uint8_t)scaled_width;
    if (pulse_width_ms[motor] == DRIVE_PULSE_MIN_WIDTH_MS &&
        pulse_pwm[motor] >= DRIVE_PULSE_MIN_PWM + 100)
    {
      pulse_pwm[motor] -= 100;
    }
  }
  else if (response_counts + 2L < desired &&
           pulse_width_ms[motor] < DRIVE_PULSE_MAX_WIDTH_MS)
  {
    ++pulse_width_ms[motor];
  }
  return 1U;
}

static void service_position_pulses(uint32_t now,
                                    const WheelEncoderCounts *current)
{
  uint8_t motor;

  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    int32_t current_count = count_for_motor(current, motor);
    int32_t response;
    int32_t live_error;
    int32_t remaining;

    if (pulse_active[motor] != 0U)
    {
      response = (current_count - pulse_start_count[motor]) *
                 pulse_direction[motor];
      if (position_low_mode[motor] == 0U ||
          position_state != DRIVE_POSITION_RUNNING ||
          response >= pulse_target_counts[motor] ||
          tick_reached(now, pulse_deadline_ms[motor]) != 0U)
      {
        finish_position_pulse(motor, response, now);
      }
      continue;
    }

    if (current_count != pulse_last_observed_count[motor])
    {
      pulse_last_observed_count[motor] = current_count;
      pulse_stable_since_ms[motor] = now;
    }
    if (tick_reached(now, pulse_restart_not_before_ms[motor]) == 0U ||
        tick_reached(now, pulse_stable_since_ms[motor] +
                          DRIVE_PULSE_STABLE_MS) == 0U)
    {
      continue;
    }

    if (pulse_response_pending[motor] != 0U)
    {
      response = (current_count - pulse_start_count[motor]) *
                 pulse_direction[motor];
      if (evaluate_position_pulse_response(motor, response, now) == 0U)
      {
        return;
      }
    }
    if (position_low_mode[motor] == 0U ||
        position_state != DRIVE_POSITION_RUNNING)
    {
      continue;
    }

    live_error = position_target_counts[motor] -
        (current_count - count_for_motor(&position_start_counts, motor));
    position_remaining_counts[motor] = live_error;
    remaining = abs_i32(live_error);
    if (remaining <= (int32_t)position_tolerance_counts)
    {
      apply_motor_output(motor, 0);
      continue;
    }
    if ((int64_t)live_error *
        position_target_counts[motor] > 0LL &&
        normalized_progress_permille[motor] >
            position_mean_progress_permille +
            DRIVE_POSITION_SYNC_HOLD_PERMILLE)
    {
      /* This wheel is ahead along the requested path. Hold it until the
         normalized four-wheel progress catches up. */
      continue;
    }
    {
      int8_t next_direction = sign_i32(live_error);

      if (pulse_direction[motor] != 0 &&
          pulse_direction[motor] != next_direction)
      {
        /* An overshoot correction must start with the smallest learned-safe
           impulse, not the original high-energy launch pulse. */
        pulse_width_ms[motor] = DRIVE_PULSE_MIN_WIDTH_MS;
        pulse_pwm[motor] = DRIVE_PULSE_MIN_PWM;
        pulse_last_response_counts[motor] = 0L;
      }
      pulse_direction[motor] = next_direction;
    }
    pulse_target_counts[motor] = clamp_i32(remaining / 5L,
                                           DRIVE_PULSE_MIN_COUNTS,
                                           DRIVE_PULSE_MAX_COUNTS);
    if (remaining < pulse_target_counts[motor])
      pulse_target_counts[motor] = remaining;
    if (pulse_last_response_counts[motor] > pulse_target_counts[motor] &&
        pulse_width_ms[motor] > DRIVE_PULSE_MIN_WIDTH_MS)
    {
      int32_t scaled_width =
          ((int32_t)pulse_width_ms[motor] * pulse_target_counts[motor] +
           pulse_last_response_counts[motor] - 1L) /
          pulse_last_response_counts[motor];

      pulse_width_ms[motor] = (uint8_t)clamp_i32(
          scaled_width,
          DRIVE_PULSE_MIN_WIDTH_MS,
          pulse_width_ms[motor]);
    }
    pulse_start_count[motor] = current_count;
    pulse_last_observed_count[motor] = current_count;
    pulse_deadline_ms[motor] = now + pulse_width_ms[motor];
    pulse_active[motor] = 1U;
    {
      int32_t compensated_pwm =
          ((int32_t)pulse_pwm[motor] * voltage_compensation_permille +
           500L) / 1000L;
      int16_t minimum_pwm =
          minimum_continuous_pwm[pulse_direction[motor] > 0 ? 1U : 0U]
                                [motor];

      compensated_pwm = clamp_i32(compensated_pwm,
                                   minimum_pwm,
                                   DRIVE_PULSE_MAX_PWM);
      apply_motor_output(motor,
          (int16_t)((int32_t)pulse_direction[motor] * compensated_pwm));
    }
  }
}

static void control_speed_mode(const int32_t delta[4],
                               uint32_t elapsed_ms,
                               uint32_t now)
{
  uint8_t motor;

  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    controlled_cps[motor] = ramp_toward(controlled_cps[motor],
                                        requested_cps[motor]);
    update_command_direction_guard(motor, controlled_cps[motor], now);
  }
  update_encoder_fault_inputs(&previous_counts, delta, elapsed_ms, now);
  if (fault_mask != 0U)
  {
    enter_fault(fault_mask, now);
    return;
  }
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    apply_motor_output(motor,
        speed_control_output(motor, controlled_cps[motor],
                             delta[motor], elapsed_ms));
  }
}

static void control_position_mode(const WheelEncoderCounts *current,
                                  const int32_t delta[4],
                                  uint32_t elapsed_ms,
                                  uint32_t now)
{
  int64_t progress_sum = 0LL;
  int32_t mean_progress;
  uint8_t active = 0U;
  uint8_t all_complete = 1U;
  uint8_t some_complete = 0U;
  uint8_t motor;

  update_position_geometry(current);
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    if (position_target_counts[motor] != 0L)
    {
      progress_sum += normalized_progress_permille[motor];
      ++active;
      if (abs_i32(position_remaining_counts[motor]) >
          (int32_t)position_tolerance_counts)
      {
        all_complete = 0U;
      }
      else
      {
        some_complete = 1U;
        if (pulse_active[motor] != 0U ||
            pulse_response_pending[motor] != 0U ||
            abs_i32(measured_cps[motor]) >
                DRIVE_POSITION_COMPLETE_MAX_CPS ||
            tick_reached(now, pulse_stable_since_ms[motor] +
                              DRIVE_PULSE_STABLE_MS) == 0U)
        {
          all_complete = 0U;
        }
      }
    }
  }
  mean_progress = active != 0U ? (int32_t)(progress_sum / active) : 1000L;
  position_mean_progress_permille = (int16_t)mean_progress;

  if (all_complete != 0U)
  {
    position_state = DRIVE_POSITION_SETTLING;
    begin_brake(position_completion_stop_mode, 1U, now);
    return;
  }
  if (tick_reached(now, position_timeout_deadline_ms) != 0U)
  {
    enter_fault(DRIVE_FAULT_TIMEOUT, now);
    return;
  }

  if (maximum_progress_spread_permille >
      DRIVE_POSITION_SYNC_FAULT_PERMILLE)
  {
    if (sync_fault_pending == 0U)
    {
      sync_fault_pending = 1U;
      sync_fault_since_ms = now;
    }
    else if (tick_reached(now, sync_fault_since_ms +
                               DRIVE_POSITION_SYNC_FAULT_MS) != 0U)
    {
      sync_fault = 1U;
      enter_fault(DRIVE_FAULT_SYNC, now);
      return;
    }
  }
  else
  {
    sync_fault_pending = 0U;
  }

  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    controlled_cps[motor] =
        position_command_for_motor(motor, mean_progress, some_complete);
    requested_cps[motor] = controlled_cps[motor];
    update_command_direction_guard(motor, controlled_cps[motor], now);
    position_low_mode[motor] =
        (controlled_cps[motor] != 0L &&
         (abs_i32(controlled_cps[motor]) < DRIVE_CONTINUOUS_MIN_CPS ||
          abs_i32(position_remaining_counts[motor]) <
              DRIVE_POSITION_PULSE_ZONE_COUNTS)) ? 1U : 0U;
  }

  update_encoder_fault_inputs(current, delta, elapsed_ms, now);
  if (fault_mask != 0U)
  {
    enter_fault(fault_mask, now);
    return;
  }
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    if (position_low_mode[motor] == 0U)
    {
      pulse_active[motor] = 0U;
      apply_motor_output(motor,
          speed_control_output(motor, controlled_cps[motor],
                               delta[motor], elapsed_ms));
    }
  }
}

void DriveBase_Init(void)
{
  WheelEncoderDiagnostics diagnostics;
  uint32_t now = HAL_GetTick();
  uint8_t motor;

  drive_mode = DRIVE_BASE_STOPPED;
  position_state = DRIVE_POSITION_IDLE;
  brake_phase = DRIVE_BRAKE_NONE;
  reset_line_assist();
  fault_mask = 0U;
  direction_fault_mask = 0U;
  encoder_signal_fault_mask = 0U;
  sync_fault = 0U;
  position_completion_pending = 0U;
  sync_fault_pending = 0U;
  battery_mv = 0U;
  voltage_compensation_permille = 1000U;
  WheelEncoder_Start();
  WheelEncoder_GetDiagnostics(&diagnostics);
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    requested_cps[motor] = 0L;
    controlled_cps[motor] = 0L;
    measured_cps[motor] = 0L;
    output_pwm[motor] = 0;
    direction_mismatch_count[motor] = 0U;
    last_valid_motion_ms[motor] = now;
    previous_illegal_count[motor] =
        diagnostics.illegal_transition_count[motor];
    position_target_counts[motor] = 0L;
    position_maximum_cps[motor] = 0L;
    position_moved_counts[motor] = 0L;
    position_remaining_counts[motor] = 0L;
    normalized_progress_permille[motor] = 0;
    pulse_width_ms[motor] = DRIVE_PULSE_INITIAL_WIDTH_MS;
    pulse_pwm[motor] = initial_position_pulse_pwm[1][motor];
  }
  maximum_progress_spread_permille = 0;
  position_mean_progress_permille = 0;
  position_tolerance_counts = DRIVE_POSITION_DEFAULT_TOLERANCE;
  position_completion_stop_mode = DRIVE_STOP_COAST;
  reset_controller_state(now);
  coast_all();
}

void DriveBase_PrepareLineTurnAssist(int32_t left_cps, int32_t right_cps)
{
  line_assist_pending = (uint8_t)(left_cps != right_cps &&
      left_cps != 0L && right_cps != 0L &&
      left_cps >= -DRIVE_MAX_CPS && left_cps <= DRIVE_MAX_CPS &&
      right_cps >= -DRIVE_MAX_CPS && right_cps <= DRIVE_MAX_CPS);
  line_assist_left = left_cps;
  line_assist_right = right_cps;
  line_assist_prepared_ms = HAL_GetTick();
}

void DriveBase_SetWheelCps(int32_t motor1_cps,
                           int32_t motor2_cps,
                           int32_t motor3_cps,
                           int32_t motor4_cps)
{
  int32_t next[DRIVE_BASE_WHEEL_COUNT] =
      {motor1_cps, motor2_cps, motor3_cps, motor4_cps};
  uint32_t now = HAL_GetTick();
  uint8_t motor;
  uint8_t line_claim = (uint8_t)(line_assist_pending &&
      now - line_assist_prepared_ms <= 20U &&
      motor1_cps == line_assist_left && motor2_cps == line_assist_left &&
      motor3_cps == line_assist_right && motor4_cps == line_assist_right);

  line_assist_pending = 0U;

  if (fault_mask != 0U || brake_phase != DRIVE_BRAKE_NONE ||
      position_state == DRIVE_POSITION_RUNNING ||
      position_state == DRIVE_POSITION_SETTLING)
  {
    reset_line_assist();
    return;
  }
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    requested_cps[motor] = clamp_i32(next[motor],
                                     -DRIVE_MAX_CPS,
                                     DRIVE_MAX_CPS);
  }
  if (all_targets_zero() != 0U)
  {
    DriveBase_Stop(DRIVE_STOP_COAST);
    return;
  }
  if (drive_mode != DRIVE_BASE_SPEED)
  {
    position_state = DRIVE_POSITION_IDLE;
    clear_motion_targets();
    for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
      requested_cps[motor] = clamp_i32(next[motor],
                                       -DRIVE_MAX_CPS,
                                       DRIVE_MAX_CPS);
    reset_controller_state(now);
    drive_mode = DRIVE_BASE_SPEED;
  }
  if (!line_claim) reset_line_assist();
  else
  {
    if (now - line_assist_accepted_ms >= 60U)
    {
      /* A delayed caller cannot revive effort learned before its pause. */
      reset_line_assist();
    }
    line_assist_active = 1U;
    line_assist_accepted_ms = now;
  }
}

void DriveBase_SetSideCps(int32_t left_cps, int32_t right_cps)
{
  DriveBase_SetWheelCps(left_cps, left_cps, right_cps, right_cps);
}

uint8_t DriveBase_StartPositionMove(const DrivePositionCommand *command)
{
  WheelEncoderCounts current;
  uint32_t now = HAL_GetTick();
  uint8_t active = 0U;
  uint8_t motor;

  if (command == 0 || command->timeout_ms == 0U || fault_mask != 0U ||
      brake_phase != DRIVE_BRAKE_NONE)
  {
    return 0U;
  }
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    if (command->delta_counts[motor] != 0L &&
        command->maximum_cps[motor] != 0L)
    {
      ++active;
    }
  }
  if (active == 0U) return 0U;

  coast_all();
  clear_motion_targets();
  WheelEncoder_GetCounts(&current);
  position_start_counts = current;
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    int32_t target = command->delta_counts[motor];
    int32_t maximum = abs_i32(command->maximum_cps[motor]);

    position_target_counts[motor] = target;
    position_maximum_cps[motor] =
        target == 0L ? 0L : (int32_t)sign_i32(target) * maximum;
    position_moved_counts[motor] = 0L;
    position_remaining_counts[motor] = target;
    normalized_progress_permille[motor] = target == 0L ? 1000 : 0;
    pulse_width_ms[motor] = DRIVE_PULSE_INITIAL_WIDTH_MS;
    pulse_pwm[motor] = initial_position_pulse_pwm[
        target < 0L ? 0U : 1U][motor];
    pulse_direction[motor] = 0;
    pulse_response_pending[motor] = 0U;
    pulse_last_response_counts[motor] = 0L;
    pulse_no_response_streak[motor] = 0U;
    pulse_wrong_direction_streak[motor] = 0U;
    pulse_last_observed_count[motor] = count_for_motor(&current, motor);
    last_valid_motion_ms[motor] = now;
  }
  position_tolerance_counts = command->tolerance_counts != 0U ?
      command->tolerance_counts : DRIVE_POSITION_DEFAULT_TOLERANCE;
  position_completion_stop_mode = command->completion_stop_mode;
  position_timeout_deadline_ms = now + command->timeout_ms;
  position_settle_deadline_ms = 0U;
  position_completion_pending = 0U;
  sync_fault_pending = 0U;
  maximum_progress_spread_permille = 0;
  position_mean_progress_permille = 0;
  position_state = DRIVE_POSITION_RUNNING;
  drive_mode = DRIVE_BASE_POSITION;
  reset_controller_state(now);
  return 1U;
}

uint8_t DriveBase_RequestPositionStop(DriveStopMode mode)
{
  if (position_state != DRIVE_POSITION_RUNNING)
  {
    return position_state == DRIVE_POSITION_SETTLING ? 1U : 0U;
  }
  position_state = DRIVE_POSITION_SETTLING;
  begin_brake(mode, 1U, HAL_GetTick());
  return 1U;
}

DrivePositionState DriveBase_GetPositionState(void)
{
  return position_state;
}

uint8_t DriveBase_GetFaultMask(void)
{
  return fault_mask;
}

void DriveBase_ClearFault(void)
{
  WheelEncoderDiagnostics diagnostics;
  uint8_t motor;

  if (brake_phase != DRIVE_BRAKE_NONE) return;
  WheelEncoder_GetDiagnostics(&diagnostics);
  fault_mask = 0U;
  direction_fault_mask = 0U;
  encoder_signal_fault_mask = 0U;
  sync_fault = 0U;
  sync_fault_pending = 0U;
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    previous_illegal_count[motor] =
        diagnostics.illegal_transition_count[motor];
    illegal_transition_streak[motor] = 0U;
    direction_mismatch_streak[motor] = 0U;
    no_motion_ms[motor] = 0U;
    recovery_boost_used[motor] = 0U;
    pulse_response_pending[motor] = 0U;
    pulse_last_response_counts[motor] = 0L;
    pulse_no_response_streak[motor] = 0U;
    pulse_wrong_direction_streak[motor] = 0U;
  }
  if (position_state == DRIVE_POSITION_FAULT)
    position_state = DRIVE_POSITION_IDLE;
  drive_mode = DRIVE_BASE_STOPPED;
}

void DriveBase_Stop(DriveStopMode mode)
{
  uint32_t now = HAL_GetTick();

  if (mode == DRIVE_STOP_BRAKE && brake_phase != DRIVE_BRAKE_NONE)
  {
    return;
  }
  position_completion_pending = 0U;
  if (position_state == DRIVE_POSITION_RUNNING ||
      position_state == DRIVE_POSITION_SETTLING ||
      position_state == DRIVE_POSITION_DONE)
  {
    position_state = DRIVE_POSITION_IDLE;
  }
  if (mode == DRIVE_STOP_COAST)
  {
    brake_phase = DRIVE_BRAKE_NONE;
    clear_motion_targets();
    coast_all();
    drive_mode = fault_mask != 0U ? DRIVE_BASE_FAULT : DRIVE_BASE_STOPPED;
  }
  else
  {
    begin_brake(mode, 0U, now);
  }
}

void DriveBase_Task(uint32_t now_ms)
{
  WheelEncoderCounts current;
  int32_t delta[DRIVE_BASE_WHEEL_COUNT];
  uint32_t elapsed_ms;
  uint8_t motor;

  service_brake(now_ms);
  if (brake_phase != DRIVE_BRAKE_NONE) return;

  if (position_state == DRIVE_POSITION_SETTLING)
  {
    if (position_completion_pending != 0U &&
        tick_reached(now_ms, position_settle_deadline_ms) != 0U)
    {
      WheelEncoder_GetCounts(&current);
      update_position_geometry(&current);
      position_completion_pending = 0U;
      position_state = DRIVE_POSITION_DONE;
      drive_mode = DRIVE_BASE_STOPPED;
    }
    return;
  }
  WheelEncoder_GetCounts(&current);
  if (drive_mode == DRIVE_BASE_POSITION)
  {
    service_position_pulses(now_ms, &current);
    if (brake_phase != DRIVE_BRAKE_NONE ||
        drive_mode != DRIVE_BASE_POSITION)
    {
      return;
    }
  }
  elapsed_ms = now_ms - last_control_ms;
  if (elapsed_ms < DRIVE_CONTROL_PERIOD_MS) return;
  last_control_ms = now_ms;

  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    delta[motor] = count_for_motor(&current, motor) -
                   count_for_motor(&previous_counts, motor);
    measured_cps[motor] = elapsed_ms != 0U ?
        (delta[motor] * 1000L) / (int32_t)elapsed_ms : 0L;
  }
  update_voltage_compensation();
  if (drive_mode == DRIVE_BASE_SPEED)
  {
    control_speed_mode(delta, elapsed_ms, now_ms);
  }
  else
  {
    if (drive_mode == DRIVE_BASE_POSITION)
    {
      control_position_mode(&current, delta, elapsed_ms, now_ms);
    }
  }
  previous_counts = current;
}

void DriveBase_GetTelemetry(DriveBaseTelemetry *telemetry)
{
  WheelEncoderDiagnostics diagnostics;
  uint8_t motor;

  if (telemetry == 0) return;
  WheelEncoder_GetDiagnostics(&diagnostics);
  telemetry->mode = drive_mode;
  telemetry->position_state = position_state;
  telemetry->fault_mask = fault_mask;
  telemetry->direction_fault_mask = direction_fault_mask;
  telemetry->encoder_signal_fault_mask = encoder_signal_fault_mask;
  telemetry->sync_fault = sync_fault;
  telemetry->maximum_progress_spread_permille =
      maximum_progress_spread_permille;
  telemetry->battery_mv = battery_mv;
  telemetry->voltage_compensation_permille =
      voltage_compensation_permille;
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    telemetry->requested_cps[motor] = requested_cps[motor];
    telemetry->controlled_cps[motor] = controlled_cps[motor];
    telemetry->measured_cps[motor] = measured_cps[motor];
    telemetry->output_pwm[motor] = output_pwm[motor];
    telemetry->position_target_counts[motor] =
        position_target_counts[motor];
    telemetry->position_moved_counts[motor] =
        position_moved_counts[motor];
    telemetry->position_remaining_counts[motor] =
        position_remaining_counts[motor];
    telemetry->normalized_progress_permille[motor] =
        normalized_progress_permille[motor];
    telemetry->legal_transition_count[motor] =
        diagnostics.legal_transition_count[motor];
    telemetry->illegal_transition_count[motor] =
        diagnostics.illegal_transition_count[motor];
    telemetry->direction_mismatch_count[motor] =
        direction_mismatch_count[motor];
    telemetry->last_valid_motion_ms[motor] =
        last_valid_motion_ms[motor];
  }
}
