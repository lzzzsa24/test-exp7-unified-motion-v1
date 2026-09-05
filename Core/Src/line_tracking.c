/*
 * 实验七：PF13/PF14/PF15/PG0 四路数字循迹
 *
 * 传感器输出低电平表示黑线。该文件是黑线位置外环，只生成左右目标
 * CPS；DriveBase 使用四路编码器形成各轮速度内环。丢线搜索使用四轮
 * 差速速度闭环，按探头反馈换向并扩大搜索，不以固定转角判定失败。
 */

#include "line_tracking.h"

#include "drive_base.h"
#include "main.h"
#include "motorPWM.h"

static uint8_t no_line_forward_enabled = 1U;
static uint8_t line_has_been_seen;
static int8_t predicted_turn_direction;
static int8_t direction_candidate;
static uint32_t direction_candidate_since_ms;
static uint32_t direction_last_seen_ms;
static uint8_t direction_center_active;
static uint32_t direction_center_since_ms;
static uint8_t recovery_episode_active;
static uint32_t recovery_episode_started_ms;
static uint32_t recovery_leg_started_ms;
static uint32_t recovery_leg_duration_ms;
static int8_t recovery_outer_candidate;
static uint32_t recovery_outer_since_ms;
static int8_t recovery_turn_direction;
static int8_t outer_turn_direction;
static uint32_t outer_turn_started_ms;
static uint8_t recovery_center_candidate;
static uint32_t recovery_center_since_ms;
static uint8_t smooth_mode_enabled;
static uint8_t smooth_filter_valid;
static int16_t smooth_error_q8;
static int16_t smooth_previous_error_q8;
static uint32_t smooth_last_update_ms;
static uint8_t smooth_centered_active;
static uint32_t smooth_centered_since_ms;
static uint32_t smooth_ramp_update_ms;
static int16_t smooth_straight_pwm;
static uint16_t smooth_turn_gain_percent = 100U;

typedef enum
{
  LINE_RECOVERY_NORMAL = 0U,
  LINE_RECOVERY_WAIT_SEARCH,
  LINE_RECOVERY_TURN_SEARCH,
  LINE_RECOVERY_BRAKE_CAPTURE,
  LINE_RECOVERY_BRAKE_REVERSE,
  LINE_RECOVERY_STOPPED,
  LINE_RECOVERY_WAIT_FORWARD,
  LINE_RECOVERY_SETTLE
} LineRecoveryState;

static LineRecoveryState recovery_state;
static uint32_t recovery_state_started_ms;

#define TRACKING_DIRECTION_GUARD_MS          70U
#define TRACKING_SEARCH_FIRST_LEG_MS          900U
#define TRACKING_SEARCH_PROBE_LEG_MS          250U
#define TRACKING_SEARCH_MAX_LEG_MS           2400U
#define TRACKING_SEARCH_TURN_CPS             3600L
#define TRACKING_SEARCH_TIMEOUT_MS           8000U
#define TRACKING_HINT_CONFIRM_MS               20U
#define TRACKING_HINT_MAX_AGE_MS              200U
#define TRACKING_HINT_CENTER_CLEAR_MS          80U
#define TRACKING_SEARCH_CENTER_CONFIRM_MS      12U
#define TRACKING_REACQUIRE_SETTLE_MS         250U
#define TRACKING_MIN_INNER_PWM             2200
#define TRACKING_MIN_OUTER_PWM             3000
#define TRACKING_SETTLE_INNER_PWM           2200
#define TRACKING_SETTLE_OUTER_PWM           2700
#define TRACKING_SETTLE_CENTER_PWM          2200
#define TRACKING_NORMAL_CENTER_PWM          3000
#define TRACKING_OUTER_ROLL_IN_MS            120U
#define TRACKING_OUTER_SPIN_END_MS           280U
#define TRACKING_OUTER_ROLL_INNER_PWM       2200
#define TRACKING_OUTER_ROLL_OUTER_PWM       3400
#define TRACKING_OUTER_SPIN_PWM             3000
#define TRACKING_OUTER_ARC_INNER_PWM        2100
#define TRACKING_OUTER_ARC_OUTER_PWM        3300
#define TRACKING_SMOOTH_UPDATE_MS              10U
#define TRACKING_SMOOTH_STEER_LIMIT          1400
#define TRACKING_SMOOTH_STEER_DEADBAND        100
#define TRACKING_SMOOTH_CURVE_CENTER_PWM      2800
#define TRACKING_SMOOTH_CURVE_SLOWDOWN_PWM     100
#define TRACKING_SMOOTH_STRAIGHT_BASE_PWM      2800
#define TRACKING_SMOOTH_STRAIGHT_MAX_PWM       3000
#define TRACKING_SMOOTH_CENTER_HOLD_MS          350U
#define TRACKING_SMOOTH_RAMP_INTERVAL_MS         20U
#define TRACKING_SMOOTH_RAMP_STEP_PWM             20

static int16_t clamp_speed(int32_t speed)
{
  if (speed <= 0)
  {
    return 0;
  }

  if (speed >= (int32_t)MOTOR_PWM_PERIOD)
  {
    return (int16_t)MOTOR_PWM_PERIOD;
  }

  return (int16_t)speed;
}

static int16_t scale_speed(int16_t speed, uint16_t percent)
{
  return clamp_speed(((int32_t)speed * percent) / 100);
}

static int16_t ensure_minimum_speed(int16_t speed, int16_t minimum)
{
  if (speed < minimum)
  {
    return clamp_speed(minimum);
  }

  return speed;
}

static int16_t turn_speed_for_gain(int16_t normal_speed)
{
  int32_t extra_percent;
  int32_t speed;

  if (smooth_turn_gain_percent <= 100U)
  {
    return normal_speed;
  }
  extra_percent = (int32_t)smooth_turn_gain_percent - 100L;
  speed = (int32_t)normal_speed +
      (((int32_t)MOTOR_PWM_PERIOD - normal_speed) * extra_percent) / 100L;
  return clamp_speed(speed);
}

static void command_set_pwm(LineTrackingCommand *command,
                            int16_t left_pwm,
                            int16_t right_pwm,
                            LineTrackingAction action)
{
  if (command == 0) return;
  command->left_cps = DriveBase_EquivalentCpsFromPwm(left_pwm);
  command->right_cps = DriveBase_EquivalentCpsFromPwm(right_pwm);
  command->action = action;
  command->valid = 1U;
}

static void command_stop(LineTrackingCommand *command)
{
  command_set_pwm(command, 0, 0, LINE_ACTION_STOP);
}

static void command_release_to_drive(LineTrackingCommand *command,
                                        LineTrackingAction action)
{
  if (command == 0) return;
  command->left_cps = 0L;
  command->right_cps = 0L;
  command->action = action;
  command->valid = 0U;
}

static uint8_t read_black(GPIO_TypeDef *port, uint16_t pin)
{
  return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET ? 1U : 0U;
}

/* Observe sensor position, never the filtered motor correction. A newly
   opposing observation invalidates the old hint while it is being confirmed. */
static void update_direction_hint(const LineTrackingReading *reading,
                                  uint8_t active_count, uint32_t now)
{
  int16_t position = -3 * reading->x2_black - reading->x1_black +
                       reading->x3_black + 3 * reading->x4_black;
  int8_t side = position < 0 ? -1 : (position > 0 ? 1 : 0);

  if (reading->x2_black && reading->x4_black)
  {
    predicted_turn_direction = 0;
    direction_candidate = 0;
    direction_center_active = 0U;
    return;
  }
  if (active_count == 0U)
  {
    direction_candidate = 0;
    direction_center_active = 0U;
    return;
  }
  if (side == 0)
  {
    direction_candidate = 0;
    if (direction_center_active == 0U)
    {
      direction_center_active = 1U;
      direction_center_since_ms = now;
    }
    if (now - direction_center_since_ms >= TRACKING_HINT_CENTER_CLEAR_MS)
      predicted_turn_direction = 0;
    return;
  }
  direction_center_active = 0U;
  if (side != direction_candidate)
  {
    direction_candidate = side;
    direction_candidate_since_ms = now;
    if (predicted_turn_direction != side) predicted_turn_direction = 0;
  }
  if (now - direction_candidate_since_ms >= TRACKING_HINT_CONFIRM_MS)
  {
    predicted_turn_direction = side;
    direction_last_seen_ms = now;
  }
}

/* A leg timeout means try the other side, not a failed corner. A matching
   outer sensor can keep the current sweep alive until a middle sensor arrives.
   The episode watchdog remains independent of all leg/capture restarts. */
static void reverse_search(uint32_t now)
{
  DriveBase_Stop(DRIVE_STOP_BRAKE);
  recovery_turn_direction = (int8_t)-recovery_turn_direction;
  if (recovery_leg_duration_ms < TRACKING_SEARCH_MAX_LEG_MS)
  {
    recovery_leg_duration_ms *= 2U;
    if (recovery_leg_duration_ms > TRACKING_SEARCH_MAX_LEG_MS)
      recovery_leg_duration_ms = TRACKING_SEARCH_MAX_LEG_MS;
  }
  recovery_outer_candidate = 0;
  recovery_center_candidate = 0U;
  recovery_state = LINE_RECOVERY_BRAKE_REVERSE;
  recovery_state_started_ms = now;
}

void line_tracking_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = TRACK_X1_Pin | TRACK_X2_Pin | TRACK_X3_Pin;
  HAL_GPIO_Init(TRACK_X1_GPIO_Port, &gpio);

  gpio.Pin = TRACK_X4_Pin;
  HAL_GPIO_Init(TRACK_X4_GPIO_Port, &gpio);

  line_tracking_reset();
}

void line_tracking_reset(void)
{
  if (recovery_episode_active != 0U)
  {
    DriveBase_Stop(DRIVE_STOP_COAST);
  }
  line_has_been_seen = 0U;
  predicted_turn_direction = 0;
  direction_candidate = 0;
  direction_center_active = 0U;
  direction_candidate_since_ms = HAL_GetTick();
  direction_last_seen_ms = HAL_GetTick();
  direction_center_since_ms = HAL_GetTick();
  recovery_episode_active = 0U;
  recovery_episode_started_ms = HAL_GetTick();
  recovery_leg_started_ms = HAL_GetTick();
  recovery_leg_duration_ms = TRACKING_SEARCH_PROBE_LEG_MS;
  recovery_outer_candidate = 0;
  recovery_outer_since_ms = HAL_GetTick();
  recovery_turn_direction = 0;
  outer_turn_direction = 0;
  outer_turn_started_ms = HAL_GetTick();
  recovery_center_candidate = 0U;
  recovery_center_since_ms = HAL_GetTick();
  smooth_filter_valid = 0U;
  smooth_error_q8 = 0;
  smooth_previous_error_q8 = 0;
  smooth_last_update_ms = HAL_GetTick();
  smooth_centered_active = 0U;
  smooth_centered_since_ms = HAL_GetTick();
  smooth_ramp_update_ms = HAL_GetTick();
  smooth_straight_pwm = TRACKING_SMOOTH_STRAIGHT_BASE_PWM;
  recovery_state = LINE_RECOVERY_NORMAL;
  recovery_state_started_ms = HAL_GetTick();
}

void line_tracking_set_no_line_forward(uint8_t enable)
{
  no_line_forward_enabled = enable != 0U ? 1U : 0U;
}

void line_tracking_set_smooth_mode(uint8_t enable)
{
  smooth_mode_enabled = enable != 0U ? 1U : 0U;
  smooth_filter_valid = 0U;
  smooth_error_q8 = 0;
  smooth_previous_error_q8 = 0;
  smooth_last_update_ms = HAL_GetTick();
  smooth_centered_active = 0U;
  smooth_centered_since_ms = HAL_GetTick();
  smooth_ramp_update_ms = HAL_GetTick();
  smooth_straight_pwm = TRACKING_SMOOTH_STRAIGHT_BASE_PWM;
}

void line_tracking_set_turn_gain_percent(uint16_t percent)
{
  if (percent < 50U)
  {
    percent = 50U;
  }
  else if (percent > 200U)
  {
    percent = 200U;
  }
  smooth_turn_gain_percent = percent;
}

LineTrackingReading line_tracking_read(void)
{
  LineTrackingReading reading;

  reading.x1_black = read_black(TRACK_X1_GPIO_Port, TRACK_X1_Pin);
  reading.x2_black = read_black(TRACK_X2_GPIO_Port, TRACK_X2_Pin);
  reading.x3_black = read_black(TRACK_X3_GPIO_Port, TRACK_X3_Pin);
  reading.x4_black = read_black(TRACK_X4_GPIO_Port, TRACK_X4_Pin);
  return reading;
}

LineTrackingAction line_tracking_compute(const LineTrackingReading *reading,
                                         int16_t base_speed,
                                         LineTrackingCommand *command)
{
  int16_t turn_inner_speed;
  int16_t turn_outer_speed;
  int16_t center_speed;
  int16_t crossing_speed;
  int16_t weighted_sum;
  int16_t line_position;
  uint8_t active_count;
  uint8_t center_visible;
  uint8_t settling = 0U;
  uint32_t now = HAL_GetTick();

  if (command == 0 || reading == 0)
  {
    return LINE_ACTION_STOP;
  }
  command_stop(command);

  if (base_speed <= 0)
  {
    line_tracking_reset();
    command_stop(command);
    return LINE_ACTION_STOP;
  }

  active_count = (uint8_t)(reading->x1_black + reading->x2_black +
                           reading->x3_black + reading->x4_black);
  center_visible = (reading->x1_black || reading->x3_black) ? 1U : 0U;
  if (active_count != 0U)
  {
    line_has_been_seen = 1U;
  }

  if (recovery_state == LINE_RECOVERY_NORMAL)
  {
    update_direction_hint(reading, active_count, now);
  }

  if (recovery_episode_active != 0U &&
      (DriveBase_GetFaultMask() != 0U ||
       now - recovery_episode_started_ms >= TRACKING_SEARCH_TIMEOUT_MS))
  {
    DriveBase_Stop(DRIVE_STOP_COAST);
    recovery_state = LINE_RECOVERY_STOPPED;
    command_stop(command);
    return LINE_ACTION_STOP;
  }

  /* Search owns DriveBase, using the same four-wheel speed feedback as normal
     tracking. No position target or chassis-angle estimate ends a corner. */
  if (recovery_state == LINE_RECOVERY_TURN_SEARCH)
  {
    int8_t outer_side = reading->x2_black && !reading->x4_black ? -1 :
                       (reading->x4_black && !reading->x2_black ? 1 : 0);
    LineTrackingAction search_action = recovery_turn_direction < 0
                                     ? LINE_ACTION_SEARCH_LEFT
                                     : LINE_ACTION_SEARCH_RIGHT;
    command_release_to_drive(command, search_action);
    DriveBase_Task(now);
    if (DriveBase_GetFaultMask() != 0U)
    {
      DriveBase_Stop(DRIVE_STOP_COAST);
      recovery_state = LINE_RECOVERY_STOPPED;
      command_stop(command);
      return LINE_ACTION_STOP;
    }

    if (center_visible != 0U)
    {
      recovery_outer_candidate = 0;
      if (recovery_center_candidate == 0U)
      {
        recovery_center_candidate = 1U;
        recovery_center_since_ms = now;
      }
      else if (now - recovery_center_since_ms >=
               TRACKING_SEARCH_CENTER_CONFIRM_MS)
      {
        DriveBase_Stop(DRIVE_STOP_BRAKE);
        recovery_center_candidate = 0U;
        recovery_state = LINE_RECOVERY_BRAKE_CAPTURE;
        recovery_state_started_ms = now;
        return search_action;
      }
    }
    else
    {
      recovery_center_candidate = 0U;
      if (outer_side != 0 && outer_side != recovery_turn_direction)
      {
        if (recovery_outer_candidate != outer_side)
        {
          recovery_outer_candidate = outer_side;
          recovery_outer_since_ms = now;
        }
        else if (now - recovery_outer_since_ms >= TRACKING_HINT_CONFIRM_MS)
        {
          reverse_search(now);
          return search_action;
        }
      }
      else
      {
        recovery_outer_candidate = 0;
      }
      /* A line on the side we are approaching is useful evidence even beyond
         the nominal leg duration. All-white or ambiguous outer hits are not. */
      if (now - recovery_leg_started_ms >= recovery_leg_duration_ms &&
          outer_side != recovery_turn_direction)
      {
        reverse_search(now);
        return search_action;
      }
    }

    DriveBase_SetSideCps(recovery_turn_direction < 0
                        ? -TRACKING_SEARCH_TURN_CPS : TRACKING_SEARCH_TURN_CPS,
                        recovery_turn_direction < 0
                        ? TRACKING_SEARCH_TURN_CPS : -TRACKING_SEARCH_TURN_CPS);
    return search_action;
  }

  if (recovery_state == LINE_RECOVERY_BRAKE_CAPTURE ||
      recovery_state == LINE_RECOVERY_BRAKE_REVERSE)
  {
    DriveBaseTelemetry drive_telemetry;
    uint8_t reversing = recovery_state == LINE_RECOVERY_BRAKE_REVERSE;
    LineTrackingAction search_action = recovery_turn_direction < 0
                                     ? LINE_ACTION_SEARCH_LEFT
                                     : LINE_ACTION_SEARCH_RIGHT;
    command_release_to_drive(command, search_action);
    DriveBase_Task(now);
    DriveBase_GetTelemetry(&drive_telemetry);
    if (drive_telemetry.fault_mask != 0U)
    {
      DriveBase_Stop(DRIVE_STOP_COAST);
      recovery_state = LINE_RECOVERY_STOPPED;
      command_stop(command);
      return LINE_ACTION_STOP;
    }
    if (drive_telemetry.mode == DRIVE_BASE_BRAKING)
      return search_action;

    command_stop(command);
    recovery_state = (reversing == 0U && center_visible != 0U)
                   ? LINE_RECOVERY_SETTLE : LINE_RECOVERY_WAIT_SEARCH;
    recovery_state_started_ms = now;
    recovery_center_candidate = 0U;
    return LINE_ACTION_STOP;
  }

  if (recovery_state == LINE_RECOVERY_WAIT_SEARCH)
  {
    command_stop(command);
    if (center_visible != 0U)
    {
      if (recovery_center_candidate == 0U)
      {
        recovery_center_candidate = 1U;
        recovery_center_since_ms = now;
      }
      else if (now - recovery_center_since_ms >=
               TRACKING_SEARCH_CENTER_CONFIRM_MS)
      {
        recovery_state = LINE_RECOVERY_SETTLE;
        recovery_state_started_ms = now;
        recovery_center_candidate = 0U;
      }
      return LINE_ACTION_STOP;
    }
    recovery_center_candidate = 0U;
    if (now - recovery_state_started_ms >= TRACKING_DIRECTION_GUARD_MS)
    {
      recovery_outer_candidate = 0;
      recovery_state = LINE_RECOVERY_TURN_SEARCH;
      recovery_leg_started_ms = now;
      recovery_state_started_ms = now;
      command_release_to_drive(command,
          recovery_turn_direction < 0 ? LINE_ACTION_SEARCH_LEFT
                                      : LINE_ACTION_SEARCH_RIGHT);
      DriveBase_SetSideCps(recovery_turn_direction < 0
                          ? -TRACKING_SEARCH_TURN_CPS : TRACKING_SEARCH_TURN_CPS,
                          recovery_turn_direction < 0
                          ? TRACKING_SEARCH_TURN_CPS : -TRACKING_SEARCH_TURN_CPS);
      return command->action;
    }
    return LINE_ACTION_STOP;
  }

  /* During the stationary guard and low-speed settle, an outer-only hit is
     still not a captured line. Re-enter the same-direction search without
     allowing the old forward/arc command to produce a one-step lurch. */
  if ((recovery_state == LINE_RECOVERY_WAIT_FORWARD ||
       recovery_state == LINE_RECOVERY_SETTLE) &&
      center_visible == 0U)
  {
    command_stop(command);
    recovery_center_candidate = 0U;
    recovery_state = LINE_RECOVERY_WAIT_SEARCH;
    recovery_state_started_ms = now;
    return LINE_ACTION_STOP;
  }

  if (recovery_state == LINE_RECOVERY_STOPPED)
  {
    command_stop(command);
    /* Only a drive fault or the whole-episode watchdog requires mode reset. */
    return LINE_ACTION_STOP;
  }

  if (active_count == 0U)
  {
    outer_turn_direction = 0;
    smooth_filter_valid = 0U;
    smooth_centered_active = 0U;
    smooth_straight_pwm = TRACKING_SMOOTH_STRAIGHT_BASE_PWM;
    /* KEY1 上电后从未见过黑线时仍按无黑线方案前进；一旦已经进入
       寻线，或处于 KEY2 纯寻线模式，全白就按预测方向原地搜线。 */
    if (no_line_forward_enabled != 0U && line_has_been_seen == 0U)
    {
      recovery_state = LINE_RECOVERY_NORMAL;
      command_set_pwm(command, base_speed, base_speed,
                      LINE_ACTION_FORWARD);
      return LINE_ACTION_FORWARD;
    }

    if (recovery_state == LINE_RECOVERY_NORMAL ||
        recovery_state == LINE_RECOVERY_WAIT_FORWARD ||
        recovery_state == LINE_RECOVERY_SETTLE)
    {
      command_stop(command);
      recovery_turn_direction = predicted_turn_direction;
      if (now - direction_last_seen_ms > TRACKING_HINT_MAX_AGE_MS)
        recovery_turn_direction = 0;
      recovery_episode_active = 1U;
      recovery_episode_started_ms = now;
      recovery_leg_duration_ms = recovery_turn_direction != 0
                               ? TRACKING_SEARCH_FIRST_LEG_MS
                               : TRACKING_SEARCH_PROBE_LEG_MS;
      /* No hint is uncertainty, not a stop condition. Start a short left
         probe, then alternate increasingly long legs if no sensor responds. */
      if (recovery_turn_direction == 0) recovery_turn_direction = -1;
      recovery_state = LINE_RECOVERY_WAIT_SEARCH;
      recovery_center_candidate = 0U;
      recovery_state_started_ms = now;
      return LINE_ACTION_STOP;
    }

    if (recovery_state == LINE_RECOVERY_STOPPED)
    {
      command_stop(command);
      return LINE_ACTION_STOP;
    }

    command_stop(command);
    return LINE_ACTION_STOP;
  }

  if (recovery_state == LINE_RECOVERY_WAIT_FORWARD)
  {
    command_stop(command);
    if (now - recovery_state_started_ms < TRACKING_DIRECTION_GUARD_MS)
    {
      return LINE_ACTION_STOP;
    }
    recovery_state = LINE_RECOVERY_SETTLE;
    recovery_state_started_ms = now;
    settling = 1U;
  }
  else if (recovery_state == LINE_RECOVERY_SETTLE)
  {
    if (now - recovery_state_started_ms < TRACKING_REACQUIRE_SETTLE_MS)
    {
      settling = 1U;
    }
    else
    {
      recovery_state = LINE_RECOVERY_NORMAL;
      predicted_turn_direction = 0;
      recovery_turn_direction = 0;
      direction_candidate = 0;
      direction_center_active = 0U;
      recovery_episode_active = 0U;
    }
  }

  if (settling != 0U)
  {
    /* 重新捕线后的 250 ms 内限制速度，转向仅按当前探头位置。 */
    turn_inner_speed = TRACKING_SETTLE_INNER_PWM;
    turn_outer_speed = TRACKING_SETTLE_OUTER_PWM;
    center_speed = TRACKING_SETTLE_CENTER_PWM;
    crossing_speed = TRACKING_SETTLE_CENTER_PWM;
  }
  else
  {
    /* Stable profile shared by KEY1 and KEY2. */
    turn_inner_speed = ensure_minimum_speed(
        scale_speed(base_speed, 55U), TRACKING_MIN_INNER_PWM);
    turn_outer_speed = ensure_minimum_speed(
        scale_speed(base_speed, 90U), TRACKING_MIN_OUTER_PWM);
    turn_outer_speed = turn_speed_for_gain(turn_outer_speed);
    center_speed = base_speed > TRACKING_NORMAL_CENTER_PWM
                 ? TRACKING_NORMAL_CENTER_PWM : base_speed;
    crossing_speed = center_speed;
  }

  /* 两个外侧探头同时压线或四路全黑，通常是宽线/交叉口。 */
  if ((reading->x2_black && reading->x4_black) || active_count == 4U)
  {
    outer_turn_direction = 0;
    smooth_filter_valid = 0U;
    smooth_centered_active = 0U;
    smooth_straight_pwm = TRACKING_SMOOTH_STRAIGHT_BASE_PWM;
    command_set_pwm(command, crossing_speed, crossing_speed,
                    LINE_ACTION_CROSSING);
    return LINE_ACTION_CROSSING;
  }

  if (active_count != 0U)
  {
    /* 物理从左到右按 X2、X1、X3、X4 排列。用位置加权处理组合状态，
       避免多个探头同时压线时在离散规则之间突然跳变。 */
    weighted_sum = (int16_t)(-3 * reading->x2_black - reading->x1_black +
                              reading->x3_black + 3 * reading->x4_black);
    line_position = weighted_sum / active_count;

    if (line_position <= -2)
    {
      smooth_filter_valid = 0U;
      smooth_centered_active = 0U;
      smooth_straight_pwm = TRACKING_SMOOTH_STRAIGHT_BASE_PWM;
      /* 四轮车从静止直接原地转向需要克服较大横向摩擦。先用 140 ms
         强差速滚动建立角速度，再切到高 PWM 原地转向。 */
      if (outer_turn_direction != -1)
      {
        outer_turn_direction = -1;
        outer_turn_started_ms = now;
      }
      if (now - outer_turn_started_ms < TRACKING_OUTER_ROLL_IN_MS)
      {
        command_set_pwm(command,
                        TRACKING_OUTER_ROLL_INNER_PWM,
                        turn_speed_for_gain(
                            TRACKING_OUTER_ROLL_OUTER_PWM),
                        LINE_ACTION_LEFT_SHARP);
      }
      else if (now - outer_turn_started_ms < TRACKING_OUTER_SPIN_END_MS)
      {
        {
          int16_t spin = turn_speed_for_gain(TRACKING_OUTER_SPIN_PWM);
          command_set_pwm(command, (int16_t)-spin, spin,
                          LINE_ACTION_LEFT_SHARP);
        }
      }
      else
      {
        /* 转角已经建立后改回高曲率前进，避免整个弯道都原地磨轮。 */
        command_set_pwm(command,
                        TRACKING_OUTER_ARC_INNER_PWM,
                        turn_speed_for_gain(
                            TRACKING_OUTER_ARC_OUTER_PWM),
                        LINE_ACTION_LEFT_SHARP);
      }
      return LINE_ACTION_LEFT_SHARP;
    }


    if (smooth_mode_enabled != 0U && settling == 0U &&
        line_position > -2 && line_position < 2)
    {
      int16_t raw_error_q8 = (int16_t)(((int32_t)weighted_sum * 256) /
                                       active_count);
      int16_t derivative_q8;
      int16_t steering;
      int32_t steering_work;
      int16_t magnitude;
      int16_t curve_center;
      int16_t left_target;
      int16_t right_target;
      uint8_t stable_center = (line_position == 0 &&
                               reading->x2_black == 0U &&
                               reading->x4_black == 0U) ? 1U : 0U;

      if (stable_center != 0U)
      {
        if (smooth_centered_active == 0U)
        {
          smooth_centered_active = 1U;
          smooth_centered_since_ms = now;
          smooth_ramp_update_ms = now;
          smooth_straight_pwm = TRACKING_SMOOTH_STRAIGHT_BASE_PWM;
        }
        else if (now - smooth_centered_since_ms >=
                 TRACKING_SMOOTH_CENTER_HOLD_MS &&
                 now - smooth_ramp_update_ms >=
                 TRACKING_SMOOTH_RAMP_INTERVAL_MS)
        {
          smooth_ramp_update_ms = now;
          if (smooth_straight_pwm < TRACKING_SMOOTH_STRAIGHT_MAX_PWM)
          {
            smooth_straight_pwm = clamp_speed(
                (int32_t)smooth_straight_pwm +
                TRACKING_SMOOTH_RAMP_STEP_PWM);
          }
        }
      }
      else
      {
        /* Raw sensor departure wins immediately over the filtered error so
           the car never carries straight-line boost into a bend. */
        smooth_centered_active = 0U;
        smooth_straight_pwm = TRACKING_SMOOTH_STRAIGHT_BASE_PWM;
      }

      if (smooth_filter_valid == 0U)
      {
        smooth_error_q8 = raw_error_q8;
        smooth_previous_error_q8 = raw_error_q8;
        smooth_last_update_ms = now;
        smooth_filter_valid = 1U;
      }
      else if (now - smooth_last_update_ms >= TRACKING_SMOOTH_UPDATE_MS)
      {
        smooth_previous_error_q8 = smooth_error_q8;
        /* 1/4 new sample, 3/4 history: suppress edge chatter without adding
           a long delay at the 10 ms control update rate. */
        smooth_error_q8 = (int16_t)(((int32_t)smooth_error_q8 * 3 +
                                    raw_error_q8) / 4);
        smooth_last_update_ms = now;
      }

      derivative_q8 = (int16_t)(smooth_error_q8 -
                                smooth_previous_error_q8);
      /* The derivative term previously amplified X1/X3 one-frame chatter and
         made a centred car alternate left/right.  Keep enough derivative for
         a real bend, but require a wider centre deadband. */
      steering_work = ((int32_t)smooth_error_q8 * 3) / 2 +
                      derivative_q8 / 2;
      steering_work = (steering_work * smooth_turn_gain_percent) / 100L;
      if (steering_work > TRACKING_SMOOTH_STEER_LIMIT)
      {
        steering = TRACKING_SMOOTH_STEER_LIMIT;
      }
      else if (steering_work < -TRACKING_SMOOTH_STEER_LIMIT)
      {
        steering = -TRACKING_SMOOTH_STEER_LIMIT;
      }
      else
      {
        steering = (int16_t)steering_work;
      }

      magnitude = smooth_error_q8 < 0
                ? (int16_t)-smooth_error_q8 : smooth_error_q8;
      curve_center = TRACKING_SMOOTH_CURVE_CENTER_PWM -
          (int16_t)(((int32_t)magnitude *
                     TRACKING_SMOOTH_CURVE_SLOWDOWN_PWM) / 256);
      left_target = clamp_speed((int32_t)curve_center + steering);
      right_target = clamp_speed((int32_t)curve_center - steering);
      left_target = ensure_minimum_speed(left_target,
                                         TRACKING_MIN_INNER_PWM);
      right_target = ensure_minimum_speed(right_target,
                                          TRACKING_MIN_INNER_PWM);

      outer_turn_direction = 0;
      if (steering < -TRACKING_SMOOTH_STEER_DEADBAND)
      {
        command_set_pwm(command, left_target, right_target,
                        LINE_ACTION_LEFT_ADJUST);
        return LINE_ACTION_LEFT_ADJUST;
      }
      if (steering > TRACKING_SMOOTH_STEER_DEADBAND)
      {
        command_set_pwm(command, left_target, right_target,
                        LINE_ACTION_RIGHT_ADJUST);
        return LINE_ACTION_RIGHT_ADJUST;
      }

      /* Only a continuously centred run earns the gradual straight boost;
         otherwise this remains at the 2800 PWM migration baseline. */
      command_set_pwm(command, smooth_straight_pwm, smooth_straight_pwm,
                      LINE_ACTION_FORWARD);
      return LINE_ACTION_FORWARD;
    }

    if (line_position < 0)
    {
      outer_turn_direction = 0;
      command_set_pwm(command, turn_inner_speed, turn_outer_speed,
                      LINE_ACTION_LEFT_ADJUST);
      return LINE_ACTION_LEFT_ADJUST;
    }

    if (line_position >= 2)
    {
      smooth_filter_valid = 0U;
      smooth_centered_active = 0U;
      smooth_straight_pwm = TRACKING_SMOOTH_STRAIGHT_BASE_PWM;
      if (outer_turn_direction != 1)
      {
        outer_turn_direction = 1;
        outer_turn_started_ms = now;
      }
      if (now - outer_turn_started_ms < TRACKING_OUTER_ROLL_IN_MS)
      {
        command_set_pwm(command,
                        turn_speed_for_gain(
                            TRACKING_OUTER_ROLL_OUTER_PWM),
                        TRACKING_OUTER_ROLL_INNER_PWM,
                        LINE_ACTION_RIGHT_SHARP);
      }
      else if (now - outer_turn_started_ms < TRACKING_OUTER_SPIN_END_MS)
      {
        {
          int16_t spin = turn_speed_for_gain(TRACKING_OUTER_SPIN_PWM);
          command_set_pwm(command, spin, (int16_t)-spin,
                          LINE_ACTION_RIGHT_SHARP);
        }
      }
      else
      {
        command_set_pwm(command,
                        turn_speed_for_gain(
                            TRACKING_OUTER_ARC_OUTER_PWM),
                        TRACKING_OUTER_ARC_INNER_PWM,
                        LINE_ACTION_RIGHT_SHARP);
      }
      return LINE_ACTION_RIGHT_SHARP;
    }

    if (line_position > 0)
    {
      outer_turn_direction = 0;
      command_set_pwm(command, turn_outer_speed, turn_inner_speed,
                      LINE_ACTION_RIGHT_ADJUST);
      return LINE_ACTION_RIGHT_ADJUST;
    }

    outer_turn_direction = 0;

    command_set_pwm(command, center_speed, center_speed,
                    LINE_ACTION_FORWARD);
    return LINE_ACTION_FORWARD;
  }

  /* active_count==0 已在函数前半段处理，此处只作防御。 */
  command_stop(command);
  return LINE_ACTION_STOP;
}
