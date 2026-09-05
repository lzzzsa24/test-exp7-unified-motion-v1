/*
 * 实验七：PF13/PF14/PF15/PG0 四路数字循迹
 *
 * 传感器输出低电平表示黑线。控制器采用离散差速动作，和指导书第 5
 * 节给出的 X1～X4 动作逻辑一致；没有编码器时，速度参数仍需在实际
 * 场地上按电池电量和地面摩擦做一次小范围标定。
 */

#include "line_tracking.h"

#include "main.h"
#include "motion_advanced.h"
#include "motorPWM.h"
#include "drive_base.h"

/* Restored 2026-09-03 14:09 decisions, adapted to the current speed controller.
 * PWM requests below are equivalent inputs, not direct motor PWM writes. */
static int16_t requested_left_pwm;
static int16_t requested_right_pwm;
static void capture_stop(void) { requested_left_pwm = 0; requested_right_pwm = 0; }
static void capture_forward(int16_t left, int16_t right)
{ requested_left_pwm = left; requested_right_pwm = right; }
static void capture_left(int16_t inner, int16_t outer) { capture_forward(inner, outer); }
static void capture_right(int16_t inner, int16_t outer) { capture_forward(outer, inner); }
static void capture_spin_left(int16_t speed) { capture_forward((int16_t)-speed, speed); }
static void capture_spin_right(int16_t speed) { capture_forward(speed, (int16_t)-speed); }
#define advanced_stop capture_stop
#define advanced_drive_forward capture_forward
#define advanced_turn_left capture_left
#define advanced_turn_right capture_right
#define advanced_spin_left capture_spin_left
#define advanced_spin_right capture_spin_right

static int8_t last_error;
static uint8_t no_line_forward_enabled = 1U;
static uint8_t line_has_been_seen;
static uint8_t middle_pair_armed;
static int8_t predicted_turn_direction;
static int8_t recovery_turn_direction;
static int8_t last_turn_direction;
static int8_t outer_turn_direction;
static uint32_t outer_turn_started_ms;

typedef enum
{
  LINE_RECOVERY_NORMAL = 0U,
  LINE_RECOVERY_WAIT_SEARCH,
  LINE_RECOVERY_TURN_SEARCH,
  LINE_RECOVERY_STOPPED,
  LINE_RECOVERY_WAIT_FORWARD,
  LINE_RECOVERY_SETTLE
} LineRecoveryState;

static LineRecoveryState recovery_state;
static uint32_t recovery_state_started_ms;

#define TRACKING_DIRECTION_GUARD_MS          70U
#define TRACKING_TURN_SEARCH_TIMEOUT_MS     1500U
#define TRACKING_REACQUIRE_SETTLE_MS         250U
#define TRACKING_MIN_INNER_PWM             2200
#define TRACKING_MIN_OUTER_PWM             3000
#define TRACKING_SETTLE_INNER_PWM           2200
#define TRACKING_SETTLE_OUTER_PWM           2700
#define TRACKING_SETTLE_CENTER_PWM          2200
#define TRACKING_NORMAL_CENTER_PWM          2500
#define TRACKING_OUTER_ROLL_IN_MS            120U
#define TRACKING_OUTER_SPIN_END_MS           280U
#define TRACKING_OUTER_ROLL_INNER_PWM       2200
#define TRACKING_OUTER_ROLL_OUTER_PWM       3400
#define TRACKING_OUTER_SPIN_PWM             3000
#define TRACKING_OUTER_ARC_INNER_PWM        2100
#define TRACKING_OUTER_ARC_OUTER_PWM        3300
#define TRACKING_SEARCH_SPIN_PWM            2700

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

static uint8_t read_black(GPIO_TypeDef *port, uint16_t pin)
{
  return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET ? 1U : 0U;
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
  DriveBase_Stop(DRIVE_STOP_COAST);
  last_error = 0;
  line_has_been_seen = 0U;
  middle_pair_armed = 0U;
  predicted_turn_direction = 0;
  recovery_turn_direction = 0;
  last_turn_direction = 0;
  outer_turn_direction = 0;
  outer_turn_started_ms = HAL_GetTick();
  recovery_state = LINE_RECOVERY_NORMAL;
  recovery_state_started_ms = HAL_GetTick();
}

void line_tracking_set_no_line_forward(uint8_t enable)
{
  no_line_forward_enabled = enable != 0U ? 1U : 0U;
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

static LineTrackingAction line_tracking_apply(const LineTrackingReading *reading,
                                       int16_t base_speed)
{
  int16_t turn_inner_speed;
  int16_t turn_outer_speed;
  int16_t center_speed;
  int16_t crossing_speed;
  int16_t weighted_sum;
  int16_t line_position;
  uint8_t active_count;
  uint8_t settling = 0U;
  uint32_t now = HAL_GetTick();

  if (base_speed <= 0)
  {
    advanced_stop();
    return LINE_ACTION_STOP;
  }

  active_count = reading->x1_black + reading->x2_black +
                 reading->x3_black + reading->x4_black;

  /* 在正常寻线阶段记录中间两路离开黑线的先后顺序。X3（右中）
     先丢而 X1 仍在，说明线向左弯；X1 先丢则说明线向右弯。 */
  if (recovery_state == LINE_RECOVERY_NORMAL)
  {
    if (reading->x1_black && reading->x3_black)
    {
      if (middle_pair_armed == 0U)
      {
        predicted_turn_direction = 0;
      }
      middle_pair_armed = 1U;
    }
    else if (middle_pair_armed != 0U)
    {
      if (reading->x1_black && !reading->x3_black)
      {
        predicted_turn_direction = -1;
      }
      else if (!reading->x1_black && reading->x3_black)
      {
        predicted_turn_direction = 1;
      }
      else if (last_error < 0)
      {
        predicted_turn_direction = -1;
      }
      else if (last_error > 0)
      {
        predicted_turn_direction = 1;
      }
      middle_pair_armed = 0U;
    }
  }

  if (active_count == 0U)
  {
    outer_turn_direction = 0;
    /* KEY1 上电后从未见过黑线时仍按无黑线方案前进；一旦已经进入
       寻线，或处于 KEY2 纯寻线模式，全白就沿预测方向搜索；无法预测时停车。 */
    if (no_line_forward_enabled != 0U && line_has_been_seen == 0U)
    {
      recovery_state = LINE_RECOVERY_NORMAL;
      advanced_drive_forward(base_speed, base_speed);
      return LINE_ACTION_FORWARD;
    }

    if (recovery_state == LINE_RECOVERY_NORMAL ||
        recovery_state == LINE_RECOVERY_WAIT_FORWARD ||
        recovery_state == LINE_RECOVERY_SETTLE)
    {
      advanced_stop();
      recovery_turn_direction = predicted_turn_direction;
      if (recovery_turn_direction == 0)
      {
        recovery_turn_direction = last_error < 0 ? -1
                                : (last_error > 0 ? 1 : 0);
      }
      if (recovery_turn_direction == 0)
      {
        recovery_turn_direction = last_turn_direction;
      }
      recovery_state = recovery_turn_direction != 0
                     ? LINE_RECOVERY_WAIT_SEARCH
                     : LINE_RECOVERY_STOPPED;
      recovery_state_started_ms = now;
      return LINE_ACTION_STOP;
    }

    if (recovery_state == LINE_RECOVERY_WAIT_SEARCH)
    {
      advanced_stop();
      if (now - recovery_state_started_ms >= TRACKING_DIRECTION_GUARD_MS)
      {
        recovery_state = LINE_RECOVERY_TURN_SEARCH;
        recovery_state_started_ms = now;
      }
      return LINE_ACTION_STOP;
    }

    if (recovery_state == LINE_RECOVERY_TURN_SEARCH)
    {
      if (now - recovery_state_started_ms >= TRACKING_TURN_SEARCH_TIMEOUT_MS)
      {
        advanced_stop();
        recovery_state = LINE_RECOVERY_STOPPED;
        recovery_state_started_ms = now;
        return LINE_ACTION_STOP;
      }

      if (recovery_turn_direction < 0)
      {
        advanced_spin_left(TRACKING_SEARCH_SPIN_PWM);
        return LINE_ACTION_SEARCH_LEFT;
      }

      advanced_spin_right(TRACKING_SEARCH_SPIN_PWM);
      return LINE_ACTION_SEARCH_RIGHT;
    }

    if (recovery_state == LINE_RECOVERY_STOPPED)
    {
      advanced_stop();
      return LINE_ACTION_STOP;
    }

    advanced_stop();
    return LINE_ACTION_STOP;
  }

  line_has_been_seen = 1U;

  if (recovery_state == LINE_RECOVERY_TURN_SEARCH ||
      recovery_state == LINE_RECOVERY_WAIT_SEARCH ||
      recovery_state == LINE_RECOVERY_STOPPED)
  {
    advanced_stop();
    recovery_state = LINE_RECOVERY_WAIT_FORWARD;
    recovery_state_started_ms = now;
    return LINE_ACTION_STOP;
  }

  if (recovery_state == LINE_RECOVERY_WAIT_FORWARD)
  {
    advanced_stop();
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
    }
  }

  if (settling != 0U)
  {
    /* 重新捕线后的 250 ms 内限制速度，避免中间两路刚压线就以
       直行全速冲过黑线。中间两路同时压线时还会使用预测方向。 */
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
    center_speed = base_speed > TRACKING_NORMAL_CENTER_PWM
                 ? TRACKING_NORMAL_CENTER_PWM : base_speed;
    crossing_speed = center_speed;
  }

  /* 两个外侧探头同时压线或四路全黑，通常是宽线/交叉口。 */
  if ((reading->x2_black && reading->x4_black) || active_count == 4U)
  {
    outer_turn_direction = 0;
    advanced_drive_forward(crossing_speed, crossing_speed);
    last_error = 0;
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
      /* 四轮车从静止直接原地转向需要克服较大横向摩擦。先用 140 ms
         强差速滚动建立角速度，再切到高 PWM 原地转向。 */
      if (outer_turn_direction != -1)
      {
        outer_turn_direction = -1;
        outer_turn_started_ms = now;
      }
      if (now - outer_turn_started_ms < TRACKING_OUTER_ROLL_IN_MS)
      {
        advanced_turn_left(TRACKING_OUTER_ROLL_INNER_PWM,
                           TRACKING_OUTER_ROLL_OUTER_PWM);
      }
      else if (now - outer_turn_started_ms < TRACKING_OUTER_SPIN_END_MS)
      {
        advanced_spin_left(TRACKING_OUTER_SPIN_PWM);
      }
      else
      {
        /* 转角已经建立后改回高曲率前进，避免整个弯道都原地磨轮。 */
        advanced_turn_left(TRACKING_OUTER_ARC_INNER_PWM,
                           TRACKING_OUTER_ARC_OUTER_PWM);
      }
      last_error = -3;
      last_turn_direction = -1;
      return LINE_ACTION_LEFT_SHARP;
    }



    if (line_position < 0)
    {
      outer_turn_direction = 0;
      advanced_turn_left(turn_inner_speed, turn_outer_speed);
      last_error = -1;
      last_turn_direction = -1;
      return LINE_ACTION_LEFT_ADJUST;
    }

    if (line_position >= 2)
    {
      if (outer_turn_direction != 1)
      {
        outer_turn_direction = 1;
        outer_turn_started_ms = now;
      }
      if (now - outer_turn_started_ms < TRACKING_OUTER_ROLL_IN_MS)
      {
        advanced_turn_right(TRACKING_OUTER_ROLL_INNER_PWM,
                            TRACKING_OUTER_ROLL_OUTER_PWM);
      }
      else if (now - outer_turn_started_ms < TRACKING_OUTER_SPIN_END_MS)
      {
        advanced_spin_right(TRACKING_OUTER_SPIN_PWM);
      }
      else
      {
        advanced_turn_right(TRACKING_OUTER_ARC_INNER_PWM,
                            TRACKING_OUTER_ARC_OUTER_PWM);
      }
      last_error = 3;
      last_turn_direction = 1;
      return LINE_ACTION_RIGHT_SHARP;
    }

    if (line_position > 0)
    {
      outer_turn_direction = 0;
      advanced_turn_right(turn_inner_speed, turn_outer_speed);
      last_error = 1;
      last_turn_direction = 1;
      return LINE_ACTION_RIGHT_ADJUST;
    }

    outer_turn_direction = 0;

    if (settling != 0U && recovery_turn_direction < 0)
    {
      advanced_turn_left(turn_inner_speed, turn_outer_speed);
      last_error = -1;
      last_turn_direction = -1;
      return LINE_ACTION_LEFT_ADJUST;
    }

    if (settling != 0U && recovery_turn_direction > 0)
    {
      advanced_turn_right(turn_inner_speed, turn_outer_speed);
      last_error = 1;
      last_turn_direction = 1;
      return LINE_ACTION_RIGHT_ADJUST;
    }

    advanced_drive_forward(center_speed, center_speed);
    last_error = 0;
    return LINE_ACTION_FORWARD;
  }

  /* active_count==0 已在函数前半段处理，此处只作防御。 */
  advanced_stop();
  return LINE_ACTION_STOP;
}


/* This profile predates filtered PD and the KEY1 gain setting. */
void line_tracking_set_smooth_mode(uint8_t enable) { (void)enable; }
void line_tracking_set_turn_gain_percent(uint16_t percent) { (void)percent; }

LineTrackingAction line_tracking_compute(const LineTrackingReading *reading,
                                         int16_t base_speed,
                                         LineTrackingCommand *command)
{
  LineTrackingAction action = LINE_ACTION_STOP;
  capture_stop();
  if (reading != 0 && base_speed > 0 && DriveBase_GetFaultMask() == 0U)
  {
    action = line_tracking_apply(reading, base_speed);
  }
  else
  {
    line_tracking_reset();
  }
  if (command != 0)
  {
    command->left_cps = DriveBase_EquivalentCpsFromPwm(requested_left_pwm);
    command->right_cps = DriveBase_EquivalentCpsFromPwm(requested_right_pwm);
    command->action = action;
    command->valid = 1U;
  }
  return action;
}
