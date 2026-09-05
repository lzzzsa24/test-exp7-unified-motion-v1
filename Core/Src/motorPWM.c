/* YB-DSF01-V1.1 / STM32F103ZE: all motor polarity knowledge lives here.
 * M1 LF: TIM8 CH2 forward, CH1 reverse; M2 LR: CH4 forward, CH3 reverse.
 * M3 RF: TIM1 CH1 forward, CH2 reverse; M4 RR: CH3 forward, CH4 reverse.
 * TIM1 is fully remapped to PE9/11/13/14; TIM8 uses PC6/7/8/9.
 * 72 MHz / 3600 = 20 kHz. 0/0 on a bridge means coast, not active brake.
 */
#include "motorPWM.h"
#include "main.h"
#include "line_config.h"

static TIM_HandleTypeDef timers[2];
static int8_t last_sign[2];
static uint8_t coasting[2];
static uint32_t coast_at[2];
static int16_t applied[2];

/* Measured hardware data, 2026-09-03 wheels lifted. These points are NOT a
 * ground-speed model. Both polarities retain the same measured PWM mapping. */
static const uint16_t requested_points[] = {0, 2200, 2400, 3000, 3599};
static const uint16_t measured_pwm[4][5] = {
  {0, 2174, 2371, 2962, 3599}, {0, 2208, 2410, 3009, 3599},
  {0, 2210, 2412, 3018, 3599}, {0, 2208, 2408, 3012, 3599}
};

static uint16_t calibrated(unsigned wheel, uint32_t pwm)
{
  unsigned index;
  if (pwm >= MOTOR_PWM_PERIOD) return MOTOR_PWM_PERIOD;
  for (index = 1U; index < 5U; ++index) {
    if (pwm <= requested_points[index]) {
      uint32_t span = requested_points[index] - requested_points[index - 1U];
      uint32_t rise = measured_pwm[wheel][index] - measured_pwm[wheel][index - 1U];
      return (uint16_t)(measured_pwm[wheel][index - 1U] +
        ((pwm - requested_points[index - 1U]) * rise + span / 2U) / span);
    }
  }
  return 0U;
}

static void wheel_write(unsigned wheel, int16_t command)
{
  unsigned side = wheel / 2U;
  TIM_HandleTypeDef *timer = &timers[side];
  uint32_t channel_a = (wheel & 1U) ? TIM_CHANNEL_3 : TIM_CHANNEL_1;
  uint32_t channel_b = (wheel & 1U) ? TIM_CHANNEL_4 : TIM_CHANNEL_2;
  int32_t magnitude = command;
  uint32_t active, inactive;
  if (magnitude < 0) magnitude = -magnitude;
  /* Left motors have opposite electrical polarity to the right motors. */
  active = ((command > 0) == (side != 0U)) ? channel_a : channel_b;
  inactive = active == channel_a ? channel_b : channel_a;
  __HAL_TIM_SET_COMPARE(timer, inactive, 0U);
  __HAL_TIM_SET_COMPARE(timer, active, calibrated(wheel, (uint32_t)magnitude));
}

static void side_write(unsigned side, int16_t command, uint32_t now)
{
  int8_t sign = command > 0 ? 1 : (command < 0 ? -1 : 0);
  if (sign == 0 || (sign != last_sign[side] && !coasting[side])) {
    if (!coasting[side]) coast_at[side] = now;
    coasting[side] = 1U;
    command = 0;
  } else if (sign != last_sign[side] &&
             now - coast_at[side] < MOTOR_REVERSE_COAST_MS) {
    command = 0;
  } else {
    coasting[side] = 0U;
    last_sign[side] = sign;
  }
  wheel_write(side * 2U, command);
  wheel_write(side * 2U + 1U, command);
  applied[side] = command;
}

void motor_pwm_set_sides(int16_t left, int16_t right, uint32_t now)
{
  if (left > (int16_t)MOTOR_PWM_PERIOD) left = (int16_t)MOTOR_PWM_PERIOD;
  if (left < -(int16_t)MOTOR_PWM_PERIOD) left = -(int16_t)MOTOR_PWM_PERIOD;
  if (right > (int16_t)MOTOR_PWM_PERIOD) right = (int16_t)MOTOR_PWM_PERIOD;
  if (right < -(int16_t)MOTOR_PWM_PERIOD) right = -(int16_t)MOTOR_PWM_PERIOD;
  side_write(0U, left, now);
  side_write(1U, right, now);
}

void pwm_motor_stop(void)
{
  motor_pwm_set_sides(0, 0, HAL_GetTick());
}

int16_t motor_pwm_get_side(unsigned side)
{
  return side < 2U ? applied[side] : 0;
}

void motor_pwm_emergency_stop(void)
{
  /* Works from fault handlers without HAL, interrupts or a running tick. */
  TIM8->BDTR &= ~TIM_BDTR_MOE;
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM8->CCR1 = TIM8->CCR2 = TIM8->CCR3 = TIM8->CCR4 = 0U;
  TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0U;
}

void motor_pwm_init(void)
{
  GPIO_InitTypeDef gpio = {0};
  TIM_OC_InitTypeDef oc = {0};
  unsigned side;
  uint32_t channel;
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_AFIO_REMAP_TIM1_ENABLE();
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = M1A_Pin | M1B_Pin | M2A_Pin | M2B_Pin;
  HAL_GPIO_Init(GPIOC, &gpio);
  gpio.Pin = M3A_Pin | M3B_Pin | M4A_Pin | M4B_Pin;
  HAL_GPIO_Init(GPIOE, &gpio);
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  for (side = 0U; side < 2U; ++side) {
    TIM_HandleTypeDef *timer = &timers[side];
    timer->Instance = side == 0U ? TIM8 : TIM1;
    timer->Init.Prescaler = 0U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = MOTOR_PWM_PERIOD;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) Error_Handler();
    for (channel = TIM_CHANNEL_1; channel <= TIM_CHANNEL_4; channel += 4U) {
      if (HAL_TIM_PWM_ConfigChannel(timer, &oc, channel) != HAL_OK ||
          HAL_TIM_PWM_Start(timer, channel) != HAL_OK) Error_Handler();
    }
    last_sign[side] = 0;
    coasting[side] = 1U;
    coast_at[side] = HAL_GetTick() - MOTOR_REVERSE_COAST_MS;
    applied[side] = 0;
  }
  pwm_motor_stop();
}
