/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motorPWM.c
  * @brief   实验二：四路直流电机 PWM 调速（20 kHz）
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motorPWM.h"
#include "main.h"

/*
 * STM32F103ZET6 的定时器映射：
 *   TIM8_CH1/2/3/4 -> PC6/PC7/PC8/PC9 -> M1A/M1B/M2A/M2B
 *   TIM1 全重映射 CH1/2/3/4 -> PE9/PE11/PE13/PE14 -> M3A/M3B/M4A/M4B
 * 72 MHz / (3599 + 1) = 20 kHz。
 *
 * 车体安装后，M1/M2 与 M3/M4 的电气正方向相反。
 * 下面将它们统一封装成相同的逻辑方向：调用 forward 时四个轮子都向前。
 */

static TIM_HandleTypeDef htim1_pwm;
static TIM_HandleTypeDef htim8_pwm;

/*
 * 2026-09-03 四轮悬空自动标定。原始 PWM=2200/2400/3000 均进行了
 * 多轮独立清零测量。电机在低速区明显非线性，因此不能用一个固定
 * 百分比修正；下面记录三个逻辑 PWM 节点对应的各电机实际 PWM。
 * 只修正功率，不改变经过实车确认的方向映射。
 */
#define MOTOR1_PWM_AT_2200  2174L
#define MOTOR1_PWM_AT_2400  2371L
#define MOTOR1_PWM_AT_3000  2962L
#define MOTOR2_PWM_AT_2200  2208L
#define MOTOR2_PWM_AT_2400  2410L
#define MOTOR2_PWM_AT_3000  3009L
#define MOTOR3_PWM_AT_2200  2210L
#define MOTOR3_PWM_AT_2400  2412L
#define MOTOR3_PWM_AT_3000  3018L
#define MOTOR4_PWM_AT_2200  2208L
#define MOTOR4_PWM_AT_2400  2408L
#define MOTOR4_PWM_AT_3000  3012L

static uint32_t clamp_speed(int16_t speed)
{
  if (speed <= 0)
  {
    return 0U;
  }

  if ((uint16_t)speed > MOTOR_PWM_PERIOD)
  {
    return MOTOR_PWM_PERIOD;
  }

  return (uint32_t)speed;
}

static int32_t interpolate_pwm(int32_t value,
                               int32_t x0,
                               int32_t y0,
                               int32_t x1,
                               int32_t y1)
{
  return y0 + ((value - x0) * (y1 - y0) + (x1 - x0) / 2L) /
              (x1 - x0);
}

static int16_t apply_power_calibration(int16_t speed,
                                       int32_t pwm_at_2200,
                                       int32_t pwm_at_2400,
                                       int32_t pwm_at_3000)
{
  int32_t requested = speed;
  int32_t calibrated;

  if (requested <= 0)
  {
    return 0;
  }
  if (requested > MOTOR_PWM_PERIOD)
  {
    requested = MOTOR_PWM_PERIOD;
  }

  if (requested <= 2200L)
  {
    calibrated = interpolate_pwm(requested, 0L, 0L,
                                 2200L, pwm_at_2200);
  }
  else if (requested <= 2400L)
  {
    calibrated = interpolate_pwm(requested, 2200L, pwm_at_2200,
                                 2400L, pwm_at_2400);
  }
  else if (requested <= 3000L)
  {
    calibrated = interpolate_pwm(requested, 2400L, pwm_at_2400,
                                 3000L, pwm_at_3000);
  }
  else
  {
    calibrated = interpolate_pwm(requested, 3000L, pwm_at_3000,
                                 MOTOR_PWM_PERIOD, MOTOR_PWM_PERIOD);
  }
  return (int16_t)calibrated;
}

static void set_compare(TIM_HandleTypeDef *htim, uint32_t channel, int16_t speed)
{
  __HAL_TIM_SET_COMPARE(htim, channel, clamp_speed(speed));
}

static void configure_pwm_channel(TIM_HandleTypeDef *htim, uint32_t channel)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0U;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel) != HAL_OK)
  {
    Error_Handler();
  }
}

static void configure_timer(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
  htim->Instance = instance;
  htim->Init.Prescaler = 0U;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = MOTOR_PWM_PERIOD;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.RepetitionCounter = 0U;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_PWM_Init(htim) != HAL_OK)
  {
    Error_Handler();
  }

  configure_pwm_channel(htim, TIM_CHANNEL_1);
  configure_pwm_channel(htim, TIM_CHANNEL_2);
  configure_pwm_channel(htim, TIM_CHANNEL_3);
  configure_pwm_channel(htim, TIM_CHANNEL_4);
}

static void start_pwm_channels(TIM_HandleTypeDef *htim)
{
  if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
}

void motor_pwm_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();

  /* TIM1 的四路输出需要完全重映射到 PE9/PE11/PE13/PE14。 */
  __HAL_AFIO_REMAP_TIM1_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = M1A_Pin | M1B_Pin | M2A_Pin | M2B_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = M3A_Pin | M3B_Pin | M4A_Pin | M4B_Pin;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  configure_timer(&htim1_pwm, TIM1);
  configure_timer(&htim8_pwm, TIM8);
  start_pwm_channels(&htim1_pwm);
  start_pwm_channels(&htim8_pwm);
  pwm_motor_stop();
}

void pwm_motor_stop(void)
{
  set_compare(&htim8_pwm, TIM_CHANNEL_1, 0);
  set_compare(&htim8_pwm, TIM_CHANNEL_2, 0);
  set_compare(&htim8_pwm, TIM_CHANNEL_3, 0);
  set_compare(&htim8_pwm, TIM_CHANNEL_4, 0);
  set_compare(&htim1_pwm, TIM_CHANNEL_1, 0);
  set_compare(&htim1_pwm, TIM_CHANNEL_2, 0);
  set_compare(&htim1_pwm, TIM_CHANNEL_3, 0);
  set_compare(&htim1_pwm, TIM_CHANNEL_4, 0);
}

void pwm_motor1_forward(int16_t speed)
{
  set_compare(&htim8_pwm, TIM_CHANNEL_1, 0);
  set_compare(&htim8_pwm, TIM_CHANNEL_2,
              apply_power_calibration(speed,
                                      MOTOR1_PWM_AT_2200,
                                      MOTOR1_PWM_AT_2400,
                                      MOTOR1_PWM_AT_3000));
}

void pwm_motor1_backward(int16_t speed)
{
  set_compare(&htim8_pwm, TIM_CHANNEL_1,
              apply_power_calibration(speed,
                                      MOTOR1_PWM_AT_2200,
                                      MOTOR1_PWM_AT_2400,
                                      MOTOR1_PWM_AT_3000));
  set_compare(&htim8_pwm, TIM_CHANNEL_2, 0);
}

void pwm_motor2_forward(int16_t speed)
{
  set_compare(&htim8_pwm, TIM_CHANNEL_3, 0);
  set_compare(&htim8_pwm, TIM_CHANNEL_4,
              apply_power_calibration(speed,
                                      MOTOR2_PWM_AT_2200,
                                      MOTOR2_PWM_AT_2400,
                                      MOTOR2_PWM_AT_3000));
}

void pwm_motor2_backward(int16_t speed)
{
  set_compare(&htim8_pwm, TIM_CHANNEL_3,
              apply_power_calibration(speed,
                                      MOTOR2_PWM_AT_2200,
                                      MOTOR2_PWM_AT_2400,
                                      MOTOR2_PWM_AT_3000));
  set_compare(&htim8_pwm, TIM_CHANNEL_4, 0);
}

void pwm_motor3_forward(int16_t speed)
{
  set_compare(&htim1_pwm, TIM_CHANNEL_1,
              apply_power_calibration(speed,
                                      MOTOR3_PWM_AT_2200,
                                      MOTOR3_PWM_AT_2400,
                                      MOTOR3_PWM_AT_3000));
  set_compare(&htim1_pwm, TIM_CHANNEL_2, 0);
}

void pwm_motor3_backward(int16_t speed)
{
  set_compare(&htim1_pwm, TIM_CHANNEL_1, 0);
  set_compare(&htim1_pwm, TIM_CHANNEL_2,
              apply_power_calibration(speed,
                                      MOTOR3_PWM_AT_2200,
                                      MOTOR3_PWM_AT_2400,
                                      MOTOR3_PWM_AT_3000));
}

void pwm_motor4_forward(int16_t speed)
{
  set_compare(&htim1_pwm, TIM_CHANNEL_3,
              apply_power_calibration(speed,
                                      MOTOR4_PWM_AT_2200,
                                      MOTOR4_PWM_AT_2400,
                                      MOTOR4_PWM_AT_3000));
  set_compare(&htim1_pwm, TIM_CHANNEL_4, 0);
}

void pwm_motor4_backward(int16_t speed)
{
  set_compare(&htim1_pwm, TIM_CHANNEL_3, 0);
  set_compare(&htim1_pwm, TIM_CHANNEL_4,
              apply_power_calibration(speed,
                                      MOTOR4_PWM_AT_2200,
                                      MOTOR4_PWM_AT_2400,
                                      MOTOR4_PWM_AT_3000));
}
