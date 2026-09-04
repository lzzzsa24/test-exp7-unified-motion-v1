#include "wheel_encoder.h"

#include "main.h"

/* 72 MHz / 72 / 50 = 20 kHz。高频采样避免编码器 GPIO 与三个按键
 * 的 EXTI3/4/5 复用冲突，也不占用超声波正在使用的 TIM2。 */
#define ENCODER_SAMPLE_PRESCALER  71U
#define ENCODER_SAMPLE_PERIOD     49U

/* 四个编码器的 AB 两相信号均已修复并由实车诊断确认正常，统一使用
 * 完整正交解码。M2 的正确引脚是 PA15/PB3，不能退回旧的 PD7/PB3。 */

static volatile WheelEncoderCounts encoder_counts;
static volatile uint8_t encoder_running;
static volatile uint32_t legal_transition_count[4];
static volatile uint32_t illegal_transition_count[4];
static uint8_t previous_state[4];

/* Physical suspended-wheel observations establish that left-side forward
 * rotation increments the raw decoder while right-side forward rotation
 * decrements it.  Applying the correction here gives every caller one logical
 * convention: vehicle-forward is positive, reverse is negative. */
static const int8_t logical_direction_sign[4] = {1, 1, -1, -1};

/* 合法 Gray 码相邻变化为 +/-1；跳过非法的双位同时变化。安装方向
 * 差异在本文件统一修正，所有上层都必须保留逻辑计数的正负号。 */
static const int8_t quadrature_delta[16] =
{
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

static uint8_t read_pair(GPIO_TypeDef *port, uint16_t pin_a, uint16_t pin_b)
{
  uint32_t idr = port->IDR;
  uint8_t a = (idr & pin_a) != 0U ? 1U : 0U;
  uint8_t b = (idr & pin_b) != 0U ? 1U : 0U;

  return (uint8_t)((a << 1U) | b);
}

static uint8_t read_split_pair(GPIO_TypeDef *port_a,
                               uint16_t pin_a,
                               GPIO_TypeDef *port_b,
                               uint16_t pin_b)
{
  uint8_t a = (port_a->IDR & pin_a) != 0U ? 1U : 0U;
  uint8_t b = (port_b->IDR & pin_b) != 0U ? 1U : 0U;

  return (uint8_t)((a << 1U) | b);
}

static void configure_encoder_gpio(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_INPUT;
  /* 断线时固定为高电平，避免浮空噪声被误计为“车轮仍在转”。 */
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;

  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &gpio);

  /* PA15/PB3/PB4 已由 HAL_MspInit 的 NOJTAG 配置释放，SWD 仍可使用。 */
  gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &gpio);
}

void WheelEncoder_Init(void)
{
  encoder_running = 0U;
  WheelEncoder_Reset();
  WheelEncoder_ResetDiagnostics();

  __HAL_RCC_TIM6_CLK_ENABLE();
  TIM6->CR1 = 0U;
  TIM6->CR2 = 0U;
  TIM6->DIER = 0U;
  TIM6->PSC = ENCODER_SAMPLE_PRESCALER;
  TIM6->ARR = ENCODER_SAMPLE_PERIOD;
  TIM6->CNT = 0U;
  TIM6->EGR = TIM_EGR_UG;
  TIM6->SR = 0U;

  HAL_NVIC_SetPriority(TIM6_IRQn, 2U, 0U);
}

void WheelEncoder_Start(void)
{
  if (encoder_running != 0U)
  {
    return;
  }

  configure_encoder_gpio();

  previous_state[0] = read_pair(GPIOD, GPIO_PIN_12, GPIO_PIN_13);
  previous_state[1] = read_split_pair(GPIOA, GPIO_PIN_15,
                                      GPIOB, GPIO_PIN_3);
  previous_state[2] = read_pair(GPIOA, GPIO_PIN_0, GPIO_PIN_1);
  previous_state[3] = read_pair(GPIOB, GPIO_PIN_4, GPIO_PIN_5);

  TIM6->CNT = 0U;
  TIM6->SR = 0U;
  encoder_running = 1U;
  HAL_NVIC_ClearPendingIRQ(TIM6_IRQn);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);
  TIM6->DIER = TIM_DIER_UIE;
  TIM6->CR1 = TIM_CR1_CEN;
}

void WheelEncoder_Stop(void)
{
  /* Compatibility no-op.  The unified-motion project deliberately keeps
   * TIM6 sampling alive after initialization.  Motion modules take their own
   * count snapshots instead of stopping or clearing the global odometer. */
}

void WheelEncoder_Reset(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  encoder_counts.motor1 = 0;
  encoder_counts.motor2 = 0;
  encoder_counts.motor3 = 0;
  encoder_counts.motor4 = 0;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void WheelEncoder_GetCounts(WheelEncoderCounts *counts)
{
  uint32_t primask;

  if (counts == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  counts->motor1 = encoder_counts.motor1;
  counts->motor2 = encoder_counts.motor2;
  counts->motor3 = encoder_counts.motor3;
  counts->motor4 = encoder_counts.motor4;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void WheelEncoder_GetDiagnostics(WheelEncoderDiagnostics *diagnostics)
{
  uint32_t primask;
  uint8_t motor;

  if (diagnostics == 0)
  {
    return;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  for (motor = 0U; motor < 4U; ++motor)
  {
    diagnostics->legal_transition_count[motor] =
        legal_transition_count[motor];
    diagnostics->illegal_transition_count[motor] =
        illegal_transition_count[motor];
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void WheelEncoder_ResetDiagnostics(void)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t motor;

  __disable_irq();
  for (motor = 0U; motor < 4U; ++motor)
  {
    legal_transition_count[motor] = 0U;
    illegal_transition_count[motor] = 0U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}

uint8_t WheelEncoder_IsRunning(void)
{
  return encoder_running;
}

void WheelEncoder_TIM6_IRQHandler(void)
{
  uint8_t state[4];
  uint8_t motor;

  if ((TIM6->SR & TIM_SR_UIF) == 0U)
  {
    return;
  }
  TIM6->SR &= ~TIM_SR_UIF;

  if (encoder_running == 0U)
  {
    return;
  }

  state[0] = read_pair(GPIOD, GPIO_PIN_12, GPIO_PIN_13);
  state[1] = read_split_pair(GPIOA, GPIO_PIN_15,
                             GPIOB, GPIO_PIN_3);
  state[2] = read_pair(GPIOA, GPIO_PIN_0, GPIO_PIN_1);
  state[3] = read_pair(GPIOB, GPIO_PIN_4, GPIO_PIN_5);

  for (motor = 0U; motor < 4U; ++motor)
  {
    uint8_t changed = (uint8_t)(previous_state[motor] ^ state[motor]);
    int8_t delta = quadrature_delta[(previous_state[motor] << 2U) |
                                    state[motor]];

    if (delta != 0)
    {
      ++legal_transition_count[motor];
      delta = (int8_t)(delta * logical_direction_sign[motor]);
      switch (motor)
      {
        case 0U: encoder_counts.motor1 += delta; break;
        case 1U: encoder_counts.motor2 += delta; break;
        case 2U: encoder_counts.motor3 += delta; break;
        case 3U: encoder_counts.motor4 += delta; break;
        default: break;
      }
    }
    else if (changed == 0x03U)
    {
      /* Both bits changed between adjacent 20 kHz samples.  Ignore the count
         but retain evidence so persistent noise/overspeed can stop motion. */
      ++illegal_transition_count[motor];
    }
  }

  previous_state[0] = state[0];
  previous_state[1] = state[1];
  previous_state[2] = state[2];
  previous_state[3] = state[3];
}
