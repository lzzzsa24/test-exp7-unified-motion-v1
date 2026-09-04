/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ir_avoid.c
  * @brief   实验五：PE5/PE6 红外发射控制与 ADC3 双通道采样
  ******************************************************************************
  */
/* USER CODE END Header */

#include "ir_avoid.h"

#include "main.h"

#define IR_LEFT_ADC_CHANNEL          7U
#define IR_RIGHT_ADC_CHANNEL         8U
#define IR_ADC_TIMEOUT_LOOPS    100000U

static bool left_obstacle_state;
static bool right_obstacle_state;
static uint16_t left_threshold;
static uint16_t right_threshold;
static uint16_t left_hysteresis;
static uint16_t right_hysteresis;
static bool ir_enabled;
static bool ir_adc_ready;

static void ir_set_left_rgb(GPIO_PinState red,
                            GPIO_PinState green,
                            GPIO_PinState blue)
{
  HAL_GPIO_WritePin(LRGB_R_GPIO_Port, LRGB_R_Pin, red);
  HAL_GPIO_WritePin(LRGB_G_GPIO_Port, LRGB_G_Pin, green);
  HAL_GPIO_WritePin(LRGB_B_GPIO_Port, LRGB_B_Pin, blue);
}

static void ir_set_right_rgb(GPIO_PinState red,
                             GPIO_PinState green,
                             GPIO_PinState blue)
{
  HAL_GPIO_WritePin(RRGB_R_GPIO_Port, RRGB_R_Pin, red);
  HAL_GPIO_WritePin(RRGB_G_GPIO_Port, RRGB_G_Pin, green);
  HAL_GPIO_WritePin(RRGB_B_GPIO_Port, RRGB_B_Pin, blue);
}

static uint16_t ir_adc_read_channel(uint32_t channel)
{
  uint32_t timeout = IR_ADC_TIMEOUT_LOOPS;

  ADC3->SQR1 &= ~ADC_SQR1_L;
  ADC3->SQR3 = channel & ADC_SQR3_SQ1;
  ADC3->SR &= ~ADC_SR_EOC;
  ADC3->CR2 |= ADC_CR2_SWSTART;

  while (((ADC3->SR & ADC_SR_EOC) == 0U) && (timeout > 0U))
  {
    --timeout;
  }

  if (timeout == 0U)
  {
    /* 采样失败按“检测到障碍物”处理，避免 ADC 故障时继续前冲。 */
    return 0U;
  }

  return (uint16_t)ADC3->DR;
}

static uint16_t ir_adc_read_average(uint32_t channel)
{
  uint32_t sum = 0U;
  uint32_t sample;

  for (sample = 0U; sample < IR_AVOID_SAMPLE_COUNT; ++sample)
  {
    sum += ir_adc_read_channel(channel);
  }

  return (uint16_t)(sum / IR_AVOID_SAMPLE_COUNT);
}

static bool ir_update_obstacle_state(uint16_t adc_value,
                                     bool current_state,
                                     uint16_t threshold,
                                     uint16_t hysteresis)
{
  if (current_state)
  {
    return adc_value < (uint16_t)(threshold + hysteresis);
  }

  return adc_value < (uint16_t)(threshold - hysteresis);
}

static uint16_t ir_percent_of(uint16_t value, uint32_t percent)
{
  return (uint16_t)(((uint32_t)value * percent) / 100U);
}

void ir_avoid_resume_io(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* 模式切换后防御性恢复右 RGB 和红外发射控制。当前四路编码器
     使用各自真实 AB 引脚，不占用 PE2~PE5。 */
  HAL_GPIO_WritePin(GPIOE,
                    RRGB_R_Pin | RRGB_G_Pin | RRGB_B_Pin |
                    LRGB_G_Pin | IR_LEFT_ENABLE_Pin | IR_RIGHT_ENABLE_Pin,
                    GPIO_PIN_RESET);
  gpio.Pin = RRGB_R_Pin | RRGB_G_Pin | RRGB_B_Pin |
             LRGB_G_Pin | IR_LEFT_ENABLE_Pin | IR_RIGHT_ENABLE_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &gpio);

  /* PG12 is owned by the buzzer phrase/arbitration path.  Do not reset or
     reconfigure it while restoring the infrared and RGB outputs. */
  HAL_GPIO_WritePin(GPIOG,
                    LRGB_R_Pin | LRGB_B_Pin | led1_Pin | led2_Pin,
                    GPIO_PIN_RESET);
  gpio.Pin = LRGB_R_Pin | LRGB_B_Pin | led1_Pin | led2_Pin;
  HAL_GPIO_Init(GPIOG, &gpio);

  gpio.Pin = IR_LEFT_ADC_Pin | IR_RIGHT_ADC_Pin;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &gpio);

  ir_set_left_rgb(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  ir_set_right_rgb(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
}

void ir_avoid_init(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint32_t timeout;

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_ADC3_CLK_ENABLE();

  (void)gpio;
  ir_avoid_resume_io();

  /* PCLK2=72 MHz，六分频后 ADC 时钟为 12 MHz，满足 F103 限制。 */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_DIV6);

  ADC3->CR1 = 0U;
  ADC3->CR2 = ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG;
  ADC3->SMPR2 = ADC_SMPR2_SMP7 | ADC_SMPR2_SMP8; /* 239.5 周期 */
  ADC3->SQR1 = 0U;
  ADC3->CR2 |= ADC_CR2_ADON;
  HAL_Delay(1U);

  left_obstacle_state = false;
  right_obstacle_state = false;
  left_threshold = 0U;
  right_threshold = 0U;
  left_hysteresis = IR_AVOID_MIN_HYSTERESIS;
  right_hysteresis = IR_AVOID_MIN_HYSTERESIS;
  ir_enabled = false;
  ir_adc_ready = false;

  /* ADC 校准异常不能把整个小车锁死在启动阶段；超时后交给上层决定
     是否进入降级模式。 */
  timeout = IR_ADC_TIMEOUT_LOOPS;
  ADC3->CR2 |= ADC_CR2_RSTCAL;
  while ((ADC3->CR2 & ADC_CR2_RSTCAL) != 0U && timeout > 0U)
  {
    --timeout;
  }
  if (timeout == 0U)
  {
    return;
  }

  timeout = IR_ADC_TIMEOUT_LOOPS;
  ADC3->CR2 |= ADC_CR2_CAL;
  while ((ADC3->CR2 & ADC_CR2_CAL) != 0U && timeout > 0U)
  {
    --timeout;
  }
  if (timeout == 0U)
  {
    return;
  }

  ir_adc_ready = true;
}

bool ir_avoid_calibrate(void)
{
  uint32_t left_sum = 0U;
  uint32_t right_sum = 0U;
  uint32_t sample;
  uint16_t left_baseline;
  uint16_t right_baseline;

  if (!ir_adc_ready)
  {
    ir_enabled = false;
    return false;
  }

  for (sample = 0U; sample < IR_AVOID_CALIBRATION_COUNT; ++sample)
  {
    left_sum += ir_adc_read_average(IR_LEFT_ADC_CHANNEL);
    right_sum += ir_adc_read_average(IR_RIGHT_ADC_CHANNEL);
    HAL_Delay(10U);
  }

  left_baseline = (uint16_t)(left_sum / IR_AVOID_CALIBRATION_COUNT);
  right_baseline = (uint16_t)(right_sum / IR_AVOID_CALIBRATION_COUNT);

  if ((left_baseline < IR_AVOID_MIN_VALID_BASELINE) ||
      (right_baseline < IR_AVOID_MIN_VALID_BASELINE))
  {
    ir_enabled = false;
    return false;
  }

  left_threshold = ir_percent_of(left_baseline, IR_AVOID_TRIGGER_PERCENT);
  right_threshold = ir_percent_of(right_baseline, IR_AVOID_TRIGGER_PERCENT);
  left_hysteresis = ir_percent_of(left_baseline, IR_AVOID_HYSTERESIS_PERCENT);
  right_hysteresis = ir_percent_of(right_baseline, IR_AVOID_HYSTERESIS_PERCENT);

  if (left_hysteresis < IR_AVOID_MIN_HYSTERESIS)
  {
    left_hysteresis = IR_AVOID_MIN_HYSTERESIS;
  }
  if (right_hysteresis < IR_AVOID_MIN_HYSTERESIS)
  {
    right_hysteresis = IR_AVOID_MIN_HYSTERESIS;
  }

  left_obstacle_state = false;
  right_obstacle_state = false;
  ir_enabled = true;
  return true;
}

void ir_avoid_set_enabled(bool enabled)
{
  ir_enabled = enabled;
  if (!enabled)
  {
    left_obstacle_state = false;
    right_obstacle_state = false;
  }
}

bool ir_avoid_is_enabled(void)
{
  return ir_enabled;
}

uint16_t ir_avoid_get_left_threshold(void)
{
  return left_threshold;
}

uint16_t ir_avoid_get_right_threshold(void)
{
  return right_threshold;
}

uint16_t ir_avoid_get_left_hysteresis(void)
{
  return left_hysteresis;
}

uint16_t ir_avoid_get_right_hysteresis(void)
{
  return right_hysteresis;
}

IrAvoidReading ir_avoid_read(void)
{
  IrAvoidReading reading;

  if (!ir_enabled)
  {
    reading.left_adc = 0U;
    reading.right_adc = 0U;
    reading.left_obstacle = false;
    reading.right_obstacle = false;
    return reading;
  }

  reading.left_adc = ir_adc_read_average(IR_LEFT_ADC_CHANNEL);
  reading.right_adc = ir_adc_read_average(IR_RIGHT_ADC_CHANNEL);

  left_obstacle_state = ir_update_obstacle_state(reading.left_adc,
                                                  left_obstacle_state,
                                                  left_threshold,
                                                  left_hysteresis);
  right_obstacle_state = ir_update_obstacle_state(reading.right_adc,
                                                   right_obstacle_state,
                                                   right_threshold,
                                                   right_hysteresis);
  reading.left_obstacle = left_obstacle_state;
  reading.right_obstacle = right_obstacle_state;

  return reading;
}

void ir_avoid_show_status(const IrAvoidReading *reading)
{
  GPIO_PinState blink;

  if (!ir_enabled)
  {
    /* 蓝灯慢闪：红外 ADC/基线未准备好。 */
    blink = ((HAL_GetTick() / 500U) & 1U) != 0U
          ? GPIO_PIN_SET : GPIO_PIN_RESET;
    ir_set_left_rgb(GPIO_PIN_RESET, GPIO_PIN_RESET, blink);
    ir_set_right_rgb(GPIO_PIN_RESET, GPIO_PIN_RESET, blink);
    return;
  }

  /* 左右独立显示：绿色=该侧清空，红色=该侧检测到红外障碍。 */
  ir_set_left_rgb(reading->left_obstacle ? GPIO_PIN_SET : GPIO_PIN_RESET,
                  reading->left_obstacle ? GPIO_PIN_RESET : GPIO_PIN_SET,
                  GPIO_PIN_RESET);
  ir_set_right_rgb(reading->right_obstacle ? GPIO_PIN_SET : GPIO_PIN_RESET,
                   reading->right_obstacle ? GPIO_PIN_RESET : GPIO_PIN_SET,
                   GPIO_PIN_RESET);
}
