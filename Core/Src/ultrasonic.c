/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ultrasonic.c
  * @brief   四针超声波模块的非阻塞测距驱动
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "ultrasonic.h"

#define ULTRASONIC_TRIGGER_US       10U
#define ULTRASONIC_TIMEOUT_MS       40U        /* 覆盖 HC-SR04 最长回波等待 */
#define ULTRASONIC_MIN_ECHO_US      116U       /* 约 2 cm */
#define ULTRASONIC_MAX_ECHO_US      23200U     /* 约 400 cm */

typedef enum
{
  ULTRASONIC_IDLE = 0U,
  ULTRASONIC_WAIT_RISING,
  ULTRASONIC_WAIT_FALLING,
  ULTRASONIC_RESULT_READY
} UltrasonicCaptureState;

static TIM_HandleTypeDef htim2_us;
static volatile UltrasonicCaptureState capture_state = ULTRASONIC_IDLE;
static volatile uint8_t result_code = ULTRASONIC_RESULT_NONE;
static volatile uint8_t last_result_code = ULTRASONIC_RESULT_NONE;
static volatile uint16_t result_distance_cm = 0U;
static volatile uint16_t result_distance_mm = 0U;
static volatile uint32_t echo_start_ticks = 0U;
static uint32_t measurement_start_ms = 0U;

static void delay_us(uint32_t microseconds)
{
  uint32_t start_ticks = __HAL_TIM_GET_COUNTER(&htim2_us);

  while ((uint32_t)(__HAL_TIM_GET_COUNTER(&htim2_us) - start_ticks) < microseconds)
  {
    /* TIM2 以 1 MHz 运行，计数差直接等于微秒数。 */
  }
}

static uint32_t tim2_clock_hz(void)
{
  uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();

  /* STM32F1 在 APB1 分频不为 1 时，定时器时钟为 PCLK1 的 2 倍。 */
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
  {
    clock_hz *= 2U;
  }

  return clock_hz;
}

void Ultrasonic_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint32_t timer_clock_hz;

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  GPIO_InitStruct.Pin = ULTRASONIC_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ULTRASONIC_TRIG_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port,
                   ULTRASONIC_TRIG_Pin,
                   GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = ULTRASONIC_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  /* ECHO 断线时用下拉保持低电平，避免 PF12 悬空产生假边沿。 */
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ULTRASONIC_ECHO_GPIO_Port, &GPIO_InitStruct);
  __HAL_GPIO_EXTI_CLEAR_IT(ULTRASONIC_ECHO_Pin);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  timer_clock_hz = tim2_clock_hz();
  htim2_us.Instance = TIM2;
  htim2_us.Init.Prescaler = (timer_clock_hz / 1000000U) - 1U;
  htim2_us.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2_us.Init.Period = 0xFFFFFFFFU;
  htim2_us.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2_us.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2_us) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start(&htim2_us) != HAL_OK)
  {
    Error_Handler();
  }

  capture_state = ULTRASONIC_IDLE;
  result_code = ULTRASONIC_RESULT_NONE;
  last_result_code = ULTRASONIC_RESULT_NONE;
  result_distance_cm = 0U;
  result_distance_mm = 0U;
}

uint8_t Ultrasonic_Start(void)
{
  if (capture_state != ULTRASONIC_IDLE)
  {
    return 0U;
  }

  result_code = ULTRASONIC_RESULT_NONE;
  last_result_code = ULTRASONIC_RESULT_NONE;
  result_distance_cm = 0U;
  result_distance_mm = 0U;
  echo_start_ticks = 0U;
  measurement_start_ms = HAL_GetTick();
  capture_state = ULTRASONIC_WAIT_RISING;

  /* 清除上一次边沿残留，再发出新的触发脉冲。 */
  __HAL_GPIO_EXTI_CLEAR_IT(ULTRASONIC_ECHO_Pin);

  HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port,
                   ULTRASONIC_TRIG_Pin,
                   GPIO_PIN_SET);
  delay_us(ULTRASONIC_TRIGGER_US);
  HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port,
                   ULTRASONIC_TRIG_Pin,
                   GPIO_PIN_RESET);

  return 1U;
}

void Ultrasonic_Task(void)
{
  if ((capture_state == ULTRASONIC_WAIT_RISING ||
       capture_state == ULTRASONIC_WAIT_FALLING) &&
      (HAL_GetTick() - measurement_start_ms >= ULTRASONIC_TIMEOUT_MS))
  {
    result_distance_cm = 0U;
    result_distance_mm = 0U;
    result_code = ULTRASONIC_RESULT_TIMEOUT;
    last_result_code = ULTRASONIC_RESULT_TIMEOUT;
    capture_state = ULTRASONIC_RESULT_READY;
  }
}

uint8_t Ultrasonic_GetResult(uint16_t *distance_cm)
{
  uint8_t code;
  uint16_t distance;

  if (capture_state != ULTRASONIC_RESULT_READY)
  {
    return ULTRASONIC_RESULT_NONE;
  }

  __disable_irq();
  code = result_code;
  distance = result_distance_cm;
  result_code = ULTRASONIC_RESULT_NONE;
  capture_state = ULTRASONIC_IDLE;
  __enable_irq();

  if (distance_cm != NULL)
  {
    *distance_cm = distance;
  }

  return code;
}

void Ultrasonic_EXTI_Callback(uint16_t GPIO_Pin)
{
  GPIO_PinState echo_level;

  if (GPIO_Pin != ULTRASONIC_ECHO_Pin)
  {
    return;
  }

  echo_level = HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port,
                                ULTRASONIC_ECHO_Pin);

  if (echo_level == GPIO_PIN_SET && capture_state == ULTRASONIC_WAIT_RISING)
  {
    echo_start_ticks = __HAL_TIM_GET_COUNTER(&htim2_us);
    capture_state = ULTRASONIC_WAIT_FALLING;
  }
  else if (echo_level == GPIO_PIN_RESET &&
           capture_state == ULTRASONIC_WAIT_FALLING)
  {
    uint32_t echo_width_us =
        (uint32_t)(__HAL_TIM_GET_COUNTER(&htim2_us) - echo_start_ticks);

    if (echo_width_us < ULTRASONIC_MIN_ECHO_US)
    {
      /* 过短回波既可能是小于 2 cm 的物体，也可能是尖峰。把它作为
         2 cm 近障碍交给三点中值和连续确认处理，不能永久锁死。 */
      result_distance_cm = 2U;
      result_distance_mm = 20U;
      result_code = ULTRASONIC_RESULT_OK;
      last_result_code = ULTRASONIC_RESULT_OK;
    }
    else if (echo_width_us > ULTRASONIC_MAX_ECHO_US)
    {
      /* 超长回波与“超过约 4 m”同类，按无回波超时处理，使整体工程
         能在连续三次后进入显式配置的低速开阔区降级。 */
      result_distance_cm = 0U;
      result_distance_mm = 0U;
      result_code = ULTRASONIC_RESULT_TIMEOUT;
      last_result_code = ULTRASONIC_RESULT_TIMEOUT;
    }
    else
    {
      result_distance_mm = (uint16_t)(((echo_width_us * 10U) + 29U) / 58U);
      result_distance_cm = (uint16_t)((result_distance_mm + 5U) / 10U);
      result_code = ULTRASONIC_RESULT_OK;
      last_result_code = ULTRASONIC_RESULT_OK;
    }

    capture_state = ULTRASONIC_RESULT_READY;
  }
}

uint8_t Ultrasonic_IsBusy(void)
{
  return (capture_state != ULTRASONIC_IDLE) ? 1U : 0U;
}

uint16_t Ultrasonic_GetLastDistanceMm(void)
{
  return result_distance_mm;
}

uint8_t Ultrasonic_GetLastResult(void)
{
  return last_result_code;
}
