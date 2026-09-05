#include "board.h"
#include "main.h"
#include "motorPWM.h"
#include <stdio.h>

static KeyInput keys;
static NecInput remote;
static volatile uint8_t input_events;
static uint32_t last_ir_cycle;
static char tx_buffer[128];
static volatile uint16_t tx_length, tx_index;

static uint8_t read_keys(void)
{
  uint32_t port = GPIOG->IDR;
  return (uint8_t)(((port & key1_Pin) ? 0U : 1U) |
                   ((port & key2_Pin) ? 0U : 2U) |
                   ((port & key3_Pin) ? 0U : 4U));
}

uint8_t Board_ReadLine(void)
{
  uint32_t port_f = GPIOF->IDR, port_g = GPIOG->IDR;
  return (uint8_t)(((port_f & TRACK_X2_Pin) ? 0U : LINE_LO) |
                   ((port_f & TRACK_X1_Pin) ? 0U : LINE_LI) |
                   ((port_f & TRACK_X3_Pin) ? 0U : LINE_RI) |
                   ((port_g & TRACK_X4_Pin) ? 0U : LINE_RO));
}

void Board_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  /* Inactive obstacle emitters, TRIG low, buzzer and indicators initially off. */
  HAL_GPIO_WritePin(GPIOE, IR_LEFT_ENABLE_Pin | IR_RIGHT_ENABLE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, RRGB_R_Pin | RRGB_G_Pin | RRGB_B_Pin | LRGB_G_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOG, Buzzer_Pin | led1_Pin | led2_Pin | LRGB_R_Pin | LRGB_B_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOF, ULTRASONIC_TRIG_Pin, GPIO_PIN_RESET);
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = IR_LEFT_ENABLE_Pin | IR_RIGHT_ENABLE_Pin | RRGB_R_Pin |
             RRGB_G_Pin | RRGB_B_Pin | LRGB_G_Pin;
  HAL_GPIO_Init(GPIOE, &gpio);
  gpio.Pin = Buzzer_Pin | led1_Pin | led2_Pin | LRGB_R_Pin | LRGB_B_Pin;
  HAL_GPIO_Init(GPIOG, &gpio);
  gpio.Pin = ULTRASONIC_TRIG_Pin;
  HAL_GPIO_Init(GPIOF, &gpio);

  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Pin = TRACK_X1_Pin | TRACK_X2_Pin | TRACK_X3_Pin;
  HAL_GPIO_Init(GPIOF, &gpio);
  gpio.Pin = TRACK_X4_Pin | key1_Pin | key2_Pin | key3_Pin;
  HAL_GPIO_Init(GPIOG, &gpio);
  Keys_Init(&keys, read_keys(), HAL_GetTick());

  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &gpio);
  USART1->CR1 = 0U;
  USART1->CR2 = 0U;
  USART1->CR3 = 0U;
  USART1->BRR = (HAL_RCC_GetPCLK2Freq() + 57600U) / 115200U;
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
  HAL_NVIC_SetPriority(USART1_IRQn, 2U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* Same priority as UART: both IRQs may OR events without nesting. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  last_ir_cycle = DWT->CYCCNT;
  gpio.Pin = IR_REMOTE_Pin;
  gpio.Mode = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(GPIOG, &gpio);
  __HAL_GPIO_EXTI_CLEAR_IT(IR_REMOTE_Pin);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2U, 0U);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

uint8_t Board_Inputs(uint32_t now)
{
  uint8_t events;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  events = input_events;
  input_events = 0U;
  if (!primask) __enable_irq();
  return (uint8_t)(events | Keys_Step(&keys, read_keys(), now));
}

void USART1_IRQHandler(void)
{
  uint32_t status = USART1->SR;
  if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
    uint8_t byte = (uint8_t)USART1->DR; /* SR then DR clears error flags. */
    if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
      input_events |= INPUT_STOP;
    else
      input_events |= Input_Serial(byte);
  }
  if ((status & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE)) {
    if (tx_index < tx_length) USART1->DR = (uint8_t)tx_buffer[tx_index++];
    else {
      USART1->CR1 &= ~USART_CR1_TXEIE;
      tx_length = 0U;
    }
  }
}

void EXTI15_10_IRQHandler(void)
{
  if (__HAL_GPIO_EXTI_GET_IT(IR_REMOTE_Pin)) {
    uint32_t now = DWT->CYCCNT;
    uint32_t elapsed_us = (now - last_ir_cycle) / (SystemCoreClock / 1000000U);
    __HAL_GPIO_EXTI_CLEAR_IT(IR_REMOTE_Pin);
    last_ir_cycle = now;
    input_events |= Input_NecEdge(&remote, elapsed_us);
  }
}

void Board_Report(const LineFollower *line)
{
  int count;
  uint32_t primask;
  if (tx_length != 0U) return; /* Drop busy telemetry; never delay control. */
  count = snprintf(tx_buffer, sizeof(tx_buffer),
    "SL1 %s %s raw=%u%u%u%u line=%u%u%u%u L=%d R=%d\r\n",
    Line_ModeName(line->mode), Line_ReasonName(line->reason),
    (unsigned)((line->raw >> 3U) & 1U), (unsigned)((line->raw >> 2U) & 1U),
    (unsigned)((line->raw >> 1U) & 1U), (unsigned)(line->raw & 1U),
    (unsigned)((line->filtered >> 3U) & 1U), (unsigned)((line->filtered >> 2U) & 1U),
    (unsigned)((line->filtered >> 1U) & 1U), (unsigned)(line->filtered & 1U),
    (int)motor_pwm_get_side(0U), (int)motor_pwm_get_side(1U));
  if (count <= 0) return;
  if ((unsigned)count >= sizeof(tx_buffer)) count = (int)sizeof(tx_buffer) - 1;
  primask = __get_PRIMASK();
  __disable_irq();
  tx_index = 0U;
  tx_length = (uint16_t)count;
  USART1->CR1 |= USART_CR1_TXEIE;
  if (!primask) __enable_irq();
}

void Board_Indicators(const LineFollower *line, uint32_t now)
{
  static LineReason previous_reason = LINE_USER_STOP;
  static uint8_t beeping;
  static uint32_t beep_at;
  uint8_t fault = line->mode == LINE_STOP && line->reason != LINE_USER_STOP;
  if (line->reason != previous_reason) {
    beeping = 1U;
    beep_at = now;
    previous_reason = line->reason;
  }
  if (now - beep_at >= 100U) beeping = 0U;
  HAL_GPIO_WritePin(GPIOG, Buzzer_Pin, beeping ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOG, led1_Pin, line->mode != LINE_STOP ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOG, led2_Pin, fault ? GPIO_PIN_SET : GPIO_PIN_RESET);
  /* Four RGB components mirror the raw sensors, even while stopped. */
  HAL_GPIO_WritePin(GPIOG, LRGB_R_Pin, (line->raw & LINE_LO) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, LRGB_G_Pin, (line->raw & LINE_LI) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, RRGB_G_Pin, (line->raw & LINE_RI) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, RRGB_R_Pin, (line->raw & LINE_RO) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Board_WatchdogStart(void)
{
  uint32_t began = HAL_GetTick();
  IWDG->KR = 0xCCCCU;
  IWDG->KR = 0x5555U;
  IWDG->PR = 4U; /* LSI / 64; nominal 200 ms at 40 kHz, oscillator-dependent. */
  IWDG->RLR = 124U;
  while (IWDG->SR != 0U) {
    if (HAL_GetTick() - began > 20U) Error_Handler();
  }
  Board_WatchdogFeed();
}

void Board_WatchdogFeed(void) { IWDG->KR = 0xAAAAU; }
