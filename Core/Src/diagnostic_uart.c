#include "diagnostic_uart.h"

#include "main.h"

#define DIAGNOSTIC_UART_BAUD       115200UL
#define DIAGNOSTIC_UART_TIMEOUT    200000UL

static void uart_write_char(char value)
{
  uint32_t timeout = DIAGNOSTIC_UART_TIMEOUT;

  while ((USART1->SR & USART_SR_TXE) == 0U && timeout > 0U)
  {
    --timeout;
  }
  if (timeout > 0U)
  {
    USART1->DR = (uint16_t)(uint8_t)value;
  }
}

void DiagnosticUart_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint32_t pclk;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &gpio);

  pclk = HAL_RCC_GetPCLK2Freq();
  USART1->CR1 = 0U;
  USART1->CR2 = 0U;
  USART1->CR3 = 0U;
  USART1->BRR = (pclk + (DIAGNOSTIC_UART_BAUD / 2UL)) /
                DIAGNOSTIC_UART_BAUD;
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void DiagnosticUart_WriteString(const char *text)
{
  if (text == 0)
  {
    return;
  }

  while (*text != '\0')
  {
    uart_write_char(*text++);
  }
}

void DiagnosticUart_WriteUnsigned(uint32_t value)
{
  char reverse[10];
  uint8_t count = 0U;

  do
  {
    reverse[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while (value != 0U && count < sizeof(reverse));

  while (count > 0U)
  {
    uart_write_char(reverse[--count]);
  }
}

void DiagnosticUart_WriteSigned(int32_t value)
{
  if (value < 0)
  {
    uart_write_char('-');
    DiagnosticUart_WriteUnsigned((uint32_t)(-value));
  }
  else
  {
    uart_write_char('+');
    DiagnosticUart_WriteUnsigned((uint32_t)value);
  }
}

int16_t DiagnosticUart_ReadChar(void)
{
  uint32_t status = USART1->SR;

  if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
  {
    (void)USART1->DR;
    return -1;
  }
  if ((status & USART_SR_RXNE) == 0U)
  {
    return -1;
  }
  return (int16_t)(USART1->DR & 0xFFU);
}
