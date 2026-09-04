/*
 * 实验七：USART2(PD5/PD6) 视觉指令接收
 *
 * 为了让没有接 K210 时仍能正常循迹，本模块采用非阻塞轮询，并且把
 * 无效/半截数据丢弃。K210 端推荐发送一行 ASCII，例如 "right\n"；同时
 * 保留了 0xA5 + id + 0x5A 的三字节帧，便于以后换成训练好的模型程序。
 */

#include "vision_uart.h"

#include <string.h>

#include "main.h"

#define VISION_UART_BAUD                 115200U
#define VISION_TOKEN_BUFFER_SIZE              64U
#define VISION_TX_BUFFER_SIZE                 48U
#define VISION_MAX_BYTES_PER_POLL             32U
#define VISION_REPEAT_GUARD_MS              1200U

static char token_buffer[VISION_TOKEN_BUFFER_SIZE];
static uint8_t token_length;
static uint8_t binary_state;
static uint8_t binary_id;
static VisionCommand pending_command;
static VisionCommand last_command;
static uint32_t last_command_tick;
static uint8_t tx_buffer[VISION_TX_BUFFER_SIZE];
static uint8_t tx_length;
static uint8_t tx_index;

static uint8_t append_character(uint8_t index, char value)
{
  if (index < VISION_TX_BUFFER_SIZE)
  {
    tx_buffer[index++] = (uint8_t)value;
  }
  return index;
}

static uint8_t append_unsigned(uint8_t index, uint32_t value)
{
  char reversed[10];
  uint8_t count = 0U;

  do
  {
    reversed[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while (value != 0U && count < sizeof(reversed));

  while (count > 0U)
  {
    index = append_character(index, reversed[--count]);
  }
  return index;
}

static uint8_t append_signed(uint8_t index, int32_t value)
{
  uint32_t magnitude;

  if (value < 0)
  {
    index = append_character(index, '-');
    magnitude = (uint32_t)(-value);
  }
  else
  {
    magnitude = (uint32_t)value;
  }

  return append_unsigned(index, magnitude);
}

static int token_equal(const char *token, const char *expected)
{
  while (*token != '\0' && *expected != '\0')
  {
    char left = *token;
    char right = *expected;

    if (left >= 'A' && left <= 'Z')
    {
      left = (char)(left - 'A' + 'a');
    }
    if (right >= 'A' && right <= 'Z')
    {
      right = (char)(right - 'A' + 'a');
    }

    if (left != right)
    {
      return 0;
    }
    ++token;
    ++expected;
  }

  return *token == '\0' && *expected == '\0';
}

static VisionCommand command_from_id(uint8_t id)
{
  switch (id)
  {
    case 1U:  return VISION_CMD_GREEN_LIGHT;
    case 2U:  return VISION_CMD_SCHOOL;
    case 3U:  return VISION_CMD_WALK;
    case 4U:  return VISION_CMD_RIGHT;
    case 5U:  return VISION_CMD_LEFT;
    case 6U:  return VISION_CMD_FREE_SPEED;
    case 7U:  return VISION_CMD_LIMIT_SPEED;
    case 8U:  return VISION_CMD_HORN;
    case 9U:  return VISION_CMD_GARAGE_ONE;
    case 10U: return VISION_CMD_GARAGE_TWO;
    case 11U: return VISION_CMD_CHUKU_TRACK_LINE;
    case 12U: return VISION_CMD_STOP;
    default:  return VISION_CMD_NONE;
  }
}

static void publish_command(VisionCommand command)
{
  uint32_t now;

  if (command == VISION_CMD_NONE)
  {
    return;
  }

  now = HAL_GetTick();
  if (command == last_command &&
      (uint32_t)(now - last_command_tick) < VISION_REPEAT_GUARD_MS)
  {
    return;
  }

  pending_command = command;
  last_command = command;
  last_command_tick = now;
}

static VisionCommand command_from_token(const char *token)
{
  /* 允许 K210 直接发送 "@right" 或 "V:right"，方便串口调试。 */
  if (token[0] == '@')
  {
    ++token;
  }
  else if (token[0] == 'V' && token[1] == ':')
  {
    token += 2;
  }

  if (token_equal(token, "green_light") || token_equal(token, "green"))
    return VISION_CMD_GREEN_LIGHT;
  if (token_equal(token, "school"))
    return VISION_CMD_SCHOOL;
  if (token_equal(token, "walk"))
    return VISION_CMD_WALK;
  if (token_equal(token, "right"))
    return VISION_CMD_RIGHT;
  if (token_equal(token, "left"))
    return VISION_CMD_LEFT;
  if (token_equal(token, "freeSpeed") || token_equal(token, "free_speed"))
    return VISION_CMD_FREE_SPEED;
  if (token_equal(token, "limitSpeed") || token_equal(token, "limit_speed"))
    return VISION_CMD_LIMIT_SPEED;
  if (token_equal(token, "horn") || token_equal(token, "beep"))
    return VISION_CMD_HORN;
  if (token_equal(token, "one") || token_equal(token, "garage1"))
    return VISION_CMD_GARAGE_ONE;
  if (token_equal(token, "two") || token_equal(token, "garage2"))
    return VISION_CMD_GARAGE_TWO;
  if (token_equal(token, "chuku_track_line") ||
      token_equal(token, "resume"))
    return VISION_CMD_CHUKU_TRACK_LINE;
  if (token_equal(token, "stop"))
    return VISION_CMD_STOP;

  return VISION_CMD_NONE;
}

static void finish_token(void)
{
  VisionCommand command;

  if (token_length == 0U)
  {
    return;
  }

  token_buffer[token_length] = '\0';
  command = command_from_token(token_buffer);
  publish_command(command);
  token_length = 0U;
}

static void consume_byte(uint8_t byte)
{
  if (binary_state == 1U)
  {
    binary_id = byte;
    binary_state = 2U;
    return;
  }

  if (binary_state == 2U)
  {
    if (byte == 0x5AU)
    {
      publish_command(command_from_id(binary_id));
    }
    binary_state = 0U;
    return;
  }

  if (byte == 0xA5U)
  {
    finish_token();
    binary_state = 1U;
    return;
  }

  if (byte == '\r' || byte == '\n')
  {
    finish_token();
    return;
  }

  if (byte < 0x20U || byte > 0x7EU)
  {
    /* 串口线上可能有调试乱码；丢弃当前行，下一行仍可恢复。 */
    token_length = 0U;
    return;
  }

  if (token_length < (VISION_TOKEN_BUFFER_SIZE - 1U))
  {
    token_buffer[token_length++] = (char)byte;
  }
  else
  {
    token_length = 0U;
  }
}

void vision_uart_init(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint32_t pclk1;
  uint32_t divider;

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_AFIO_REMAP_USART2_ENABLE();

  gpio.Pin = VISION_UART_TX_Pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(VISION_UART_TX_GPIO_Port, &gpio);

  gpio.Pin = VISION_UART_RX_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VISION_UART_RX_GPIO_Port, &gpio);

  pclk1 = HAL_RCC_GetPCLK1Freq();
  divider = (pclk1 + (VISION_UART_BAUD / 2U)) / VISION_UART_BAUD;
  if (divider < 16U)
  {
    divider = 16U;
  }

  USART2->CR1 = 0U;
  USART2->CR2 = 0U;
  USART2->CR3 = 0U;
  USART2->BRR = divider;
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

  token_length = 0U;
  binary_state = 0U;
  binary_id = 0U;
  pending_command = VISION_CMD_NONE;
  last_command = VISION_CMD_NONE;
  last_command_tick = 0U;
  tx_length = 0U;
  tx_index = 0U;
}

void vision_uart_poll(void)
{
  uint32_t count = 0U;

  while ((USART2->SR & USART_SR_RXNE) != 0U &&
         count < VISION_MAX_BYTES_PER_POLL)
  {
    uint32_t status = USART2->SR;
    uint8_t byte = (uint8_t)USART2->DR;

    /* 读取 DR 已清除 RXNE；错误标志也通过一次 SR/DR 读取复位。 */
    (void)status;
    consume_byte(byte);
    ++count;
  }

  /* 每次主循环最多发送一个字节，避免遥测阻塞寻线和测距。 */
  if (tx_index < tx_length && (USART2->SR & USART_SR_TXE) != 0U)
  {
    USART2->DR = tx_buffer[tx_index++];
    if (tx_index >= tx_length)
    {
      tx_index = 0U;
      tx_length = 0U;
    }
  }
}

void vision_uart_queue_motion_telemetry(uint8_t valid,
                                        uint16_t distance_mm,
                                        int16_t speed_mm_s,
                                        int32_t travel_mm)
{
  uint8_t index = 0U;

  if (tx_length != 0U)
  {
    return;
  }

  index = append_character(index, 'M');
  index = append_character(index, ',');
  index = append_unsigned(index, valid != 0U ? 1U : 0U);
  index = append_character(index, ',');
  index = append_unsigned(index, distance_mm);
  index = append_character(index, ',');
  index = append_signed(index, speed_mm_s);
  index = append_character(index, ',');
  index = append_signed(index, travel_mm);
  index = append_character(index, '\n');
  tx_length = index;
  tx_index = 0U;
}

VisionCommand vision_uart_take_event(void)
{
  VisionCommand command;

  vision_uart_poll();
  command = pending_command;
  pending_command = VISION_CMD_NONE;
  return command;
}

const char *vision_command_name(VisionCommand command)
{
  switch (command)
  {
    case VISION_CMD_GREEN_LIGHT:      return "green_light";
    case VISION_CMD_SCHOOL:           return "school";
    case VISION_CMD_WALK:             return "walk";
    case VISION_CMD_RIGHT:            return "right";
    case VISION_CMD_LEFT:             return "left";
    case VISION_CMD_FREE_SPEED:       return "freeSpeed";
    case VISION_CMD_LIMIT_SPEED:      return "limitSpeed";
    case VISION_CMD_HORN:             return "horn";
    case VISION_CMD_GARAGE_ONE:       return "one";
    case VISION_CMD_GARAGE_TWO:       return "two";
    case VISION_CMD_CHUKU_TRACK_LINE: return "chuku_track_line";
    case VISION_CMD_STOP:             return "stop";
    default:                          return "none";
  }
}
