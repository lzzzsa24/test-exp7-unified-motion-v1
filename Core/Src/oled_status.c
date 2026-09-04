/*
 * Board OLED status display.
 * Schematic: I2C1 SCL=PB6, SDA=PB7, 0.91 inch OLED connector.
 * The common SSD1306 128x32 addresses 0x3C and 0x3D are probed at startup.
 */

#include "oled_status.h"

#include "main.h"

#define OLED_WIDTH                 128U
#define OLED_PAGES                   4U
#define OLED_BUFFER_SIZE          (OLED_WIDTH * OLED_PAGES)
#define OLED_ADDRESS_PRIMARY       0x3CU
#define OLED_ADDRESS_SECONDARY     0x3DU
#define OLED_I2C_TIMEOUT_MS           4U
#define OLED_PAGE_INTERVAL_MS         6U
#define OLED_FULL_REFRESH_MS        800U

static uint8_t framebuffer[OLED_BUFFER_SIZE];
static uint8_t transfer_buffer[OLED_WIDTH + 1U];
static uint8_t oled_address;
static uint8_t oled_ready;
static uint8_t oled_dirty;
static uint8_t next_page;
static uint8_t consecutive_errors;
static uint32_t last_page_ms;
static uint32_t last_refresh_ms;
static uint8_t cached_mode = 0xFFU;
static uint8_t cached_valid = 0xFFU;
static uint16_t cached_distance_mm = 0xFFFFU;
static int16_t cached_speed_mm_s = (int16_t)0x7FFF;
static int32_t cached_displacement_mm = 0x7FFFFFFFL;
static uint16_t battery_millivolts;
static uint8_t battery_percent;
static uint8_t battery_valid;
static uint8_t battery_low;

static void short_gpio_delay(void)
{
  volatile uint32_t count;

  for (count = 0U; count < 160U; ++count)
  {
    __NOP();
  }
}

/*
 * The OLED remains powered while the MCU enters the UART bootloader.  If reset
 * interrupts an I2C transfer, SDA can remain low and the new application sees
 * a permanently busy bus while the OLED continues showing its old framebuffer.
 * Release SDA, clock out the unfinished byte, then generate a GPIO STOP before
 * assigning PB6/PB7 back to I2C1.
 */
static void oled_bus_recover(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t pulse;

  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  short_gpio_delay();
  for (pulse = 0U; pulse < 9U; ++pulse)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    short_gpio_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    short_gpio_delay();
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  short_gpio_delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  short_gpio_delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  short_gpio_delay();
}

static const uint8_t font_digits[10][5] =
{
  {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
  {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
  {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
  {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
  {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
  {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
  {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
  {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
  {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
  {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}
};

static const uint8_t font_letters[26][5] =
{
  {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU},
  {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U},
  {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U},
  {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU},
  {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U},
  {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U},
  {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU},
  {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU},
  {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U},
  {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U},
  {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U},
  {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U},
  {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU},
  {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU},
  {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU},
  {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U},
  {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU},
  {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U},
  {0x46U, 0x49U, 0x49U, 0x49U, 0x31U},
  {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U},
  {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU},
  {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU},
  {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU},
  {0x63U, 0x14U, 0x08U, 0x14U, 0x63U},
  {0x07U, 0x08U, 0x70U, 0x08U, 0x07U},
  {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}
};

static const uint8_t font_space[5] = {0U, 0U, 0U, 0U, 0U};
static const uint8_t font_plus[5] = {0x08U, 0x08U, 0x3EU, 0x08U, 0x08U};
static const uint8_t font_minus[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
static const uint8_t font_dot[5] = {0x00U, 0x60U, 0x60U, 0x00U, 0x00U};
static const uint8_t font_slash[5] = {0x20U, 0x10U, 0x08U, 0x04U, 0x02U};
static const uint8_t font_colon[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
static const uint8_t font_percent[5] = {0x62U, 0x64U, 0x08U, 0x13U, 0x23U};
static const uint8_t font_exclamation[5] = {0x00U, 0x00U, 0x5FU, 0x00U, 0x00U};

static const uint8_t *glyph_for(char character)
{
  if (character >= '0' && character <= '9')
  {
    return font_digits[(uint8_t)(character - '0')];
  }
  if (character >= 'A' && character <= 'Z')
  {
    return font_letters[(uint8_t)(character - 'A')];
  }

  switch (character)
  {
    case '+': return font_plus;
    case '-': return font_minus;
    case '.': return font_dot;
    case '/': return font_slash;
    case ':': return font_colon;
    case '%': return font_percent;
    case '!': return font_exclamation;
    default:  return font_space;
  }
}

static uint8_t wait_sr1_set(uint32_t mask, uint32_t timeout_ms)
{
  uint32_t started = HAL_GetTick();

  while ((I2C1->SR1 & mask) == 0U)
  {
    if ((I2C1->SR1 & I2C_SR1_AF) != 0U ||
        HAL_GetTick() - started >= timeout_ms)
    {
      return 0U;
    }
  }
  return 1U;
}

static void i2c_abort(void)
{
  I2C1->CR1 |= I2C_CR1_STOP;
  I2C1->SR1 &= ~I2C_SR1_AF;
}

static uint8_t oled_i2c_write(uint8_t address,
                              const uint8_t *data,
                              uint16_t length)
{
  uint32_t started = HAL_GetTick();
  uint16_t index;
  volatile uint32_t temporary;

  while ((I2C1->SR2 & I2C_SR2_BUSY) != 0U)
  {
    if (HAL_GetTick() - started >= OLED_I2C_TIMEOUT_MS)
    {
      i2c_abort();
      return 0U;
    }
  }

  I2C1->CR1 |= I2C_CR1_START;
  if (wait_sr1_set(I2C_SR1_SB, OLED_I2C_TIMEOUT_MS) == 0U)
  {
    i2c_abort();
    return 0U;
  }

  I2C1->DR = (uint8_t)(address << 1U);
  if (wait_sr1_set(I2C_SR1_ADDR, OLED_I2C_TIMEOUT_MS) == 0U)
  {
    i2c_abort();
    return 0U;
  }

  temporary = I2C1->SR1;
  temporary = I2C1->SR2;
  (void)temporary;

  for (index = 0U; index < length; ++index)
  {
    if (wait_sr1_set(I2C_SR1_TXE, OLED_I2C_TIMEOUT_MS) == 0U)
    {
      i2c_abort();
      return 0U;
    }
    I2C1->DR = data[index];
  }

  if (wait_sr1_set(I2C_SR1_BTF, OLED_I2C_TIMEOUT_MS) == 0U)
  {
    i2c_abort();
    return 0U;
  }
  I2C1->CR1 |= I2C_CR1_STOP;
  return 1U;
}

static uint8_t oled_send_commands(const uint8_t *commands, uint8_t count)
{
  uint8_t buffer[32];
  uint8_t index;

  if (count > (uint8_t)(sizeof(buffer) - 1U))
  {
    return 0U;
  }

  buffer[0] = 0x00U;
  for (index = 0U; index < count; ++index)
  {
    buffer[index + 1U] = commands[index];
  }
  return oled_i2c_write(oled_address, buffer, (uint16_t)count + 1U);
}

static void oled_i2c_init(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint32_t peripheral_clock_hz;
  uint32_t frequency_mhz;
  uint32_t ccr;

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();

  oled_bus_recover();

  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  peripheral_clock_hz = HAL_RCC_GetPCLK1Freq();
  frequency_mhz = peripheral_clock_hz / 1000000U;
  ccr = peripheral_clock_hz / (3U * 400000U);
  if (ccr == 0U)
  {
    ccr = 1U;
  }

  I2C1->CR1 = 0U;
  I2C1->CR2 = frequency_mhz & I2C_CR2_FREQ;
  I2C1->CCR = I2C_CCR_FS | ccr;
  I2C1->TRISE = ((frequency_mhz * 300U) / 1000U) + 1U;
  I2C1->CR1 = I2C_CR1_PE;
}

static void clear_framebuffer(void)
{
  uint16_t index;

  for (index = 0U; index < OLED_BUFFER_SIZE; ++index)
  {
    framebuffer[index] = 0U;
  }
}

static void draw_text(uint8_t page, uint8_t column, const char *text)
{
  uint16_t offset = (uint16_t)page * OLED_WIDTH + column;

  while (*text != '\0' && offset + 5U < OLED_BUFFER_SIZE &&
         (offset % OLED_WIDTH) + 5U < OLED_WIDTH)
  {
    const uint8_t *glyph = glyph_for(*text++);
    uint8_t glyph_column;

    for (glyph_column = 0U; glyph_column < 5U; ++glyph_column)
    {
      framebuffer[offset++] = glyph[glyph_column];
    }
    framebuffer[offset++] = 0U;
  }
}

static uint8_t append_char(char *text, uint8_t index, char value)
{
  if (index < 21U)
  {
    text[index++] = value;
  }
  return index;
}

static uint8_t append_string(char *text, uint8_t index, const char *value)
{
  while (*value != '\0' && index < 21U)
  {
    text[index++] = *value++;
  }
  return index;
}

static uint8_t append_unsigned(char *text, uint8_t index, uint32_t value)
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
    index = append_char(text, index, reverse[--count]);
  }
  return index;
}

static uint8_t append_tenths(char *text, uint8_t index, int32_t tenths)
{
  uint32_t magnitude;

  if (tenths < 0)
  {
    index = append_char(text, index, '-');
    magnitude = (uint32_t)(-tenths);
  }
  else
  {
    index = append_char(text, index, '+');
    magnitude = (uint32_t)tenths;
  }

  index = append_unsigned(text, index, magnitude / 10U);
  index = append_char(text, index, '.');
  index = append_char(text, index, (char)('0' + (magnitude % 10U)));
  return index;
}

static uint8_t append_signed(char *text, uint8_t index, int32_t value)
{
  uint32_t magnitude;

  if (value < 0)
  {
    index = append_char(text, index, '-');
    magnitude = (uint32_t)(-(int64_t)value);
  }
  else
  {
    index = append_char(text, index, '+');
    magnitude = (uint32_t)value;
  }
  return append_unsigned(text, index, magnitude);
}

static void finish_text(char *text, uint8_t index)
{
  text[index < 21U ? index : 21U] = '\0';
}

static uint8_t append_battery(char *line, uint8_t index)
{
  uint16_t rounded_mv;

  index = append_string(line, index, " B:");
  if (battery_valid == 0U)
  {
    return append_string(line, index, "---");
  }

  rounded_mv = (uint16_t)(((uint32_t)battery_millivolts + 5U) / 10U * 10U);
  index = append_unsigned(line, index, rounded_mv / 1000U);
  index = append_char(line, index, '.');
  index = append_char(line, index,
      (char)('0' + ((rounded_mv % 1000U) / 100U)));
  index = append_char(line, index,
      (char)('0' + ((rounded_mv % 100U) / 10U)));
  index = append_char(line, index, 'V');
  index = append_char(line, index, ' ');
  index = append_unsigned(line, index, battery_percent);
  index = append_char(line, index, '%');
  if (battery_low != 0U)
  {
    index = append_char(line, index, '!');
  }
  return index;
}

static void draw_battery_header(const char *prefix)
{
  char line[22];
  uint8_t index = append_string(line, 0U, prefix);

  index = append_battery(line, index);
  finish_text(line, index);
  draw_text(0U, 0U, line);
}

static void build_screen(uint8_t app_mode,
                         uint8_t motion_valid,
                         uint16_t distance_mm,
                         int16_t closing_speed_mm_s,
                         int32_t relative_displacement_mm)
{
  char line[22];
  uint8_t index;

  clear_framebuffer();
  switch (app_mode)
  {
    case 0U: draw_battery_header("INT"); break;
    case 1U: draw_battery_header("LINE"); break;
    case 2U: draw_battery_header("FIG8"); break;
    case 3U: draw_battery_header("SQUARE"); break;
    case 4U: draw_battery_header("STOP"); break;
    default: draw_battery_header("UNK"); break;
  }

  if (motion_valid == 0U)
  {
    draw_text(1U, 0U, "D:---.- CM");
    draw_text(2U, 0U, "V:---.- CM/S");
    draw_text(3U, 0U, "S:---.- CM");
    return;
  }

  index = append_string(line, 0U, "D:");
  index = append_unsigned(line, index, distance_mm / 10U);
  index = append_char(line, index, '.');
  index = append_char(line, index, (char)('0' + (distance_mm % 10U)));
  index = append_string(line, index, " CM");
  finish_text(line, index);
  draw_text(1U, 0U, line);

  index = append_string(line, 0U, "V:");
  index = append_tenths(line, index, closing_speed_mm_s);
  index = append_string(line, index, " CM/S");
  finish_text(line, index);
  draw_text(2U, 0U, line);

  index = append_string(line, 0U, "S:");
  index = append_tenths(line, index, relative_displacement_mm);
  index = append_string(line, index, " CM");
  finish_text(line, index);
  draw_text(3U, 0U, line);
}

static void build_figure8_screen(uint8_t state,
                                 uint8_t fault_mask,
                                 int32_t motor1,
                                 int32_t motor2,
                                 int32_t motor3,
                                 int32_t motor4)
{
  char line[22];
  uint8_t index;

  clear_framebuffer();
  switch (state)
  {
    case 1U: draw_battery_header("F8:L"); break;
    case 2U: draw_battery_header("F8:R"); break;
    case 3U: draw_battery_header("F8:SW"); break;
    case 4U: draw_battery_header("F8:DONE"); break;
    case 5U: draw_battery_header("F8:FAULT"); break;
    default: draw_battery_header("F8:IDLE"); break;
  }

  index = append_string(line, 0U, "M1:");
  index = append_signed(line, index, motor1);
  index = append_string(line, index, " M2:");
  index = append_signed(line, index, motor2);
  finish_text(line, index);
  draw_text(1U, 0U, line);

  index = append_string(line, 0U, "M3:");
  index = append_signed(line, index, motor3);
  index = append_string(line, index, " M4:");
  index = append_signed(line, index, motor4);
  finish_text(line, index);
  draw_text(2U, 0U, line);

  if (fault_mask != 0U)
  {
    index = append_string(line, 0U, "FAULT MASK:");
    index = append_unsigned(line, index, fault_mask);
    finish_text(line, index);
    draw_text(3U, 0U, line);
  }
  else if (state == 4U)
  {
    draw_text(3U, 0U, "ENCODERS:PASS");
  }
  else
  {
    draw_text(3U, 0U, "ENCODERS:RUNNING");
  }
}

static void build_square_screen(uint8_t state,
                                uint8_t side,
                                uint8_t fault_mask,
                                int32_t motor1,
                                int32_t motor2,
                                int32_t motor3,
                                int32_t motor4)
{
  char line[22];
  uint8_t index;

  clear_framebuffer();
  index = append_string(line, 0U, "SQ:");
  switch (state)
  {
    case 1U: index = append_char(line, index, 'D'); break;
    case 2U:
    case 4U: index = append_char(line, index, 'P'); break;
    case 3U: index = append_char(line, index, 'T'); break;
    case 5U: index = append_char(line, index, 'C'); break;
    case 6U: index = append_char(line, index, 'F'); break;
    default: index = append_char(line, index, 'I'); break;
  }
  index = append_unsigned(line, index, side);
  index = append_battery(line, index);
  finish_text(line, index);
  draw_text(0U, 0U, line);

  index = append_string(line, 0U, "M1:");
  index = append_signed(line, index, motor1);
  index = append_string(line, index, " M2:");
  index = append_signed(line, index, motor2);
  finish_text(line, index);
  draw_text(1U, 0U, line);

  index = append_string(line, 0U, "M3:");
  index = append_signed(line, index, motor3);
  index = append_string(line, index, " M4:");
  index = append_signed(line, index, motor4);
  finish_text(line, index);
  draw_text(2U, 0U, line);

  if (fault_mask != 0U)
  {
    index = append_string(line, 0U, "FAULT MASK:");
    index = append_unsigned(line, index, fault_mask);
    finish_text(line, index);
    draw_text(3U, 0U, line);
  }
  else if (state == 5U)
  {
    draw_text(3U, 0U, "SQUARE COMPLETE");
  }
  else
  {
    draw_text(3U, 0U, "ENCODERS:RUNNING");
  }
}

void OledStatus_Init(void)
{
  static const uint8_t init_commands[] =
  {
    0xAEU, 0xD5U, 0x80U, 0xA8U, 0x1FU, 0xD3U, 0x00U, 0x40U,
    0x8DU, 0x14U, 0x20U, 0x02U, 0xA1U, 0xC8U, 0xDAU, 0x02U,
    0x81U, 0x8FU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U,
    0xAFU
  };
  uint8_t probe[2] = {0x00U, 0xAEU};

  oled_ready = 0U;
  oled_dirty = 0U;
  consecutive_errors = 0U;
  oled_i2c_init();

  oled_address = OLED_ADDRESS_PRIMARY;
  if (oled_i2c_write(oled_address, probe, sizeof(probe)) == 0U)
  {
    oled_address = OLED_ADDRESS_SECONDARY;
    if (oled_i2c_write(oled_address, probe, sizeof(probe)) == 0U)
    {
      return;
    }
  }

  if (oled_send_commands(init_commands, sizeof(init_commands)) == 0U)
  {
    return;
  }

  oled_ready = 1U;
  cached_mode = 0xFFU;
  cached_valid = 0xFFU;
  clear_framebuffer();
  draw_text(0U, 0U, "OLED READY");
  oled_dirty = 1U;
  next_page = 0U;
  last_page_ms = HAL_GetTick() - OLED_PAGE_INTERVAL_MS;
  last_refresh_ms = HAL_GetTick();
}

void OledStatus_SetBattery(uint16_t millivolts,
                           uint8_t percent,
                           uint8_t valid,
                           uint8_t low)
{
  uint16_t rounded_mv = (uint16_t)(((uint32_t)millivolts + 5U) / 10U * 10U);

  valid = valid != 0U ? 1U : 0U;
  low = low != 0U ? 1U : 0U;
  if (percent > 100U)
  {
    percent = 100U;
  }
  if (battery_millivolts == rounded_mv &&
      battery_percent == percent &&
      battery_valid == valid &&
      battery_low == low)
  {
    return;
  }

  battery_millivolts = rounded_mv;
  battery_percent = percent;
  battery_valid = valid;
  battery_low = low;
  /* Force the normal-mode cache to rebuild its first row. */
  cached_mode = 0xFFU;
}

void OledStatus_SetData(uint8_t integrated_mode,
                        uint8_t motion_valid,
                        uint16_t distance_mm,
                        int16_t closing_speed_mm_s,
                        int32_t relative_displacement_mm)
{
  motion_valid = motion_valid != 0U ? 1U : 0U;

  if (cached_mode == integrated_mode && cached_valid == motion_valid &&
      cached_distance_mm == distance_mm &&
      cached_speed_mm_s == closing_speed_mm_s &&
      cached_displacement_mm == relative_displacement_mm)
  {
    return;
  }

  cached_mode = integrated_mode;
  cached_valid = motion_valid;
  cached_distance_mm = distance_mm;
  cached_speed_mm_s = closing_speed_mm_s;
  cached_displacement_mm = relative_displacement_mm;
  build_screen(integrated_mode, motion_valid, distance_mm,
               closing_speed_mm_s, relative_displacement_mm);
  oled_dirty = 1U;
  next_page = 0U;
}

void OledStatus_SetFigure8Data(uint8_t state,
                               uint8_t fault_mask,
                               int32_t motor1,
                               int32_t motor2,
                               int32_t motor3,
                               int32_t motor4)
{
  /* Invalidate the normal page cache so leaving KEY3 redraws immediately. */
  cached_mode = 0xFFU;
  build_figure8_screen(state, fault_mask,
                       motor1, motor2, motor3, motor4);
  oled_dirty = 1U;
  next_page = 0U;
}

void OledStatus_SetSquareData(uint8_t state,
                              uint8_t side,
                              uint8_t fault_mask,
                              int32_t motor1,
                              int32_t motor2,
                              int32_t motor3,
                              int32_t motor4)
{
  cached_mode = 0xFFU;
  build_square_screen(state, side, fault_mask,
                      motor1, motor2, motor3, motor4);
  oled_dirty = 1U;
  next_page = 0U;
}

void OledStatus_Task(void)
{
  uint8_t commands[3];
  uint16_t index;
  uint32_t now;

  if (oled_ready == 0U)
  {
    return;
  }

  now = HAL_GetTick();
  if (oled_dirty == 0U && now - last_refresh_ms >= OLED_FULL_REFRESH_MS)
  {
    oled_dirty = 1U;
    next_page = 0U;
  }
  if (oled_dirty == 0U || now - last_page_ms < OLED_PAGE_INTERVAL_MS)
  {
    return;
  }

  commands[0] = (uint8_t)(0xB0U + next_page);
  commands[1] = 0x00U;
  commands[2] = 0x10U;
  if (oled_send_commands(commands, sizeof(commands)) == 0U)
  {
    if (++consecutive_errors >= 3U)
    {
      oled_ready = 0U;
    }
    return;
  }

  transfer_buffer[0] = 0x40U;
  for (index = 0U; index < OLED_WIDTH; ++index)
  {
    transfer_buffer[index + 1U] =
        framebuffer[(uint16_t)next_page * OLED_WIDTH + index];
  }
  if (oled_i2c_write(oled_address, transfer_buffer,
                     (uint16_t)sizeof(transfer_buffer)) == 0U)
  {
    if (++consecutive_errors >= 3U)
    {
      oled_ready = 0U;
    }
    return;
  }

  consecutive_errors = 0U;
  last_page_ms = now;
  next_page++;
  if (next_page >= OLED_PAGES)
  {
    next_page = 0U;
    oled_dirty = 0U;
    last_refresh_ms = now;
  }
}

uint8_t OledStatus_IsReady(void)
{
  return oled_ready;
}
