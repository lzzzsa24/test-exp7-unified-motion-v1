#include "battery_monitor.h"

#include "main.h"

#define BATTERY_ADC_CHANNEL                 5U
#define BATTERY_SAMPLE_INTERVAL_MS        200U
#define BATTERY_SAMPLE_COUNT                8U
#define BATTERY_ADC_TIMEOUT_LOOPS       20000U
#define BATTERY_ADC_MIN_VALID              80U

/* Schematic divider: upper=10 kOhm, lower=3.3 kOhm. */
#define BATTERY_DIVIDER_UPPER_OHM       10000ULL
#define BATTERY_DIVIDER_LOWER_OHM        3300ULL
#define BATTERY_ADC_REFERENCE_MV         3300ULL
#define BATTERY_ADC_FULL_SCALE           4095ULL
#define BATTERY_LOW_MV                   7000U

static BatteryMonitorStatus battery_status;
static uint32_t last_sample_ms;
static uint32_t filtered_raw_x8;
static uint8_t filter_ready;

static uint16_t battery_adc_read(void)
{
  uint32_t timeout = BATTERY_ADC_TIMEOUT_LOOPS;

  /* ADC3 is initialized and calibrated by ir_avoid_init().  All accesses are
   * made sequentially from the main loop, so changing SQR3 cannot race the
   * PF9/PF10 infrared conversions. */
  ADC3->SQR1 &= ~ADC_SQR1_L;
  ADC3->SQR3 = BATTERY_ADC_CHANNEL & ADC_SQR3_SQ1;
  ADC3->SR &= ~ADC_SR_EOC;
  ADC3->CR2 |= ADC_CR2_SWSTART;
  while (((ADC3->SR & ADC_SR_EOC) == 0U) && timeout > 0U)
  {
    --timeout;
  }
  if (timeout == 0U)
  {
    return 0U;
  }
  return (uint16_t)ADC3->DR;
}

static uint16_t raw_to_millivolts(uint16_t raw)
{
  uint64_t numerator = (uint64_t)raw * BATTERY_ADC_REFERENCE_MV *
      (BATTERY_DIVIDER_UPPER_OHM + BATTERY_DIVIDER_LOWER_OHM);
  uint64_t denominator = BATTERY_ADC_FULL_SCALE *
      BATTERY_DIVIDER_LOWER_OHM;

  return (uint16_t)((numerator + denominator / 2ULL) / denominator);
}

static uint8_t interpolate_percent(uint16_t millivolts,
                                   uint16_t lower_mv,
                                   uint8_t lower_percent,
                                   uint16_t upper_mv,
                                   uint8_t upper_percent)
{
  uint32_t voltage_span = (uint32_t)upper_mv - lower_mv;
  uint32_t percent_span = (uint32_t)upper_percent - lower_percent;

  return (uint8_t)(lower_percent +
      (((uint32_t)millivolts - lower_mv) * percent_span +
       voltage_span / 2U) / voltage_span);
}

/* Approximate resting-voltage curve for a two-cell lithium-ion pack.  The
 * voltage remains authoritative; percentage will vary under motor load. */
static uint8_t voltage_to_percent(uint16_t millivolts)
{
  if (millivolts >= 8400U) return 100U;
  if (millivolts >= 8100U)
    return interpolate_percent(millivolts, 8100U, 90U, 8400U, 100U);
  if (millivolts >= 7900U)
    return interpolate_percent(millivolts, 7900U, 75U, 8100U, 90U);
  if (millivolts >= 7700U)
    return interpolate_percent(millivolts, 7700U, 60U, 7900U, 75U);
  if (millivolts >= 7500U)
    return interpolate_percent(millivolts, 7500U, 40U, 7700U, 60U);
  if (millivolts >= 7300U)
    return interpolate_percent(millivolts, 7300U, 20U, 7500U, 40U);
  if (millivolts >= 7000U)
    return interpolate_percent(millivolts, 7000U, 5U, 7300U, 20U);
  if (millivolts >= 6800U)
    return interpolate_percent(millivolts, 6800U, 0U, 7000U, 5U);
  return 0U;
}

void BatteryMonitor_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &gpio);

  /* ADC3 channel 5 uses the same long sample time as the infrared channels. */
  ADC3->SMPR2 |= ADC_SMPR2_SMP5;

  battery_status.millivolts = 0U;
  battery_status.raw_adc = 0U;
  battery_status.percent = 0U;
  battery_status.valid = 0U;
  battery_status.low = 1U;
  filtered_raw_x8 = 0U;
  filter_ready = 0U;
  last_sample_ms = HAL_GetTick() - BATTERY_SAMPLE_INTERVAL_MS;
}

void BatteryMonitor_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t sum = 0U;
  uint16_t average;
  uint16_t filtered;
  uint8_t sample;

  if (now - last_sample_ms < BATTERY_SAMPLE_INTERVAL_MS)
  {
    return;
  }
  last_sample_ms = now;

  for (sample = 0U; sample < BATTERY_SAMPLE_COUNT; ++sample)
  {
    sum += battery_adc_read();
  }
  average = (uint16_t)((sum + BATTERY_SAMPLE_COUNT / 2U) /
                       BATTERY_SAMPLE_COUNT);
  if (average < BATTERY_ADC_MIN_VALID)
  {
    battery_status.raw_adc = average;
    battery_status.millivolts = 0U;
    battery_status.percent = 0U;
    battery_status.valid = 0U;
    battery_status.low = 1U;
    filter_ready = 0U;
    return;
  }

  if (filter_ready == 0U)
  {
    filtered_raw_x8 = (uint32_t)average * 8U;
    filter_ready = 1U;
  }
  else
  {
    /* First-order low-pass: new value contributes 1/8 every 200 ms. */
    filtered_raw_x8 = (filtered_raw_x8 * 7U +
                       (uint32_t)average * 8U + 4U) / 8U;
  }
  filtered = (uint16_t)((filtered_raw_x8 + 4U) / 8U);

  battery_status.raw_adc = filtered;
  battery_status.millivolts = raw_to_millivolts(filtered);
  battery_status.percent = voltage_to_percent(battery_status.millivolts);
  battery_status.valid = 1U;
  battery_status.low = battery_status.millivolts < BATTERY_LOW_MV ? 1U : 0U;
}

void BatteryMonitor_Get(BatteryMonitorStatus *status)
{
  if (status != 0)
  {
    *status = battery_status;
  }
}
