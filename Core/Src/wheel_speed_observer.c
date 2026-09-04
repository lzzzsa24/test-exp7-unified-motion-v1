#include "wheel_speed_observer.h"

#include "main.h"
#include "wheel_encoder.h"

#define SPEED_OBSERVER_PERIOD_MS        20U
#define SPEED_OBSERVER_STALE_MS        120U
#define SPEED_OBSERVER_MAX_CPS        12000U

static uint8_t observer_running;
static uint8_t speed_valid;
static uint32_t average_cps_filtered;
static uint32_t last_sample_ms;
static uint32_t last_valid_ms;
static WheelEncoderCounts previous_counts;

static int32_t abs_i32(int32_t value)
{
  return value < 0L ? -value : value;
}

void WheelSpeedObserver_Init(void)
{
  observer_running = 0U;
  speed_valid = 0U;
  average_cps_filtered = 0U;
  last_sample_ms = 0U;
  last_valid_ms = 0U;
}

void WheelSpeedObserver_Start(void)
{
  WheelEncoder_GetCounts(&previous_counts);
  last_sample_ms = HAL_GetTick();
  last_valid_ms = last_sample_ms;
  average_cps_filtered = 0U;
  speed_valid = 0U;
  observer_running = 1U;
}

void WheelSpeedObserver_Stop(void)
{
  observer_running = 0U;
  speed_valid = 0U;
  average_cps_filtered = 0U;
}

void WheelSpeedObserver_Task(void)
{
  WheelEncoderCounts current;
  uint32_t now;
  uint32_t elapsed_ms;
  uint64_t delta_sum;
  uint32_t instant_cps;

  if (observer_running == 0U)
  {
    return;
  }
  now = HAL_GetTick();
  if (WheelEncoder_IsRunning() == 0U)
  {
    speed_valid = 0U;
    return;
  }
  elapsed_ms = now - last_sample_ms;
  if (elapsed_ms < SPEED_OBSERVER_PERIOD_MS)
  {
    if (speed_valid != 0U && now - last_valid_ms > SPEED_OBSERVER_STALE_MS)
    {
      speed_valid = 0U;
    }
    return;
  }

  WheelEncoder_GetCounts(&current);
  delta_sum = (uint32_t)abs_i32(current.motor1 - previous_counts.motor1);
  delta_sum += (uint32_t)abs_i32(current.motor2 - previous_counts.motor2);
  delta_sum += (uint32_t)abs_i32(current.motor3 - previous_counts.motor3);
  delta_sum += (uint32_t)abs_i32(current.motor4 - previous_counts.motor4);
  previous_counts = current;
  last_sample_ms = now;

  instant_cps = (uint32_t)((delta_sum * 1000ULL) /
                           ((uint64_t)elapsed_ms * 4ULL));
  if (instant_cps > SPEED_OBSERVER_MAX_CPS)
  {
    speed_valid = 0U;
    return;
  }

  if (speed_valid == 0U)
  {
    average_cps_filtered = instant_cps;
  }
  else
  {
    average_cps_filtered =
        (average_cps_filtered * 3U + instant_cps + 2U) / 4U;
  }
  speed_valid = 1U;
  last_valid_ms = now;
}

uint8_t WheelSpeedObserver_GetAverageCps(uint32_t *average_cps)
{
  uint32_t now = HAL_GetTick();

  if (average_cps == 0)
  {
    return 0U;
  }
  if (observer_running == 0U || speed_valid == 0U ||
      now - last_valid_ms > SPEED_OBSERVER_STALE_MS)
  {
    *average_cps = 0U;
    return 0U;
  }
  *average_cps = average_cps_filtered;
  return 1U;
}
