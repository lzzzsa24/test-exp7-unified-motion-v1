/*
 * 使用固定目标距离变化估计沿超声波声束方向的相对速度和位移。
 * 全部使用整数定点计算，避免在 STM32F103 上引入浮点运行时。
 */

#include "ultrasonic_motion.h"

#include <stddef.h>

#define MOTION_FILTER_SAMPLES          3U
#define MOTION_MIN_DISTANCE_MM        20U
#define MOTION_MAX_DISTANCE_MM      4000U
#define MOTION_MIN_DT_MS              40U
#define MOTION_MAX_DT_MS             300U
#define MOTION_STALE_MS              350U
#define MOTION_MAX_SPEED_MM_S       1500L
#define MOTION_JUMP_MARGIN_MM          40L
#define MOTION_OUTPUT_LIMIT_MM_S     2000L
#define MOTION_VALID_SPEED_SAMPLES      2U

static uint16_t samples[MOTION_FILTER_SAMPLES];
static uint8_t sample_count;
static uint8_t sample_index;
static uint8_t have_filtered_sample;
static uint8_t speed_sample_count;
static uint16_t reference_distance_mm;
static uint16_t previous_distance_mm;
static int16_t filtered_speed_mm_s;
static UltrasonicMotionEstimate current_estimate;

static uint16_t median_three(uint16_t a, uint16_t b, uint16_t c)
{
  uint16_t temporary;

  if (a > b)
  {
    temporary = a;
    a = b;
    b = temporary;
  }
  if (b > c)
  {
    temporary = b;
    b = c;
    c = temporary;
  }
  if (a > b)
  {
    b = a;
  }

  return b;
}

static int32_t absolute_i32(int32_t value)
{
  return value < 0 ? -value : value;
}

static int16_t clamp_speed(int32_t speed_mm_s)
{
  if (speed_mm_s > MOTION_OUTPUT_LIMIT_MM_S)
  {
    return (int16_t)MOTION_OUTPUT_LIMIT_MM_S;
  }
  if (speed_mm_s < -MOTION_OUTPUT_LIMIT_MM_S)
  {
    return (int16_t)-MOTION_OUTPUT_LIMIT_MM_S;
  }
  return (int16_t)speed_mm_s;
}

static void start_new_reference(uint16_t distance_mm, uint32_t now_ms)
{
  reference_distance_mm = distance_mm;
  previous_distance_mm = distance_mm;
  filtered_speed_mm_s = 0;
  speed_sample_count = 0U;
  have_filtered_sample = 1U;
  current_estimate.valid = 0U;
  current_estimate.target_distance_mm = distance_mm;
  current_estimate.closing_speed_mm_s = 0;
  current_estimate.relative_displacement_mm = 0;
  current_estimate.last_update_ms = now_ms;
}

void UltrasonicMotion_Init(void)
{
  UltrasonicMotion_Reset();
}

void UltrasonicMotion_Reset(void)
{
  uint8_t index;

  for (index = 0U; index < MOTION_FILTER_SAMPLES; ++index)
  {
    samples[index] = 0U;
  }

  sample_count = 0U;
  sample_index = 0U;
  have_filtered_sample = 0U;
  speed_sample_count = 0U;
  reference_distance_mm = 0U;
  previous_distance_mm = 0U;
  filtered_speed_mm_s = 0;
  current_estimate.valid = 0U;
  current_estimate.target_distance_mm = 0U;
  current_estimate.closing_speed_mm_s = 0;
  current_estimate.relative_displacement_mm = 0;
  current_estimate.last_update_ms = 0U;
}

void UltrasonicMotion_Update(uint16_t distance_mm, uint32_t now_ms)
{
  uint16_t filtered_distance_mm;
  uint32_t elapsed_ms;
  int32_t distance_delta_mm;
  int32_t maximum_delta_mm;
  int32_t instant_speed_mm_s;

  if (distance_mm < MOTION_MIN_DISTANCE_MM ||
      distance_mm > MOTION_MAX_DISTANCE_MM)
  {
    UltrasonicMotion_NoteInvalid(now_ms);
    return;
  }

  samples[sample_index] = distance_mm;
  sample_index = (uint8_t)((sample_index + 1U) % MOTION_FILTER_SAMPLES);
  if (sample_count < MOTION_FILTER_SAMPLES)
  {
    sample_count++;
  }

  if (sample_count < MOTION_FILTER_SAMPLES)
  {
    return;
  }

  filtered_distance_mm = median_three(samples[0], samples[1], samples[2]);
  if (have_filtered_sample == 0U)
  {
    start_new_reference(filtered_distance_mm, now_ms);
    return;
  }

  elapsed_ms = now_ms - current_estimate.last_update_ms;
  if (elapsed_ms < MOTION_MIN_DT_MS)
  {
    return;
  }
  if (elapsed_ms > MOTION_MAX_DT_MS)
  {
    start_new_reference(filtered_distance_mm, now_ms);
    return;
  }

  /* 正值表示距离正在减小，即小车沿声束方向接近同一目标。 */
  distance_delta_mm = (int32_t)previous_distance_mm -
                      (int32_t)filtered_distance_mm;
  maximum_delta_mm = MOTION_JUMP_MARGIN_MM +
                     (MOTION_MAX_SPEED_MM_S * (int32_t)elapsed_ms) / 1000L;

  /* 跳变通常表示墙面角度改变或反射目标切换，重新建参考点而不是
     把它累计成车辆位移。 */
  if (absolute_i32(distance_delta_mm) > maximum_delta_mm)
  {
    start_new_reference(filtered_distance_mm, now_ms);
    return;
  }

  instant_speed_mm_s = (distance_delta_mm * 1000L) /
                       (int32_t)elapsed_ms;
  if (speed_sample_count == 0U)
  {
    filtered_speed_mm_s = clamp_speed(instant_speed_mm_s);
  }
  else
  {
    filtered_speed_mm_s = clamp_speed(
        ((int32_t)filtered_speed_mm_s * 3L + instant_speed_mm_s) / 4L);
  }

  if (speed_sample_count < 255U)
  {
    speed_sample_count++;
  }

  previous_distance_mm = filtered_distance_mm;
  current_estimate.target_distance_mm = filtered_distance_mm;
  current_estimate.closing_speed_mm_s = filtered_speed_mm_s;
  current_estimate.relative_displacement_mm =
      (int32_t)reference_distance_mm - (int32_t)filtered_distance_mm;
  current_estimate.last_update_ms = now_ms;
  current_estimate.valid = speed_sample_count >= MOTION_VALID_SPEED_SAMPLES
                         ? 1U : 0U;
}

void UltrasonicMotion_NoteInvalid(uint32_t now_ms)
{
  if (have_filtered_sample == 0U ||
      now_ms - current_estimate.last_update_ms > MOTION_STALE_MS)
  {
    current_estimate.valid = 0U;
  }
}

void UltrasonicMotion_Task(uint32_t now_ms)
{
  if (have_filtered_sample != 0U &&
      now_ms - current_estimate.last_update_ms > MOTION_STALE_MS)
  {
    current_estimate.valid = 0U;
  }
}

void UltrasonicMotion_Get(UltrasonicMotionEstimate *estimate)
{
  if (estimate != NULL)
  {
    *estimate = current_estimate;
  }
}
