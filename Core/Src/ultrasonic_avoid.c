/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ultrasonic_avoid.c
  * @brief   与电机实现无关的超声波避障状态机
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "ultrasonic.h"
#include "ultrasonic_avoid.h"
#include "ultrasonic_motion.h"

#define AVOID_MEASURE_INTERVAL_MS    60U
#define AVOID_MEASURE_STALE_MS       250U
#define AVOID_CONFIRM_COUNT          2U
#define AVOID_EMERGENCY_CONFIRM_COUNT 2U
#define AVOID_CRITICAL_RAW_CM        10U
#define AVOID_FILTER_SAMPLES         3U
#define AVOID_DEFAULT_STOP_CM        25U
#define AVOID_DEFAULT_CLEAR_CM       35U
#define AVOID_DEFAULT_TURN_TIME_MS   500U
#define AVOID_DEFAULT_STOP_TIME_MS   120U
#define AVOID_DEFAULT_REVERSE_MS     300U
#define AVOID_DEFAULT_GUARD_MS        60U
#define AVOID_DEFAULT_NO_ECHO_COUNT    3U

static UltrasonicAvoidDriveCallback drive_callback;
static UltrasonicAvoidDriveCallback reverse_callback;
static UltrasonicAvoidStopCallback stop_callback;
static UltrasonicAvoidTurnCallback turn_left_callback;
static UltrasonicAvoidTurnCallback turn_right_callback;

static UltrasonicAvoidState avoid_state = ULTRASONIC_AVOID_WAIT_SAFE;
static uint16_t stop_distance_cm = AVOID_DEFAULT_STOP_CM;
static uint16_t clear_distance_cm = AVOID_DEFAULT_CLEAR_CM;
static uint16_t emergency_distance_cm = AVOID_DEFAULT_STOP_CM;
static int16_t cruise_speed = 2800;
static int16_t slow_speed = 2500;
static int16_t turn_inner_speed = 2400;
static int16_t turn_outer_speed = 3300;
static int16_t reverse_speed = 2200;
static uint32_t turn_time_ms = AVOID_DEFAULT_TURN_TIME_MS;
static uint32_t stop_time_ms = AVOID_DEFAULT_STOP_TIME_MS;
static uint32_t reverse_time_ms = AVOID_DEFAULT_REVERSE_MS;
static uint32_t guard_time_ms = AVOID_DEFAULT_GUARD_MS;

static uint16_t filtered_distance_cm = 0U;
static uint8_t distance_valid = 0U;
static uint8_t obstacle_count = 0U;
static uint8_t emergency_count = 0U;
static uint8_t sample_count = 0U;
static uint8_t sample_index = 0U;
static uint16_t distance_samples[AVOID_FILTER_SAMPLES] = {0U};
static uint32_t last_trigger_ms = 0U;
static uint32_t last_valid_measurement_ms = 0U;
static uint32_t maneuver_start_ms = 0U;
static uint32_t cooldown_start_ms = 0U;
static uint8_t next_turn_direction = ULTRASONIC_AVOID_TURN_LEFT;
static uint8_t no_echo_fallback_enabled = 0U;
static uint8_t no_echo_fallback_active = 0U;
static uint8_t no_echo_timeout_count = 0U;
static uint8_t no_echo_timeout_limit = AVOID_DEFAULT_NO_ECHO_COUNT;

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

static void stop_motors(void)
{
  if (stop_callback != NULL)
  {
    stop_callback();
  }
}

static void clear_filter(void)
{
  distance_valid = 0U;
  sample_count = 0U;
  sample_index = 0U;
  obstacle_count = 0U;
  emergency_count = 0U;
}

static void accept_distance(uint16_t distance_cm, uint32_t now_ms)
{
  distance_samples[sample_index] = distance_cm;
  sample_index = (uint8_t)((sample_index + 1U) % AVOID_FILTER_SAMPLES);

  if (sample_count < AVOID_FILTER_SAMPLES)
  {
    sample_count++;
  }

  if (sample_count == AVOID_FILTER_SAMPLES)
  {
    filtered_distance_cm = median_three(distance_samples[0],
                                        distance_samples[1],
                                        distance_samples[2]);
    distance_valid = 1U;
  }

  last_valid_measurement_ms = now_ms;
}

static void begin_turn(uint32_t now_ms)
{
  stop_motors();
  /* 转弯后声束指向的目标已经改变，墙面相对位移参考必须重建。 */
  UltrasonicMotion_Reset();
  maneuver_start_ms = now_ms;
  avoid_state = ULTRASONIC_AVOID_STOPPING;
  obstacle_count = 0U;
  emergency_count = 0U;
}

void UltrasonicAvoid_Init(UltrasonicAvoidDriveCallback drive,
                          UltrasonicAvoidStopCallback stop,
                          UltrasonicAvoidTurnCallback turn_left,
                          UltrasonicAvoidTurnCallback turn_right)
{
  drive_callback = drive;
  reverse_callback = NULL;
  stop_callback = stop;
  turn_left_callback = turn_left;
  turn_right_callback = turn_right;
  avoid_state = ULTRASONIC_AVOID_WAIT_SAFE;
  last_trigger_ms = HAL_GetTick() - AVOID_MEASURE_INTERVAL_MS;
  last_valid_measurement_ms = HAL_GetTick();
  maneuver_start_ms = 0U;
  cooldown_start_ms = 0U;
  next_turn_direction = ULTRASONIC_AVOID_TURN_LEFT;
  no_echo_fallback_active = 0U;
  no_echo_timeout_count = 0U;
  filtered_distance_cm = 0U;
  clear_filter();
  UltrasonicMotion_Init();
  stop_motors();
}

void UltrasonicAvoid_SetThresholds(uint16_t stop_cm, uint16_t clear_cm)
{
  if (stop_cm >= clear_cm || stop_cm == 0U)
  {
    return;
  }

  stop_distance_cm = stop_cm;
  clear_distance_cm = clear_cm;
  if (emergency_distance_cm < stop_distance_cm)
  {
    emergency_distance_cm = stop_distance_cm;
  }
  if (emergency_distance_cm >= clear_distance_cm)
  {
    emergency_distance_cm = (uint16_t)(clear_distance_cm - 1U);
  }
}

void UltrasonicAvoid_SetEmergencyDistance(uint16_t emergency_cm)
{
  if (emergency_cm < stop_distance_cm)
  {
    emergency_cm = stop_distance_cm;
  }
  if (emergency_cm >= clear_distance_cm)
  {
    emergency_cm = (uint16_t)(clear_distance_cm - 1U);
  }
  /* Once the first near sample has slowed the car, do not let the measured
     wheel-speed drop pull the threshold backward before the confirm sample. */
  if (emergency_count != 0U && emergency_cm < emergency_distance_cm)
  {
    return;
  }
  emergency_distance_cm = emergency_cm;
}

void UltrasonicAvoid_SetSpeeds(int16_t cruise,
                               int16_t slow,
                               int16_t turn_inner,
                               int16_t turn_outer)
{
  if (cruise > 0)
  {
    cruise_speed = cruise;
  }
  if (slow > 0)
  {
    slow_speed = slow;
  }
  if (turn_inner > 0)
  {
    turn_inner_speed = turn_inner;
  }
  if (turn_outer > 0)
  {
    turn_outer_speed = turn_outer;
  }
}

void UltrasonicAvoid_SetTurnTime(uint32_t new_turn_time_ms)
{
  if (new_turn_time_ms >= 100U && new_turn_time_ms <= 3000U)
  {
    turn_time_ms = new_turn_time_ms;
  }
}

void UltrasonicAvoid_SetEscapeManeuver(
    UltrasonicAvoidDriveCallback new_reverse_callback,
    int16_t new_reverse_speed,
    uint32_t new_stop_time_ms,
    uint32_t new_reverse_time_ms,
    uint32_t new_guard_time_ms)
{
  reverse_callback = new_reverse_callback;

  if (new_reverse_speed > 0)
  {
    reverse_speed = new_reverse_speed;
  }
  if (new_stop_time_ms <= 1000U)
  {
    stop_time_ms = new_stop_time_ms;
  }
  if (new_reverse_time_ms <= 2000U)
  {
    reverse_time_ms = new_reverse_time_ms;
  }
  if (new_guard_time_ms <= 500U)
  {
    guard_time_ms = new_guard_time_ms;
  }
}

void UltrasonicAvoid_SetNoEchoFallback(uint8_t enable,
                                       uint8_t timeout_count)
{
  no_echo_fallback_enabled = enable != 0U ? 1U : 0U;
  no_echo_timeout_limit = timeout_count == 0U
                        ? AVOID_DEFAULT_NO_ECHO_COUNT : timeout_count;
  no_echo_fallback_active = 0U;
  no_echo_timeout_count = 0U;
}

void UltrasonicAvoid_Task(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint16_t measured_distance_cm = 0U;
  uint8_t measurement_result;
  uint8_t accepted_measurement = 0U;
  uint8_t transient_timeout = 0U;

  Ultrasonic_Task();
  UltrasonicMotion_Task(now_ms);

  if (avoid_state == ULTRASONIC_AVOID_STOPPING)
  {
    stop_motors();
    if (now_ms - maneuver_start_ms >= stop_time_ms)
    {
      maneuver_start_ms = now_ms;
      avoid_state = (reverse_callback != NULL && reverse_time_ms > 0U)
                    ? ULTRASONIC_AVOID_BACKING
                    : ULTRASONIC_AVOID_GUARD;
    }
    return;
  }

  if (avoid_state == ULTRASONIC_AVOID_BACKING)
  {
    if (now_ms - maneuver_start_ms < reverse_time_ms)
    {
      reverse_callback(reverse_speed, reverse_speed);
    }
    else
    {
      stop_motors();
      maneuver_start_ms = now_ms;
      avoid_state = ULTRASONIC_AVOID_GUARD;
    }
    return;
  }

  if (avoid_state == ULTRASONIC_AVOID_GUARD)
  {
    stop_motors();
    if (now_ms - maneuver_start_ms >= guard_time_ms)
    {
      maneuver_start_ms = now_ms;
      avoid_state = ULTRASONIC_AVOID_TURNING;
    }
    return;
  }

  if (avoid_state == ULTRASONIC_AVOID_TURNING)
  {
    if (now_ms - maneuver_start_ms < turn_time_ms)
    {
      if (next_turn_direction == ULTRASONIC_AVOID_TURN_LEFT)
      {
        if (turn_left_callback != NULL)
        {
          turn_left_callback(turn_inner_speed, turn_outer_speed);
        }
      }
      else if (turn_right_callback != NULL)
      {
        turn_right_callback(turn_inner_speed, turn_outer_speed);
      }
    }
    else
    {
      stop_motors();
      next_turn_direction = (next_turn_direction == ULTRASONIC_AVOID_TURN_LEFT)
                                ? ULTRASONIC_AVOID_TURN_RIGHT
                                : ULTRASONIC_AVOID_TURN_LEFT;
      cooldown_start_ms = now_ms;
      avoid_state = ULTRASONIC_AVOID_COOLDOWN;
      clear_filter();
    }
    return;
  }

  if (avoid_state == ULTRASONIC_AVOID_COOLDOWN)
  {
    stop_motors();
    if (now_ms - cooldown_start_ms >= 250U)
    {
      avoid_state = ULTRASONIC_AVOID_WAIT_SAFE;
      last_trigger_ms = now_ms - AVOID_MEASURE_INTERVAL_MS;
    }
    return;
  }

  if (!Ultrasonic_IsBusy() &&
      (now_ms - last_trigger_ms >= AVOID_MEASURE_INTERVAL_MS))
  {
    if (Ultrasonic_Start() != 0U)
    {
      last_trigger_ms = now_ms;
    }
  }

  measurement_result = Ultrasonic_GetResult(&measured_distance_cm);
  if (measurement_result == ULTRASONIC_RESULT_OK)
  {
    accept_distance(measured_distance_cm, now_ms);
    UltrasonicMotion_Update(Ultrasonic_GetLastDistanceMm(), now_ms);
    accepted_measurement = 1U;
    no_echo_timeout_count = 0U;
    no_echo_fallback_active = 0U;

    if ((avoid_state == ULTRASONIC_AVOID_WAIT_SAFE ||
         avoid_state == ULTRASONIC_AVOID_FORWARD) &&
        measured_distance_cm <= emergency_distance_cm)
    {
      if (measured_distance_cm <= AVOID_CRITICAL_RAW_CM)
      {
        emergency_count = AVOID_EMERGENCY_CONFIRM_COUNT;
      }
      else if (emergency_count < AVOID_EMERGENCY_CONFIRM_COUNT)
      {
        ++emergency_count;
      }

      if (emergency_count >= AVOID_EMERGENCY_CONFIRM_COUNT)
      {
        begin_turn(now_ms);
        return;
      }

      /* First fast-path sample is not enough to reverse direction, but it is
         enough to remove cruise speed during the 60 ms confirmation gap. */
      if (avoid_state == ULTRASONIC_AVOID_FORWARD && drive_callback != NULL)
      {
        drive_callback(slow_speed, slow_speed);
      }
      else
      {
        stop_motors();
      }
      return;
    }
    else
    {
      emergency_count = 0U;
    }
  }
  else if (measurement_result == ULTRASONIC_RESULT_TIMEOUT)
  {
    UltrasonicMotion_NoteInvalid(now_ms);
    if (no_echo_timeout_count < no_echo_timeout_limit)
    {
      no_echo_timeout_count++;
    }

    if (no_echo_fallback_enabled != 0U &&
        no_echo_timeout_count >= no_echo_timeout_limit)
    {
      /* HC-SR04 无回波可能是开阔区超出量程，也可能是断线；整体工程
         显式启用后才允许低速降级，收到有效回波会立即重新安全测量。 */
      no_echo_fallback_active = 1U;
      avoid_state = ULTRASONIC_AVOID_FORWARD;
      clear_filter();
      if (drive_callback != NULL)
      {
        drive_callback(slow_speed, slow_speed);
      }
      return;
    }

    if (avoid_state == ULTRASONIC_AVOID_FORWARD &&
        distance_valid != 0U &&
        now_ms - last_valid_measurement_ms <= AVOID_MEASURE_STALE_MS)
    {
      /* 单次漏回波很常见。最近的三点中值仍在有效期内时不急停，
         本周期降为慢速；连续三次超时仍会进入上面的降级分支。 */
      transient_timeout = 1U;
    }
    else
    {
      /* 启动阶段没有可信历史距离时仍先停车，避免直接盲目前进。 */
      stop_motors();
      avoid_state = ULTRASONIC_AVOID_WAIT_SAFE;
      clear_filter();
      return;
    }
  }
  else if (measurement_result == ULTRASONIC_RESULT_OUT_RANGE)
  {
    UltrasonicMotion_NoteInvalid(now_ms);
    /* 有 ECHO 但脉宽越界，按传感器/接线异常处理，不进入开阔区降级。 */
    no_echo_timeout_count = 0U;
    no_echo_fallback_active = 0U;
    stop_motors();
    avoid_state = ULTRASONIC_AVOID_WAIT_SAFE;
    clear_filter();
    return;
  }

  if (no_echo_fallback_active != 0U)
  {
    /* 降级期间没有新回波时保持低速；有效回波会在上面的分支清除
       降级标志，随后重新要求连续三次有效测量。 */
    if (drive_callback != NULL)
    {
      drive_callback(slow_speed, slow_speed);
    }
    return;
  }

  if (transient_timeout != 0U)
  {
    if (drive_callback != NULL)
    {
      drive_callback(slow_speed, slow_speed);
    }
    return;
  }

  if (avoid_state == ULTRASONIC_AVOID_WAIT_SAFE)
  {
    stop_motors();
    if (!distance_valid)
    {
      return;
    }

    /* 开机正对障碍，或一次转向后仍未离开墙面时，不能永远停在
       WAIT_SAFE；用新的有效测量再次确认并继续脱困。 */
    if (filtered_distance_cm <= stop_distance_cm)
    {
      if (accepted_measurement != 0U && obstacle_count < AVOID_CONFIRM_COUNT)
      {
        obstacle_count++;
      }
      if (accepted_measurement != 0U && obstacle_count >= AVOID_CONFIRM_COUNT)
      {
        begin_turn(now_ms);
      }
      return;
    }

    obstacle_count = 0U;
    avoid_state = ULTRASONIC_AVOID_FORWARD;
    if (filtered_distance_cm < clear_distance_cm)
    {
      if (drive_callback != NULL)
      {
        drive_callback(slow_speed, slow_speed);
      }
    }
    else if (drive_callback != NULL)
    {
      drive_callback(cruise_speed, cruise_speed);
    }
    return;
  }

  if (avoid_state != ULTRASONIC_AVOID_FORWARD)
  {
    stop_motors();
    return;
  }

  if (!distance_valid ||
      (now_ms - last_valid_measurement_ms > AVOID_MEASURE_STALE_MS))
  {
    stop_motors();
    avoid_state = ULTRASONIC_AVOID_WAIT_SAFE;
    clear_filter();
    return;
  }

  if (filtered_distance_cm <= stop_distance_cm)
  {
    /* 确认次数按“有效测量”计数，而不是按主循环次数计数，
       防止同一个旧距离在 1～2 ms 内被重复算作两次。 */
    if (accepted_measurement != 0U && obstacle_count < AVOID_CONFIRM_COUNT)
    {
      obstacle_count++;
    }

    if (accepted_measurement != 0U && obstacle_count >= AVOID_CONFIRM_COUNT)
    {
      begin_turn(now_ms);
      return;
    }
  }
  else if (accepted_measurement != 0U)
  {
    obstacle_count = 0U;
  }

  if (filtered_distance_cm < clear_distance_cm)
  {
    if (drive_callback != NULL)
    {
      drive_callback(slow_speed, slow_speed);
    }
  }
  else if (drive_callback != NULL)
  {
    drive_callback(cruise_speed, cruise_speed);
  }
}

UltrasonicAvoidState UltrasonicAvoid_GetState(void)
{
  return avoid_state;
}

uint16_t UltrasonicAvoid_GetLastDistanceCm(void)
{
  return filtered_distance_cm;
}

uint16_t UltrasonicAvoid_GetEmergencyDistanceCm(void)
{
  return emergency_distance_cm;
}

uint8_t UltrasonicAvoid_IsNoEchoFallbackActive(void)
{
  return no_echo_fallback_active;
}
