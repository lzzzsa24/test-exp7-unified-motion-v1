#include "encoder_straight.h"

#include "drive_base.h"
#include "main.h"
#include "vehicle_geometry.h"

#define STRAIGHT_COUNTS_PER_WHEEL_REV       1040L
#define STRAIGHT_PI_X10000                  31416L
#define STRAIGHT_TIMEOUT_MULTIPLIER              3U
#define STRAIGHT_MIN_TIMEOUT_MS               2500U
#define STRAIGHT_POSITION_TOLERANCE_COUNTS      14U

static EncoderStraightState straight_state;
static uint8_t fault_mask;

static uint8_t module_fault_mask(uint8_t drive_fault)
{
  uint8_t motors = drive_fault & 0x0FU;

  return motors != 0U ? motors : 0x10U;
}

void EncoderStraight_Init(void)
{
  straight_state = ENCODER_STRAIGHT_IDLE;
  fault_mask = 0U;
}

uint8_t EncoderStraight_Start(uint32_t distance_mm, int32_t requested_cps)
{
  DrivePositionCommand command = {0};
  int64_t numerator;
  int64_t denominator;
  int32_t target_counts;
  int32_t cruise_cps;
  uint32_t expected_ms;
  uint32_t timeout_ms;
  uint8_t motor;

  if (distance_mm == 0U || requested_cps <= 0L) return 0U;
  numerator = (int64_t)distance_mm *
              STRAIGHT_COUNTS_PER_WHEEL_REV * 10000LL;
  denominator = (int64_t)STRAIGHT_PI_X10000 *
                VEHICLE_WHEEL_DIAMETER_MM;
  target_counts = (int32_t)((numerator + denominator / 2LL) /
                            denominator);
  if (target_counts <= 0L) return 0U;

  cruise_cps = requested_cps;
  expected_ms = (uint32_t)(((int64_t)target_counts * 1000LL) /
                           cruise_cps);
  timeout_ms = expected_ms * STRAIGHT_TIMEOUT_MULTIPLIER + 1500U;
  if (timeout_ms < STRAIGHT_MIN_TIMEOUT_MS)
    timeout_ms = STRAIGHT_MIN_TIMEOUT_MS;

  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    command.delta_counts[motor] = target_counts;
    command.maximum_cps[motor] = cruise_cps;
  }
  command.timeout_ms = timeout_ms;
  command.tolerance_counts = STRAIGHT_POSITION_TOLERANCE_COUNTS;
  command.completion_stop_mode = DRIVE_STOP_BRAKE;
  fault_mask = 0U;
  if (DriveBase_StartPositionMove(&command) == 0U) return 0U;
  straight_state = ENCODER_STRAIGHT_RUNNING;
  return 1U;
}

void EncoderStraight_Task(void)
{
  DrivePositionState state;

  if (straight_state != ENCODER_STRAIGHT_RUNNING &&
      straight_state != ENCODER_STRAIGHT_SETTLE)
  {
    return;
  }
  DriveBase_Task(HAL_GetTick());
  state = DriveBase_GetPositionState();
  if (state == DRIVE_POSITION_SETTLING)
  {
    straight_state = ENCODER_STRAIGHT_SETTLE;
  }
  else if (state == DRIVE_POSITION_DONE)
  {
    straight_state = ENCODER_STRAIGHT_DONE;
  }
  else if (state == DRIVE_POSITION_FAULT)
  {
    fault_mask = module_fault_mask(DriveBase_GetFaultMask());
    straight_state = ENCODER_STRAIGHT_FAULT;
  }
}

void EncoderStraight_Stop(void)
{
  if (straight_state == ENCODER_STRAIGHT_RUNNING ||
      straight_state == ENCODER_STRAIGHT_SETTLE)
  {
    DriveBase_Stop(DRIVE_STOP_COAST);
  }
  straight_state = ENCODER_STRAIGHT_IDLE;
}

EncoderStraightState EncoderStraight_GetState(void)
{
  return straight_state;
}

uint8_t EncoderStraight_GetFaultMask(void)
{
  return fault_mask;
}
