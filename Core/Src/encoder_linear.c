#include "encoder_linear.h"

#include "drive_base.h"
#include "main.h"
#include "vehicle_geometry.h"

#define LINEAR_COUNTS_PER_WHEEL_REV       1040L
#define LINEAR_PI_X10000                  31416L
#define LINEAR_TIMEOUT_MULTIPLIER              4U
#define LINEAR_MIN_TIMEOUT_MS               2000U
#define LINEAR_POSITION_TOLERANCE_COUNTS      12U

static EncoderLinearState linear_state;
static uint8_t fault_mask;
static int32_t target_counts;
static int32_t progress_counts;

static int32_t abs_i32(int32_t value)
{
  return value < 0L ? -value : value;
}

static uint8_t module_fault_mask(uint8_t drive_fault)
{
  uint8_t motors = drive_fault & 0x0FU;

  return motors != 0U ? motors : 0x10U;
}

void EncoderLinear_Init(void)
{
  linear_state = ENCODER_LINEAR_IDLE;
  fault_mask = 0U;
  target_counts = 0L;
  progress_counts = 0L;
}

uint8_t EncoderLinear_Start(int32_t distance_mm, int32_t requested_cps)
{
  DrivePositionCommand command = {0};
  int64_t numerator;
  int64_t denominator;
  int32_t signed_counts;
  int32_t magnitude_cps;
  uint32_t expected_ms;
  uint32_t timeout_ms;
  uint8_t motor;

  if (distance_mm == 0L || requested_cps <= 0L)
  {
    return 0U;
  }
  numerator = (int64_t)abs_i32(distance_mm) *
              LINEAR_COUNTS_PER_WHEEL_REV * 10000LL;
  denominator = (int64_t)LINEAR_PI_X10000 *
                VEHICLE_WHEEL_DIAMETER_MM;
  target_counts = (int32_t)((numerator + denominator / 2LL) /
                            denominator);
  if (target_counts <= 0L) return 0U;

  signed_counts = distance_mm > 0L ? target_counts : -target_counts;
  magnitude_cps = abs_i32(requested_cps);
  expected_ms = (uint32_t)(((int64_t)target_counts * 1000LL) /
                           magnitude_cps);
  timeout_ms = expected_ms * LINEAR_TIMEOUT_MULTIPLIER + 1200U;
  if (timeout_ms < LINEAR_MIN_TIMEOUT_MS)
    timeout_ms = LINEAR_MIN_TIMEOUT_MS;

  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    command.delta_counts[motor] = signed_counts;
    command.maximum_cps[motor] =
        signed_counts > 0L ? magnitude_cps : -magnitude_cps;
  }
  command.timeout_ms = timeout_ms;
  command.tolerance_counts = LINEAR_POSITION_TOLERANCE_COUNTS;
  command.completion_stop_mode = DRIVE_STOP_COAST;

  fault_mask = 0U;
  progress_counts = 0L;
  if (DriveBase_StartPositionMove(&command) == 0U)
  {
    return 0U;
  }
  linear_state = ENCODER_LINEAR_RUNNING;
  return 1U;
}

void EncoderLinear_Task(void)
{
  DriveBaseTelemetry telemetry;
  DrivePositionState state;
  int64_t sum = 0LL;
  uint8_t motor;

  if (linear_state != ENCODER_LINEAR_RUNNING &&
      linear_state != ENCODER_LINEAR_SETTLE)
  {
    return;
  }
  DriveBase_Task(HAL_GetTick());
  DriveBase_GetTelemetry(&telemetry);
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    sum += abs_i32(telemetry.position_moved_counts[motor]);
  }
  progress_counts = (int32_t)((sum + 2LL) / 4LL);
  if (progress_counts > target_counts) progress_counts = target_counts;

  state = DriveBase_GetPositionState();
  if (state == DRIVE_POSITION_SETTLING)
  {
    linear_state = ENCODER_LINEAR_SETTLE;
  }
  else if (state == DRIVE_POSITION_DONE)
  {
    progress_counts = target_counts;
    linear_state = ENCODER_LINEAR_DONE;
  }
  else if (state == DRIVE_POSITION_FAULT)
  {
    fault_mask = module_fault_mask(DriveBase_GetFaultMask());
    linear_state = ENCODER_LINEAR_FAULT;
  }
}

void EncoderLinear_Stop(void)
{
  if (linear_state == ENCODER_LINEAR_RUNNING ||
      linear_state == ENCODER_LINEAR_SETTLE)
  {
    DriveBase_Stop(DRIVE_STOP_COAST);
  }
  linear_state = ENCODER_LINEAR_IDLE;
}

EncoderLinearState EncoderLinear_GetState(void)
{
  return linear_state;
}

uint8_t EncoderLinear_GetFaultMask(void)
{
  return fault_mask;
}

uint32_t EncoderLinear_GetProgressMm(void)
{
  int64_t numerator;
  int64_t denominator;

  if (progress_counts <= 0L) return 0U;
  numerator = (int64_t)progress_counts *
              LINEAR_PI_X10000 * VEHICLE_WHEEL_DIAMETER_MM;
  denominator = (int64_t)LINEAR_COUNTS_PER_WHEEL_REV * 10000LL;
  return (uint32_t)((numerator + denominator / 2LL) / denominator);
}
