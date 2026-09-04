#include "encoder_turn.h"

#include "drive_base.h"
#include "main.h"
#include "vehicle_geometry.h"

#define TURN_COUNTS_PER_WHEEL_REV       1040L
#define TURN_WHEEL_DIAMETER_MM   VEHICLE_WHEEL_DIAMETER_MM
/* 129 mm physical track width multiplied by the existing ground-calibrated
   skid factor. Encoder counts remain wheel-angle evidence, not chassis yaw. */
#define TURN_SKID_COMP_PERMILLE          2619L
#define TURN_TRACK_WIDTH_MM \
  ((VEHICLE_TRACK_WIDTH_MM * TURN_SKID_COMP_PERMILLE + 500L) / 1000L)
#define TURN_TIMEOUT_MULTIPLIER             3U
#define TURN_LOW_SPEED_TIMEOUT_MULTIPLIER   7U
#define TURN_MIN_TIMEOUT_MS              2500U
#define TURN_LOW_SPEED_MIN_TIMEOUT_MS     4000U
#define TURN_CONTINUOUS_MIN_CPS           1412L
#define TURN_POSITION_TOLERANCE_COUNTS      14U

static EncoderTurnState turn_state;
static uint8_t fault_mask;
static int32_t target_counts[DRIVE_BASE_WHEEL_COUNT];
static int32_t requested_angle_mdeg;
static int32_t achieved_angle_mdeg;

static int32_t abs_i32(int32_t value)
{
  return value < 0L ? -value : value;
}

static int32_t sign_i32(int32_t value)
{
  return value > 0L ? 1L : (value < 0L ? -1L : 0L);
}

static uint8_t module_fault_mask(uint8_t drive_fault)
{
  uint8_t motors = drive_fault & 0x0FU;

  return motors != 0U ? motors : 0x10U;
}

static int32_t path_counts(int32_t angle_mdeg, int32_t radius_times_two)
{
  int64_t numerator = (int64_t)abs_i32(angle_mdeg) *
                      (int64_t)abs_i32(radius_times_two) *
                      (int64_t)TURN_COUNTS_PER_WHEEL_REV;
  int64_t denominator = 360000LL * (int64_t)TURN_WHEEL_DIAMETER_MM;

  return (int32_t)((numerator + denominator / 2LL) / denominator);
}

static uint8_t start_turn(int32_t angle_mdeg,
                          int32_t center_radius_mm,
                          int32_t maximum_wheel_cps,
                          uint8_t rear_pivot)
{
  DrivePositionCommand command = {0};
  int32_t left_radius_x2;
  int32_t right_radius_x2;
  int32_t maximum_radius_x2;
  int32_t left_count_magnitude;
  int32_t right_count_magnitude;
  int32_t left_cps;
  int32_t right_cps;
  int32_t angle_sign;
  int32_t maximum_counts;
  uint32_t expected_ms;
  uint32_t timeout_ms;

  if (angle_mdeg == 0L || maximum_wheel_cps <= 0L) return 0U;
  left_radius_x2 = 2L * center_radius_mm - TURN_TRACK_WIDTH_MM;
  right_radius_x2 = 2L * center_radius_mm + TURN_TRACK_WIDTH_MM;
  maximum_radius_x2 = abs_i32(left_radius_x2) > abs_i32(right_radius_x2) ?
      abs_i32(left_radius_x2) : abs_i32(right_radius_x2);
  if (maximum_radius_x2 == 0L) return 0U;

  left_count_magnitude = path_counts(angle_mdeg, left_radius_x2);
  right_count_magnitude = path_counts(angle_mdeg, right_radius_x2);
  angle_sign = sign_i32(angle_mdeg);
  left_cps = (maximum_wheel_cps * abs_i32(left_radius_x2)) /
             maximum_radius_x2;
  right_cps = (maximum_wheel_cps * abs_i32(right_radius_x2)) /
              maximum_radius_x2;
  left_cps *= angle_sign * sign_i32(left_radius_x2);
  right_cps *= angle_sign * sign_i32(right_radius_x2);

  /* Physical wheel order confirmed on this car:
     M1=front-left, M2=rear-left, M3=front-right, M4=rear-right.  A normal
     turn drives both axles.  Rear-pivot search deliberately leaves M2/M4 at
     zero so the tail is not the powered end of the sweep. */
  target_counts[0] = sign_i32(left_cps) * left_count_magnitude;
  target_counts[1] = rear_pivot == 0U
                   ? sign_i32(left_cps) * left_count_magnitude : 0L;
  target_counts[2] = sign_i32(right_cps) * right_count_magnitude;
  target_counts[3] = rear_pivot == 0U
                   ? sign_i32(right_cps) * right_count_magnitude : 0L;
  command.delta_counts[0] = target_counts[0];
  command.delta_counts[1] = target_counts[1];
  command.delta_counts[2] = target_counts[2];
  command.delta_counts[3] = target_counts[3];
  command.maximum_cps[0] = left_cps;
  command.maximum_cps[1] = rear_pivot == 0U ? left_cps : 0L;
  command.maximum_cps[2] = right_cps;
  command.maximum_cps[3] = rear_pivot == 0U ? right_cps : 0L;

  maximum_counts = left_count_magnitude > right_count_magnitude ?
      left_count_magnitude : right_count_magnitude;
  expected_ms = (uint32_t)(((int64_t)maximum_counts * 1000LL) /
                           maximum_wheel_cps);
  if (maximum_wheel_cps < TURN_CONTINUOUS_MIN_CPS)
  {
    timeout_ms = expected_ms * TURN_LOW_SPEED_TIMEOUT_MULTIPLIER + 2000U;
    if (timeout_ms < TURN_LOW_SPEED_MIN_TIMEOUT_MS)
      timeout_ms = TURN_LOW_SPEED_MIN_TIMEOUT_MS;
  }
  else
  {
    timeout_ms = expected_ms * TURN_TIMEOUT_MULTIPLIER + 1500U;
    if (timeout_ms < TURN_MIN_TIMEOUT_MS)
      timeout_ms = TURN_MIN_TIMEOUT_MS;
  }
  command.timeout_ms = timeout_ms;
  command.tolerance_counts = TURN_POSITION_TOLERANCE_COUNTS;
  command.completion_stop_mode = DRIVE_STOP_BRAKE;

  requested_angle_mdeg = angle_mdeg;
  achieved_angle_mdeg = 0L;
  fault_mask = 0U;
  if (DriveBase_StartPositionMove(&command) == 0U) return 0U;
  turn_state = ENCODER_TURN_RUNNING;
  return 1U;
}

static void update_achieved_angle(void)
{
  DriveBaseTelemetry telemetry;
  int64_t accumulated_mdeg = 0LL;
  int32_t requested_magnitude = abs_i32(requested_angle_mdeg);
  uint8_t active_motors = 0U;
  uint8_t motor;

  DriveBase_GetTelemetry(&telemetry);
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    if (target_counts[motor] != 0L)
    {
      int32_t moved = telemetry.position_moved_counts[motor];
      int32_t along_path = moved * sign_i32(target_counts[motor]);
      int32_t target_magnitude = abs_i32(target_counts[motor]);

      accumulated_mdeg +=
          ((int64_t)requested_magnitude * along_path +
           target_magnitude / 2L) / target_magnitude;
      ++active_motors;
    }
  }
  if (active_motors == 0U)
  {
    achieved_angle_mdeg = 0L;
  }
  else
  {
    int32_t magnitude = (int32_t)(accumulated_mdeg / active_motors);
    if (magnitude < 0L) magnitude = 0L;
    achieved_angle_mdeg = sign_i32(requested_angle_mdeg) * magnitude;
  }
}

void EncoderTurn_Init(void)
{
  uint8_t motor;

  turn_state = ENCODER_TURN_IDLE;
  fault_mask = 0U;
  requested_angle_mdeg = 0L;
  achieved_angle_mdeg = 0L;
  for (motor = 0U; motor < DRIVE_BASE_WHEEL_COUNT; ++motor)
  {
    target_counts[motor] = 0L;
  }
}

uint8_t EncoderTurn_Start(int32_t angle_mdeg,
                          int32_t center_radius_mm,
                          int32_t maximum_wheel_cps)
{
  return start_turn(angle_mdeg, center_radius_mm,
                    maximum_wheel_cps, 0U);
}

uint8_t EncoderTurn_StartRearPivot(int32_t angle_mdeg,
                                   int32_t maximum_front_wheel_cps)
{
  return start_turn(angle_mdeg, 0L,
                    maximum_front_wheel_cps, 1U);
}

void EncoderTurn_Task(void)
{
  DrivePositionState state;

  if (turn_state != ENCODER_TURN_RUNNING) return;
  DriveBase_Task(HAL_GetTick());
  state = DriveBase_GetPositionState();
  if (state == DRIVE_POSITION_DONE)
  {
    update_achieved_angle();
    turn_state = ENCODER_TURN_DONE;
  }
  else if (state == DRIVE_POSITION_FAULT)
  {
    update_achieved_angle();
    fault_mask = module_fault_mask(DriveBase_GetFaultMask());
    turn_state = ENCODER_TURN_FAULT;
  }
}

uint8_t EncoderTurn_RequestStop(void)
{
  if (turn_state != ENCODER_TURN_RUNNING) return 0U;
  return DriveBase_RequestPositionStop(DRIVE_STOP_BRAKE);
}

void EncoderTurn_Stop(void)
{
  if (turn_state == ENCODER_TURN_RUNNING)
  {
    DriveBase_Stop(DRIVE_STOP_COAST);
  }
  turn_state = ENCODER_TURN_IDLE;
}

EncoderTurnState EncoderTurn_GetState(void)
{
  return turn_state;
}

uint8_t EncoderTurn_GetFaultMask(void)
{
  return fault_mask;
}

int32_t EncoderTurn_GetAchievedAngleMdeg(void)
{
  return achieved_angle_mdeg;
}
