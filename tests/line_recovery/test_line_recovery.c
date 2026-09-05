/* Host regression: real line_tracking.c and encoder_turn.c, mocked HAL and
   DriveBase. Sensor histories and ownership are tested; no physical yaw or
   wheel-speed-loop validation is implied. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "line_tracking.h"
#include "encoder_turn.h"
#include "drive_base.h"
#include "wheel_encoder.h"

static uint32_t tick;
static DriveBaseTelemetry telemetry;
static DrivePositionCommand move;
static unsigned starts;
static int reject_start;
static LineTrackingCommand output;
static WheelEncoderCounts counts, start_counts;
void WheelEncoder_GetCounts(WheelEncoderCounts *c) { *c = counts; }

uint32_t HAL_GetTick(void) { return tick; }
int HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t n)
{ (void)p; (void)n; return 1; }
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g)
{ (void)p; (void)g; }
int32_t DriveBase_EquivalentCpsFromPwm(int16_t p) { return p; }
void DriveBase_GetTelemetry(DriveBaseTelemetry *t) { *t = telemetry; }
void DriveBase_Task(uint32_t now) { (void)now; }
DrivePositionState DriveBase_GetPositionState(void)
{ return telemetry.position_state; }
uint8_t DriveBase_GetFaultMask(void) { return telemetry.fault_mask; }
void DriveBase_Stop(DriveStopMode mode)
{
  (void)mode;
  telemetry.mode = DRIVE_BASE_STOPPED;
  telemetry.position_state = DRIVE_POSITION_IDLE;
}
uint8_t DriveBase_StartPositionMove(const DrivePositionCommand *c)
{
  unsigned i;
  if (reject_start) return 0;
  move = *c;
  start_counts = counts;
  ++starts;
  telemetry.mode = DRIVE_BASE_POSITION;
  telemetry.position_state = DRIVE_POSITION_RUNNING;
  for (i = 0; i < 4; ++i)
  {
    telemetry.position_target_counts[i] = c->delta_counts[i];
    telemetry.position_moved_counts[i] = 0;
  }
  return 1;
}
uint8_t DriveBase_RequestPositionStop(DriveStopMode mode)
{
  (void)mode;
  telemetry.mode = DRIVE_BASE_BRAKING;
  telemetry.position_state = DRIVE_POSITION_SETTLING;
  return 1;
}
static LineTrackingAction sample(unsigned mask, uint32_t dt)
{
  LineTrackingReading r = {mask & 1, (mask >> 1) & 1,
                          (mask >> 2) & 1, (mask >> 3) & 1};
  tick += dt;
  return line_tracking_compute(&r, 3000, &output);
}
static void reset(uint8_t forward, uint8_t smooth)
{
  line_tracking_reset();
  EncoderTurn_Init();
  memset(&telemetry, 0, sizeof telemetry);
  memset(&counts, 0, sizeof counts);
  starts = 0;
  reject_start = 0;
  line_tracking_set_no_line_forward(forward);
  line_tracking_set_smooth_mode(smooth);
}
static void hint(unsigned mask)
{
  sample(mask, 1);
  sample(mask, 25);
}
static void start_search(int side)
{
  assert(sample(0, 1) == LINE_ACTION_STOP);
  sample(0, 70);
  assert(starts == 1 && !output.valid);
  assert(sample(0, 1) == (side < 0 ? LINE_ACTION_SEARCH_LEFT :
                                    LINE_ACTION_SEARCH_RIGHT));
  assert(move.delta_counts[0] == move.delta_counts[1]);
  assert(move.delta_counts[2] == move.delta_counts[3]);
  assert(move.delta_counts[0] == -move.delta_counts[2]);
  assert(side < 0 ? move.delta_counts[0] < 0 : move.delta_counts[0] > 0);
  assert(move.maximum_cps[0] == move.maximum_cps[1]);
  assert(move.maximum_cps[2] == move.maximum_cps[3]);
}
static void progress(unsigned percent)
{
  unsigned i;
  for (i = 0; i < 4; ++i)
    telemetry.position_moved_counts[i] =
        move.delta_counts[i] * (int32_t)percent / 100;
  counts.motor1 = start_counts.motor1 + telemetry.position_moved_counts[0];
  counts.motor2 = start_counts.motor2 + telemetry.position_moved_counts[1];
  counts.motor3 = start_counts.motor3 + telemetry.position_moved_counts[2];
  counts.motor4 = start_counts.motor4 + telemetry.position_moved_counts[3];
}
int main(void)
{
  unsigned smooth, forward;
  int32_t first_target;
  for (smooth = 0; smooth <= 1; ++smooth)
  for (forward = 0; forward <= 1; ++forward)
  {
    reset((uint8_t)forward, (uint8_t)smooth);
    sample(5, 1); hint(1); hint(8); start_search(1);
    reset((uint8_t)forward, (uint8_t)smooth);
    sample(5, 1); hint(4); hint(2); start_search(-1);

    /* Centred travel and intersections must clear a previous bend. */
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); sample(5, 1); sample(5, 81); sample(0, 1); sample(0, 100);
    assert(starts == 0 && output.valid && output.left_cps == 0);
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); sample(15, 1); sample(0, 1); sample(0, 100);
    assert(starts == 0);

    /* A fleeting opposite hit must not reuse the old direction. */
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); sample(8, 1); sample(0, 1); sample(0, 100);
    assert(starts == 0);
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); sample(0, 201); sample(0, 100); assert(starts == 0);

    /* Outer-only detection cannot take the motors away from search. */
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); start_search(-1);
    assert(sample(2, 10) == LINE_ACTION_SEARCH_LEFT && !output.valid);
    first_target = move.delta_counts[2];
    progress(40);
    sample(5, 1); sample(5, 12);
    assert(!output.valid && telemetry.mode == DRIVE_BASE_BRAKING);
    /* Braking travel is charged even while position telemetry is stale. */
    progress(55);
    memset(telemetry.position_moved_counts, 0,
           sizeof telemetry.position_moved_counts);
    telemetry.mode = DRIVE_BASE_STOPPED;
    sample(5, 1); sample(5, 1);
    assert(output.valid && output.left_cps > 0 &&
           output.left_cps == output.right_cps);
    /* Losing the capture retains the remaining angle and episode clock. */
    sample(0, 1); sample(0, 70);
    assert(starts == 2 && move.delta_counts[2] < first_target / 2);
    progress(101);
    assert(sample(0, 1) == LINE_ACTION_STOP && output.valid);
    sample(5, 1); sample(5, 300); assert(output.left_cps == 0 && starts == 2);

    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); start_search(-1);
    assert(sample(5, 2500) == LINE_ACTION_STOP && output.valid);
    sample(5, 300); assert(output.left_cps == 0);

    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); start_search(-1);
    telemetry.position_state = DRIVE_POSITION_FAULT;
    telemetry.fault_mask = DRIVE_FAULT_MOTOR1;
    assert(sample(0, 1) == LINE_ACTION_STOP && output.valid);
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); reject_start = 1; sample(0, 1); sample(0, 70);
    assert(starts == 0 && output.valid && output.left_cps == 0);
  }
  reset(1, 1); assert(sample(0, 1) == LINE_ACTION_FORWARD);
  reset(0, 1); assert(sample(0, 1) == LINE_ACTION_STOP);
  /* HAL millisecond rollover still permits a fresh stable hint. */
  tick = UINT32_MAX - 15;
  reset(0, 1); hint(1); start_search(-1);
  puts("PASS: direction history, four-wheel targets, capture ownership, cumulative budget, timeout, faults, mode reset, tick rollover");
  return 0;
}
