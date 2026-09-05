/* Real line controller with mocked HAL/DriveBase. These tests validate
   sensor histories and motor ownership, not wheel traction or physical yaw. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "line_tracking.h"
#include "line_search_model.h"
#include "drive_base.h"

static uint32_t tick;
static DriveBaseTelemetry telemetry;
static LineTrackingCommand output;
static unsigned speed_commands, brake_commands;
static uint32_t brake_started;
static uint8_t fault_on_task;

uint32_t HAL_GetTick(void) { return tick; }
int HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t n)
{ (void)p; (void)n; return 1; }
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g)
{ (void)p; (void)g; }
int32_t DriveBase_EquivalentCpsFromPwm(int16_t p) { return p; }
void DriveBase_GetTelemetry(DriveBaseTelemetry *t) { *t = telemetry; }
void DriveBase_Task(uint32_t now)
{
  if (fault_on_task)
  {
    telemetry.fault_mask = DRIVE_FAULT_MOTOR1;
    telemetry.mode = DRIVE_BASE_FAULT;
  }
  else if (telemetry.mode == DRIVE_BASE_BRAKING && now - brake_started >= 52U)
    telemetry.mode = DRIVE_BASE_STOPPED;
}
uint8_t DriveBase_GetFaultMask(void) { return telemetry.fault_mask; }
void DriveBase_Stop(DriveStopMode mode)
{
  if (mode == DRIVE_STOP_BRAKE)
  {
    ++brake_commands;
    brake_started = tick;
    telemetry.mode = DRIVE_BASE_BRAKING;
  }
  else telemetry.mode = DRIVE_BASE_STOPPED;
  memset(telemetry.requested_cps, 0, sizeof telemetry.requested_cps);
}
void DriveBase_SetWheelCps(int32_t m1, int32_t m2, int32_t m3, int32_t m4)
{
  /* A new speed command during active braking would violate ownership even
     though the production DriveBase rejects it. Make that visible here. */
  assert(telemetry.mode != DRIVE_BASE_BRAKING && telemetry.fault_mask == 0);
  ++speed_commands;
  telemetry.mode = DRIVE_BASE_SPEED;
  telemetry.requested_cps[0] = m1;
  telemetry.requested_cps[1] = m2;
  telemetry.requested_cps[2] = m3;
  telemetry.requested_cps[3] = m4;
}
void DriveBase_SetSideCps(int32_t left, int32_t right)
{
  DriveBase_SetWheelCps(left, left, right, right);
}
static LineTrackingAction sample(unsigned mask, uint32_t dt)
{
  LineTrackingReading r = {mask & 1, (mask >> 1) & 1,
                          (mask >> 2) & 1, (mask >> 3) & 1};
  LineTrackingAction action;
  tick += dt;
  action = line_tracking_compute(&r, 3000, &output);
  /* Production caller applies valid commands after each compute. */
  if (output.valid)
  {
    if (output.left_cps == 0 && output.right_cps == 0)
      DriveBase_Stop(DRIVE_STOP_COAST);
    else DriveBase_SetSideCps(output.left_cps, output.right_cps);
  }
  return action;
}
static void reset(uint8_t forward, uint8_t smooth)
{
  line_tracking_reset();
  memset(&telemetry, 0, sizeof telemetry);
  speed_commands = brake_commands = 0;
  fault_on_task = 0;
  line_tracking_set_no_line_forward(forward);
  line_tracking_set_smooth_mode(smooth);
}
static void hint(unsigned mask) { sample(mask, 1); sample(mask, 25); }
static void assert_search(int side)
{
  int32_t magnitude = LINE_SEARCH_TARGET_CPS;
  assert(!output.valid && telemetry.mode == DRIVE_BASE_SPEED);
  assert(output.action == (side < 0 ? LINE_ACTION_SEARCH_LEFT :
                                      LINE_ACTION_SEARCH_RIGHT));
  assert(telemetry.requested_cps[1] ==
         (side < 0 ? -magnitude : magnitude));
  assert(telemetry.requested_cps[3] ==
         (side < 0 ? magnitude : -magnitude));
  assert(telemetry.requested_cps[0] == -telemetry.requested_cps[2]);
  assert(telemetry.requested_cps[1] == -telemetry.requested_cps[3]);
  assert(telemetry.requested_cps[0] == (side < 0 ? -magnitude : magnitude));
  assert(telemetry.requested_cps[0] == telemetry.requested_cps[1]);
  assert(telemetry.requested_cps[2] == telemetry.requested_cps[3]);
  assert(side < 0 ? telemetry.requested_cps[0] < 0 :
                   telemetry.requested_cps[0] > 0);
}
static void start_search(int side)
{
  assert(sample(0, 1) == LINE_ACTION_STOP);
  sample(0, 70);
  assert_search(side);
}
static void finish_reverse(int side)
{
  unsigned before = speed_commands;
  assert(!output.valid && telemetry.mode == DRIVE_BASE_BRAKING);
  sample(0, 30);
  assert(telemetry.mode == DRIVE_BASE_BRAKING && speed_commands == before);
  sample(0, 22);
  assert(output.valid && telemetry.mode == DRIVE_BASE_STOPPED);
  sample(0, 69); assert(speed_commands == before);
  sample(0, 1); assert_search(side);
}
static void capture(void)
{
  sample(5, 1); sample(5, 12);
  assert(!output.valid && telemetry.mode == DRIVE_BASE_BRAKING);
  sample(5, 30); assert(!output.valid);
  sample(5, 22); sample(5, 1);
  assert(output.valid && output.left_cps > 0 &&
         output.left_cps == output.right_cps);
  assert(telemetry.requested_cps[0] == telemetry.requested_cps[1]);
  assert(telemetry.requested_cps[2] == telemetry.requested_cps[3]);
}
int main(void)
{
  unsigned smooth, forward;
  /* Independent numeric references for the measured 129/129/47 mm chassis
     and the existing rounded effective track of 338 mm. */
  assert(VEHICLE_TRACK_WIDTH_MM == 129 && VEHICLE_WHEELBASE_MM == 129 &&
         VEHICLE_WHEEL_DIAMETER_MM == 47);
  assert(LINE_SEARCH_EFFECTIVE_TRACK_MM == 338);
#if LINE_SEARCH_NOMINAL_YAW_MDEG_S == 120000L
  const uint32_t probe_ms = 362U, first_ms = 1300U;
  assert(LINE_SEARCH_TARGET_CPS == 2493);
  assert(LINE_SEARCH_LEG_MS(2400U) == 3466U);
#elif LINE_SEARCH_NOMINAL_YAW_MDEG_S == 90000L
  const uint32_t probe_ms = 482U, first_ms = 1733U;
  assert(LINE_SEARCH_TARGET_CPS == 1870);
  assert(LINE_SEARCH_LEG_MS(2400U) == 4621U);
#else
#error "Add an independent numeric reference for a new test profile"
#endif
  assert(LINE_SEARCH_LEG_MS(250U) == probe_ms);
  assert(LINE_SEARCH_LEG_MS(900U) == first_ms);
  for (smooth = 0; smooth <= 1; ++smooth)
  for (forward = 0; forward <= 1; ++forward)
  {
    /* Most recent stable side wins over earlier departure history. */
    reset((uint8_t)forward, (uint8_t)smooth);
    sample(5, 1);
    assert(telemetry.requested_cps[0] == telemetry.requested_cps[1]);
    assert(telemetry.requested_cps[2] == telemetry.requested_cps[3]);
    /* On-line sharp turns still use the original equal-axle commands. */
    sample(2, 1); sample(2, 130);
    assert(output.valid && output.action == LINE_ACTION_LEFT_SHARP);
    assert(telemetry.requested_cps[0] == telemetry.requested_cps[1]);
    assert(telemetry.requested_cps[2] == telemetry.requested_cps[3]);
    reset((uint8_t)forward, (uint8_t)smooth);
    sample(5, 1); hint(1); hint(8); start_search(1);
    reset((uint8_t)forward, (uint8_t)smooth);
    sample(5, 1); hint(4); hint(2); start_search(-1);

    /* No reliable direction must produce exploration, not latched STOP. */
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(4); sample(5, 1); sample(5, 81); start_search(-1);
    sample(0, 250); assert_search(-1);
    sample(0, probe_ms - 250); finish_reverse(1);
    sample(0, 2U * probe_ms); finish_reverse(-1);
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(4); sample(15, 1); start_search(-1);
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(4); sample(2, 1); start_search(-1);
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(4); sample(0, 201); sample(0, 70); assert_search(-1);

    /* Search can continue beyond both old limits when sensors guide it.
       Wheel counts are intentionally irrelevant to recovery termination. */
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); start_search(-1);
    telemetry.position_moved_counts[0] = -100000;
    telemetry.position_moved_counts[2] = 100000;
    sample(2, 3000); assert_search(-1);
    capture();
    sample(5, 250); assert(output.left_cps > 0);
    sample(5, 5000); assert(output.left_cps > 0);

    /* A confirmed opposite outer hit corrects the initial prediction. */
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); start_search(-1);
    sample(8, 1); sample(8, 19); assert_search(-1);
    sample(8, 1); finish_reverse(1);
    capture();

    /* An empty leg reverses and widens, rather than ending in STOP. */
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); start_search(-1);
    sample(0, 900); assert_search(-1);
    sample(0, first_ms - 900); finish_reverse(1);
    sample(0, first_ms); assert_search(1);
    sample(0, first_ms); finish_reverse(-1);

    /* One-frame centre chatter cannot capture while stationary either. */
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); sample(0, 1); sample(5, 10); sample(0, 60);
    assert_search(-1);
    sample(5, 1); sample(0, 1); assert_search(-1);

    /* Failed capture keeps the episode watchdog, but has no angle budget. */
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); start_search(-1); capture();
    sample(0, 1); sample(0, 70); assert_search(-1);
    sample(2, 7800);
    assert(output.valid && output.left_cps == 0);
    sample(5, 300); assert(output.left_cps == 0);

    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); start_search(-1); fault_on_task = 1;
    sample(0, 1); assert(output.valid && output.left_cps == 0);
    sample(5, 300); assert(output.left_cps == 0);

    /* User mode reset must stop a directly owned search or brake. */
    reset((uint8_t)forward, (uint8_t)smooth);
    hint(1); start_search(-1);
    line_tracking_reset(); assert(telemetry.mode == DRIVE_BASE_STOPPED);
    sample(5, 1); assert(output.left_cps > 0);
  }
  reset(1, 1); assert(sample(0, 1) == LINE_ACTION_FORWARD);
  reset(0, 1); start_search(-1);
  tick = UINT32_MAX - 15;
  reset(0, 1); hint(1); start_search(-1);
  printf("PASS nominal_yaw=%ld cps=%ld: geometry conversion, symmetric wheel targets, on-line isolation, direction, reversals, capture, watchdog, faults, reset, tick rollover\n",
         (long)LINE_SEARCH_NOMINAL_YAW_MDEG_S, (long)LINE_SEARCH_TARGET_CPS);
  return 0;
}
