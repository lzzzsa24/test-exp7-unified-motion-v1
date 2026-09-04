#include "line_tracking_lift_test.h"

#if defined(LINE_TRACKING_LIFT_TEST)

#include "diagnostic_uart.h"
#include "drive_base.h"
#include "line_tracking.h"
#include "main.h"
#include "wheel_encoder.h"

#define LIFT_TEST_BASE_PWM               3000
#define LIFT_TEST_CENTER_MS               250U
#define LIFT_TEST_DIRECTION_HINT_MS       120U
#define LIFT_TEST_LOST_SEARCH_MS          350U
#define LIFT_TEST_OUTER_ONLY_MS           300U
#define LIFT_TEST_CENTER_CAPTURE_MS       450U
#define LIFT_TEST_FINAL_CENTER_MS         350U
#define LIFT_TEST_MAX_CAPTURE_PAUSE_MS     120U

enum
{
  LIFT_TEST_FAIL_INITIAL_FORWARD = 0x01U,
  LIFT_TEST_FAIL_SEARCH_NOT_STARTED = 0x02U,
  LIFT_TEST_FAIL_OUTER_RELEASED = 0x04U,
  LIFT_TEST_FAIL_OUTER_NOT_SEARCHING = 0x08U,
  LIFT_TEST_FAIL_ENCODER_DIRECTION = 0x10U,
  LIFT_TEST_FAIL_CENTER_NOT_CAPTURED = 0x20U,
  LIFT_TEST_FAIL_DRIVE_BASE = 0x40U,
  LIFT_TEST_FAIL_CAPTURE_PAUSE = 0x80U
};

typedef struct
{
  uint32_t search_samples;
  uint32_t valid_samples;
  uint32_t forward_samples;
  uint32_t wrong_samples;
  uint32_t first_forward_ms;
  uint8_t first_forward_seen;
} LiftPhaseStats;

static LineTrackingReading reading_from_mask(uint8_t mask)
{
  LineTrackingReading reading;

  reading.x1_black = (mask & 0x01U) != 0U ? 1U : 0U;
  reading.x2_black = (mask & 0x02U) != 0U ? 1U : 0U;
  reading.x3_black = (mask & 0x04U) != 0U ? 1U : 0U;
  reading.x4_black = (mask & 0x08U) != 0U ? 1U : 0U;
  return reading;
}

static int32_t abs_i32(int32_t value)
{
  return value < 0L ? -value : value;
}

static void apply_command(const LineTrackingCommand *command)
{
  if (command == 0 || command->valid == 0U)
  {
    return;
  }
  if (command->left_cps == 0L && command->right_cps == 0L)
  {
    DriveBase_Stop(DRIVE_STOP_COAST);
  }
  else
  {
    DriveBase_SetSideCps(command->left_cps, command->right_cps);
  }
}

static void run_phase(uint8_t mask,
                      uint32_t duration_ms,
                      uint8_t require_search_ownership,
                      LiftPhaseStats *stats)
{
  LineTrackingReading reading = reading_from_mask(mask);
  uint32_t started = HAL_GetTick();

  while (HAL_GetTick() - started < duration_ms)
  {
    LineTrackingCommand command;
    LineTrackingAction action;

    DriveBase_Task(HAL_GetTick());
    action = line_tracking_compute(&reading, LIFT_TEST_BASE_PWM, &command);
    apply_command(&command);

    if (action == LINE_ACTION_SEARCH_LEFT ||
        action == LINE_ACTION_SEARCH_RIGHT)
    {
      ++stats->search_samples;
    }
    if (command.valid != 0U)
    {
      ++stats->valid_samples;
      if (command.left_cps > 0L && command.right_cps > 0L)
      {
        ++stats->forward_samples;
        if (stats->first_forward_seen == 0U)
        {
          stats->first_forward_seen = 1U;
          stats->first_forward_ms = HAL_GetTick() - started;
        }
      }
      if (require_search_ownership != 0U)
      {
        ++stats->wrong_samples;
      }
    }
    else if (require_search_ownership != 0U &&
             action != LINE_ACTION_SEARCH_LEFT &&
             action != LINE_ACTION_SEARCH_RIGHT)
    {
      ++stats->wrong_samples;
    }
    HAL_Delay(1U);
  }
}

static void print_counts(const char *label,
                         const WheelEncoderCounts *counts)
{
  DiagnosticUart_WriteString(label);
  DiagnosticUart_WriteString(" M1=");
  DiagnosticUart_WriteSigned(counts->motor1);
  DiagnosticUart_WriteString(" M2=");
  DiagnosticUart_WriteSigned(counts->motor2);
  DiagnosticUart_WriteString(" M3=");
  DiagnosticUart_WriteSigned(counts->motor3);
  DiagnosticUart_WriteString(" M4=");
  DiagnosticUart_WriteSigned(counts->motor4);
  DiagnosticUart_WriteString("\r\n");
}

void LineTrackingLiftTest_Run(void)
{
  LiftPhaseStats initial = {0};
  LiftPhaseStats hint = {0};
  LiftPhaseStats lost = {0};
  LiftPhaseStats outer = {0};
  LiftPhaseStats capture = {0};
  LiftPhaseStats final_center = {0};
  WheelEncoderCounts outer_start;
  WheelEncoderCounts outer_end;
  DriveBaseTelemetry telemetry;
  uint8_t fail_mask = 0U;

  DiagnosticUart_WriteString("LTST START SYNTHETIC LINE; WHEELS MUST BE LIFTED\r\n");
  DriveBase_ClearFault();
  DriveBase_Stop(DRIVE_STOP_COAST);
  line_tracking_set_no_line_forward(0U);
  line_tracking_set_smooth_mode(1U);
  line_tracking_set_turn_gain_percent(100U);
  line_tracking_reset();

  /* X1+X3 centred, then X1-only records a predicted left recovery. */
  run_phase(0x05U, LIFT_TEST_CENTER_MS, 0U, &initial);
  run_phase(0x01U, LIFT_TEST_DIRECTION_HINT_MS, 0U, &hint);
  if (initial.forward_samples == 0U)
  {
    fail_mask |= LIFT_TEST_FAIL_INITIAL_FORWARD;
  }

  /* All white starts the encoder-bounded left rear-pivot search. */
  run_phase(0x00U, LIFT_TEST_LOST_SEARCH_MS, 0U, &lost);
  if (lost.search_samples == 0U)
  {
    fail_mask |= LIFT_TEST_FAIL_SEARCH_NOT_STARTED;
  }

  WheelEncoder_GetCounts(&outer_start);
  /* X2 is left outer only. It must not release EncoderTurn or go forward. */
  run_phase(0x02U, LIFT_TEST_OUTER_ONLY_MS, 1U, &outer);
  WheelEncoder_GetCounts(&outer_end);
  if (outer.valid_samples != 0U)
  {
    fail_mask |= LIFT_TEST_FAIL_OUTER_RELEASED;
  }
  if (outer.search_samples == 0U || outer.wrong_samples != 0U)
  {
    fail_mask |= LIFT_TEST_FAIL_OUTER_NOT_SEARCHING;
  }
  if (outer_end.motor1 >= outer_start.motor1 - 20L ||
      outer_end.motor3 <= outer_start.motor3 + 20L ||
      abs_i32(outer_end.motor2 - outer_start.motor2) > 20L ||
      abs_i32(outer_end.motor4 - outer_start.motor4) > 20L)
  {
    fail_mask |= LIFT_TEST_FAIL_ENCODER_DIRECTION;
  }

  /* X1 is a middle hit. After 12 ms confirmation and active braking, the
     controller may hand back to low-speed line capture. */
  run_phase(0x01U, LIFT_TEST_CENTER_CAPTURE_MS, 0U, &capture);
  run_phase(0x05U, LIFT_TEST_FINAL_CENTER_MS, 0U, &final_center);
  if (capture.valid_samples == 0U || final_center.forward_samples == 0U)
  {
    fail_mask |= LIFT_TEST_FAIL_CENTER_NOT_CAPTURED;
  }
  if (capture.first_forward_seen == 0U ||
      capture.first_forward_ms > LIFT_TEST_MAX_CAPTURE_PAUSE_MS)
  {
    fail_mask |= LIFT_TEST_FAIL_CAPTURE_PAUSE;
  }

  DriveBase_GetTelemetry(&telemetry);
  if (telemetry.fault_mask != 0U)
  {
    fail_mask |= LIFT_TEST_FAIL_DRIVE_BASE;
  }

  DriveBase_Stop(DRIVE_STOP_COAST);
  line_tracking_reset();
  DriveBase_Task(HAL_GetTick());

  DiagnosticUart_WriteString("LTST OUTER search=");
  DiagnosticUart_WriteUnsigned(outer.search_samples);
  DiagnosticUart_WriteString(" valid=");
  DiagnosticUart_WriteUnsigned(outer.valid_samples);
  DiagnosticUart_WriteString(" wrong=");
  DiagnosticUart_WriteUnsigned(outer.wrong_samples);
  DiagnosticUart_WriteString("\r\n");
  DiagnosticUart_WriteString("LTST CAPTURE valid=");
  DiagnosticUart_WriteUnsigned(capture.valid_samples);
  DiagnosticUart_WriteString(" first-ms=");
  if (capture.first_forward_seen != 0U)
  {
    DiagnosticUart_WriteUnsigned(capture.first_forward_ms);
  }
  else
  {
    DiagnosticUart_WriteString("NONE");
  }
  DiagnosticUart_WriteString(" final-forward=");
  DiagnosticUart_WriteUnsigned(final_center.forward_samples);
  DiagnosticUart_WriteString("\r\n");
  print_counts("LTST OUTER BEGIN", &outer_start);
  print_counts("LTST OUTER END", &outer_end);
  DiagnosticUart_WriteString("LTST RESULT ");
  DiagnosticUart_WriteString(fail_mask == 0U ? "PASS" : "FAIL");
  DiagnosticUart_WriteString(" MASK=");
  DiagnosticUart_WriteUnsigned(fail_mask);
  DiagnosticUart_WriteString(" DRIVE_F=");
  DiagnosticUart_WriteUnsigned(telemetry.fault_mask);
  DiagnosticUart_WriteString("\r\nLTST STOPPED; NORMAL APP DEFAULT STOP\r\n");
}

#endif /* LINE_TRACKING_LIFT_TEST */
