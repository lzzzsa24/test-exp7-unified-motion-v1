#ifndef __LINE_OBSTACLE_BYPASS_H
#define __LINE_OBSTACLE_BYPASS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  LINE_BYPASS_IDLE = 0U,
  LINE_BYPASS_STOPPING,
  LINE_BYPASS_REVERSING,
  LINE_BYPASS_DIRECTION_GUARD,
  LINE_BYPASS_TURNING,
  LINE_BYPASS_DRIVING,
  LINE_BYPASS_EVALUATING,
  LINE_BYPASS_DONE,
  LINE_BYPASS_FAULT
} LineObstacleBypassState;

typedef struct
{
  uint16_t reverse_max_mm;
  uint16_t forward_step_mm;
  uint16_t clear_probe_mm;
  uint16_t return_step_mm;
  uint16_t post_turn_step_mm;
  uint16_t stop_time_ms;
  uint16_t emergency_stop_time_ms;
  uint16_t direction_guard_ms;
  uint16_t minimum_band_adc;
  uint16_t minimum_flank_travel_mm;
  uint16_t acquire_max_travel_mm;
  uint16_t blind_parallel_travel_mm;
  uint16_t sensor_filter_interval_ms;
  uint16_t reverse_cps;
  uint16_t emergency_reverse_cps;
  uint16_t forward_cps;
  uint16_t clear_probe_cps;
  uint16_t return_cps;
  uint16_t turn_cps;
  uint32_t emergency_speed_cps;
  int32_t turn_step_mdeg;
  int32_t continuous_turn_limit_mdeg;
  int32_t acquire_turn_limit_mdeg;
  int32_t return_heading_mdeg;
  int32_t return_heading_tolerance_mdeg;
  int32_t maximum_net_turn_mdeg;
  uint8_t clear_confirm_steps;
  uint8_t sensor_filter_samples;
  uint8_t line_clear_samples;
  uint8_t line_confirm_samples;
} LineObstacleBypassConfig;

typedef struct
{
  uint8_t line_mask;
  uint8_t infrared_valid;
  uint16_t left_ir_adc;
  uint16_t right_ir_adc;
  uint16_t left_ir_threshold;
  uint16_t right_ir_threshold;
  uint16_t left_ir_hysteresis;
  uint16_t right_ir_hysteresis;
} LineObstacleBypassInput;

typedef struct
{
  LineObstacleBypassState state;
  int8_t bypass_direction;
  uint8_t motion_intent;
  uint8_t fault_mask;
  uint8_t line_mask;
  uint8_t original_line_cleared;
  uint8_t flank_acquired;
  uint8_t acquire_escape_committed;
  uint8_t return_aligned;
  uint8_t clear_probe_steps;
  uint32_t acquire_travel_mm;
  uint32_t flank_travel_mm;
  uint32_t return_travel_mm;
  uint32_t entry_speed_cps;
  uint16_t inside_ir_adc;
  uint16_t inside_ir_lower;
  uint16_t inside_ir_upper;
  uint32_t segment_progress_mm;
  int32_t net_turn_mdeg;
  int32_t return_target_mdeg;
  uint8_t emergency_brake_active;
  uint8_t guided_turn_active;
} LineObstacleBypassTelemetry;

void LineObstacleBypass_GetDefaultConfig(LineObstacleBypassConfig *config);
void LineObstacleBypass_Init(const LineObstacleBypassConfig *config);
/* direction: negative=bypass left, positive=bypass right. */
uint8_t LineObstacleBypass_Start(int8_t direction);
uint8_t LineObstacleBypass_StartWithSpeed(int8_t direction,
                                          uint32_t entry_speed_cps);
void LineObstacleBypass_Task(const LineObstacleBypassInput *input);
void LineObstacleBypass_Stop(void);
LineObstacleBypassState LineObstacleBypass_GetState(void);
uint8_t LineObstacleBypass_GetFaultMask(void);
void LineObstacleBypass_GetTelemetry(LineObstacleBypassTelemetry *telemetry);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_OBSTACLE_BYPASS_H */
