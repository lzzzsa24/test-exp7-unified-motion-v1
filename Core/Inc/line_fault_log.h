#ifndef LINE_FAULT_LOG_H
#define LINE_FAULT_LOG_H
#include <stdint.h>

#define LINE_FAULT_LOG_CAPACITY 32U
typedef struct
{
  uint32_t sequence, first_ms, last_ms, occurrences, elapsed_ms;
  uint8_t stall_mask, direction_mask, signal_mask, sensor_mask;
  uint8_t recovery_state, degraded_mask;
  uint16_t battery_mv;
  int32_t requested[4], controlled[4], measured[4], delta[4];
  int16_t pwm[4];
  uint32_t illegal_delta[4], no_motion_ms[4];
} LineFaultRecord;

/* Fixed-size RAM history, retained by STOP/mode change/ClearFault, not reboot. */
void LineFaultLog_Init(void);
void LineFaultLog_Record(const LineFaultRecord *record);
uint32_t LineFaultLog_Count(void);
uint32_t LineFaultLog_Overwritten(void);
uint8_t LineFaultLog_Get(uint32_t oldest_index, LineFaultRecord *record);
void LineFaultLog_RequestDump(void);
/* One short line per call, only while operator STOP is active. */
void LineFaultLog_Task(uint8_t operator_stopped);
#endif
