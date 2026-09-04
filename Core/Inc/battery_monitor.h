#ifndef __BATTERY_MONITOR_H
#define __BATTERY_MONITOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint16_t millivolts;
  uint16_t raw_adc;
  uint8_t percent;
  uint8_t valid;
  uint8_t low;
} BatteryMonitorStatus;

/* Board divider: VM -- 10 kOhm -- PF7/ADC3_IN5 -- 3.3 kOhm -- GND. */
void BatteryMonitor_Init(void);
void BatteryMonitor_Task(void);
void BatteryMonitor_Get(BatteryMonitorStatus *status);

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_MONITOR_H */
