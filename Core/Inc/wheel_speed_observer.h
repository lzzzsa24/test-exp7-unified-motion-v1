#ifndef WHEEL_SPEED_OBSERVER_H
#define WHEEL_SPEED_OBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Passive four-wheel speed estimate for direct-PWM line-following modes.
 * It never writes a motor output and must be stopped before an encoder motion
 * controller takes ownership of WheelEncoder. */
void WheelSpeedObserver_Init(void);
void WheelSpeedObserver_Start(void);
void WheelSpeedObserver_Stop(void);
void WheelSpeedObserver_Task(void);
uint8_t WheelSpeedObserver_GetAverageCps(uint32_t *average_cps);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SPEED_OBSERVER_H */
