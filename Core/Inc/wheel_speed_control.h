#ifndef __WHEEL_SPEED_CONTROL_H
#define __WHEEL_SPEED_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  int32_t motor1_cps;
  int32_t motor2_cps;
  int32_t motor3_cps;
  int32_t motor4_cps;
} WheelSpeedMeasurements;

/* Deprecated compatibility API. New code must call DriveBase directly.
   Signed targets use logical PWM units: positive=forward, negative=backward. */
void WheelSpeedControl_Init(void);
void WheelSpeedControl_Start(void);
void WheelSpeedControl_Stop(void);
void WheelSpeedControl_SetTargets(int16_t motor1,
                                  int16_t motor2,
                                  int16_t motor3,
                                  int16_t motor4);
/* Signed physical targets in encoder counts per second. */
void WheelSpeedControl_SetCpsTargets(int32_t motor1_cps,
                                     int32_t motor2_cps,
                                     int32_t motor3_cps,
                                     int32_t motor4_cps);
void WheelSpeedControl_Task(void);
void WheelSpeedControl_GetMeasurements(WheelSpeedMeasurements *measurements);
uint8_t WheelSpeedControl_IsRunning(void);

#ifdef __cplusplus
}
#endif

#endif /* __WHEEL_SPEED_CONTROL_H */
