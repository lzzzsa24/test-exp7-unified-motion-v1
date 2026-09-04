/* Compatibility facade retained for source compatibility with older modules.
 * The unified-motion project has one motor owner: drive_base.c. */

#include "wheel_speed_control.h"

#include "drive_base.h"
#include "main.h"

static uint8_t compatibility_running;

void WheelSpeedControl_Init(void)
{
  compatibility_running = 0U;
}

void WheelSpeedControl_Start(void)
{
  compatibility_running = 1U;
}

void WheelSpeedControl_Stop(void)
{
  compatibility_running = 0U;
  DriveBase_Stop(DRIVE_STOP_COAST);
}

void WheelSpeedControl_SetTargets(int16_t motor1,
                                  int16_t motor2,
                                  int16_t motor3,
                                  int16_t motor4)
{
  compatibility_running = 1U;
  DriveBase_SetWheelCps(DriveBase_EquivalentCpsFromPwm(motor1),
                        DriveBase_EquivalentCpsFromPwm(motor2),
                        DriveBase_EquivalentCpsFromPwm(motor3),
                        DriveBase_EquivalentCpsFromPwm(motor4));
}

void WheelSpeedControl_SetCpsTargets(int32_t motor1_cps,
                                     int32_t motor2_cps,
                                     int32_t motor3_cps,
                                     int32_t motor4_cps)
{
  compatibility_running = 1U;
  DriveBase_SetWheelCps(motor1_cps, motor2_cps,
                        motor3_cps, motor4_cps);
}

void WheelSpeedControl_Task(void)
{
  DriveBase_Task(HAL_GetTick());
}

void WheelSpeedControl_GetMeasurements(WheelSpeedMeasurements *measurements)
{
  DriveBaseTelemetry telemetry;

  if (measurements == 0)
  {
    return;
  }
  DriveBase_GetTelemetry(&telemetry);
  measurements->motor1_cps = telemetry.measured_cps[0];
  measurements->motor2_cps = telemetry.measured_cps[1];
  measurements->motor3_cps = telemetry.measured_cps[2];
  measurements->motor4_cps = telemetry.measured_cps[3];
}

uint8_t WheelSpeedControl_IsRunning(void)
{
  return compatibility_running;
}
