#ifndef __ENCODER_TURN_H
#define __ENCODER_TURN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  ENCODER_TURN_IDLE = 0U,
  ENCODER_TURN_RUNNING,
  ENCODER_TURN_DONE,
  ENCODER_TURN_FAULT
} EncoderTurnState;

void EncoderTurn_Init(void);

/*
 * angle_mdeg: positive=left, negative=right.
 * center_radius_mm: signed radius of the vehicle center path; positive=left
 * center, negative=right center, 0=in-place turn.  For a forward arc, radius
 * and angle have the same sign.
 * maximum_wheel_cps: target speed of the wheel on the largest path radius.
 */
uint8_t EncoderTurn_Start(int32_t angle_mdeg,
                          int32_t center_radius_mm,
                          int32_t maximum_wheel_cps);
void EncoderTurn_Task(void);
/* Non-blocking early finish for sensor-guided continuous turns.  Motor PWM is
 * removed immediately, encoder sampling remains active through the short
 * settle window, and GetAchievedAngleMdeg() is updated before DONE. */
uint8_t EncoderTurn_RequestStop(void);
void EncoderTurn_Stop(void);
EncoderTurnState EncoderTurn_GetState(void);
uint8_t EncoderTurn_GetFaultMask(void);
/* Encoder-derived signed angle after the motion has settled.  This reports
 * wheel motion, not an IMU-measured chassis yaw angle. */
int32_t EncoderTurn_GetAchievedAngleMdeg(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_TURN_H */
