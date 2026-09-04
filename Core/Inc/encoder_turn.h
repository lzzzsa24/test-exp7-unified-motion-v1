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
/* Line-sensor sweep which biases the ground pivot toward the rear axle.
 * M1/M3 are the front-left/front-right motors; the outside front wheel moves
 * forward at maximum speed while the reversing inside front wheel is scaled
 * by inner_reverse_percent. M2/M4 remain unpowered so the rear tyre contact
 * patch acts as the approximate pivot. The forward bias limits unintended
 * reverse translation when left/right traction differs. */
uint8_t EncoderTurn_StartRearPivot(int32_t angle_mdeg,
                                   int32_t maximum_front_wheel_cps,
                                   uint16_t inner_reverse_percent);
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
