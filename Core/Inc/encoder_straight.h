#ifndef __ENCODER_STRAIGHT_H
#define __ENCODER_STRAIGHT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  ENCODER_STRAIGHT_IDLE = 0U,
  ENCODER_STRAIGHT_RUNNING,
  ENCODER_STRAIGHT_SETTLE,
  ENCODER_STRAIGHT_DONE,
  ENCODER_STRAIGHT_FAULT
} EncoderStraightState;

void EncoderStraight_Init(void);
uint8_t EncoderStraight_Start(uint32_t distance_mm, int32_t cruise_cps);
void EncoderStraight_Task(void);
void EncoderStraight_Stop(void);
EncoderStraightState EncoderStraight_GetState(void);
uint8_t EncoderStraight_GetFaultMask(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_STRAIGHT_H */
