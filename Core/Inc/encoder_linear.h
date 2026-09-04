#ifndef __ENCODER_LINEAR_H
#define __ENCODER_LINEAR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  ENCODER_LINEAR_IDLE = 0U,
  ENCODER_LINEAR_RUNNING,
  ENCODER_LINEAR_SETTLE,
  ENCODER_LINEAR_DONE,
  ENCODER_LINEAR_FAULT
} EncoderLinearState;

void EncoderLinear_Init(void);

/*
 * distance_mm: positive=forward, negative=backward.
 * cruise_cps: positive magnitude in encoder counts per second.
 */
uint8_t EncoderLinear_Start(int32_t distance_mm, int32_t cruise_cps);
void EncoderLinear_Task(void);
void EncoderLinear_Stop(void);
EncoderLinearState EncoderLinear_GetState(void);
uint8_t EncoderLinear_GetFaultMask(void);
uint32_t EncoderLinear_GetProgressMm(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_LINEAR_H */
