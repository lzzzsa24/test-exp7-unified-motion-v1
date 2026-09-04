#ifndef __FIGURE8_ENCODER_H
#define __FIGURE8_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  FIGURE8_IDLE = 0U,
  FIGURE8_LEFT_LOOP,
  FIGURE8_RIGHT_LOOP,
  FIGURE8_PAUSE,
  FIGURE8_DONE,
  FIGURE8_FAULT
} Figure8EncoderState;

void Figure8Encoder_Init(void);
void Figure8Encoder_Start(void);
void Figure8Encoder_Stop(void);
void Figure8Encoder_Task(void);
Figure8EncoderState Figure8Encoder_GetState(void);
uint8_t Figure8Encoder_GetFaultMask(void);

#ifdef __cplusplus
}
#endif

#endif /* __FIGURE8_ENCODER_H */
