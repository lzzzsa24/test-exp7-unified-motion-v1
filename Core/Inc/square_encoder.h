#ifndef __SQUARE_ENCODER_H
#define __SQUARE_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  SQUARE_IDLE = 0U,
  SQUARE_DRIVE,
  SQUARE_PAUSE_BEFORE_TURN,
  SQUARE_TURN,
  SQUARE_PAUSE_BEFORE_DRIVE,
  SQUARE_DONE,
  SQUARE_FAULT
} SquareEncoderState;

void SquareEncoder_Init(void);
void SquareEncoder_Start(void);
void SquareEncoder_Stop(void);
void SquareEncoder_Task(void);
SquareEncoderState SquareEncoder_GetState(void);
uint8_t SquareEncoder_GetFaultMask(void);
uint8_t SquareEncoder_GetSide(void);

#ifdef __cplusplus
}
#endif

#endif /* __SQUARE_ENCODER_H */
