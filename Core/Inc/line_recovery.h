#ifndef LINE_RECOVERY_H
#define LINE_RECOVERY_H
#include "line_tracking.h"

typedef enum { LINE_RECOVERY_BUSY, LINE_RECOVERY_CAPTURED,
               LINE_RECOVERY_FAILED } LineRecoveryResult;

typedef enum
{
  LINE_REC_STOP_NONE = 0,
  LINE_REC_STOP_DRIVE_FAULT = 9
} LineRecoveryStopReason;

/* Preserved until a successful capture is committed or the mode is reset.
   Only drive faults latch; search has no timeout, distance or attempt limit. */
LineRecoveryStopReason LineRecovery_GetStopReason(void);
void LineRecovery_Stop(LineRecoveryStopReason reason);

void LineRecovery_Reset(void);
void LineRecovery_Begin(int8_t preferred_side, uint32_t now);
LineRecoveryResult LineRecovery_Step(const LineTrackingReading *reading,
                                     LineTrackingCommand *command, uint32_t now);
void LineRecovery_Commit(void);
#endif
