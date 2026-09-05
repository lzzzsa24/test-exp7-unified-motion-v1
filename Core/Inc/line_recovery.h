#ifndef LINE_RECOVERY_H
#define LINE_RECOVERY_H
#include "line_tracking.h"

typedef enum { LINE_RECOVERY_BUSY, LINE_RECOVERY_CAPTURED,
               LINE_RECOVERY_FAILED } LineRecoveryResult;

typedef enum
{
  LINE_REC_STOP_NONE = 0,
  LINE_REC_STOP_SEARCH_TIMEOUT,
  LINE_REC_STOP_SCAN_LIMIT,
  LINE_REC_STOP_RETURN_REJECTED,
  LINE_REC_STOP_RETURN_TIMEOUT,
  LINE_REC_STOP_CAPTURE_TIMEOUT,
  LINE_REC_STOP_REJOIN_LOST,
  LINE_REC_STOP_NO_MOTION,
  LINE_REC_STOP_ENCODER_RANGE,
  LINE_REC_STOP_DRIVE_FAULT
} LineRecoveryStopReason;

/* Preserved until a successful capture is committed or the mode is reset.
   Stopping never clears DriveBase faults or restarts the search clock. */
LineRecoveryStopReason LineRecovery_GetStopReason(void);
void LineRecovery_Stop(LineRecoveryStopReason reason);

void LineRecovery_Reset(void);
void LineRecovery_Record(const LineTrackingReading *reading, uint32_t now);
void LineRecovery_Begin(int8_t preferred_side, uint32_t now);
LineRecoveryResult LineRecovery_Step(const LineTrackingReading *reading,
                                     LineTrackingCommand *command, uint32_t now);
uint8_t LineRecovery_Expired(uint32_t now);
void LineRecovery_Commit(void);
#endif
