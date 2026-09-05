#ifndef LINE_RECOVERY_H
#define LINE_RECOVERY_H
#include "line_tracking.h"

typedef enum { LINE_RECOVERY_BUSY, LINE_RECOVERY_CAPTURED,
               LINE_RECOVERY_FAILED } LineRecoveryResult;

void LineRecovery_Reset(void);
void LineRecovery_Record(const LineTrackingReading *reading, uint32_t now);
void LineRecovery_Begin(int8_t preferred_side, uint32_t now);
LineRecoveryResult LineRecovery_Step(const LineTrackingReading *reading,
                                     LineTrackingCommand *command, uint32_t now);
uint8_t LineRecovery_Expired(uint32_t now);
void LineRecovery_Commit(void);
#endif
