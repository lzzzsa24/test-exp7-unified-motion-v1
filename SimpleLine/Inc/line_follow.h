#ifndef SIMPLE_LINE_FOLLOW_H
#define SIMPLE_LINE_FOLLOW_H

#include <stdint.h>
#include "line_config.h"

/* Bit order on screen and in tests: left outer, left inner, right inner,
 * right outer. This is X2, X1, X3, X4, NOT connector number order. */
#define LINE_LO 8U
#define LINE_LI 4U
#define LINE_RI 2U
#define LINE_RO 1U

typedef enum { LINE_STOP, LINE_TRACK, LINE_TURN, LINE_SEARCH, LINE_WIDE } LineMode;
typedef enum {
  LINE_USER_STOP, LINE_OK, LINE_NO_START_LINE, LINE_LOST,
  LINE_TURN_TIMEOUT, LINE_WIDE_TIMEOUT, LINE_LOOP_TIMEOUT, LINE_UNSTABLE
} LineReason;

typedef struct {
  LineMode mode;
  LineReason reason;
  int16_t left_pwm, right_pwm;
  uint8_t raw, filtered, candidate, sample_count, ready;
  int8_t last_direction;
  uint8_t maneuver, center_confirming, wide;
  uint32_t last_tick, stable_at, direction_at, maneuver_at, center_at, wide_at;
} LineFollower;

void Line_Init(LineFollower *line, uint32_t now);
void Line_Start(LineFollower *line, uint32_t now);
void Line_Stop(LineFollower *line, LineReason reason);
/* Call once per LINE_PERIOD_MS. Unsigned differences support tick wraparound. */
void Line_Step(LineFollower *line, uint8_t mask, uint32_t now);
const char *Line_ModeName(LineMode mode);
const char *Line_ReasonName(LineReason reason);

#endif
