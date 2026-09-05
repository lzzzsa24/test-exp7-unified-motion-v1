#ifndef LINE_TURN_LOAD_H
#define LINE_TURN_LOAD_H
#include <stdint.h>

/* Extra PWM for a lagging wheel, never a wheel-speed target or raw output. */
typedef struct
{
  uint32_t slow_ms;
  int16_t extra_pwm;
} LineTurnLoadState;

int16_t LineTurnLoad_Update(LineTurnLoadState *state, uint8_t enabled,
                           int32_t target_cps, int32_t measured_cps,
                           uint32_t elapsed_ms);
#endif
