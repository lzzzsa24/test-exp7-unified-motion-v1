#include "line_turn_load.h"

#define LOAD_CONFIRM_MS       40U
#define LOAD_MAX_EXTRA_PWM   600L
#define LOAD_RISE_PWM_PER_MS   5L
#define LOAD_MIN_CPS        1412L
#define LOAD_MAX_CPS        9000L

int16_t LineTurnLoad_Update(LineTurnLoadState *s, uint8_t enabled,
                           int32_t target, int32_t measured, uint32_t elapsed)
{
  int32_t magnitude, desired, next;
  int64_t actual;
  if (!s) return 0;
  /* Reject stale samples as well as reversal, overspeed and low-speed pulses.
     Clear immediately on recovery; do not keep a timed open-loop kick. */
  if (!enabled || elapsed == 0U || elapsed > 60U ||
      target < -LOAD_MAX_CPS || target > LOAD_MAX_CPS) goto clear;
  magnitude = target < 0 ? -target : target;
  actual = target < 0 ? -(int64_t)measured : (int64_t)measured;
  if (magnitude < LOAD_MIN_CPS || actual < 0 ||
      (int64_t)actual * 100 >= (int64_t)magnitude * 85) goto clear;

  if (s->slow_ms < LOAD_CONFIRM_MS) s->slow_ms += elapsed;
  if (s->slow_ms < LOAD_CONFIRM_MS) return 0;
  desired = (magnitude - (int32_t)actual) / 2;
  if (desired > LOAD_MAX_EXTRA_PWM) desired = LOAD_MAX_EXTRA_PWM;
  next = s->extra_pwm + (int32_t)elapsed * LOAD_RISE_PWM_PER_MS;
  s->extra_pwm = (int16_t)(next < desired ? next : desired);
  return s->extra_pwm;
clear:
  s->slow_ms = 0U;
  s->extra_pwm = 0;
  return 0;
}
