#include "line_follow.h"
#include <string.h>

static void output(LineFollower *line, LineMode mode, int16_t left, int16_t right)
{
  line->mode = mode;
  line->left_pwm = left;
  line->right_pwm = right;
}

void Line_Stop(LineFollower *line, LineReason reason)
{
  output(line, LINE_STOP, 0, 0);
  line->reason = reason;
}

void Line_Init(LineFollower *line, uint32_t now)
{
  memset(line, 0, sizeof(*line));
  line->last_tick = now;
  line->stable_at = now;
  Line_Stop(line, LINE_USER_STOP);
}

void Line_Start(LineFollower *line, uint32_t now)
{
  /* A repeated start while driving must not reset a maneuver deadline. */
  if (line->mode != LINE_STOP) return;
  Line_Init(line, now);
  line->mode = LINE_TRACK;
  line->reason = LINE_OK;
}

static void begin_maneuver(LineFollower *line, uint32_t now)
{
  if (!line->maneuver) {
    line->maneuver = 1U;
    line->maneuver_at = now;
  }
  line->center_confirming = 0U;
}

static void turn(LineFollower *line, LineMode mode, int8_t direction)
{
  int16_t left = direction < 0 ? -LINE_TURN_PWM : LINE_TURN_PWM;
  output(line, mode, left, (int16_t)-left);
}

void Line_Step(LineFollower *line, uint8_t mask, uint32_t now)
{
  uint32_t elapsed = now - line->last_tick;
  uint8_t value;
  int8_t direction;
  line->last_tick = now;
  line->raw = mask & 15U;
  if (line->raw != line->candidate) {
    line->candidate = line->raw;
    line->sample_count = 1U;
  } else if (line->sample_count < LINE_FILTER_SAMPLES) {
    ++line->sample_count;
  }
  if (line->sample_count >= LINE_FILTER_SAMPLES) {
    line->filtered = line->candidate;
    line->ready = 1U;
    line->stable_at = now;
  }
  if (line->mode == LINE_STOP) return;
  if (elapsed > LINE_LOOP_TIMEOUT_MS) {
    Line_Stop(line, LINE_LOOP_TIMEOUT);
    return;
  }
  if (now - line->stable_at >= LINE_SENSOR_TIMEOUT_MS) {
    Line_Stop(line, LINE_UNSTABLE);
    return;
  }
  if (!line->ready) return;
  value = line->filtered;

  /* One shared bound covers outer-only turns, white search and transient
   * centre hits. Only 30 ms of centre evidence releases this deadline. */
  if (line->maneuver && now - line->maneuver_at >= LINE_MANEUVER_MS) {
    Line_Stop(line, value == 0U ? LINE_LOST : LINE_TURN_TIMEOUT);
    return;
  }

  if (value == LINE_LI || value == LINE_RI || value == (LINE_LI | LINE_RI)) {
    line->wide = 0U;
    if (line->maneuver) {
      if (!line->center_confirming) {
        line->center_confirming = 1U;
        line->center_at = now;
      } else if (now - line->center_at >= LINE_CENTER_CONFIRM_MS) {
        line->maneuver = 0U;
      }
    }
    if (value == (LINE_LI | LINE_RI)) {
      int16_t speed = line->maneuver ? LINE_SLOW_PWM : LINE_STRAIGHT_PWM;
      output(line, LINE_TRACK, speed, speed);
    } else {
      direction = value == LINE_LI ? -1 : 1;
      line->last_direction = direction;
      line->direction_at = now;
      output(line, LINE_TRACK,
             direction < 0 ? LINE_SLOW_PWM : LINE_OUTER_PWM,
             direction < 0 ? LINE_OUTER_PWM : LINE_SLOW_PWM);
    }
    return;
  }

  line->center_confirming = 0U;
  if (value == LINE_LO || value == (LINE_LO | LINE_LI) ||
      value == LINE_RO || value == (LINE_RO | LINE_RI)) {
    line->wide = 0U;
    direction = (value & LINE_LO) ? -1 : 1;
    line->last_direction = direction;
    line->direction_at = now;
    begin_maneuver(line, now);
    turn(line, LINE_TURN, direction);
    return;
  }

  if (value == 0U) {
    line->wide = 0U;
    if (!line->maneuver) {
      /* No movement at all if start was requested on all white. */
      if (line->left_pwm == 0 && line->right_pwm == 0) {
        Line_Stop(line, LINE_NO_START_LINE);
        return;
      }
      if (now - line->direction_at > LINE_HINT_MAX_AGE_MS)
        line->last_direction = 0;
      begin_maneuver(line, now);
    }
    if (line->last_direction != 0) {
      turn(line, LINE_SEARCH, line->last_direction);
    } else if (now - line->maneuver_at < LINE_GAP_MS) {
      output(line, LINE_SEARCH, LINE_SLOW_PWM, LINE_SLOW_PWM);
    } else {
      Line_Stop(line, LINE_LOST);
    }
    return;
  }

  /* All other masks are wide line, a junction or disjoint detections. There
   * is no route-selection information: cross straight briefly, then stop. */
  if (!line->wide) {
    line->wide = 1U;
    line->wide_at = now;
  }
  if (now - line->wide_at >= LINE_WIDE_MS) {
    Line_Stop(line, LINE_WIDE_TIMEOUT);
  } else {
    output(line, LINE_WIDE, LINE_SLOW_PWM, LINE_SLOW_PWM);
  }
}

const char *Line_ModeName(LineMode mode)
{
  static const char *const names[] = {"STOP", "TRACK", "TURN", "SEARCH", "WIDE"};
  return (unsigned)mode < sizeof(names) / sizeof(names[0]) ? names[mode] : "?";
}

const char *Line_ReasonName(LineReason reason)
{
  static const char *const names[] = {
    "USER", "OK", "NO_LINE", "LOST", "TURN_TIME", "WIDE_TIME", "LOOP_TIME", "NOISY"
  };
  return (unsigned)reason < sizeof(names) / sizeof(names[0]) ? names[reason] : "?";
}
