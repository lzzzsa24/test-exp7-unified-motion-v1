#include "line_follow.h"
#include <string.h>

#if LINE_DEFAULT_SEARCH_DIRECTION != -1 && LINE_DEFAULT_SEARCH_DIRECTION != 1
#error "Search direction must be -1 (left) or +1 (right)."
#endif

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
  (void)now;
  memset(line, 0, sizeof(*line));
  line->last_direction = LINE_DEFAULT_SEARCH_DIRECTION;
  Line_Stop(line, LINE_USER_STOP);
}

void Line_Start(LineFollower *line, uint32_t now)
{
  /* A repeated START must not change the ongoing search direction. */
  if (line->mode != LINE_STOP) return;
  Line_Init(line, now);
  line->mode = LINE_TRACK;
  line->reason = LINE_OK;
}

static void turn(LineFollower *line, LineMode mode, int8_t direction)
{
  int16_t left = direction < 0 ? -LINE_TURN_PWM : LINE_TURN_PWM;
  output(line, mode, left, (int16_t)-left);
}

void Line_Step(LineFollower *line, uint8_t mask, uint32_t now)
{
  uint8_t value;
  int8_t direction;
  (void)now; /* Neither elapsed time nor input instability can latch STOP. */
  line->raw = mask & 15U;
  if (!line->ready) {
    /* Use the first current reading after explicit START. Do not wait stopped
     * indefinitely when startup samples keep changing. Later changes require
     * two identical samples, and noise holds the last accepted decision. */
    line->candidate = line->filtered = line->raw;
    line->sample_count = 1U;
    line->ready = 1U;
  } else if (line->raw != line->candidate) {
    line->candidate = line->raw;
    line->sample_count = 1U;
  } else if (line->sample_count < LINE_FILTER_SAMPLES) {
    ++line->sample_count;
  }
  if (line->sample_count >= LINE_FILTER_SAMPLES)
    line->filtered = line->candidate;
  if (line->mode == LINE_STOP) return;
  value = line->filtered;

  if (value == (LINE_LI | LINE_RI)) {
    output(line, LINE_TRACK, LINE_STRAIGHT_PWM, LINE_STRAIGHT_PWM);
  } else if (value == LINE_LI || value == LINE_RI) {
    direction = value == LINE_LI ? -1 : 1;
    line->last_direction = direction;
    output(line, LINE_TRACK,
           direction < 0 ? LINE_SLOW_PWM : LINE_OUTER_PWM,
           direction < 0 ? LINE_OUTER_PWM : LINE_SLOW_PWM);
  } else if (value == LINE_LO || value == (LINE_LO | LINE_LI) ||
             value == LINE_RO || value == (LINE_RO | LINE_RI)) {
    direction = (value & LINE_LO) ? -1 : 1;
    line->last_direction = direction;
    turn(line, LINE_TURN, direction);
  } else if (value == 0U) {
    /* Never expire or alternate a search merely because time has passed.
     * The first confirmed line reading above resumes tracking automatically. */
    turn(line, LINE_SEARCH, line->last_direction);
  } else {
    /* Wide/junction/disjoint readings: keep crossing slowly, without a timer. */
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
  static const char *const names[] = {"USER", "OK"};
  return (unsigned)reason < sizeof(names) / sizeof(names[0]) ? names[reason] : "?";
}
