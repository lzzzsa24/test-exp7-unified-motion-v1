#include "line_follow.h"
#include "operator_input.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;
static uint32_t now;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
  fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

static void step(LineFollower *line, uint8_t mask)
{
  now += LINE_PERIOD_MS;
  Line_Step(line, mask, now);
}

static void run(LineFollower *line, uint8_t mask, uint32_t duration)
{
  uint32_t elapsed;
  for (elapsed = 0U; elapsed < duration; elapsed += LINE_PERIOD_MS) step(line, mask);
}

static void start(LineFollower *line, uint8_t mask)
{
  Line_Init(line, now);
  CHECK(line->mode == LINE_STOP && line->left_pwm == 0 && line->right_pwm == 0);
  Line_Start(line, now);
  step(line, mask);
  CHECK(line->left_pwm == 0 && line->right_pwm == 0);
  step(line, mask);
}

static void test_patterns(void)
{
  /* Independent expected decisions, in physical left-to-right binary order. */
  static const int16_t left[16] = {
    0,2700,2600,2700,2200,2200,2400,2200,-2700,2200,2200,2200,-2700,2200,2200,2200
  };
  static const int16_t right[16] = {
    0,-2700,2200,-2700,2600,2200,2400,2200,2700,2200,2200,2200,2700,2200,2200,2200
  };
  unsigned mask;
  for (mask = 0U; mask < 16U; ++mask) {
    LineFollower line;
    now = 0U;
    start(&line, (uint8_t)mask);
    CHECK(line.left_pwm == left[mask] && line.right_pwm == right[mask]);
    if (mask == 0U) CHECK(line.reason == LINE_NO_START_LINE);
    else CHECK(line.mode != LINE_STOP);
    Line_Stop(&line, LINE_USER_STOP);
    run(&line, 6U, 1000U);
    CHECK(line.mode == LINE_STOP && line.left_pwm == 0 && line.right_pwm == 0);
  }
}

static void test_filter_and_gap(void)
{
  LineFollower line;
  unsigned i;
  now = 0U;
  start(&line, 6U);
  step(&line, 8U); /* A single false outer pulse must not reverse a motor. */
  CHECK(line.mode == LINE_TRACK && line.left_pwm > 0);
  step(&line, 6U);
  CHECK(line.mode == LINE_TRACK);
  run(&line, 0U, 40U);
  CHECK(line.mode == LINE_SEARCH && line.left_pwm == line.right_pwm);
  run(&line, 6U, 40U);
  CHECK(line.mode == LINE_TRACK && !line.maneuver);
  run(&line, 0U, 80U);
  CHECK(line.mode == LINE_STOP && line.reason == LINE_LOST);
  run(&line, 6U, 50U);
  CHECK(line.mode == LINE_STOP);
  Line_Start(&line, now);
  run(&line, 6U, 20U);
  CHECK(line.mode == LINE_TRACK);
  /* Never retain a stale straight command under endlessly bouncing input. */
  for (i = 0; i < 12U; ++i) step(&line, (uint8_t)((i & 1U) ? 1U : 8U));
  CHECK(line.mode == LINE_STOP && line.reason == LINE_UNSTABLE);
}

static void test_search_limits(void)
{
  LineFollower line;
  uint32_t began;
  now = 0U;
  start(&line, 4U);
  run(&line, 0U, 20U);
  CHECK(line.mode == LINE_SEARCH && line.left_pwm < 0 && line.right_pwm > 0);
  began = line.maneuver_at;
  /* Brief centre sightings and repeated start commands cannot renew timeout. */
  while (line.mode != LINE_STOP && now - began < 1000U) {
    Line_Start(&line, now);
    run(&line, 6U, 15U);
    run(&line, 0U, 25U);
  }
  CHECK(line.mode == LINE_STOP);
  CHECK(now - began >= LINE_MANEUVER_MS && now - began < LINE_MANEUVER_MS + 40U);
  start(&line, 8U);
  run(&line, 8U, 950U);
  CHECK(line.reason == LINE_TURN_TIMEOUT);
  /* A sufficiently old left correction must not choose left after a straight. */
  start(&line, 4U);
  run(&line, 6U, 300U);
  run(&line, 0U, 20U);
  CHECK(line.mode == LINE_SEARCH && line.left_pwm == line.right_pwm);
  run(&line, 0U, 60U);
  CHECK(line.mode == LINE_STOP && line.reason == LINE_LOST);
}

static void test_reacquire_wide_and_time(void)
{
  LineFollower line;
  now = UINT32_MAX - 20U;
  start(&line, 1U);
  run(&line, 0U, 25U); /* Includes millisecond counter wrap. */
  CHECK(line.mode == LINE_SEARCH && line.left_pwm > 0 && line.right_pwm < 0);
  run(&line, 6U, 15U);
  CHECK(line.mode == LINE_TRACK && line.maneuver && line.left_pwm == 2200);
  run(&line, 6U, 40U);
  CHECK(!line.maneuver && line.left_pwm == 2400);
  run(&line, 15U, 100U);
  CHECK(line.mode == LINE_WIDE && line.left_pwm == line.right_pwm);
  run(&line, 6U, 20U);
  CHECK(line.mode == LINE_TRACK);
  run(&line, 9U, 330U);
  CHECK(line.mode == LINE_STOP && line.reason == LINE_WIDE_TIMEOUT);
  start(&line, 6U);
  now += 51U;
  Line_Step(&line, 6U, now);
  CHECK(line.mode == LINE_STOP && line.reason == LINE_LOOP_TIMEOUT);
  CHECK(!strcmp(Line_ModeName(LINE_SEARCH), "SEARCH"));
  CHECK(!strcmp(Line_ReasonName(LINE_UNSTABLE), "NOISY"));
}

static uint8_t nec_frame(NecInput *nec, uint32_t bits)
{
  uint8_t events = Input_NecEdge(nec, 13500U);
  unsigned i;
  for (i = 0; i < 32U; ++i)
    events |= Input_NecEdge(nec, (bits & ((uint32_t)1U << i)) ? 2250U : 1125U);
  return events;
}

static void test_controls(void)
{
  KeyInput key;
  NecInput nec = {0};
  uint32_t t;
  Keys_Init(&key, 1U, 0U); /* KEY1 held during boot. */
  for (t = 5U; t <= 500U; t += 5U) CHECK(Keys_Step(&key, 1U, t) == 0U);
  CHECK(Keys_Step(&key, 0U, 505U) == 0U);
  CHECK(Keys_Step(&key, 0U, 525U) == 0U);
  CHECK(Keys_Step(&key, 1U, 530U) == 0U);
  CHECK(Keys_Step(&key, 0U, 535U) == 0U); /* Contact bounce. */
  CHECK(Keys_Step(&key, 1U, 540U) == 0U);
  CHECK(Keys_Step(&key, 1U, 560U) == INPUT_START);
  CHECK(Keys_Step(&key, 1U, 580U) == 0U);
  CHECK(Keys_Step(&key, 5U, 585U) == INPUT_STOP);
  CHECK(Keys_Step(&key, 1U, 605U) == 0U);
  CHECK(Input_Serial('1') == INPUT_START && Input_Serial('2') == INPUT_START);
  CHECK(Input_Serial('0') == INPUT_STOP && Input_Serial('3') == INPUT_STOP);
  CHECK(Input_Serial('s') == INPUT_STOP && Input_Serial('?') == INPUT_QUERY);
  CHECK(Input_Serial('\n') == 0U && Input_Serial('4') == 0U);
  CHECK(nec_frame(&nec, 0xEF10FF00U) == INPUT_START);
  CHECK(Input_NecEdge(&nec, 11250U) == 0U); /* NEC repeat. */
  CHECK(nec_frame(&nec, 0xF20DFF00U) == INPUT_STOP);
  CHECK(nec_frame(&nec, 0xED12FF00U) == INPUT_STOP);
  CHECK(nec_frame(&nec, 0xEE11FF00U) == INPUT_START);
  CHECK(nec_frame(&nec, 0xEF10FF01U) == 0U); /* Bad address complement. */
  CHECK(nec_frame(&nec, 0xEE10FF00U) == 0U); /* Bad command complement. */
  CHECK(Input_NecEdge(&nec, 13500U) == 0U);
  CHECK(Input_NecEdge(&nec, 4000U) == 0U && !nec.receiving);
}

int main(void)
{
  test_patterns();
  test_filter_and_gap();
  test_search_limits();
  test_reacquire_wide_and_time();
  test_controls();
  printf("PASS controller and operator input: %u checks\n", checks);
  return 0;
}
