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

static void run_active(LineFollower *line, uint8_t mask, uint32_t duration)
{
  uint32_t elapsed;
  for (elapsed = 0U; elapsed < duration; elapsed += LINE_PERIOD_MS) {
    step(line, mask);
    CHECK(line->mode != LINE_STOP && line->left_pwm != 0 && line->right_pwm != 0);
  }
}

static void start(LineFollower *line, uint8_t mask)
{
  Line_Init(line, now);
  CHECK(line->mode == LINE_STOP && line->left_pwm == 0 && line->right_pwm == 0);
  Line_Start(line, now);
  step(line, mask);
  CHECK(line->mode != LINE_STOP && line->left_pwm != 0 && line->right_pwm != 0);
  step(line, mask);
}

static void test_patterns(void)
{
  /* Physical left-to-right order. White now searches left from a fresh start. */
  static const int16_t left[16] = {
    -2700,2700,2600,2700,2200,2200,2400,2200,-2700,2200,2200,2200,-2700,2200,2200,2200
  };
  static const int16_t right[16] = {
    2700,-2700,2200,-2700,2600,2200,2400,2200,2700,2200,2200,2200,2700,2200,2200,2200
  };
  unsigned mask;
  for (mask = 0U; mask < 16U; ++mask) {
    LineFollower line;
    now = 0U;
    start(&line, (uint8_t)mask);
    CHECK(line.left_pwm == left[mask] && line.right_pwm == right[mask]);
    CHECK(line.reason == LINE_OK);
    /* Reproduce the reported "moves then stops" for EVERY stable input. */
    run_active(&line, (uint8_t)mask, 10000U);
    CHECK(line.left_pwm == left[mask] && line.right_pwm == right[mask]);
    Line_Stop(&line, LINE_USER_STOP);
    run(&line, 6U, 1000U);
    run(&line, 0U, 1000U);
    CHECK(line.mode == LINE_STOP && line.left_pwm == 0 && line.right_pwm == 0);
  }
}

static void test_filter_without_stop(void)
{
  LineFollower line;
  unsigned i;
  now = 0U;
  start(&line, 6U);
  step(&line, 8U); /* A single false outer pulse must not reverse a motor. */
  CHECK(line.mode == LINE_TRACK && line.left_pwm > 0);
  step(&line, 6U);
  CHECK(line.mode == LINE_TRACK);
  for (i = 0; i < 2000U; ++i) {
    step(&line, (uint8_t)((i & 1U) ? 1U : 8U));
    CHECK(line.mode == LINE_TRACK && line.left_pwm == line.right_pwm);
  }
  run_active(&line, 0U, 100U);
  CHECK(line.mode == LINE_SEARCH && line.left_pwm < 0 && line.right_pwm > 0);
  /* During search, noisy input must also keep running without a re-arm. */
  for (i = 0; i < 2000U; ++i) {
    step(&line, (uint8_t)((i & 1U) ? 0U : 6U));
    CHECK(line.mode == LINE_SEARCH);
  }
  /* Noise from the very first sample must not leave START waiting at zero. */
  Line_Stop(&line, LINE_USER_STOP);
  Line_Start(&line, now);
  for (i = 0; i < 200U; ++i) {
    step(&line, (uint8_t)((i & 1U) ? 1U : 8U));
    CHECK(line.mode != LINE_STOP && line.left_pwm != 0 && line.right_pwm != 0);
  }
}

static void test_continuous_search(void)
{
  LineFollower line;
  unsigned i;
  now = 0U;
  start(&line, 0U); /* Explicit START on all-white must search, not latch STOP. */
  run_active(&line, 0U, 300000U); /* Five simulated minutes, no wall-clock wait. */
  CHECK(line.mode == LINE_SEARCH && line.left_pwm == -2700 && line.right_pwm == 2700);
  run_active(&line, 6U, 20U);
  CHECK(line.mode == LINE_TRACK && line.left_pwm == 2400);
  run_active(&line, 2U, 20U); /* Learn right. */
  run_active(&line, 6U, 60000U); /* A long straight does not expire the hint. */
  run_active(&line, 0U, 300000U);
  CHECK(line.mode == LINE_SEARCH && line.left_pwm == 2700 && line.right_pwm == -2700);
  for (i = 0; i < 100U; ++i) {
    Line_Start(&line, now);
    run_active(&line, 0U, 20U);
    CHECK(line.left_pwm == 2700);
  }
  Line_Stop(&line, LINE_USER_STOP);
  run(&line, 0U, 1000U);
  CHECK(line.mode == LINE_STOP && line.left_pwm == 0 && line.right_pwm == 0);
  Line_Start(&line, now);
  step(&line, 0U);
  CHECK(line.mode == LINE_SEARCH && line.left_pwm < 0);
}

static void test_reacquire_and_time(void)
{
  LineFollower line;
  now = UINT32_MAX - 20U;
  start(&line, 1U);
  run_active(&line, 0U, 100U); /* Includes millisecond counter wrap. */
  CHECK(line.mode == LINE_SEARCH && line.left_pwm > 0 && line.right_pwm < 0);
  run_active(&line, 6U, 15U);
  CHECK(line.mode == LINE_TRACK && line.left_pwm == 2400);
  run_active(&line, 15U, 2000U);
  CHECK(line.mode == LINE_WIDE && line.left_pwm == line.right_pwm);
  now += 60000U; /* A late control pass must not change RUN into STOP. */
  Line_Step(&line, 15U, now);
  CHECK(line.mode == LINE_WIDE && line.left_pwm > 0);
  run_active(&line, 0U, 2000U);
  now += 60000U;
  Line_Step(&line, 0U, now);
  CHECK(line.mode == LINE_SEARCH && line.left_pwm > 0 && line.right_pwm < 0);
  CHECK(!strcmp(Line_ModeName(LINE_SEARCH), "SEARCH"));
  CHECK(!strcmp(Line_ReasonName(LINE_USER_STOP), "USER"));
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
  test_filter_without_stop();
  test_continuous_search();
  test_reacquire_and_time();
  test_controls();
  printf("PASS controller and operator input: %u checks\n", checks);
  return 0;
}
