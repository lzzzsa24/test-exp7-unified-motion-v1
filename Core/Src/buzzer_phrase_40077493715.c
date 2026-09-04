#include "buzzer_phrase_40077493715.h"

#include "main.h"

typedef struct
{
  uint16_t duration_ms;
  GPIO_PinState output;
} BuzzerPhraseStep;

/*
 * Frozen 40077493715 phrase envelope.  The board carries an active buzzer,
 * so pitch cannot be changed in software; these steps reproduce the five
 * audible attacks and their pauses.  One complete repeat is 1530 ms.
 */
static const BuzzerPhraseStep phrase_steps[] =
{
  {110U, GPIO_PIN_SET},
  { 30U, GPIO_PIN_RESET},
  {150U, GPIO_PIN_SET},
  { 10U, GPIO_PIN_RESET},
  {140U, GPIO_PIN_SET},
  { 60U, GPIO_PIN_RESET},
  {170U, GPIO_PIN_SET},
  { 12U, GPIO_PIN_RESET},
  {280U, GPIO_PIN_SET},
  {568U, GPIO_PIN_RESET}
};

#define BUZZER_PHRASE_STEP_COUNT \
  ((uint8_t)(sizeof(phrase_steps) / sizeof(phrase_steps[0])))
#define BUZZER_PHRASE_MAX_REPEATS 24U

static uint8_t phrase_playing;
static uint8_t phrase_step_index;
static uint8_t phrase_repeat_index;
static uint8_t phrase_repeat_target;
static uint32_t phrase_deadline_ms;

static uint8_t phrase_tick_reached(uint32_t now_ms, uint32_t deadline_ms)
{
  return (int32_t)(now_ms - deadline_ms) >= 0 ? 1U : 0U;
}

void BuzzerPhrase400_Init(void)
{
  phrase_playing = 0U;
  phrase_step_index = 0U;
  phrase_repeat_index = 0U;
  phrase_repeat_target = 0U;
  phrase_deadline_ms = 0U;
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
}

uint8_t BuzzerPhrase400_Start(uint8_t repeat_count)
{
  uint32_t now_ms;

  if (repeat_count == 0U || repeat_count > BUZZER_PHRASE_MAX_REPEATS)
  {
    return 0U;
  }

  now_ms = HAL_GetTick();
  phrase_step_index = 0U;
  phrase_repeat_index = 0U;
  phrase_repeat_target = repeat_count;
  phrase_playing = 1U;
  HAL_GPIO_WritePin(Buzzer_GPIO_Port,
                    Buzzer_Pin,
                    phrase_steps[0].output);
  phrase_deadline_ms = now_ms + phrase_steps[0].duration_ms;
  return 1U;
}

void BuzzerPhrase400_Stop(void)
{
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
  phrase_playing = 0U;
  phrase_step_index = 0U;
  phrase_repeat_index = 0U;
  phrase_repeat_target = 0U;
  phrase_deadline_ms = 0U;
}

void BuzzerPhrase400_Task(uint32_t now_ms)
{
  while (phrase_playing != 0U &&
         phrase_tick_reached(now_ms, phrase_deadline_ms) != 0U)
  {
    ++phrase_step_index;
    if (phrase_step_index >= BUZZER_PHRASE_STEP_COUNT)
    {
      phrase_step_index = 0U;
      ++phrase_repeat_index;
      if (phrase_repeat_index >= phrase_repeat_target)
      {
        BuzzerPhrase400_Stop();
        return;
      }
    }

    HAL_GPIO_WritePin(Buzzer_GPIO_Port,
                      Buzzer_Pin,
                      phrase_steps[phrase_step_index].output);
    /* Accumulate from the previous deadline so loop jitter cannot stretch
       the phrase.  The signed deadline comparison remains safe at tick wrap. */
    phrase_deadline_ms += phrase_steps[phrase_step_index].duration_ms;
  }
}

uint8_t BuzzerPhrase400_IsPlaying(void)
{
  return phrase_playing;
}
