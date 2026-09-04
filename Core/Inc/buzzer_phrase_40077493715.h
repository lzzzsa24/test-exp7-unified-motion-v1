#ifndef BUZZER_PHRASE_40077493715_H
#define BUZZER_PHRASE_40077493715_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * Non-blocking rhythm player for the active-high PG12 buzzer.
 * Call BuzzerPhrase400_Task() every main-loop pass (1 ms recommended,
 * no more than 5 ms between calls while playing).
 */
void BuzzerPhrase400_Init(void);
uint8_t BuzzerPhrase400_Start(uint8_t repeat_count);
void BuzzerPhrase400_Stop(void);
void BuzzerPhrase400_Task(uint32_t now_ms);
uint8_t BuzzerPhrase400_IsPlaying(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_PHRASE_40077493715_H */
