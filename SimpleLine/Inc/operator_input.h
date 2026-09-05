#ifndef SIMPLE_LINE_OPERATOR_INPUT_H
#define SIMPLE_LINE_OPERATOR_INPUT_H
#include <stdint.h>

#define INPUT_START 1U
#define INPUT_STOP  2U
#define INPUT_QUERY 4U

typedef struct {
  uint8_t candidate, stable, armed;
  uint32_t changed_at;
} KeyInput;

typedef struct { uint32_t bits; uint8_t count, receiving; } NecInput;

void Keys_Init(KeyInput *keys, uint8_t pressed, uint32_t now);
uint8_t Keys_Step(KeyInput *keys, uint8_t pressed, uint32_t now);
uint8_t Input_Serial(uint8_t byte);
/* NEC falling-edge intervals in microseconds; repeats do not start motion. */
uint8_t Input_NecEdge(NecInput *nec, uint32_t interval_us);
#endif
