#include "operator_input.h"

void Keys_Init(KeyInput *keys, uint8_t pressed, uint32_t now)
{
  keys->candidate = keys->stable = pressed;
  keys->armed = 0U; /* First require release, including a key held at reset. */
  keys->changed_at = now;
}

uint8_t Keys_Step(KeyInput *keys, uint8_t pressed, uint32_t now)
{
  uint8_t event = 0U;
  if (pressed != keys->candidate) {
    keys->candidate = pressed;
    keys->changed_at = now;
  }
  if (now - keys->changed_at >= 20U) {
    keys->stable = pressed;
    if (pressed == 0U) keys->armed = 1U;
    if ((pressed & 3U) && keys->armed) {
      event = INPUT_START;
      keys->armed = 0U;
    }
  }
  /* KEY3 is level-sensitive and wins immediately over either start key. */
  if (pressed & 4U) {
    keys->armed = 0U;
    event = INPUT_STOP;
  }
  return event;
}

uint8_t Input_Serial(uint8_t byte)
{
  switch (byte) {
    case '1': case '2': return INPUT_START;
    case '0': case '3': case 's': case 'S': case ' ': return INPUT_STOP;
    case '?': return INPUT_QUERY;
    default: return 0U;
  }
}

uint8_t Input_NecEdge(NecInput *nec, uint32_t interval_us)
{
  uint8_t address, inverse, command, command_inverse;
  if (interval_us >= 12500U && interval_us <= 14500U) {
    nec->bits = 0U;
    nec->count = 0U;
    nec->receiving = 1U;
    return 0U;
  }
  if (!nec->receiving) return 0U;
  if (interval_us >= 1750U && interval_us <= 2800U) {
    nec->bits |= (uint32_t)1U << nec->count;
  } else if (interval_us < 800U || interval_us > 1500U) {
    nec->receiving = 0U; /* Invalid pulse or repeat frame. */
    return 0U;
  }
  if (++nec->count < 32U) return 0U;
  nec->receiving = 0U;
  address = (uint8_t)nec->bits;
  inverse = (uint8_t)(nec->bits >> 8U);
  command = (uint8_t)(nec->bits >> 16U);
  command_inverse = (uint8_t)(nec->bits >> 24U);
  if ((uint8_t)(address ^ inverse) != 255U ||
      (uint8_t)(command ^ command_inverse) != 255U) return 0U;
  switch (command) {
    case 0x10: case 0x11: return INPUT_START; /* Yahboom 1 / 2 */
    case 0x0D: case 0x12: return INPUT_STOP;  /* Yahboom 0 / 3 */
    default: return 0U;
  }
}
