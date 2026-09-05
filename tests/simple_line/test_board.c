#include "main.h"
#include "board.h"
#include "motorPWM.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

GPIO_TypeDef test_gpio[7];
TIM_TypeDef test_timer1, test_timer8;
USART_TypeDef test_usart;
IWDG_TypeDef test_iwdg;
DWT_Type test_dwt;
CoreDebug_Type test_debug;
uint32_t SystemCoreClock = 72000000U, test_tick, test_exti, test_remap;
static uint32_t pin_modes[7][16], pin_pulls[7][16];
static uint32_t output_pins[7];
static unsigned checks;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
  fprintf(stderr, "FAIL board line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

void Error_Handler(void) { fprintf(stderr, "Unexpected Error_Handler\n"); exit(1); }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config)
{
  unsigned bit;
  for (bit = 0; bit < 16U; ++bit) if (config->Pin & (1U << bit)) {
    pin_modes[port - test_gpio][bit] = config->Mode;
    pin_pulls[port - test_gpio][bit] = config->Pull;
  }
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pins, GPIO_PinState state)
{
  if (state) output_pins[port - test_gpio] |= pins;
  else output_pins[port - test_gpio] &= ~pins;
}
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t sub)
{
  CHECK((irq == USART1_IRQn || irq == EXTI15_10_IRQn) && priority == 2U && sub == 0U);
}
void HAL_NVIC_EnableIRQ(int irq) { CHECK(irq == USART1_IRQn || irq == EXTI15_10_IRQn); }
int HAL_TIM_PWM_Init(TIM_HandleTypeDef *timer)
{
  CHECK(timer->Init.Prescaler == 0U && timer->Init.Period == 3599U);
  return HAL_OK;
}
int HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *timer, TIM_OC_InitTypeDef *config, uint32_t channel)
{
  CHECK(config->Pulse == 0U && config->OCMode == TIM_OCMODE_PWM1);
  test_set_compare(timer, channel, 0U);
  return HAL_OK;
}
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
  CHECK(channel <= TIM_CHANNEL_4);
  timer->Instance->BDTR |= TIM_BDTR_MOE;
  return HAL_OK;
}
void test_set_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{
  switch (channel) {
    case TIM_CHANNEL_1: timer->Instance->CCR1 = value; break;
    case TIM_CHANNEL_2: timer->Instance->CCR2 = value; break;
    case TIM_CHANNEL_3: timer->Instance->CCR3 = value; break;
    case TIM_CHANNEL_4: timer->Instance->CCR4 = value; break;
    default: CHECK(0);
  }
}

static void motors_zero(void)
{
  CHECK(TIM1->CCR1 == 0U && TIM1->CCR2 == 0U && TIM1->CCR3 == 0U && TIM1->CCR4 == 0U);
  CHECK(TIM8->CCR1 == 0U && TIM8->CCR2 == 0U && TIM8->CCR3 == 0U && TIM8->CCR4 == 0U);
}

static void test_motors(void)
{
  unsigned pwm;
  uint32_t previous[4] = {0};
  test_tick = 0U;
  motor_pwm_init();
  motors_zero();
  CHECK(test_remap && pin_modes[2][6] == GPIO_MODE_AF_PP && pin_modes[4][14] == GPIO_MODE_AF_PP);
  motor_pwm_set_sides(2200, 2200, 10U);
  CHECK(TIM8->CCR1 == 0U && TIM8->CCR2 == 2174U && TIM8->CCR3 == 0U && TIM8->CCR4 == 2208U);
  CHECK(TIM1->CCR1 == 2210U && TIM1->CCR2 == 0U && TIM1->CCR3 == 2208U && TIM1->CCR4 == 0U);
  motor_pwm_set_sides(-2200, -2200, 20U);
  motors_zero();
  motor_pwm_set_sides(-2200, -2200, 29U);
  motors_zero();
  motor_pwm_set_sides(-2200, -2200, 30U);
  CHECK(TIM8->CCR1 == 2174U && TIM8->CCR2 == 0U && TIM8->CCR3 == 2208U && TIM8->CCR4 == 0U);
  CHECK(TIM1->CCR1 == 0U && TIM1->CCR2 == 2210U && TIM1->CCR3 == 0U && TIM1->CCR4 == 2208U);
  motor_pwm_set_sides(-2400, 2400, 40U);
  CHECK(TIM8->CCR1 == 2371U && TIM1->CCR1 == 0U && TIM1->CCR2 == 0U);
  motor_pwm_set_sides(-2400, 2400, 50U);
  CHECK(TIM8->CCR1 == 2371U && TIM1->CCR1 == 2412U);
  test_tick = 51U;
  pwm_motor_stop();
  motors_zero();
  motor_pwm_set_sides(32767, -32768, 52U);
  motors_zero();
  motor_pwm_set_sides(32767, -32768, 61U);
  CHECK(TIM8->CCR2 == 3599U && TIM1->CCR2 == 3599U);
  CHECK(motor_pwm_get_side(0U) == 3599 && motor_pwm_get_side(1U) == -3599);
  test_tick = 70U;
  pwm_motor_stop();
  for (pwm = 0; pwm <= 3599U; ++pwm) {
    uint32_t current[4];
    unsigned wheel;
    motor_pwm_set_sides((int16_t)pwm, (int16_t)pwm, 80U + pwm);
    current[0] = TIM8->CCR2; current[1] = TIM8->CCR4;
    current[2] = TIM1->CCR1; current[3] = TIM1->CCR3;
    CHECK(TIM8->CCR1 == 0U && TIM8->CCR3 == 0U && TIM1->CCR2 == 0U && TIM1->CCR4 == 0U);
    for (wheel = 0; wheel < 4U; ++wheel) {
      CHECK(current[wheel] >= previous[wheel] && current[wheel] <= 3599U);
      previous[wheel] = current[wheel];
    }
  }
  motor_pwm_set_sides(-2400, -2400, UINT32_MAX - 4U);
  motors_zero();
  motor_pwm_set_sides(-2400, -2400, 4U);
  motors_zero();
  motor_pwm_set_sides(-2400, -2400, 5U);
  CHECK(TIM8->CCR1 == 2371U && TIM1->CCR2 == 2412U);
  motor_pwm_emergency_stop();
  motors_zero();
  CHECK(!(TIM1->BDTR & TIM_BDTR_MOE) && !(TIM8->BDTR & TIM_BDTR_MOE));
}

void USART1_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
static void receive(uint8_t command)
{
  USART1->SR = USART_SR_RXNE;
  USART1->DR = command;
  USART1_IRQHandler();
}

static void test_board_io(void)
{
  unsigned i, mask;
  LineFollower line;
  char text[128] = {0};
  for (i = 0; i < 7U; ++i) test_gpio[i].IDR = 0xFFFFU;
  test_tick = 0U;
  Board_Init();
  CHECK(USART1->BRR == 625U && (USART1->CR1 & USART_CR1_RXNEIE));
  CHECK(pin_modes[5][13] == GPIO_MODE_INPUT && pin_pulls[5][13] == GPIO_PULLUP);
  CHECK(pin_modes[6][11] == GPIO_MODE_IT_FALLING && pin_modes[6][12] == GPIO_MODE_OUTPUT_PP);
  CHECK((output_pins[4] & (GPIO_PIN_5 | GPIO_PIN_6)) == (GPIO_PIN_5 | GPIO_PIN_6));
  CHECK(Board_ReadLine() == 0U);
  for (mask = 0; mask < 16U; ++mask) {
    GPIOF->IDR = GPIOG->IDR = 0xFFFFU;
    /* Expected physical GPIOs, independent of TRACK_X* macros. */
    if (mask & 8U) GPIOF->IDR &= ~(1U << 14);
    if (mask & 4U) GPIOF->IDR &= ~(1U << 13);
    if (mask & 2U) GPIOF->IDR &= ~(1U << 15);
    if (mask & 1U) GPIOG->IDR &= ~1U;
    CHECK(Board_ReadLine() == mask);
  }
  receive('1'); receive('0');
  CHECK((Board_Inputs(5U) & (INPUT_START | INPUT_STOP)) == (INPUT_START | INPUT_STOP));
  CHECK(Board_Inputs(10U) == 0U);
  receive('2');
  GPIOG->IDR &= ~GPIO_PIN_5;
  CHECK(Board_Inputs(15U) & INPUT_STOP); /* Physical STOP beats incoming START. */
  GPIOG->IDR |= GPIO_PIN_5;
  USART1->SR = USART_SR_ORE;
  USART1_IRQHandler();
  CHECK(Board_Inputs(20U) == INPUT_STOP);
  /* IRQ-driven telemetry can be busy while incoming STOP is still accepted. */
  Line_Init(&line, 20U);
  line.raw = line.filtered = 10U;
  Board_Report(&line);
  Board_Report(&line); /* Drops busy report. */
  for (i = 0; i < sizeof(text) - 1U && (USART1->CR1 & USART_CR1_TXEIE); ++i) {
    USART1->SR = USART_SR_TXE;
    USART1_IRQHandler();
    if (USART1->CR1 & USART_CR1_TXEIE) text[i] = (char)USART1->DR;
    if (i == 2U) receive('0');
  }
  CHECK(strstr(text, "raw=1010 line=1010") != NULL);
  CHECK(Board_Inputs(25U) == INPUT_STOP);
  line.raw = 8U;
  Board_Indicators(&line, 30U);
  CHECK(output_pins[6] & GPIO_PIN_1);
  CHECK(!(output_pins[4] & (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_7)));
  Board_WatchdogStart();
  CHECK(IWDG->PR == 4U && IWDG->RLR == 124U && IWDG->KR == 0xAAAAU);
}

int main(void)
{
  test_motors();
  test_board_io();
  printf("PASS real board/motor sources with HAL register stubs: %u checks\n", checks);
  return 0;
}
