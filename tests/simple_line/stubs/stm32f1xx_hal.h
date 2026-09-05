#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H
#include <stdint.h>
typedef struct { uint32_t IDR; } GPIO_TypeDef;
typedef struct { uint32_t BDTR, CCR1, CCR2, CCR3, CCR4; } TIM_TypeDef;
typedef struct { uint32_t CR1, CR2, CR3, BRR, SR, DR; } USART_TypeDef;
typedef struct { uint32_t CYCCNT, CTRL; } DWT_Type;
typedef struct { uint32_t DEMCR; } CoreDebug_Type;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef struct {
  uint32_t Prescaler, CounterMode, Period, ClockDivision, AutoReloadPreload;
} TIM_Base_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_Base_InitTypeDef Init; } TIM_HandleTypeDef;
typedef struct {
  uint32_t OCMode, OCPolarity, OCNPolarity, OCIdleState, OCNIdleState, OCFastMode, Pulse;
} TIM_OC_InitTypeDef;
extern GPIO_TypeDef test_gpio[7];
extern TIM_TypeDef test_timer1, test_timer8;
extern USART_TypeDef test_usart;
extern DWT_Type test_dwt;
extern CoreDebug_Type test_debug;
extern uint32_t SystemCoreClock, test_tick, test_exti, test_remap;
#define GPIOA (&test_gpio[0])
#define GPIOB (&test_gpio[1])
#define GPIOC (&test_gpio[2])
#define GPIOD (&test_gpio[3])
#define GPIOE (&test_gpio[4])
#define GPIOF (&test_gpio[5])
#define GPIOG (&test_gpio[6])
#define TIM1 (&test_timer1)
#define TIM8 (&test_timer8)
#define USART1 (&test_usart)
#define DWT (&test_dwt)
#define CoreDebug (&test_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define GPIO_PIN_0 0x0001U
#define GPIO_PIN_1 0x0002U
#define GPIO_PIN_2 0x0004U
#define GPIO_PIN_3 0x0008U
#define GPIO_PIN_4 0x0010U
#define GPIO_PIN_5 0x0020U
#define GPIO_PIN_6 0x0040U
#define GPIO_PIN_7 0x0080U
#define GPIO_PIN_8 0x0100U
#define GPIO_PIN_9 0x0200U
#define GPIO_PIN_10 0x0400U
#define GPIO_PIN_11 0x0800U
#define GPIO_PIN_12 0x1000U
#define GPIO_PIN_13 0x2000U
#define GPIO_PIN_14 0x4000U
#define GPIO_PIN_15 0x8000U
#define GPIO_MODE_AF_PP 1U
#define GPIO_MODE_OUTPUT_PP 2U
#define GPIO_MODE_INPUT 3U
#define GPIO_MODE_IT_FALLING 4U
#define GPIO_NOPULL 0U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_LOW 1U
#define GPIO_SPEED_FREQ_HIGH 3U
#define TIM_BDTR_MOE (1U << 15)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define TIM_OCMODE_PWM1 1U
#define TIM_OCPOLARITY_HIGH 0U
#define TIM_OCNPOLARITY_HIGH 0U
#define TIM_OCIDLESTATE_RESET 0U
#define TIM_OCNIDLESTATE_RESET 0U
#define TIM_OCFAST_DISABLE 0U
#define TIM_COUNTERMODE_UP 0U
#define TIM_CLOCKDIVISION_DIV1 0U
#define TIM_AUTORELOAD_PRELOAD_ENABLE 1U
#define HAL_OK 0
#define USART_SR_PE (1U << 0)
#define USART_SR_FE (1U << 1)
#define USART_SR_NE (1U << 2)
#define USART_SR_ORE (1U << 3)
#define USART_SR_RXNE (1U << 5)
#define USART_SR_TXE (1U << 7)
#define USART_CR1_RE (1U << 2)
#define USART_CR1_TE (1U << 3)
#define USART_CR1_RXNEIE (1U << 5)
#define USART_CR1_TXEIE (1U << 7)
#define USART_CR1_UE (1U << 13)
#define USART1_IRQn 37
#define EXTI15_10_IRQn 40
#define __HAL_RCC_AFIO_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOF_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOG_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM8_CLK_ENABLE() ((void)0)
#define __HAL_AFIO_REMAP_TIM1_ENABLE() (test_remap = 1U)
#define __HAL_TIM_SET_COMPARE(t,c,v) test_set_compare((t), (c), (v))
#define __HAL_GPIO_EXTI_GET_IT(p) (test_exti & (p))
#define __HAL_GPIO_EXTI_CLEAR_IT(p) (test_exti &= ~(p))
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) { }
static inline void __enable_irq(void) { }
static inline uint32_t HAL_GetTick(void) { return test_tick; }
static inline uint32_t HAL_RCC_GetPCLK2Freq(void) { return 72000000U; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pins, GPIO_PinState state);
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t sub);
void HAL_NVIC_EnableIRQ(int irq);
int HAL_TIM_PWM_Init(TIM_HandleTypeDef *timer);
int HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *timer, TIM_OC_InitTypeDef *config, uint32_t channel);
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
void test_set_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value);
#endif
