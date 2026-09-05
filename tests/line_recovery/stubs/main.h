#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
typedef int GPIO_TypeDef;
typedef struct { uint32_t Mode, Pull, Speed, Pin; } GPIO_InitTypeDef;
#define GPIO_MODE_INPUT 0
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
typedef enum { GPIO_PIN_RESET=0, GPIO_PIN_SET=1 } GPIO_PinState;
#define Buzzer_Pin 16
#define Buzzer_GPIO_Port ((GPIO_TypeDef *)0)
#define TRACK_X1_Pin 1
#define TRACK_X2_Pin 2
#define TRACK_X3_Pin 4
#define TRACK_X4_Pin 8
#define TRACK_X1_GPIO_Port ((GPIO_TypeDef *)0)
#define TRACK_X2_GPIO_Port ((GPIO_TypeDef *)0)
#define TRACK_X3_GPIO_Port ((GPIO_TypeDef *)0)
#define TRACK_X4_GPIO_Port ((GPIO_TypeDef *)0)
#define __HAL_RCC_GPIOF_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOG_CLK_ENABLE() ((void)0)
uint32_t HAL_GetTick(void);
int HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
#endif
