/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RRGB_R_Pin GPIO_PIN_2
#define RRGB_R_GPIO_Port GPIOE
#define RRGB_G_Pin GPIO_PIN_3
#define RRGB_G_GPIO_Port GPIOE
#define RRGB_B_Pin GPIO_PIN_4
#define RRGB_B_GPIO_Port GPIOE
#define LRGB_R_Pin GPIO_PIN_1
#define LRGB_R_GPIO_Port GPIOG
#define LRGB_G_Pin GPIO_PIN_7
#define LRGB_G_GPIO_Port GPIOE
#define LRGB_B_Pin GPIO_PIN_2
#define LRGB_B_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

#ifndef Buzzer_Pin
#define Buzzer_Pin GPIO_PIN_12
#endif
#ifndef Buzzer_GPIO_Port
#define Buzzer_GPIO_Port GPIOG
#endif
#ifndef led1_Pin
#define led1_Pin GPIO_PIN_13
#endif
#ifndef led1_GPIO_Port
#define led1_GPIO_Port GPIOG
#endif
#ifndef led2_Pin
#define led2_Pin GPIO_PIN_15
#endif
#ifndef led2_GPIO_Port
#define led2_GPIO_Port GPIOG
#endif
#ifndef key1_Pin
#define key1_Pin GPIO_PIN_3
#endif
#ifndef key1_GPIO_Port
#define key1_GPIO_Port GPIOG
#endif
#ifndef key2_Pin
#define key2_Pin GPIO_PIN_4
#endif
#ifndef key2_GPIO_Port
#define key2_GPIO_Port GPIOG
#endif
#ifndef key3_Pin
#define key3_Pin GPIO_PIN_5
#endif
#ifndef key3_GPIO_Port
#define key3_GPIO_Port GPIOG
#endif

/* 实验二：四路直流电机驱动引脚。若以后由 CubeMX 重新生成，
   这些保护宏会自动让位给生成的同名定义。 */
#ifndef M1A_Pin
#define M1A_Pin GPIO_PIN_6
#endif
#ifndef M1A_GPIO_Port
#define M1A_GPIO_Port GPIOC
#endif
#ifndef M1B_Pin
#define M1B_Pin GPIO_PIN_7
#endif
#ifndef M1B_GPIO_Port
#define M1B_GPIO_Port GPIOC
#endif
#ifndef M2A_Pin
#define M2A_Pin GPIO_PIN_8
#endif
#ifndef M2A_GPIO_Port
#define M2A_GPIO_Port GPIOC
#endif
#ifndef M2B_Pin
#define M2B_Pin GPIO_PIN_9
#endif
#ifndef M2B_GPIO_Port
#define M2B_GPIO_Port GPIOC
#endif
#ifndef M3A_Pin
#define M3A_Pin GPIO_PIN_9
#endif
#ifndef M3A_GPIO_Port
#define M3A_GPIO_Port GPIOE
#endif
#ifndef M3B_Pin
#define M3B_Pin GPIO_PIN_11
#endif
#ifndef M3B_GPIO_Port
#define M3B_GPIO_Port GPIOE
#endif
#ifndef M4A_Pin
#define M4A_Pin GPIO_PIN_13
#endif
#ifndef M4A_GPIO_Port
#define M4A_GPIO_Port GPIOE
#endif
#ifndef M4B_Pin
#define M4B_Pin GPIO_PIN_14
#endif
#ifndef M4B_GPIO_Port
#define M4B_GPIO_Port GPIOE
#endif

/* 实验五：底板红外避障模块。PE5/PE6 低电平开启发射管，
   PF9/PF10 分别接 ADC3_IN7/ADC3_IN8。 */
#ifndef IR_LEFT_ENABLE_Pin
#define IR_LEFT_ENABLE_Pin GPIO_PIN_5
#endif
#ifndef IR_LEFT_ENABLE_GPIO_Port
#define IR_LEFT_ENABLE_GPIO_Port GPIOE
#endif
#ifndef IR_RIGHT_ENABLE_Pin
#define IR_RIGHT_ENABLE_Pin GPIO_PIN_6
#endif
#ifndef IR_RIGHT_ENABLE_GPIO_Port
#define IR_RIGHT_ENABLE_GPIO_Port GPIOE
#endif
#ifndef IR_LEFT_ADC_Pin
#define IR_LEFT_ADC_Pin GPIO_PIN_9
#endif
#ifndef IR_LEFT_ADC_GPIO_Port
#define IR_LEFT_ADC_GPIO_Port GPIOF
#endif
#ifndef IR_RIGHT_ADC_Pin
#define IR_RIGHT_ADC_Pin GPIO_PIN_10
#endif
#ifndef IR_RIGHT_ADC_GPIO_Port
#define IR_RIGHT_ADC_GPIO_Port GPIOF
#endif

/* 实验七：四路循迹接口（指导书附录 B.1/B.2 的 X1～X4）。
   传感器输出为低电平时表示检测到黑线。X1/X3 为中间两路，
   X2/X4 为左右外侧两路，便于沿用指导书中的动作逻辑。 */
#ifndef TRACK_X1_Pin
#define TRACK_X1_Pin GPIO_PIN_13
#endif
#ifndef TRACK_X1_GPIO_Port
#define TRACK_X1_GPIO_Port GPIOF
#endif
#ifndef TRACK_X2_Pin
#define TRACK_X2_Pin GPIO_PIN_14
#endif
#ifndef TRACK_X2_GPIO_Port
#define TRACK_X2_GPIO_Port GPIOF
#endif
#ifndef TRACK_X3_Pin
#define TRACK_X3_Pin GPIO_PIN_15
#endif
#ifndef TRACK_X3_GPIO_Port
#define TRACK_X3_GPIO_Port GPIOF
#endif
#ifndef TRACK_X4_Pin
#define TRACK_X4_Pin GPIO_PIN_0
#endif
#ifndef TRACK_X4_GPIO_Port
#define TRACK_X4_GPIO_Port GPIOG
#endif

/* 指导书的串口 2 接口用于外接 K210：STM32 USART2 为 PD5(TX)/PD6(RX)。
   视觉通信采用 3.3 V TTL，TX/RX 必须交叉连接并共地。 */
#ifndef VISION_UART_TX_Pin
#define VISION_UART_TX_Pin GPIO_PIN_5
#endif
#ifndef VISION_UART_TX_GPIO_Port
#define VISION_UART_TX_GPIO_Port GPIOD
#endif
#ifndef VISION_UART_RX_Pin
#define VISION_UART_RX_Pin GPIO_PIN_6
#endif
#ifndef VISION_UART_RX_GPIO_Port
#define VISION_UART_RX_GPIO_Port GPIOD
#endif

/* 实验七扩展：J1 四针超声波。
   该工程的 .ioc 未声明 PF11/PF12/TIM2，初始化由 ultrasonic.c 手动完成；
   以后若用 CubeMX 重新生成，请确认不要覆盖这组 User Code 和中断合并。 */
#ifndef ULTRASONIC_TRIG_Pin
#define ULTRASONIC_TRIG_Pin GPIO_PIN_11
#endif
#ifndef ULTRASONIC_TRIG_GPIO_Port
#define ULTRASONIC_TRIG_GPIO_Port GPIOF
#endif
#ifndef ULTRASONIC_ECHO_Pin
#define ULTRASONIC_ECHO_Pin GPIO_PIN_12
#endif
#ifndef ULTRASONIC_ECHO_GPIO_Port
#define ULTRASONIC_ECHO_GPIO_Port GPIOF
#endif

/* 板载 HS0038B 红外遥控接收头，PG11 下降沿 EXTI；与 PF12 共用
   EXTI15_10 IRQ，由 ir_remote.c 手动初始化。 */
#ifndef IR_REMOTE_Pin
#define IR_REMOTE_Pin GPIO_PIN_11
#endif
#ifndef IR_REMOTE_GPIO_Port
#define IR_REMOTE_GPIO_Port GPIOG
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
