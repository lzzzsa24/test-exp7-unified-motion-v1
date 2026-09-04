/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ultrasonic.h
  * @brief   四针超声波模块的非阻塞测距驱动
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ULTRASONIC_RESULT_NONE       0U
#define ULTRASONIC_RESULT_OK         1U
#define ULTRASONIC_RESULT_TIMEOUT    2U
#define ULTRASONIC_RESULT_OUT_RANGE  3U

/* 初始化 PF11/PF12、EXTI15_10 和 TIM2 微秒计时器。 */
void Ultrasonic_Init(void);

/* 发起一次测量；返回 1 表示已启动，返回 0 表示仍有测量未完成。 */
uint8_t Ultrasonic_Start(void);

/* 在主循环中高频调用，用于处理超时。 */
void Ultrasonic_Task(void);

/* 读取一次完成结果；没有新结果时返回 ULTRASONIC_RESULT_NONE。 */
uint8_t Ultrasonic_GetResult(uint16_t *distance_cm);

/* 读取最近一次结果，用于故障指示；不会消费待处理结果。 */
uint8_t Ultrasonic_GetLastResult(void);

/* 最近一次成功回波的毫米距离；仅用于相对运动估计，普通避障继续使用 cm。 */
uint16_t Ultrasonic_GetLastDistanceMm(void);

/* 在 HAL_GPIO_EXTI_Callback 中调用，不要在本模块重复定义该 HAL 回调。 */
void Ultrasonic_EXTI_Callback(uint16_t GPIO_Pin);

uint8_t Ultrasonic_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_H */
