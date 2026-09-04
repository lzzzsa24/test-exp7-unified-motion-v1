/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motion_advanced.h
  * @brief   实验三：差速转弯、轨迹和变速运动接口
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MOTION_ADVANCED_H
#define __MOTION_ADVANCED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void advanced_drive_forward(int16_t left_speed, int16_t right_speed);
void advanced_drive_backward(int16_t left_speed, int16_t right_speed);
void advanced_turn_left(int16_t inner_speed, int16_t outer_speed);
void advanced_turn_right(int16_t inner_speed, int16_t outer_speed);
void advanced_spin_left(int16_t speed);
void advanced_spin_right(int16_t speed);
void advanced_stop(void);
/* Apply an already converted signed left/right wheel-speed command. This is
   the execution boundary used by the line-tracking outer loop. */
void advanced_drive_cps(int32_t left_cps, int32_t right_cps);

/* 仅限制正向/差速转向的 PWM，不改变电机方向映射；原地旋转保持
   避障所需的独立速度。传入 MOTOR_PWM_PERIOD 可恢复正向无限制。 */
void advanced_set_forward_speed_limit(int16_t max_speed);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_ADVANCED_H */
