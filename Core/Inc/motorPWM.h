/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motorPWM.h
  * @brief   实验二：四路直流电机 PWM 调速接口
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MOTOR_PWM_H
#define __MOTOR_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_PWM_PERIOD 3599U

void motor_pwm_init(void);
void pwm_motor_stop(void);
void pwm_motor1_forward(int16_t speed);
void pwm_motor1_backward(int16_t speed);
void pwm_motor2_forward(int16_t speed);
void pwm_motor2_backward(int16_t speed);
void pwm_motor3_forward(int16_t speed);
void pwm_motor3_backward(int16_t speed);
void pwm_motor4_forward(int16_t speed);
void pwm_motor4_backward(int16_t speed);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PWM_H */
