/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.h
  * @brief   实验二：四路直流电机 GPIO 运动控制接口
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

void motor_init(void);

void motor1_forward(void);
void motor1_backward(void);
void motor2_forward(void);
void motor2_backward(void);
void motor3_forward(void);
void motor3_backward(void);
void motor4_forward(void);
void motor4_backward(void);
void motor_stop(void);

void car_forward(void);
void car_backward(void);
void car_left(void);
void car_right(void);
void car_spin_left(void);
void car_spin_right(void);
void car_brake(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
