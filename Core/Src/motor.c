/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.c
  * @brief   实验二：四路直流电机 GPIO 运动控制
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motor.h"
#include "main.h"

/*
 * AT8236 的 IN1/IN2 逻辑：
 *   1/0：正转，0/1：反转，0/0：滑行/休眠，1/1：刹车。
 * 这里把“正转/反转”定义集中放在本文件，若某个车轮安装方向相反，
 * 只需交换该电机的 forward/backward 两个输出即可。
 */

static void motor_write_pair(GPIO_TypeDef *port,
                             uint16_t pin_a,
                             uint16_t pin_b,
                             GPIO_PinState state_a,
                             GPIO_PinState state_b)
{
  HAL_GPIO_WritePin(port, pin_a, state_a);
  HAL_GPIO_WritePin(port, pin_b, state_b);
}

static void motor_all(GPIO_PinState state_a, GPIO_PinState state_b)
{
  motor_write_pair(M1A_GPIO_Port, M1A_Pin, M1B_Pin, state_a, state_b);
  motor_write_pair(M2A_GPIO_Port, M2A_Pin, M2B_Pin, state_a, state_b);
  motor_write_pair(M3A_GPIO_Port, M3A_Pin, M3B_Pin, state_a, state_b);
  motor_write_pair(M4A_GPIO_Port, M4A_Pin, M4B_Pin, state_a, state_b);
}

void motor_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* 上电先让四路驱动处于 0/0，避免初始化瞬间转动。 */
  HAL_GPIO_WritePin(GPIOC, M1A_Pin | M1B_Pin | M2A_Pin | M2B_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, M3A_Pin | M3B_Pin | M4A_Pin | M4B_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = M1A_Pin | M1B_Pin | M2A_Pin | M2B_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = M3A_Pin | M3B_Pin | M4A_Pin | M4B_Pin;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  motor_stop();
}

void motor1_forward(void)
{
  motor_write_pair(M1A_GPIO_Port, M1A_Pin, M1B_Pin, GPIO_PIN_SET, GPIO_PIN_RESET);
}

void motor1_backward(void)
{
  motor_write_pair(M1A_GPIO_Port, M1A_Pin, M1B_Pin, GPIO_PIN_RESET, GPIO_PIN_SET);
}

void motor2_forward(void)
{
  motor_write_pair(M2A_GPIO_Port, M2A_Pin, M2B_Pin, GPIO_PIN_SET, GPIO_PIN_RESET);
}

void motor2_backward(void)
{
  motor_write_pair(M2A_GPIO_Port, M2A_Pin, M2B_Pin, GPIO_PIN_RESET, GPIO_PIN_SET);
}

void motor3_forward(void)
{
  motor_write_pair(M3A_GPIO_Port, M3A_Pin, M3B_Pin, GPIO_PIN_SET, GPIO_PIN_RESET);
}

void motor3_backward(void)
{
  motor_write_pair(M3A_GPIO_Port, M3A_Pin, M3B_Pin, GPIO_PIN_RESET, GPIO_PIN_SET);
}

void motor4_forward(void)
{
  motor_write_pair(M4A_GPIO_Port, M4A_Pin, M4B_Pin, GPIO_PIN_SET, GPIO_PIN_RESET);
}

void motor4_backward(void)
{
  motor_write_pair(M4A_GPIO_Port, M4A_Pin, M4B_Pin, GPIO_PIN_RESET, GPIO_PIN_SET);
}

/* 0/0：滑行/休眠，和指导书中的 motor_stop 示例一致。 */
void motor_stop(void)
{
  motor_all(GPIO_PIN_RESET, GPIO_PIN_RESET);
}

/* 四路 1/1：主动刹车。 */
void car_brake(void)
{
  motor_all(GPIO_PIN_SET, GPIO_PIN_SET);
}

/* 默认按 M1、M2 为左侧，M3、M4 为右侧组织四轮小车。 */
void car_forward(void)
{
  motor1_forward();
  motor2_forward();
  motor3_forward();
  motor4_forward();
}

void car_backward(void)
{
  motor1_backward();
  motor2_backward();
  motor3_backward();
  motor4_backward();
}

/* 左转：左侧停止，右侧前进。 */
void car_left(void)
{
  motor_write_pair(M1A_GPIO_Port, M1A_Pin, M1B_Pin, GPIO_PIN_RESET, GPIO_PIN_RESET);
  motor_write_pair(M2A_GPIO_Port, M2A_Pin, M2B_Pin, GPIO_PIN_RESET, GPIO_PIN_RESET);
  motor3_forward();
  motor4_forward();
}

/* 右转：右侧停止，左侧前进。 */
void car_right(void)
{
  motor1_forward();
  motor2_forward();
  motor_write_pair(M3A_GPIO_Port, M3A_Pin, M3B_Pin, GPIO_PIN_RESET, GPIO_PIN_RESET);
  motor_write_pair(M4A_GPIO_Port, M4A_Pin, M4B_Pin, GPIO_PIN_RESET, GPIO_PIN_RESET);
}

/* 原地左转：左侧后退，右侧前进。 */
void car_spin_left(void)
{
  motor1_backward();
  motor2_backward();
  motor3_forward();
  motor4_forward();
}

/* 原地右转：左侧前进，右侧后退。 */
void car_spin_right(void)
{
  motor1_forward();
  motor2_forward();
  motor3_backward();
  motor4_backward();
}
