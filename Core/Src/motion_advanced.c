/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motion_advanced.c
  * @brief   实验三：基于 PWM 的差速运动控制
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motion_advanced.h"
#include "drive_base.h"
#include "motorPWM.h"

/* 默认约定：M1、M2 为左侧电机，M3、M4 为右侧电机。 */

static int16_t forward_speed_limit = MOTOR_PWM_PERIOD;

static int16_t clamp_positive_speed(int16_t speed)
{
  if (speed <= 0)
  {
    return 0;
  }

  if (speed > (int16_t)MOTOR_PWM_PERIOD)
  {
    return (int16_t)MOTOR_PWM_PERIOD;
  }

  return speed;
}

static void limit_forward_pair(int16_t *left_speed, int16_t *right_speed)
{
  int16_t left = clamp_positive_speed(*left_speed);
  int16_t right = clamp_positive_speed(*right_speed);
  int16_t maximum = left > right ? left : right;

  /* 先限制到硬件 PWM 范围，再将两侧按同一比例缩放到超声波发布的
     安全上限。不能预先分别截到 forward_speed_limit，否则较大的
     转向差速会被压平，甚至把转弯错误地变成直行。 */
  if (maximum > forward_speed_limit && maximum > 0)
  {
    left = (int16_t)(((int32_t)left * forward_speed_limit) / maximum);
    right = (int16_t)(((int32_t)right * forward_speed_limit) / maximum);
  }

  *left_speed = left;
  *right_speed = right;
}

void advanced_drive_cps(int32_t left_cps, int32_t right_cps)
{
  if (left_cps >= 0L && right_cps >= 0L)
  {
    int32_t limit_cps = DriveBase_EquivalentCpsFromPwm(forward_speed_limit);
    int32_t maximum = left_cps > right_cps ? left_cps : right_cps;

    if (maximum > limit_cps && maximum > 0L)
    {
      left_cps = (left_cps * limit_cps) / maximum;
      right_cps = (right_cps * limit_cps) / maximum;
    }
  }
  DriveBase_SetSideCps(left_cps, right_cps);
}

void advanced_drive_forward(int16_t left_speed, int16_t right_speed)
{
  limit_forward_pair(&left_speed, &right_speed);
  advanced_drive_cps(DriveBase_EquivalentCpsFromPwm(left_speed),
                     DriveBase_EquivalentCpsFromPwm(right_speed));
}

void advanced_drive_backward(int16_t left_speed, int16_t right_speed)
{
  advanced_drive_cps(DriveBase_EquivalentCpsFromPwm(-left_speed),
                     DriveBase_EquivalentCpsFromPwm(-right_speed));
}

/* 左侧慢、右侧快，车辆向左弯。 */
void advanced_turn_left(int16_t inner_speed, int16_t outer_speed)
{
  advanced_drive_forward(inner_speed, outer_speed);
}

/* 左侧快、右侧慢，车辆向右弯。 */
void advanced_turn_right(int16_t inner_speed, int16_t outer_speed)
{
  advanced_drive_forward(outer_speed, inner_speed);
}

void advanced_spin_left(int16_t speed)
{
  /* 原地旋转是红外/视觉的避障动作，不属于“前进限速”。如果在
     超声波开阔区降级时也把它压到慢速，车辆会表现为只停不转。 */
  advanced_drive_cps(DriveBase_EquivalentCpsFromPwm(-speed),
                     DriveBase_EquivalentCpsFromPwm(speed));
}

void advanced_spin_right(int16_t speed)
{
  /* 同上：保留实验五的原地转向力度，避免安全层误把避障动作变成
     低速爬行。 */
  advanced_drive_cps(DriveBase_EquivalentCpsFromPwm(speed),
                     DriveBase_EquivalentCpsFromPwm(-speed));
}

void advanced_stop(void)
{
  /* PWM 输入全部为 0，对应驱动器 0/0 的滑行状态。 */
  DriveBase_Stop(DRIVE_STOP_COAST);
}

void advanced_set_forward_speed_limit(int16_t max_speed)
{
  if (max_speed <= 0)
  {
    forward_speed_limit = 0;
  }
  else if (max_speed > (int16_t)MOTOR_PWM_PERIOD)
  {
    forward_speed_limit = (int16_t)MOTOR_PWM_PERIOD;
  }
  else
  {
    forward_speed_limit = max_speed;
  }
}
