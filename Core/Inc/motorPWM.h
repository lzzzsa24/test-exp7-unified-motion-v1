#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H
#include <stdint.h>
#define MOTOR_PWM_PERIOD 3599U
void motor_pwm_init(void);
/* Positive means vehicle-forward. M1/M2 receive left, M3/M4 receive right. */
void motor_pwm_set_sides(int16_t left, int16_t right, uint32_t now);
void pwm_motor_stop(void);
int16_t motor_pwm_get_side(unsigned side);
void motor_pwm_emergency_stop(void);
#endif
