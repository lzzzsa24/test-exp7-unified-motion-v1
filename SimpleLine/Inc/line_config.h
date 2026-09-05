#ifndef SIMPLE_LINE_CONFIG_H
#define SIMPLE_LINE_CONFIG_H

/* Logical PWM, 0..3599. Initial values require a ground test. */
#define LINE_PERIOD_MS           5U
#define LINE_FILTER_SAMPLES      2U
#define LINE_STRAIGHT_PWM     2400
#define LINE_SLOW_PWM         2200
#define LINE_OUTER_PWM        2600
#define LINE_TURN_PWM         2700
#define LINE_GAP_MS             60U
#define LINE_HINT_MAX_AGE_MS    250U
#define LINE_MANEUVER_MS        900U
#define LINE_CENTER_CONFIRM_MS   30U
#define LINE_WIDE_MS            300U
#define LINE_LOOP_TIMEOUT_MS     50U
#define LINE_SENSOR_TIMEOUT_MS   50U
#define MOTOR_REVERSE_COAST_MS   10U

#endif
