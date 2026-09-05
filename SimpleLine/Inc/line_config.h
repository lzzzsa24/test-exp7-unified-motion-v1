#ifndef SIMPLE_LINE_CONFIG_H
#define SIMPLE_LINE_CONFIG_H

/* Logical PWM, 0..3599. Initial values require a ground test. */
#define LINE_PERIOD_MS           5U
#define LINE_FILTER_SAMPLES      2U
#define LINE_STRAIGHT_PWM     2400
#define LINE_SLOW_PWM         2200
#define LINE_OUTER_PWM        2600
#define LINE_TURN_PWM         2700
/* No search/turn/line/loop deadline: only operator commands stop driving.
 * With no previous left/right evidence, search left (-1); +1 selects right. */
#define LINE_DEFAULT_SEARCH_DIRECTION (-1)
#define MOTOR_REVERSE_COAST_MS   10U

#endif
