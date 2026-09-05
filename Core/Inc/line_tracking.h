/*
 * 实验七：四路红外循迹接口
 *
 * X1～X4 的电平由指导书定义：低电平表示探头位于黑线上，高电平表示
 * 探头位于白底。这里将 X1/X3 作为中间两路，X2/X4 作为左右外侧路。
 */

#ifndef __LINE_TRACKING_H
#define __LINE_TRACKING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t x1_black;
  uint8_t x2_black;
  uint8_t x3_black;
  uint8_t x4_black;
} LineTrackingReading;

typedef enum
{
  LINE_ACTION_STOP = 0,
  LINE_ACTION_FORWARD,
  LINE_ACTION_LEFT_ADJUST,
  LINE_ACTION_RIGHT_ADJUST,
  LINE_ACTION_LEFT_SHARP,
  LINE_ACTION_RIGHT_SHARP,
  LINE_ACTION_CROSSING,
  LINE_ACTION_SEARCH_LEFT,
  LINE_ACTION_SEARCH_RIGHT
} LineTrackingAction;

typedef struct
{
  int32_t left_cps;
  int32_t right_cps;
  LineTrackingAction action;
  /* This test profile returns side commands for both tracking and recovery. */
  uint8_t valid;
} LineTrackingCommand;

void line_tracking_init(void);
void line_tracking_reset(void);
/* enable=1：尚未见过黑线时允许无黑线直行；enable=0 或已经见过黑线：
   全白时只沿预测方向原地搜索，最多1500 ms；不后退、不反向搜索。
   无方向时停车。任一路见线后停车70 ms，再低速恢复250 ms。 */
void line_tracking_set_no_line_forward(uint8_t enable);
/* Compatibility no-op: this test restores the pre-PD discrete controller. */
void line_tracking_set_smooth_mode(uint8_t enable);
/* Compatibility no-op: use the historical fixed steering parameters. */
void line_tracking_set_turn_gain_percent(uint16_t percent);
LineTrackingReading line_tracking_read(void);
LineTrackingAction line_tracking_compute(const LineTrackingReading *reading,
                                         int16_t base_speed,
                                         LineTrackingCommand *command);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_TRACKING_H */
