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
  /* valid=0 while EncoderTurn owns DriveBase during lost-line search/brake. */
  uint8_t valid;
} LineTrackingCommand;

void line_tracking_init(void);
void line_tracking_reset(void);
/* enable=1：尚未见过黑线时允许无黑线直行；enable=0 或已经见过黑线：
   全白时按近期稳定位置执行四轮差速搜索，单次丢线累计不超过编码器
   估算 85 度且最长 2500 ms（包含捕线重试）。外侧 X2/X4 只预告黑线，
   必须由中间 X1/X3 稳定确认后才退出搜索；方向不明确或耗尽预算则
   停车，需停止后重新选择模式。编码器角度不代表真实车身角度。 */
void line_tracking_set_no_line_forward(uint8_t enable);
/* enable=1: use filtered PD differential steering as the line-position outer
   loop. Wheel-speed feedback remains in DriveBase. */
void line_tracking_set_smooth_mode(uint8_t enable);
/* 100 keeps the normal KEY2 steering gain; 200 doubles KEY1's requested
   steering component before the safe PWM saturation. */
void line_tracking_set_turn_gain_percent(uint16_t percent);
LineTrackingReading line_tracking_read(void);
LineTrackingAction line_tracking_compute(const LineTrackingReading *reading,
                                         int16_t base_speed,
                                         LineTrackingCommand *command);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_TRACKING_H */
