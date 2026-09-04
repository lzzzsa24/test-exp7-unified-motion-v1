/* 实验七：STM32 USART2 与 K210 的视觉指令接口 */

#ifndef __VISION_UART_H
#define __VISION_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  VISION_CMD_NONE = 0,
  VISION_CMD_GREEN_LIGHT,
  VISION_CMD_SCHOOL,
  VISION_CMD_WALK,
  VISION_CMD_RIGHT,
  VISION_CMD_LEFT,
  VISION_CMD_FREE_SPEED,
  VISION_CMD_LIMIT_SPEED,
  VISION_CMD_HORN,
  VISION_CMD_GARAGE_ONE,
  VISION_CMD_GARAGE_TWO,
  VISION_CMD_CHUKU_TRACK_LINE,
  VISION_CMD_STOP
} VisionCommand;

void vision_uart_init(void);
void vision_uart_poll(void);
VisionCommand vision_uart_take_event(void);
const char *vision_command_name(VisionCommand command);

/* 非阻塞排队一帧运动估计：M,valid,distance_mm,speed_mm_s,travel_mm。 */
void vision_uart_queue_motion_telemetry(uint8_t valid,
                                        uint16_t distance_mm,
                                        int16_t speed_mm_s,
                                        int32_t travel_mm);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_UART_H */
