/*
 * 单前向超声波的墙面相对运动估计。
 *
 * 该模块只表示传感器声束方向上、相对同一静止反射面的距离变化；
 * 不能代替轮速编码器，也不能给出转弯或任意场地中的绝对里程。
 */

#ifndef __ULTRASONIC_MOTION_H
#define __ULTRASONIC_MOTION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t valid;
  uint16_t target_distance_mm;
  int16_t closing_speed_mm_s;
  int32_t relative_displacement_mm;
  uint32_t last_update_ms;
} UltrasonicMotionEstimate;

void UltrasonicMotion_Init(void);
void UltrasonicMotion_Reset(void);
void UltrasonicMotion_Update(uint16_t distance_mm, uint32_t now_ms);
void UltrasonicMotion_NoteInvalid(uint32_t now_ms);
void UltrasonicMotion_Task(uint32_t now_ms);
void UltrasonicMotion_Get(UltrasonicMotionEstimate *estimate);

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_MOTION_H */
