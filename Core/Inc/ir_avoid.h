/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ir_avoid.h
  * @brief   实验五：双路红外避障传感与状态指示接口
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __IR_AVOID_H
#define __IR_AVOID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 障碍物使接收端电压下降。上电时会以“前方无遮挡”的读数为基线，
   再为左右传感器分别计算阈值，避免不同底板使用同一个固定 ADC 值。 */
#define IR_AVOID_TRIGGER_PERCENT          70U
#define IR_AVOID_HYSTERESIS_PERCENT        5U
#define IR_AVOID_MIN_HYSTERESIS           20U
#define IR_AVOID_MIN_VALID_BASELINE      200U
#define IR_AVOID_SAMPLE_COUNT              8U
#define IR_AVOID_CALIBRATION_COUNT        32U

typedef struct
{
  uint16_t left_adc;
  uint16_t right_adc;
  bool left_obstacle;
  bool right_obstacle;
} IrAvoidReading;

void ir_avoid_init(void);
/* 退出轨迹模式后防御性恢复红外发射和左右 RGB 的 GPIO 方向，
   不清除已经完成的红外标定值。编码器不再占用 PE2~PE5。 */
void ir_avoid_resume_io(void);
bool ir_avoid_calibrate(void);
void ir_avoid_set_enabled(bool enabled);
bool ir_avoid_is_enabled(void);
uint16_t ir_avoid_get_left_threshold(void);
uint16_t ir_avoid_get_right_threshold(void);
uint16_t ir_avoid_get_left_hysteresis(void);
uint16_t ir_avoid_get_right_hysteresis(void);
IrAvoidReading ir_avoid_read(void);
void ir_avoid_show_status(const IrAvoidReading *reading);

#ifdef __cplusplus
}
#endif

#endif /* __IR_AVOID_H */
