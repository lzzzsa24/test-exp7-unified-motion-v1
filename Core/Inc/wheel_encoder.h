/* 四路 AB 相霍尔编码器采集。
 *
 * YB-DSF01-V1.1 原理图网络：
 *   M1: H1A=PD12, H1B=PD13
 *   M2: H2A=PA15, H2B=PB3
 *   M3: H3A=PA0,  H3B=PA1
 *   M4: H4A=PB4,  H4B=PB5
 *
 * PA15/PB3/PB4 默认属于 JTAG，工程在 HAL_MspInit 中保留 SWD、关闭
 * JTAG，从而允许它们作为普通输入；四路均使用完整 AB 正交解码，
 * 编码器采样不占用 EXTI。
 */

#ifndef __WHEEL_ENCODER_H
#define __WHEEL_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  int32_t motor1;
  int32_t motor2;
  int32_t motor3;
  int32_t motor4;
} WheelEncoderCounts;

typedef struct
{
  uint32_t legal_transition_count[4];
  uint32_t illegal_transition_count[4];
} WheelEncoderDiagnostics;

void WheelEncoder_Init(void);
void WheelEncoder_Start(void);
/* Compatibility no-op in unified-motion: sampling remains persistent. */
void WheelEncoder_Stop(void);
/* Reserved for startup/diagnostics; motion modules use count snapshots. */
void WheelEncoder_Reset(void);
void WheelEncoder_GetCounts(WheelEncoderCounts *counts);
void WheelEncoder_GetDiagnostics(WheelEncoderDiagnostics *diagnostics);
void WheelEncoder_ResetDiagnostics(void);
uint8_t WheelEncoder_IsRunning(void);

/* 由唯一的 TIM6_IRQHandler 调用。 */
void WheelEncoder_TIM6_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __WHEEL_ENCODER_H */
