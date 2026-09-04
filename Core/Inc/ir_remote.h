#ifndef __IR_REMOTE_H
#define __IR_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define IR_REMOTE_VIRTUAL_KEY_NONE  0U
#define IR_REMOTE_VIRTUAL_KEY1      1U
#define IR_REMOTE_VIRTUAL_KEY2      2U
#define IR_REMOTE_VIRTUAL_KEY3      3U
#define IR_REMOTE_VIRTUAL_STOP      4U
#define IR_REMOTE_VIRTUAL_KEY4      5U

/* 初始化板载 HS0038B：PG11 下降沿 EXTI，DWT 周期计数器负责微秒计时。 */
void IrRemote_Init(void);

/* 从主循环读取一次按键事件。长按产生的 NEC repeat 帧不会重复上报。 */
uint8_t IrRemote_TakeVirtualKey(void);

/* 最近一次通过 NEC 反码校验的原始命令字节。 */
uint8_t IrRemote_GetLastCommand(void);

/* 在工程唯一的 HAL_GPIO_EXTI_Callback() 中转发 GPIO_Pin。 */
void IrRemote_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* __IR_REMOTE_H */
