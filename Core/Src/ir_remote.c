#include "ir_remote.h"

#include "main.h"

/* Yahboom 21 键遥控器按 NEC LSB-first 还原后的数字键命令。 */
#define IR_COMMAND_NUMBER_1          0x10U
#define IR_COMMAND_NUMBER_2          0x11U
#define IR_COMMAND_NUMBER_3          0x12U
#define IR_COMMAND_NUMBER_4          0x14U
#define IR_COMMAND_NUMBER_0          0x0DU
/* Yahboom remote direction-pad centre / buzzer button.  Its 0x05 command
   belongs to the same map as number 1=0x10 and number 2=0x11 above. */
#define IR_COMMAND_CENTER_BUZZER     0x05U

#define IR_LEADER_MIN_US            12500U
#define IR_LEADER_MAX_US            14500U
#define IR_REPEAT_MIN_US            10200U
#define IR_REPEAT_MAX_US            12100U
#define IR_BIT_ZERO_MIN_US            800U
#define IR_BIT_ZERO_MAX_US           1500U
#define IR_BIT_ONE_MIN_US            1750U
#define IR_BIT_ONE_MAX_US            2800U

static volatile uint32_t ir_last_falling_cycle;
static volatile uint32_t ir_frame;
static volatile uint8_t ir_bit_index;
static volatile uint8_t ir_receiving;
static volatile uint8_t ir_pending_command;
static volatile uint8_t ir_pending_valid;
static volatile uint8_t ir_last_command;
static uint32_t ir_cycles_per_us;

void IrRemote_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitStruct.Pin = IR_REMOTE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_REMOTE_GPIO_Port, &GPIO_InitStruct);
  __HAL_GPIO_EXTI_CLEAR_IT(IR_REMOTE_Pin);

  /* DWT 不占用 TIM1/TIM8 电机、TIM2 超声波或 TIM6 编码器。 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  ir_cycles_per_us = SystemCoreClock / 1000000U;
  if (ir_cycles_per_us == 0U)
  {
    ir_cycles_per_us = 1U;
  }

  ir_last_falling_cycle = DWT->CYCCNT;
  ir_frame = 0U;
  ir_bit_index = 0U;
  ir_receiving = 0U;
  ir_pending_command = 0U;
  ir_pending_valid = 0U;
  ir_last_command = 0U;

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1U, 0U);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

uint8_t IrRemote_TakeVirtualKey(void)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t command = 0U;

  __disable_irq();
  if (ir_pending_valid != 0U)
  {
    command = ir_pending_command;
    ir_pending_valid = 0U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  switch (command)
  {
    case IR_COMMAND_NUMBER_1:
      return IR_REMOTE_VIRTUAL_KEY1;
    case IR_COMMAND_NUMBER_2:
      return IR_REMOTE_VIRTUAL_KEY2;
    case IR_COMMAND_NUMBER_3:
      return IR_REMOTE_VIRTUAL_KEY3;
    case IR_COMMAND_NUMBER_4:
      return IR_REMOTE_VIRTUAL_KEY4;
    case IR_COMMAND_NUMBER_0:
      return IR_REMOTE_VIRTUAL_STOP;
    case IR_COMMAND_CENTER_BUZZER:
      return IR_REMOTE_VIRTUAL_AUDIO_ONCE;
    default:
      return IR_REMOTE_VIRTUAL_KEY_NONE;
  }
}

uint8_t IrRemote_GetLastCommand(void)
{
  return ir_last_command;
}

void IrRemote_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now_cycle;
  uint32_t interval_us;
  uint8_t address;
  uint8_t address_inverse;
  uint8_t command;
  uint8_t command_inverse;

  if (GPIO_Pin != IR_REMOTE_Pin)
  {
    return;
  }

  now_cycle = DWT->CYCCNT;
  interval_us = (now_cycle - ir_last_falling_cycle) / ir_cycles_per_us;
  ir_last_falling_cycle = now_cycle;

  if (interval_us >= IR_LEADER_MIN_US && interval_us <= IR_LEADER_MAX_US)
  {
    ir_frame = 0U;
    ir_bit_index = 0U;
    ir_receiving = 1U;
    return;
  }

  if (interval_us >= IR_REPEAT_MIN_US && interval_us <= IR_REPEAT_MAX_US)
  {
    ir_receiving = 0U;
    return;
  }

  if (ir_receiving == 0U)
  {
    return;
  }

  if (interval_us >= IR_BIT_ZERO_MIN_US && interval_us <= IR_BIT_ZERO_MAX_US)
  {
    /* 当前位为 0。 */
  }
  else if (interval_us >= IR_BIT_ONE_MIN_US &&
           interval_us <= IR_BIT_ONE_MAX_US)
  {
    ir_frame |= (1UL << ir_bit_index);
  }
  else
  {
    ir_receiving = 0U;
    ir_bit_index = 0U;
    return;
  }

  ++ir_bit_index;
  if (ir_bit_index < 32U)
  {
    return;
  }

  ir_receiving = 0U;
  ir_bit_index = 0U;
  address = (uint8_t)(ir_frame & 0xFFU);
  address_inverse = (uint8_t)((ir_frame >> 8U) & 0xFFU);
  command = (uint8_t)((ir_frame >> 16U) & 0xFFU);
  command_inverse = (uint8_t)((ir_frame >> 24U) & 0xFFU);

  if (((uint8_t)(address ^ address_inverse) == 0xFFU) &&
      ((uint8_t)(command ^ command_inverse) == 0xFFU))
  {
    ir_last_command = command;
    ir_pending_command = command;
    ir_pending_valid = 1U;
  }
}
