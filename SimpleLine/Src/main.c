#include "main.h"
#include "board.h"
#include "motorPWM.h"

static void clock_init(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clock = {0};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  osc.HSIState = RCC_HSI_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
  clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clock.APB1CLKDivider = RCC_HCLK_DIV2;
  clock.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

int main(void)
{
  LineFollower line;
  uint32_t last_control, last_report;
  HAL_Init();
  clock_init();
  motor_pwm_init();
  Board_Init();
  last_control = last_report = HAL_GetTick();
  Line_Init(&line, last_control);
  Board_WatchdogStart();
  Board_Report(&line);
  for (;;) {
    uint32_t now = HAL_GetTick();
    if (now - last_control >= LINE_PERIOD_MS) {
      uint8_t events = Board_Inputs(now);
      LineMode old_mode = line.mode;
      if (events & INPUT_STOP) Line_Stop(&line, LINE_USER_STOP);
      else if (events & INPUT_START) Line_Start(&line, now);
      Line_Step(&line, Board_ReadLine(), now);
      motor_pwm_set_sides(line.left_pwm, line.right_pwm, now);
      last_control = now;
      /* Only a completed control pass feeds the hardware watchdog. */
      Board_WatchdogFeed();
      Board_Indicators(&line, now);
      if (events || line.mode != old_mode || now - last_report >= 200U) {
        Board_Report(&line);
        last_report = now;
      }
    }
    __WFI();
  }
}

void SysTick_Handler(void) { HAL_IncTick(); }

void Error_Handler(void)
{
  __disable_irq();
  motor_pwm_emergency_stop();
  /* If IWDG was enabled it resets into STOP; otherwise outputs stay off. */
  for (;;) { }
}

void NMI_Handler(void) { Error_Handler(); }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
