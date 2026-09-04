#include "figure8_encoder.h"

#include "encoder_turn.h"
#include "main.h"

/*
 * 两个相切圆构成 8 字：先绕左侧圆 +360°，再绕右侧圆 -360°。
 * 实测轮距129 mm、轴距129 mm、轮径47 mm。结合已有地面转角
 * 标定，精确转向模型使用约338 mm的滑移补偿等效轮距。
 * 旧的113 mm中心半径小于半等效轮距，会命令内侧轮反转，实车更像
 * 原地扭转而不是8字。改用220 mm中心半径后，内外侧等效半径约
 * 51 mm / 389 mm，四轮都向前，理论整体占地约880 mm x 440 mm。
 */
#define FIG8_LOOP_ANGLE_MDEG          360000L
#define FIG8_CENTER_RADIUS_MM            220L
#define FIG8_MAX_WHEEL_CPS              2357L
#define FIG8_BETWEEN_LOOPS_PAUSE_MS      100U

static Figure8EncoderState figure8_state;
static Figure8EncoderState next_loop_state;
static uint8_t fault_mask;
static uint32_t pause_deadline_ms;

static uint8_t tick_reached(uint32_t now, uint32_t deadline)
{
  return (int32_t)(now - deadline) >= 0 ? 1U : 0U;
}

static void enter_fault(uint8_t mask)
{
  EncoderTurn_Stop();
  fault_mask = mask != 0U ? mask : 0x10U;
  figure8_state = FIGURE8_FAULT;
}

static void start_loop(Figure8EncoderState loop_state)
{
  int32_t angle;
  int32_t radius;

  angle = loop_state == FIGURE8_LEFT_LOOP ?
      FIG8_LOOP_ANGLE_MDEG : -FIG8_LOOP_ANGLE_MDEG;
  radius = loop_state == FIGURE8_LEFT_LOOP ?
      FIG8_CENTER_RADIUS_MM : -FIG8_CENTER_RADIUS_MM;
  if (EncoderTurn_Start(angle,
                        radius,
                        FIG8_MAX_WHEEL_CPS) == 0U)
  {
    enter_fault(0x10U);
    return;
  }
  figure8_state = loop_state;
}

void Figure8Encoder_Init(void)
{
  figure8_state = FIGURE8_IDLE;
  next_loop_state = FIGURE8_LEFT_LOOP;
  fault_mask = 0U;
  pause_deadline_ms = 0U;
}

void Figure8Encoder_Start(void)
{
  EncoderTurn_Stop();
  fault_mask = 0U;
  next_loop_state = FIGURE8_LEFT_LOOP;
  pause_deadline_ms = 0U;
  start_loop(FIGURE8_LEFT_LOOP);
}

void Figure8Encoder_Stop(void)
{
  EncoderTurn_Stop();
  figure8_state = FIGURE8_IDLE;
  next_loop_state = FIGURE8_LEFT_LOOP;
  pause_deadline_ms = 0U;
}

void Figure8Encoder_Task(void)
{
  EncoderTurnState turn_state;
  uint32_t now = HAL_GetTick();

  if (figure8_state == FIGURE8_LEFT_LOOP ||
      figure8_state == FIGURE8_RIGHT_LOOP)
  {
    Figure8EncoderState completed_loop = figure8_state;

    EncoderTurn_Task();
    turn_state = EncoderTurn_GetState();
    if (turn_state == ENCODER_TURN_FAULT)
    {
      enter_fault(EncoderTurn_GetFaultMask());
      return;
    }
    if (turn_state == ENCODER_TURN_DONE)
    {
      EncoderTurn_Stop();
      if (completed_loop == FIGURE8_LEFT_LOOP)
      {
        next_loop_state = FIGURE8_RIGHT_LOOP;
        figure8_state = FIGURE8_PAUSE;
        pause_deadline_ms = now + FIG8_BETWEEN_LOOPS_PAUSE_MS;
      }
      else
      {
        /* 离线测试只运行一个完整8字，右环完成后锁存停车。 */
        figure8_state = FIGURE8_DONE;
        pause_deadline_ms = 0U;
      }
    }
    return;
  }

  if (figure8_state == FIGURE8_PAUSE &&
      tick_reached(now, pause_deadline_ms))
  {
    start_loop(next_loop_state);
  }
}

Figure8EncoderState Figure8Encoder_GetState(void)
{
  return figure8_state;
}

uint8_t Figure8Encoder_GetFaultMask(void)
{
  return fault_mask;
}
