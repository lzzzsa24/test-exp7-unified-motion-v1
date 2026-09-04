#include "square_encoder.h"

#include "encoder_straight.h"
#include "encoder_turn.h"
#include "main.h"

#define SQUARE_SIDE_LENGTH_MM          400U
#define SQUARE_STRAIGHT_CPS           2357L
#define SQUARE_TURN_ANGLE_MDEG       90000L
#define SQUARE_TURN_CPS               1200L
#define SQUARE_PHASE_PAUSE_MS          150U

static SquareEncoderState square_state;
static uint8_t side_number;
static uint8_t fault_mask;
static uint32_t pause_deadline_ms;

static uint8_t tick_reached(uint32_t now, uint32_t deadline)
{
  return (int32_t)(now - deadline) >= 0 ? 1U : 0U;
}

static void enter_fault(uint8_t mask)
{
  EncoderStraight_Stop();
  EncoderTurn_Stop();
  fault_mask = mask != 0U ? mask : 0x10U;
  square_state = SQUARE_FAULT;
}

static void start_drive(void)
{
  if (EncoderStraight_Start(SQUARE_SIDE_LENGTH_MM,
                            SQUARE_STRAIGHT_CPS) == 0U)
  {
    enter_fault(0x10U);
    return;
  }
  square_state = SQUARE_DRIVE;
}

static void start_turn(void)
{
  if (EncoderTurn_Start(SQUARE_TURN_ANGLE_MDEG, 0L,
                        SQUARE_TURN_CPS) == 0U)
  {
    enter_fault(0x10U);
    return;
  }
  square_state = SQUARE_TURN;
}

void SquareEncoder_Init(void)
{
  square_state = SQUARE_IDLE;
  side_number = 1U;
  fault_mask = 0U;
  pause_deadline_ms = 0U;
}

void SquareEncoder_Start(void)
{
  EncoderStraight_Stop();
  EncoderTurn_Stop();
  side_number = 1U;
  fault_mask = 0U;
  pause_deadline_ms = 0U;
  start_drive();
}

void SquareEncoder_Stop(void)
{
  EncoderStraight_Stop();
  EncoderTurn_Stop();
  square_state = SQUARE_IDLE;
  side_number = 1U;
  pause_deadline_ms = 0U;
}

void SquareEncoder_Task(void)
{
  uint32_t now = HAL_GetTick();

  if (square_state == SQUARE_DRIVE)
  {
    EncoderStraightState state;

    EncoderStraight_Task();
    state = EncoderStraight_GetState();
    if (state == ENCODER_STRAIGHT_FAULT)
    {
      enter_fault(EncoderStraight_GetFaultMask());
    }
    else if (state == ENCODER_STRAIGHT_DONE)
    {
      EncoderStraight_Stop();
      square_state = SQUARE_PAUSE_BEFORE_TURN;
      pause_deadline_ms = now + SQUARE_PHASE_PAUSE_MS;
    }
    return;
  }

  if (square_state == SQUARE_PAUSE_BEFORE_TURN)
  {
    if (tick_reached(now, pause_deadline_ms))
    {
      start_turn();
    }
    return;
  }

  if (square_state == SQUARE_TURN)
  {
    EncoderTurnState state;

    EncoderTurn_Task();
    state = EncoderTurn_GetState();
    if (state == ENCODER_TURN_FAULT)
    {
      enter_fault(EncoderTurn_GetFaultMask());
    }
    else if (state == ENCODER_TURN_DONE)
    {
      EncoderTurn_Stop();
      if (side_number >= 4U)
      {
        square_state = SQUARE_DONE;
      }
      else
      {
        ++side_number;
        square_state = SQUARE_PAUSE_BEFORE_DRIVE;
        pause_deadline_ms = now + SQUARE_PHASE_PAUSE_MS;
      }
    }
    return;
  }

  if (square_state == SQUARE_PAUSE_BEFORE_DRIVE &&
      tick_reached(now, pause_deadline_ms))
  {
    start_drive();
  }
}

SquareEncoderState SquareEncoder_GetState(void)
{
  return square_state;
}

uint8_t SquareEncoder_GetFaultMask(void)
{
  return fault_mask;
}

uint8_t SquareEncoder_GetSide(void)
{
  return side_number;
}
