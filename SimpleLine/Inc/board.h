#ifndef SIMPLE_LINE_BOARD_H
#define SIMPLE_LINE_BOARD_H
#include "line_follow.h"
#include "operator_input.h"
void Board_Init(void);
uint8_t Board_ReadLine(void);
uint8_t Board_Inputs(uint32_t now);
void Board_Indicators(const LineFollower *line, uint32_t now);
void Board_Report(const LineFollower *line);
#endif
