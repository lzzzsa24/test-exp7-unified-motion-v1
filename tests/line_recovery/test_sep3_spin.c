#include <assert.h>
#include <stdio.h>
#include "main.h"
#include "drive_base.h"
#include "line_tracking.h"
static uint32_t tick;
static uint8_t fault;
uint32_t HAL_GetTick(void) { return tick; }
int HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin) { (void)p; (void)pin; return 1; }
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g) { (void)p; (void)g; }
void DriveBase_Stop(DriveStopMode mode) { (void)mode; }
uint8_t DriveBase_GetFaultMask(void) { return fault; }
/* Identity conversion makes assertions describe the historical PWM request. */
int32_t DriveBase_EquivalentCpsFromPwm(int16_t pwm) { return pwm; }
static LineTrackingCommand step(unsigned mask, uint32_t time)
{
  LineTrackingReading r = {(uint8_t)(mask&1U), (uint8_t)((mask>>1)&1U),
                          (uint8_t)((mask>>2)&1U), (uint8_t)((mask>>3)&1U)};
  LineTrackingCommand c;
  tick = time;
  line_tracking_compute(&r,2600,&c);
  assert(c.valid);
  /* Historical recovery must never request both sides backwards. */
  assert(c.left_cps >= 0 || c.right_cps >= 0);
  return c;
}
static void stopped(LineTrackingCommand c) { assert(c.left_cps==0 && c.right_cps==0); }
static void begin(int right, uint32_t base)
{
  tick=base; fault=0; line_tracking_reset(); line_tracking_set_no_line_forward(0);
  step(5,base);
  step(right ? 4U : 1U,base+10);
  stopped(step(0,base+20));
  stopped(step(0,base+89));
  stopped(step(0,base+90));
}
int main(void)
{
  int right;
  unsigned mask;
  for(right=0;right<2;right++)
  {
    LineTrackingCommand c;
    begin(right,0);
    c=step(0,91);
    assert(c.left_cps==(right?2700:-2700));
    assert(c.right_cps==-c.left_cps);
    c=step(0,1589); assert(c.left_cps==(right?2700:-2700));
    stopped(step(0,1590)); stopped(step(0,4000));
    for(mask=1;mask<16;mask++)
    {
      begin(right,0); step(0,91);
      stopped(step(mask,100)); /* Includes outer-only detection. */
    }
  }
  begin(0,0); step(0,91); stopped(step(5,100));
  stopped(step(5,169));
  { LineTrackingCommand c=step(5,170); assert(c.left_cps==2200 && c.right_cps==2700); }
  { LineTrackingCommand c=step(5,420); assert(c.left_cps==2500 && c.right_cps==2500); }
  tick=0; line_tracking_reset(); stopped(step(0,0)); stopped(step(0,10000));
  line_tracking_set_no_line_forward(1);
  { LineTrackingCommand c=step(0,10001); assert(c.left_cps==2600 && c.right_cps==2600); }
  begin(1,0); step(0,91); fault=1; stopped(step(0,100)); fault=0;
  stopped(step(0,200)); /* Fault cleared history, so no restart from stale hint. */
  begin(1,0); step(0,91);
  { LineTrackingCommand c; LineTrackingReading r={0}; line_tracking_compute(&r,0,&c); stopped(c); }
  stopped(step(0,200));
  begin(0,0xFFFFFF80U);
  { LineTrackingCommand c=step(0,0xFFFFFFDBU); assert(c.left_cps==-2700); }
  stopped(step(0,0xFFFFFF80U+1590U));
  puts("PASS: direction, 1500ms timeout, 15 reacquire masks, settle, no reverse, unknown, KEY1, fault, reset, rollover");
  return 0;
}
