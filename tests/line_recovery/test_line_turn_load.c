/* Real DriveBase + line-load policy. Only physical encoders, battery and PWM
   pins are mocked. These tests validate control decisions, not tyre friction. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "drive_base.h"
#include "line_turn_load.h"
#include "wheel_encoder.h"
#include "battery_monitor.h"
#include "motorPWM.h"
#include "main.h"
#include "line_tracking.h"

static uint32_t tick;
static int32_t counts[4];
static int16_t pins[4];
static WheelEncoderDiagnostics diagnostics;
uint32_t HAL_GetTick(void) { return tick; }
int HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t n) { (void)p; (void)n; return 1; }
void HAL_GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *g) { (void)p; (void)g; }
void WheelEncoder_Start(void) {}
void WheelEncoder_GetCounts(WheelEncoderCounts *c)
{ c->motor1=counts[0]; c->motor2=counts[1]; c->motor3=counts[2]; c->motor4=counts[3]; }
void WheelEncoder_GetDiagnostics(WheelEncoderDiagnostics *d) { *d=diagnostics; }
void BatteryMonitor_Get(BatteryMonitorStatus *b)
{ memset(b,0,sizeof *b); b->valid=1; b->millivolts=7800; }
#define PWM_STUB(n,i) \
  void pwm_motor##n##_forward(int16_t p) { assert(p>=0 && p<=MOTOR_PWM_PERIOD); pins[i]=p; } \
  void pwm_motor##n##_backward(int16_t p) { assert(p>=0 && p<=MOTOR_PWM_PERIOD); pins[i]=(int16_t)-p; }
PWM_STUB(1,0)
PWM_STUB(2,1)
PWM_STUB(3,2)
PWM_STUB(4,3)
static int32_t absolute(int32_t x) { return x<0?-x:x; }
static void reset(void)
{
  memset(counts,0,sizeof counts);
  memset(&diagnostics,0,sizeof diagnostics);
  DriveBase_Init();
}
static void command(int32_t left,int32_t right,uint8_t assist)
{
  if(assist) DriveBase_PrepareLineTurnAssist(left,right);
  DriveBase_SetSideCps(left,right);
}
static void sample(const int32_t delta[4])
{
  unsigned i;
  for(i=0;i<4;++i) counts[i]+=delta[i];
  tick+=20;
  DriveBase_Task(tick);
}
static DriveBaseTelemetry trace(int32_t l,int32_t r,unsigned lag,uint8_t assist)
{
  DriveBaseTelemetry t;
  int32_t delta[4]={l/50,l/50,r/50,r/50};
  unsigned k;
  reset();
  delta[lag]=delta[lag]<0?-1:1; /* slow rolling, never a zero-count stall */
  for(k=0;k<30;++k) { command(l,r,assist); sample(delta); }
  DriveBase_GetTelemetry(&t);
  assert(!t.fault_mask && t.mode==DRIVE_BASE_SPEED);
  assert(t.requested_cps[0]==l && t.requested_cps[1]==l);
  assert(t.requested_cps[2]==r && t.requested_cps[3]==r);
  return t;
}
static void compare_load(int32_t l,int32_t r,unsigned lag)
{
  DriveBaseTelemetry baseline=trace(l,r,lag,0), assisted=trace(l,r,lag,1);
  unsigned i;
  for(i=0;i<4;++i)
  {
    if(i==lag)
      assert(absolute(assisted.output_pwm[i])>absolute(baseline.output_pwm[i])+400);
    else assert(assisted.output_pwm[i]==baseline.output_pwm[i]);
  }
  printf("slow wheel %u (%ld,%ld): PWM %d -> %d, same targets\n",
         lag+1,(long)l,(long)r,baseline.output_pwm[lag],assisted.output_pwm[lag]);
}
static DriveBaseTelemetry trace_line_curve(uint8_t assist)
{
  LineTrackingReading reading={1,0,0,0};
  LineTrackingCommand output;
  DriveBaseTelemetry t;
  int32_t delta[4]={1,28,105,105};
  unsigned i;
  line_tracking_reset(); reset();
  line_tracking_set_smooth_mode(0);
  for(i=0;i<30;++i)
  {
    sample(delta);
    line_tracking_compute(&reading,3000,&output);
    assert(output.valid && output.left_cps==1412 && output.right_cps==5273);
    /* Baseline keeps the exact real line targets but withdraws the claim. */
    if(!assist) DriveBase_PrepareLineTurnAssist(0,0);
    DriveBase_SetSideCps(output.left_cps,output.right_cps);
  }
  DriveBase_GetTelemetry(&t);
  assert(!t.fault_mask);
  return t;
}
int main(void)
{
  LineTurnLoadState s={0};
  DriveBaseTelemetry t, baseline;
  int32_t creep[4]={1,50,-50,-50}, stopped[4]={0}, wrong[4]={-20,50,-50,-50};
  unsigned i;
  /* A single bad sample gets no assistance; the ramp and cap are finite. */
  assert(LineTurnLoad_Update(&s,1,2500,50,20)==0);
  assert(LineTurnLoad_Update(&s,1,2500,50,20)==100);
  for(i=0;i<10;++i) (void)LineTurnLoad_Update(&s,1,2500,50,20);
  assert(s.extra_pwm==600);
  assert(LineTurnLoad_Update(&s,1,2500,2600,20)==0);
  assert(LineTurnLoad_Update(&s,1,2500,-50,20)==0);
  assert(LineTurnLoad_Update(&s,1,350,0,20)==0);
  assert(LineTurnLoad_Update(&s,1,2500,0,100)==0);
  assert(LineTurnLoad_Update(&s,0,2500,0,20)==0);

  compare_load(1412,5273,0); /* normal left curve */
  compare_load(5273,1412,2); /* normal right curve */
  for(i=0;i<4;++i) compare_load(-2500,2500,i);
  for(i=0;i<4;++i) compare_load(2500,-2500,i);
  baseline=trace_line_curve(0); t=trace_line_curve(1);
  assert(t.output_pwm[0]>baseline.output_pwm[0]+400);
  for(i=1;i<4;++i) assert(t.output_pwm[i]==baseline.output_pwm[i]);

  /* Removing the line owner cancels effort even for exactly the same targets. */
  (void)trace(2500,-2500,0,1);
  command(2500,-2500,0); sample(creep);
  DriveBase_GetTelemetry(&t);
  assert(t.output_pwm[0]<3000);
  /* A renewed command can rebuild effort; overspeed removes it immediately. */
  for(i=0;i<10;++i) { command(2500,-2500,1); sample(creep); }
  assert(pins[0]>3300);
  creep[0]=100; command(2500,-2500,1); sample(creep);
  assert(pins[0]<3000);
  creep[0]=1;

  /* An ignored command expires, and a stale preparation cannot be consumed. */
  (void)trace(2500,-2500,0,1);
  for(i=0;i<4;++i) sample(creep);
  assert(pins[0]<3000);
  reset(); DriveBase_PrepareLineTurnAssist(2500,-2500); tick+=21;
  DriveBase_SetSideCps(2500,-2500);
  for(i=0;i<15;++i) sample(creep);
  assert(pins[0]<3000);

  /* Capped/mismatched commands cannot accidentally claim the prepared power. */
  reset();
  for(i=0;i<30;++i)
  {
    DriveBase_PrepareLineTurnAssist(3000,-3000);
    DriveBase_SetSideCps(2500,-2500); sample(creep);
  }
  assert(pins[0]<3000);
  /* Equal targets never opt in (straight/crossing/straight capture). */
  baseline=trace(2500,2500,0,0); t=trace(2500,2500,0,1);
  for(i=0;i<4;++i) assert(t.output_pwm[i]==baseline.output_pwm[i]);

  /* Brake, position and fault ownership survive an attempted line command. */
  (void)trace(2500,-2500,0,1);
  DriveBase_Stop(DRIVE_STOP_BRAKE);
  command(2500,-2500,1); DriveBase_GetTelemetry(&t);
  assert(t.mode==DRIVE_BASE_BRAKING);
  DriveBase_Stop(DRIVE_STOP_COAST);
  for(i=0;i<4;++i) assert(pins[i]==0);
  {
    DrivePositionCommand move={{1000,1000,1000,1000},{2500,2500,2500,2500},1000,12,DRIVE_STOP_COAST};
    assert(DriveBase_StartPositionMove(&move));
    command(2500,-2500,1); DriveBase_GetTelemetry(&t);
    assert(t.mode==DRIVE_BASE_POSITION && t.position_state==DRIVE_POSITION_RUNNING);
    tick+=1001; DriveBase_Task(tick);
    assert(DriveBase_GetFaultMask() & DRIVE_FAULT_TIMEOUT);
  }
  /* No encoder response still latches stall; no automatic clearing/retry. */
  reset();
  for(i=0;i<100;++i) { command(2500,-2500,1); sample(stopped); }
  assert((DriveBase_GetFaultMask() & 0x0fU)!=0);
  tick+=100; DriveBase_Task(tick);
  command(2500,-2500,1);
  for(i=0;i<4;++i) assert(pins[i]==0);
  /* Real reverse/sign and illegal-transition guards still latch their bits. */
  reset();
  for(i=0;i<20;++i) { command(2500,-2500,1); sample(wrong); }
  assert(DriveBase_GetFaultMask() & DRIVE_FAULT_DIRECTION);
  reset();
  for(i=0;i<5;++i)
  {
    diagnostics.illegal_transition_count[1]+=6;
    command(2500,-2500,1); sample(creep);
  }
  assert(DriveBase_GetFaultMask() & DRIVE_FAULT_ENCODER_SIGNAL);

  tick=UINT32_MAX-200;
  (void)trace(2500,-2500,0,1);
  for(i=0;i<4;++i) sample(creep);
  assert(pins[0]<3000);
  puts("PASS: real speed loop, independent lagging wheels, ownership/expiry, overspeed, rails, faults, wrap");
  return 0;
}
