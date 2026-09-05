/* Real line + recovery + frozen phrase. HAL and wheel motion are simulated. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "line_tracking.h"
#include "line_recovery.h"
#include "line_search_model.h"
#include "drive_base.h"
#include "buzzer_phrase_40077493715.h"
static uint32_t tick, brake_started;
static DriveBaseTelemetry telemetry;
static LineTrackingCommand output;
static GPIO_PinState buzzer;
static unsigned attacks, spins, reversals;
static int32_t previous_spin;
uint32_t HAL_GetTick(void) { return tick; }
int HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t n) { (void)p; (void)n; return 1; }
void HAL_GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *g) { (void)p; (void)g; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t n,GPIO_PinState s)
{ assert(p==Buzzer_GPIO_Port && n==Buzzer_Pin); if(s && !buzzer) ++attacks; buzzer=s; }
int32_t DriveBase_EquivalentCpsFromPwm(int16_t p) { return p; }
void DriveBase_PrepareLineTurnAssist(int32_t l,int32_t r) { (void)l; (void)r; }
void DriveBase_SetLineFaultObservation(uint8_t e,uint8_t s,uint8_t r)
{ (void)e; (void)s; (void)r; }
void DriveBase_GetTelemetry(DriveBaseTelemetry *t) { *t=telemetry; }
uint8_t DriveBase_GetFaultMask(void) { return telemetry.fault_mask; }
void DriveBase_Task(uint32_t now)
{ if(telemetry.mode==DRIVE_BASE_BRAKING && now-brake_started>=52) telemetry.mode=DRIVE_BASE_STOPPED; }
void DriveBase_Stop(DriveStopMode mode)
{
  telemetry.mode=mode==DRIVE_STOP_BRAKE?DRIVE_BASE_BRAKING:DRIVE_BASE_STOPPED;
  brake_started=tick; memset(telemetry.requested_cps,0,sizeof telemetry.requested_cps);
}
void DriveBase_SetWheelCps(int32_t a,int32_t b,int32_t c,int32_t d)
{
  assert(telemetry.mode!=DRIVE_BASE_BRAKING && !telemetry.fault_mask);
  assert(a==b && c==d);
  if(!output.valid)
  {
    assert(a==-c && (a==LINE_SEARCH_TARGET_CPS || a==-LINE_SEARCH_TARGET_CPS));
    if(previous_spin && a!=previous_spin) ++reversals;
    previous_spin=a; ++spins;
  }
  telemetry.mode=DRIVE_BASE_SPEED;
  telemetry.requested_cps[0]=a; telemetry.requested_cps[1]=b;
  telemetry.requested_cps[2]=c; telemetry.requested_cps[3]=d;
}
void DriveBase_SetSideCps(int32_t l,int32_t r) { DriveBase_SetWheelCps(l,l,r,r); }
uint8_t DriveBase_StartPositionMove(const DrivePositionCommand *c)
{ (void)c; assert(!"Persistent search must not use position moves"); return 0; }
static void sample(unsigned mask,uint32_t dt,int16_t base)
{
  LineTrackingReading r={mask&1,(mask>>1)&1,(mask>>2)&1,(mask>>3)&1};
  tick+=dt; BuzzerPhrase400_Task(tick); DriveBase_Task(tick);
  line_tracking_compute(&r,base,&output);
  if(output.valid)
  {
    if(!output.left_cps && !output.right_cps) DriveBase_Stop(DRIVE_STOP_COAST);
    else DriveBase_SetSideCps(output.left_cps,output.right_cps);
  }
}
static void hold(unsigned mask,uint32_t ms)
{ while(ms) { uint32_t dt=ms>10?10:ms; sample(mask,dt,3000); ms-=dt; } }
static void reset(uint8_t forward,uint8_t smooth)
{
  line_tracking_reset(); memset(&telemetry,0,sizeof telemetry); BuzzerPhrase400_Init();
  attacks=spins=reversals=0; previous_spin=0;
  line_tracking_set_no_line_forward(forward); line_tracking_set_smooth_mode(smooth);
}
static void assert_search(void)
{
  assert(!output.valid && telemetry.mode==DRIVE_BASE_SPEED && BuzzerPhrase400_IsPlaying());
  assert(telemetry.requested_cps[0]==-telemetry.requested_cps[2] && spins && !reversals);
}
int main(void)
{
  unsigned smooth,forward,i;
  for(smooth=0;smooth<=1;++smooth) for(forward=0;forward<=1;++forward)
  {
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300); sample(0,10,3000);
    assert(!output.valid && telemetry.mode==DRIVE_BASE_BRAKING && BuzzerPhrase400_IsPlaying());
    hold(0,90000); assert_search(); assert(attacks>250); /* Beyond 8 s and 24 phrase repeats. */
    hold(2,10000); assert_search(); hold(8,10000); assert_search();
    hold(15,1000); assert_search(); /* Wide crossing is not a confirmed middle line. */
    sample(5,10,3000); assert(telemetry.mode==DRIVE_BASE_BRAKING);
    hold(0,200); assert_search(); /* False contact resumes, never latches stop. */
    hold(5,180); assert(output.valid && output.left_cps>0 && !BuzzerPhrase400_IsPlaying() && !buzzer);
    hold(2,2000); assert(output.left_cps>0 && output.right_cps>0); /* No 800-ms capture stop. */
    hold(5,550); assert(output.left_cps>2400 && !BuzzerPhrase400_IsPlaying());
    sample(0,10,3000); hold(0,100); assert_search();
    /* Same cancellation hook that main calls on remote STOP / mode handoff. */
    line_tracking_reset(); assert(telemetry.mode==DRIVE_BASE_STOPPED && !BuzzerPhrase400_IsPlaying() && !buzzer);
    for(i=0;i<100;++i) sample(5,10,0);
    assert(output.valid && !output.left_cps && !BuzzerPhrase400_IsPlaying());

    /* Last confirmed right hint determines rotation, not timed side swapping. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(4,40); sample(0,10,3000); hold(0,500);
    assert_search(); assert(telemetry.requested_cps[0]>0);
    sample(5,10,3000); line_tracking_reset();
    assert(!BuzzerPhrase400_IsPlaying() && telemetry.mode==DRIVE_BASE_STOPPED);
    /* All real drive faults still stop audio and movement; never clear/retry. */
    for(i=0;i<3;++i)
    {
      reset(0,(uint8_t)smooth); hold(0,100);
      telemetry.fault_mask=(uint8_t)(i==0?1:(i==1?32:64)); sample(0,10,3000);
      assert(output.valid && !output.left_cps && !BuzzerPhrase400_IsPlaying());
      assert(LineRecovery_GetStopReason()==LINE_REC_STOP_DRIVE_FAULT);
      telemetry.fault_mask=0; hold(5,1000);
      assert(!output.left_cps && !BuzzerPhrase400_IsPlaying());
    }
  }
  /* Verify the actual predefined envelope, not a replacement alarm pattern. */
  reset(0,0); sample(0,0,3000); assert(buzzer==GPIO_PIN_SET);
  hold(0,110); assert(!buzzer); hold(0,30); assert(buzzer);
  hold(0,150); assert(!buzzer); hold(0,10); assert(buzzer);
  hold(0,140); assert(!buzzer); hold(0,60); assert(buzzer);
  hold(0,170); assert(!buzzer); hold(0,12); assert(buzzer);
  hold(0,280); assert(!buzzer); hold(0,568); assert(buzzer && attacks==6);
  /* Normal mode reset must not claim an unrelated manually started phrase. */
  line_tracking_reset(); BuzzerPhrase400_Start(1); line_tracking_reset();
  assert(BuzzerPhrase400_IsPlaying()); BuzzerPhrase400_Stop();
  reset(1,1); sample(0,10,3000); assert(output.left_cps>0 && !BuzzerPhrase400_IsPlaying());
  tick=UINT32_MAX-500; reset(0,1); hold(0,10000); assert_search();
  hold(5,700); assert(output.left_cps>2400 && !BuzzerPhrase400_IsPlaying());
  printf("PASS CPS=%ld: persistent rotation, actual repeating phrase, capture/re-loss, reset/STOP, faults, wrap\n",(long)LINE_SEARCH_TARGET_CPS);
  return 0;
}
