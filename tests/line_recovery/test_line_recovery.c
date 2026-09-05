/* Real line/recovery sources; velocity/count plant. Any position API use fails. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "line_tracking.h"
#include "line_search_model.h"
#include "drive_base.h"
#include "wheel_encoder.h"
typedef struct { char kind; int32_t cps[4], origin[4], maximum; uint32_t start,end; } Leg;
static uint32_t tick,brake_started;
static DriveBaseTelemetry telemetry;
static LineTrackingCommand output;
static int32_t counts[4];
static Leg legs[64];
static unsigned leg_count;
static uint8_t blocked;
static int32_t gain[4];
static int32_t abs32(int32_t x) { return x<0?-x:x; }
uint32_t HAL_GetTick(void) { return tick; }
int HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t n) { (void)p; (void)n; return 1; }
void HAL_GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *g) { (void)p; (void)g; }
int32_t DriveBase_EquivalentCpsFromPwm(int16_t p) { return p; }
void DriveBase_PrepareLineTurnAssist(int32_t l,int32_t r) { (void)l; (void)r; }
void WheelEncoder_GetCounts(WheelEncoderCounts *c)
{ c->motor1=counts[0]; c->motor2=counts[1]; c->motor3=counts[2]; c->motor4=counts[3]; }
void DriveBase_GetTelemetry(DriveBaseTelemetry *t) { *t=telemetry; }
uint8_t DriveBase_GetFaultMask(void) { return telemetry.fault_mask; }
void DriveBase_Task(uint32_t now)
{ if(telemetry.mode==DRIVE_BASE_BRAKING && now-brake_started>=52) telemetry.mode=DRIVE_BASE_STOPPED; }
void DriveBase_Stop(DriveStopMode mode)
{
  if(telemetry.mode==DRIVE_BASE_SPEED && leg_count) legs[leg_count-1].end=tick;
  telemetry.mode=mode==DRIVE_STOP_BRAKE?DRIVE_BASE_BRAKING:DRIVE_BASE_STOPPED;
  brake_started=tick; memset(telemetry.requested_cps,0,sizeof telemetry.requested_cps);
}
void DriveBase_SetWheelCps(int32_t a,int32_t b,int32_t c,int32_t d)
{
  assert(telemetry.mode!=DRIVE_BASE_BRAKING && !telemetry.fault_mask);
  assert(telemetry.mode!=DRIVE_BASE_POSITION);
  assert(a==b && c==d);
  if(telemetry.mode!=DRIVE_BASE_SPEED)
  {
    Leg *l;
    assert(leg_count<64); l=&legs[leg_count++]; memset(l,0,sizeof *l); l->kind='N';
    if(a<0 && c<0) l->kind='B';
    if(a==-c && a!=0 && (output.action==LINE_ACTION_SEARCH_LEFT || output.action==LINE_ACTION_SEARCH_RIGHT))
    {
      int32_t sign=output.action==LINE_ACTION_SEARCH_LEFT?-1:1;
      l->kind=a*sign>0?'P':'R';
    }
    l->cps[0]=a; l->cps[1]=b; l->cps[2]=c; l->cps[3]=d;
    memcpy(l->origin,counts,sizeof counts); l->start=tick;
  }
  telemetry.mode=DRIVE_BASE_SPEED;
  telemetry.requested_cps[0]=a; telemetry.requested_cps[1]=b;
  telemetry.requested_cps[2]=c; telemetry.requested_cps[3]=d;
}
void DriveBase_SetSideCps(int32_t l,int32_t r) { DriveBase_SetWheelCps(l,l,r,r); }
uint8_t DriveBase_StartPositionMove(const DrivePositionCommand *m)
{ (void)m; assert(!"Line recovery must never enter exact position control"); return 0; }
uint8_t DriveBase_RequestPositionStop(DriveStopMode m)
{ (void)m; assert(!"Line recovery must not own a position move"); return 0; }
static void sample(unsigned mask,uint32_t dt)
{
  unsigned i;
  LineTrackingReading r={mask&1,(mask>>1)&1,(mask>>2)&1,(mask>>3)&1};
  if(telemetry.mode==DRIVE_BASE_SPEED) for(i=0;i<4;++i)
  {
    if(!(blocked&(1U<<i))) counts[i]+=telemetry.requested_cps[i]*(int32_t)dt*gain[i]/100000;
    if(leg_count && abs32(counts[i]-legs[leg_count-1].origin[i])>legs[leg_count-1].maximum)
      legs[leg_count-1].maximum=abs32(counts[i]-legs[leg_count-1].origin[i]);
  }
  tick+=dt; DriveBase_Task(tick); line_tracking_compute(&r,3000,&output);
  if(output.valid)
  {
    if(!output.left_cps && !output.right_cps) DriveBase_Stop(DRIVE_STOP_COAST);
    else DriveBase_SetSideCps(output.left_cps,output.right_cps);
  }
}
static void hold(unsigned mask,uint32_t ms)
{ while(ms) { uint32_t dt=ms>10?10:ms; sample(mask,dt); ms-=dt; } }
static void reset(uint8_t forward,uint8_t smooth)
{
  line_tracking_reset(); memset(&telemetry,0,sizeof telemetry); memset(counts,0,sizeof counts);
  leg_count=0; blocked=0; gain[0]=gain[1]=gain[2]=gain[3]=100;
  line_tracking_set_no_line_forward(forward); line_tracking_set_smooth_mode(smooth);
}
static Leg *nth(char kind,unsigned n)
{
  unsigned i;
  for(i=0;i<leg_count;++i) if(legs[i].kind==kind && --n==0) return &legs[i];
  return 0;
}
static Leg *wait_leg(char kind,unsigned n,unsigned mask)
{
  unsigned tries=0;
  while(!nth(kind,n) && tries++<900) sample(mask,10);
  assert(nth(kind,n)); return nth(kind,n);
}
static void wait_stop(void)
{
  unsigned tries=0;
  while((!output.valid || output.left_cps || output.right_cps) && tries++<900) sample(0,10);
  assert(output.valid && !output.left_cps && !output.right_cps);
}
int main(void)
{
  unsigned smooth,forward,i;
  for(smooth=0;smooth<=1;++smooth) for(forward=0;forward<=1;++forward)
  {
    /* Known history: continuous group reverse, middle capture, slow edge follow. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300); sample(0,10);
    assert(!output.valid && telemetry.mode==DRIVE_BASE_BRAKING);
    wait_leg('B',1,0);
    for(i=0;i<4;++i) assert(telemetry.requested_cps[i]==-1870);
    hold(5,150); assert(output.valid && output.left_cps>0 && output.right_cps>0);
    hold(2,160); assert(output.left_cps>0 && output.right_cps>0 && output.left_cps<=2400 && output.right_cps<=2400);
    hold(5,550); assert(output.left_cps>2400 && output.left_cps==output.right_cps);

    /* Outer hit stops retreat first, then guides a turn without forward release. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300); sample(0,10); wait_leg('B',1,0);
    sample(8,10); assert(telemetry.mode==DRIVE_BASE_BRAKING && !output.valid);
    assert(wait_leg('P',1,8)->cps[0]>0); assert(!output.valid);

    /* All white: three widening scans, coarse returns, no tiny position pulses. */
    reset(0,(uint8_t)smooth); wait_leg('P',1,0); hold(0,250);
    assert(!nth('R',1) && !nth('P',2));
    wait_leg('P',2,0); wait_leg('P',3,0); wait_stop(); assert(!nth('P',4));
    for(i=1;i<=3;++i)
    {
      Leg *p=nth('P',i), *r=nth('R',i); unsigned w;
      assert(p && r && p->maximum>=980*(int32_t)i);
      assert(p->maximum<=987*(int32_t)i+LINE_SEARCH_TARGET_CPS/100);
      assert(p->end-p->start>=300 && r->cps[0]==-p->cps[0]);
      if(i<3)
      {
        Leg *next=nth('P',i+1); assert(next->cps[0]==-p->cps[0]);
        for(w=0;w<4;++w) assert(abs32(next->origin[w]-p->origin[w])<=56);
      }
    }
    hold(5,100); assert(!output.left_cps);

    /* Unequal wheel response may leave coarse error; it must not trigger
       wheel-by-wheel correction or position synchronization faults. */
    reset(0,(uint8_t)smooth); gain[0]=60;
    wait_leg('P',1,0); wait_leg('P',2,0);
    for(i=0;i<4;++i) assert(abs32(nth('P',2)->origin[i]-nth('P',1)->origin[i])<=150);
    assert(!telemetry.fault_mask);
    /* If the return cannot progress far enough, stop instead of searching
       from a claimed-but-unreached anchor or clearing a controller fault. */
    reset(0,(uint8_t)smooth); wait_leg('R',1,0);
    gain[0]=gain[1]=gain[2]=gain[3]=10; wait_stop();
    assert(!nth('P',2) && !telemetry.fault_mask);
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300);
    for(i=0;i<4;++i) counts[i]+=1000;
    sample(0,10); wait_leg('B',1,0); wait_leg('P',1,0);
    assert(nth('B',1)->maximum<=316+1870/100);

    /* Chatter during brake, stale history and normal reverse-turn invalidation. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300); sample(0,10); hold(5,150);
    assert(!nth('B',1) && output.left_cps>0);
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300); tick+=700; sample(0,0);
    wait_leg('P',1,0); assert(!nth('B',1));
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300); hold(2,150); sample(0,10);
    wait_leg('P',1,0); assert(!nth('B',1));

    /* Early opposite evidence can correct direction before much travel occurs. */
    reset(0,(uint8_t)smooth); wait_leg('P',1,0); hold(8,30);
    assert(telemetry.mode==DRIVE_BASE_BRAKING); assert(wait_leg('P',2,8)->cps[0]>0);
    hold(5,150); assert(output.valid && output.left_cps>0);
    /* Matching evidence extends the first blind sweep, with middle confirmation. */
    reset(0,(uint8_t)smooth); wait_leg('P',1,0); hold(2,650);
    assert(!nth('R',1) && telemetry.mode==DRIVE_BASE_SPEED && !output.valid);
    hold(5,150); assert(output.left_cps>0);
    reset(0,(uint8_t)smooth); wait_leg('P',1,0);
    hold(2,8100); assert(output.valid && !output.left_cps && !telemetry.fault_mask);
    reset(0,(uint8_t)smooth); wait_leg('P',1,0); hold(0,200); sample(5,10);
    assert(telemetry.mode==DRIVE_BASE_BRAKING); hold(0,250);
    assert(!(output.valid && output.left_cps>0 && output.right_cps>0));

    /* Blocked wheel: stop locally instead of repeating blind reversals. */
    reset(0,(uint8_t)smooth); blocked=1; wait_leg('P',1,0); wait_stop();
    assert(!nth('P',2) && !telemetry.fault_mask);
    /* Real reported 1/5/8 fault classes remain latched; no clear-fault API exists. */
    for(i=0;i<3;++i)
    {
      reset(0,(uint8_t)smooth); wait_leg('P',1,0);
      telemetry.fault_mask=(uint8_t)(i==0?1:(i==1?16:128)); sample(0,10);
      assert(output.valid && !output.left_cps); hold(5,100); assert(!output.left_cps && telemetry.fault_mask);
    }
    reset(0,(uint8_t)smooth); wait_leg('P',1,0); counts[0]+=10000; sample(0,10);
    assert(output.valid && !output.left_cps);
    /* Capture/retry must retain the original episode deadline. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300);
    {
      uint32_t lost=tick+10;
      sample(0,10); wait_leg('B',1,0); hold(5,150); assert(output.left_cps>0);
      sample(0,10); tick=lost+8001; sample(0,0); assert(output.valid && !output.left_cps);
    }
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300); sample(0,10); wait_leg('B',1,0);
    line_tracking_reset(); assert(telemetry.mode==DRIVE_BASE_STOPPED);
  }
  reset(1,1); sample(0,10); assert(output.left_cps>0);
  tick=UINT32_MAX-30; reset(0,1); wait_leg('P',1,0); wait_stop();
  printf("PASS CPS=%ld: continuous retreat, widening scans, coarse returns, outer/middle capture, no position API, faults, watchdog, wrap\n",(long)LINE_SEARCH_TARGET_CPS);
  return 0;
}
