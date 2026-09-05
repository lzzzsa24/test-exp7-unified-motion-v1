/* Real line/recovery sources, a deterministic wheel-count plant and mocked
   DriveBase. Count retracing is verified; physical slip cancellation is not. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "line_tracking.h"
#include "line_search_model.h"
#include "drive_base.h"
#include "wheel_encoder.h"

static uint32_t tick, brake_started;
static DriveBaseTelemetry telemetry;
static LineTrackingCommand output;
static int32_t counts[4], goal[4];
static DrivePositionCommand moves[16];
static int32_t move_origins[16][4], probe_origins[8][4];
static unsigned move_count, probe_count;
static uint8_t reject_move;
static int32_t max_probe_travel;
static int32_t abs32(int32_t n) { return n < 0 ? -n : n; }
uint32_t HAL_GetTick(void) { return tick; }
int HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t n) { (void)p; (void)n; return 1; }
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g) { (void)p; (void)g; }
int32_t DriveBase_EquivalentCpsFromPwm(int16_t p) { return p; }
void WheelEncoder_GetCounts(WheelEncoderCounts *c)
{ c->motor1=counts[0]; c->motor2=counts[1]; c->motor3=counts[2]; c->motor4=counts[3]; }
void DriveBase_GetTelemetry(DriveBaseTelemetry *t) { *t=telemetry; }
uint8_t DriveBase_GetFaultMask(void) { return telemetry.fault_mask; }
DrivePositionState DriveBase_GetPositionState(void) { return telemetry.position_state; }
void DriveBase_Task(uint32_t now)
{
  if (telemetry.mode==DRIVE_BASE_BRAKING && now-brake_started>=52U)
  {
    telemetry.mode=DRIVE_BASE_STOPPED;
    if (telemetry.position_state==DRIVE_POSITION_SETTLING)
      telemetry.position_state=DRIVE_POSITION_DONE;
  }
}
void DriveBase_Stop(DriveStopMode mode)
{
  if (mode==DRIVE_STOP_BRAKE)
  { telemetry.mode=DRIVE_BASE_BRAKING; brake_started=tick; }
  else
  { telemetry.mode=DRIVE_BASE_STOPPED; telemetry.position_state=DRIVE_POSITION_IDLE; }
  memset(telemetry.requested_cps,0,sizeof telemetry.requested_cps);
}
void DriveBase_SetWheelCps(int32_t m1,int32_t m2,int32_t m3,int32_t m4)
{
  assert(telemetry.mode!=DRIVE_BASE_BRAKING && !telemetry.fault_mask);
  assert(telemetry.position_state!=DRIVE_POSITION_RUNNING);
  if (m1==-m3 && m2==-m4 && m1!=0 && telemetry.mode!=DRIVE_BASE_SPEED)
  {
    assert(probe_count<8);
    memcpy(probe_origins[probe_count++],counts,sizeof counts);
  }
  telemetry.mode=DRIVE_BASE_SPEED;
  telemetry.requested_cps[0]=m1; telemetry.requested_cps[1]=m2;
  telemetry.requested_cps[2]=m3; telemetry.requested_cps[3]=m4;
}
void DriveBase_SetSideCps(int32_t l,int32_t r) { DriveBase_SetWheelCps(l,l,r,r); }
uint8_t DriveBase_StartPositionMove(const DrivePositionCommand *m)
{
  unsigned i;
  assert(telemetry.mode!=DRIVE_BASE_BRAKING && !telemetry.fault_mask);
  if (reject_move) return 0;
  assert(move_count<16);
  moves[move_count]=*m;
  memcpy(move_origins[move_count],counts,sizeof counts);
  ++move_count;
  telemetry.mode=DRIVE_BASE_POSITION;
  telemetry.position_state=DRIVE_POSITION_RUNNING;
  for(i=0;i<4;++i)
  {
    goal[i]=counts[i]+m->delta_counts[i];
    telemetry.position_target_counts[i]=m->delta_counts[i];
    telemetry.position_moved_counts[i]=0;
    telemetry.requested_cps[i]=m->delta_counts[i]<0?-m->maximum_cps[i]:m->maximum_cps[i];
  }
  return 1;
}
uint8_t DriveBase_RequestPositionStop(DriveStopMode mode)
{
  assert(telemetry.position_state==DRIVE_POSITION_RUNNING);
  DriveBase_Stop(mode);
  telemetry.position_state=DRIVE_POSITION_SETTLING;
  return 1;
}
static void plant(uint32_t dt)
{
  unsigned i;
  uint8_t done=1;
  for(i=0;i<4;++i)
  {
    int32_t step=telemetry.requested_cps[i]*(int32_t)dt/1000L;
    if(telemetry.mode==DRIVE_BASE_SPEED) counts[i]+=step;
    if(telemetry.mode==DRIVE_BASE_POSITION)
    {
      int32_t remaining=goal[i]-counts[i];
      int32_t magnitude=abs32(step);
      if(magnitude>abs32(remaining)) magnitude=abs32(remaining);
      counts[i]+=remaining<0?-magnitude:magnitude;
      telemetry.position_moved_counts[i]=counts[i]-move_origins[move_count-1][i];
      if(abs32(goal[i]-counts[i])>12) done=0;
    }
    if(telemetry.mode==DRIVE_BASE_SPEED && telemetry.requested_cps[0]==-telemetry.requested_cps[2] && probe_count)
    {
      int32_t distance=abs32(counts[i]-probe_origins[probe_count-1][i]);
      if(distance>max_probe_travel) max_probe_travel=distance;
    }
  }
  if(telemetry.mode==DRIVE_BASE_POSITION && done)
  {
    memset(telemetry.requested_cps,0,sizeof telemetry.requested_cps);
    telemetry.mode=DRIVE_BASE_STOPPED;
    telemetry.position_state=DRIVE_POSITION_DONE;
  }
}
static void sample(unsigned mask,uint32_t dt)
{
  LineTrackingReading r={mask&1,(mask>>1)&1,(mask>>2)&1,(mask>>3)&1};
  plant(dt);
  tick+=dt;
  DriveBase_Task(tick);
  line_tracking_compute(&r,3000,&output);
  if(output.valid)
  {
    if(!output.left_cps&&!output.right_cps) DriveBase_Stop(DRIVE_STOP_COAST);
    else DriveBase_SetSideCps(output.left_cps,output.right_cps);
  }
}
static void hold(unsigned mask,uint32_t duration)
{
  while(duration) { uint32_t dt=duration>10?10:duration; sample(mask,dt); duration-=dt; }
}
static void reset(uint8_t forward,uint8_t smooth)
{
  line_tracking_reset();
  memset(&telemetry,0,sizeof telemetry);
  memset(counts,0,sizeof counts);
  move_count=probe_count=0;
  reject_move=0; max_probe_travel=0;
  line_tracking_set_no_line_forward(forward);
  line_tracking_set_smooth_mode(smooth);
}
static void wait_for_probe(unsigned number)
{
  unsigned tries=0;
  while(probe_count<number && tries++<500) sample(0,10);
  assert(probe_count==number);
  assert(!output.valid && telemetry.mode==DRIVE_BASE_SPEED);
  assert(telemetry.requested_cps[0]==telemetry.requested_cps[1]);
  assert(telemetry.requested_cps[2]==telemetry.requested_cps[3]);
  assert(telemetry.requested_cps[0]==-telemetry.requested_cps[2]);
  assert(abs32(telemetry.requested_cps[0])==LINE_SEARCH_TARGET_CPS);
}
static void wait_for_stop(void)
{
  unsigned tries=0;
  while((!output.valid || output.left_cps || output.right_cps) && tries++<800) sample(0,10);
  assert(output.valid&&!output.left_cps&&!output.right_cps);
}
int main(void)
{
  unsigned smooth,forward,i;
  for(smooth=0;smooth<=1;++smooth) for(forward=0;forward<=1;++forward)
  {
    /* Loss immediately brakes, then reverses a known recent forward segment. */
    reset((uint8_t)forward,(uint8_t)smooth);
    hold(5,300); sample(0,10);
    assert(!output.valid&&telemetry.mode==DRIVE_BASE_BRAKING);
    hold(0,60);
    assert(move_count==1&&telemetry.position_state==DRIVE_POSITION_RUNNING);
    for(i=0;i<4;++i) assert(moves[0].delta_counts[i]<0&&moves[0].delta_counts[i]>=-316);
    hold(5,160);
    assert(output.valid&&output.left_cps>0&&output.right_cps>0);
    /* The edge after capture must remain slow and forward, never sharp-spin. */
    hold(2,160);
    assert(output.valid&&output.left_cps>0&&output.right_cps>0);
    assert(output.left_cps<=2400&&output.right_cps<=2400);
    hold(5,550); assert(output.valid&&output.left_cps>2400&&output.left_cps==output.right_cps);

    /* A new loss during uncommitted capture must not reset the deadline. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300);
    {
      uint32_t lost_at=tick+10;
      sample(0,10); hold(0,80); hold(5,150);
      sample(0,10); assert(!output.valid&&telemetry.mode==DRIVE_BASE_BRAKING);
      tick=lost_at+8001; sample(0,0);
      assert(output.valid&&output.left_cps==0);
    }

    /* Backtrack distance is capped even if counts report a large overshoot. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300);
    for(i=0;i<4;++i) counts[i]+=1000;
    sample(0,10); hold(0,60);
    assert(move_count==1);
    for(i=0;i<4;++i) assert(moves[0].delta_counts[i]==-316);

    /* White chatter that clears during braking must not force reversal. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300);
    sample(0,10); hold(5,130);
    assert(move_count==0&&probe_count==0&&output.left_cps>0);

    /* No history: two local probes, each undone before trying the other side.
       There is no position-reset fiction: actual four-wheel counts are used. */
    reset(0,(uint8_t)smooth);
    wait_for_probe(1);
    assert(telemetry.requested_cps[0]<0);
    wait_for_probe(2);
    assert(telemetry.requested_cps[0]>0&&move_count==1);
    for(i=0;i<4;++i)
    {
      assert(move_origins[0][i]+moves[0].delta_counts[i]==probe_origins[0][i]);
      assert(abs32(probe_origins[1][i]-probe_origins[0][i])<=12);
    }
    wait_for_stop(); assert(move_count==2);
    for(i=0;i<4;++i) assert(abs32(counts[i])<=24);
    assert(max_probe_travel<=197+LINE_SEARCH_TARGET_CPS/100);
    hold(5,100); assert(output.left_cps==0);

    /* Do not reverse a history that crossed an ordinary in-place sharp turn. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300); hold(2,150);
    assert(telemetry.requested_cps[0]<0);
    sample(0,10); hold(0,60); assert(move_count==0);

    /* A real opposite outer hit rolls back first rather than sweeping farther. */
    reset((uint8_t)forward,(uint8_t)smooth);
    hold(8,40); wait_for_probe(1);
    assert(telemetry.requested_cps[0]>0);
    hold(2,30); assert(telemetry.mode==DRIVE_BASE_BRAKING);
    wait_for_probe(2); assert(telemetry.requested_cps[0]<0);
    hold(5,150); assert(output.valid&&output.left_cps>0&&output.right_cps>0);

    /* Constant matching outer evidence has a finite travel budget too. */
    reset(0,(uint8_t)smooth); wait_for_probe(1);
    hold(2,300);
    assert(move_count>=1&&max_probe_travel>197);
    assert(max_probe_travel<=394+LINE_SEARCH_TARGET_CPS/100);

    /* Brief centre noise in a probe does not release to forward mode. */
    reset(0,(uint8_t)smooth); wait_for_probe(1);
    hold(5,10); assert(telemetry.mode==DRIVE_BASE_BRAKING);
    hold(0,10); assert(!output.valid);
    hold(0,240); assert(!(output.valid&&output.left_cps>0&&output.right_cps>0));

    /* False contact during rollback must still finish returning to the
       original probe reference before the opposite probe starts. */
    reset(0,(uint8_t)smooth); wait_for_probe(1);
    {
      unsigned tries=0;
      while(telemetry.mode!=DRIVE_BASE_POSITION && tries++<100) sample(0,10);
      assert(telemetry.mode==DRIVE_BASE_POSITION);
      hold(5,10); assert(telemetry.mode==DRIVE_BASE_BRAKING);
      wait_for_probe(2);
      for(i=0;i<4;++i) assert(abs32(probe_origins[1][i]-probe_origins[0][i])<=12);
    }

    /* Old line evidence cannot authorize a blind retreat. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300);
    tick+=700; sample(0,0); hold(0,60); assert(move_count==0);
    wait_for_probe(1);

    /* Fault and rejected replay start cannot be overridden by recovery. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300);
    reject_move=1; sample(0,10); hold(0,60);
    assert(output.valid&&output.left_cps==0);
    reset(0,(uint8_t)smooth); wait_for_probe(1);
    reject_move=1; hold(0,600);
    assert(output.valid&&output.left_cps==0&&probe_count==1);
    reset(0,(uint8_t)smooth); wait_for_probe(1);
    telemetry.fault_mask=DRIVE_FAULT_MOTOR3; sample(0,10);
    assert(output.valid&&output.left_cps==0);
    hold(5,100); assert(output.left_cps==0);

    /* Unexpected large encoder displacement never causes a huge rollback. */
    reset(0,(uint8_t)smooth); wait_for_probe(1);
    counts[0]+=2000; sample(0,1); hold(0,70);
    assert(output.valid&&output.left_cps==0&&move_count==0);

    /* Whole-episode watchdog remains active while forward capture is pending. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300);
    sample(0,10); hold(0,80); hold(5,150); assert(output.left_cps>0);
    tick+=8001; sample(2,0); assert(output.valid&&output.left_cps==0);

    /* Reset during position motion releases motor ownership. */
    reset((uint8_t)forward,(uint8_t)smooth); hold(5,300);
    sample(0,10); hold(0,60);
    line_tracking_reset(); assert(telemetry.mode==DRIVE_BASE_STOPPED);
  }
  reset(1,1); sample(0,10); assert(output.left_cps>0);
  tick=UINT32_MAX-30; reset(0,1); wait_for_probe(1); wait_for_stop();
  printf("PASS CPS=%ld: initial brake, bounded history retreat, probe rollback, opposite-side retry, stationary capture, slow edge follow, limits, faults, reset, wrap\n",(long)LINE_SEARCH_TARGET_CPS);
  return 0;
}
