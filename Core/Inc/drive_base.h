#ifndef __DRIVE_BASE_H
#define __DRIVE_BASE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRIVE_BASE_WHEEL_COUNT 4U

typedef enum
{
  DRIVE_STOP_COAST = 0U,
  /* Non-blocking 0-PWM guard followed by a short, bounded reverse-torque
     pulse.  The unverified AT8236 1/1 hardware-brake state is not used. */
  DRIVE_STOP_BRAKE
} DriveStopMode;

typedef enum
{
  DRIVE_BASE_STOPPED = 0U,
  DRIVE_BASE_SPEED,
  DRIVE_BASE_POSITION,
  DRIVE_BASE_BRAKING,
  DRIVE_BASE_FAULT
} DriveBaseMode;

typedef enum
{
  DRIVE_POSITION_IDLE = 0U,
  DRIVE_POSITION_RUNNING,
  DRIVE_POSITION_SETTLING,
  DRIVE_POSITION_DONE,
  DRIVE_POSITION_FAULT
} DrivePositionState;

enum
{
  DRIVE_FAULT_MOTOR1 = 0x01U,
  DRIVE_FAULT_MOTOR2 = 0x02U,
  DRIVE_FAULT_MOTOR3 = 0x04U,
  DRIVE_FAULT_MOTOR4 = 0x08U,
  DRIVE_FAULT_TIMEOUT = 0x10U,
  DRIVE_FAULT_DIRECTION = 0x20U,
  DRIVE_FAULT_ENCODER_SIGNAL = 0x40U,
  DRIVE_FAULT_SYNC = 0x80U
};

typedef struct
{
  /* Signed logical wheel travel. Positive means vehicle-forward rotation. */
  int32_t delta_counts[DRIVE_BASE_WHEEL_COUNT];
  /* Wheel-speed magnitude limit. DriveBase derives direction from
     delta_counts, so a legacy caller's sign cannot reverse the move. */
  int32_t maximum_cps[DRIVE_BASE_WHEEL_COUNT];
  uint32_t timeout_ms;
  uint16_t tolerance_counts;
  DriveStopMode completion_stop_mode;
} DrivePositionCommand;

typedef struct
{
  DriveBaseMode mode;
  DrivePositionState position_state;
  uint8_t fault_mask;
  uint8_t direction_fault_mask;
  uint8_t encoder_signal_fault_mask;
  uint8_t sync_fault;
  int32_t requested_cps[DRIVE_BASE_WHEEL_COUNT];
  int32_t controlled_cps[DRIVE_BASE_WHEEL_COUNT];
  int32_t measured_cps[DRIVE_BASE_WHEEL_COUNT];
  int16_t output_pwm[DRIVE_BASE_WHEEL_COUNT];
  int32_t position_target_counts[DRIVE_BASE_WHEEL_COUNT];
  int32_t position_moved_counts[DRIVE_BASE_WHEEL_COUNT];
  int32_t position_remaining_counts[DRIVE_BASE_WHEEL_COUNT];
  int16_t normalized_progress_permille[DRIVE_BASE_WHEEL_COUNT];
  int16_t maximum_progress_spread_permille;
  uint32_t legal_transition_count[DRIVE_BASE_WHEEL_COUNT];
  uint32_t illegal_transition_count[DRIVE_BASE_WHEEL_COUNT];
  uint32_t direction_mismatch_count[DRIVE_BASE_WHEEL_COUNT];
  uint32_t last_valid_motion_ms[DRIVE_BASE_WHEEL_COUNT];
  uint16_t battery_mv;
  uint16_t voltage_compensation_permille;
} DriveBaseTelemetry;

void DriveBase_Init(void);
void DriveBase_Task(uint32_t now_ms);

void DriveBase_SetWheelCps(int32_t motor1_cps,
                           int32_t motor2_cps,
                           int32_t motor3_cps,
                           int32_t motor4_cps);
void DriveBase_SetSideCps(int32_t left_cps, int32_t right_cps);

/* Arm only the next matching SetWheel/SideCps call (within 20 ms). The claim
   is consumed even on rejection; ordinary callers never renew it. Assistance
   expires after 60 ms without a newly armed, accepted command. No motion here. */
void DriveBase_PrepareLineTurnAssist(int32_t left_cps, int32_t right_cps);

/* Explicit line ownership. Encoder-derived speed faults are logged without
   stopping; direction/signal faults use bounded feedforward until mode reset.
   Zero motion alone retains bounded PI and existing line-turn assistance.
   Position control never inherits this policy. Does not clear hard faults. */
void DriveBase_SetLineFaultObservation(uint8_t enabled, uint8_t sensors,
                                      uint8_t recovery_state);
uint8_t DriveBase_GetLineDegradedMask(void);

/* Compatibility conversion for existing high-level modules whose tuned
   parameters are still expressed in logical PWM units. */
int32_t DriveBase_EquivalentCpsFromPwm(int16_t signed_pwm);

uint8_t DriveBase_StartPositionMove(const DrivePositionCommand *command);
uint8_t DriveBase_RequestPositionStop(DriveStopMode mode);
DrivePositionState DriveBase_GetPositionState(void);
uint8_t DriveBase_GetFaultMask(void);
void DriveBase_ClearFault(void);

void DriveBase_Stop(DriveStopMode mode);
void DriveBase_GetTelemetry(DriveBaseTelemetry *telemetry);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVE_BASE_H */
