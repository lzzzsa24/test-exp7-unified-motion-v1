# Shared project state

This is the handoff record for humans, Codex tasks, and delegated models. It is
not a substitute for Git: the checker resolves the live branch and HEAD every
time. Only the integration coordinator updates this file after a merge, flash,
or physical test.

## Machine-readable snapshot

```text
state_schema_version: 1
state_updated_at: 2026-09-05
integration_branch: feature/line-reacquire-lock
repository_head_at_update: a88790b
latest_code_commit: b424189
flashed_source_commit: b424189
flash_record_commit: a88790b
deployed_tag: deployed/2026-09-05-position-pulse-coast
formal_bin_path: manual-build-unified-motion/exp7_unified_motion.bin
formal_hex_path: manual-build-unified-motion/exp7_unified_motion.hex
formal_bin_size_bytes: 67288
flashed_bin_sha256: D0AE046B81547FA366B35A4F1DEDF7349786CDA6D57585E422F05F384F189780
flashed_hex_sha256: 6B96B414B87AAED19E6F158737B3B87334F5B33C9C285B77CD4E3180A04E9B9A
ground_test_status: position_pulse_coast_flashed_ground_test_pending_buzzer_passed
k210_status: removed
candidate_source_commit: b424189
candidate_bin_size_bytes: 67288
candidate_bin_sha256: D0AE046B81547FA366B35A4F1DEDF7349786CDA6D57585E422F05F384F189780
candidate_hex_sha256: 6B96B414B87AAED19E6F158737B3B87334F5B33C9C285B77CD4E3180A04E9B9A
user_reported_flash: tool_verified_current_candidate
```

`repository_head_at_update` is the source/history anchor present when this
snapshot was written. Documentation-only governance commits may be newer; the
checker requires the anchor to remain an ancestor and prints the live HEAD.

## Active project and deployed firmware

- Repository: `F:\myproject\jidian\project\test-exp7-unified-motion-v1`
- Integration branch: `feature/line-reacquire-lock`
- Latest firmware source commit: `b424189` (`Coast before handing position travel to pulse control`), now flashed. It applies the requested `3ad26cd` change on top of the current integration history, retaining bounded retrace, per-wheel line-turn assistance, the buzzer GPIO fix and IR centre-key audio.
- Flash/readback record: `a88790b` (`Record position pulse coast handoff firmware flash`).
- The formal BIN above was rebuilt from the clean integration checkout, then
  written through the STM32 ROM bootloader on USB-SERIAL CH340K COM11 at
  57600 baud. Selective erase covered 33 firmware pages, preserved the final
  calibration page, wrote and read back 67288 bytes with `VERIFY OK`, and
  completed `GO OK: 0x08000000`.
- Build products under `manual-build-*` are intentionally ignored by Git. A
  different computer must rebuild the named source commit rather than assume
  the artifact was transferred.
- Current formal BIN/HEX were built in this integration checkout from
  `b424189`. The preceding adaptive-search build remains under
  `manual-build-adaptive-line-search`.
  Previous isolated candidates remain under the validation directory.
- Immediate rollback tag: `rollback/2026-09-05-before-position-pulse-coast`
  points to `887e6b9`. The buzzer GPIO fix and earlier adaptive-search rollback
  points remain available.

## Current mode map

| Input | Mode | Motor owner |
|---|---|---|
| power-on / stop command | STOP | stop latch |
| KEY1 / `1` | integrated black-line tracking plus infrared/ultrasonic bypass | line controller or bypass state machine |
| KEY2 / `2` | black-line tracking only; obstacle sensors do not take the motors | line controller |
| KEY3 / `3` | one encoder-controlled figure eight, then stop | figure-eight controller |
| KEY4 / `4` | one encoder-controlled square, then stop | square controller |
| remote direction-pad centre (`0x05`) | play the preset buzzer phrase once without changing mode | non-blocking phrase player; safety warnings retain priority |

The infrared remote also supplies the virtual mode keys and a stop command.

## Current line-loss behavior

Latest user ground observation before this deployment (2026-09-05): the prior
lost-line rotation could move the chassis away from the track even when the
right-front wheel itself rotated normally. The new recovery therefore bounds
local motion rather than enlarging a blind angular sweep.

The `aebf230` recovery retained in deployed `b424189` changes KEY1/KEY2 as follows:

- Normal tracking records a four-encoder snapshot every 20 ms while a middle
  line sensor sees the line, retaining up to 16 reliable forward snapshots.
  Reverse-command and wide-crossing samples are excluded from this history.
- On line loss the car actively brakes. If the line does not return while
  stopping, it retraces toward a recent recorded snapshot, with each wheel
  limited to 45 mm equivalent rim travel and at most two retrace attempts.
  With no usable history it does not invent a blind reverse move.
- If retracing does not find the line, the car performs at most one bounded
  probe per side at the geometry-derived 2493-CPS search target. An all-white
  probe is limited to 28 mm or 350 ms; matching outer-sensor guidance may
  extend it only to 56 mm or 600 ms.
- A failed probe is actively braked and each wheel is commanded back toward
  that probe's encoder start count before the opposite side is tried. A return
  demand above 85 mm per wheel is rejected rather than causing a large move.
- A middle-sensor hit brakes first and is confirmed while stationary. After a
  valid capture, at least 500 ms of low-speed line following is used before
  normal tracking resumes. False captures return to the bounded recovery path.
- The whole recovery keeps an independent 8-second deadline. Two failed local
  probes, abnormal encoder movement, DriveBase fault, or timeout stop the car
  until the mode is selected again. Encoder return only proves wheel motion;
  slip can prevent exact chassis-position restoration.

## Current line-turn load assistance

- Commit `63dbfe6`, retained in `b424189`, keeps the requested four-wheel CPS targets and adds a
  bounded PWM supplement only to an accepted line-tracking differential or
  counter-rotation command. Straight travel, wide-line travel, stop, encoder
  position retrace/rollback and non-line modes do not receive this supplement.
- Each wheel is evaluated independently. Assistance begins only after its
  target ramp reaches at least 1412 CPS and measured same-direction speed stays
  below 85 percent of target for 40 ms.
- The extra PWM is half the same-direction speed deficit, capped at 600 and
  ramped upward by at most 5 PWM per millisecond. Total output remains clamped
  to the existing 3599 hardware limit; the existing startup pulse has priority.
- Reaching the 85-percent speed threshold, reverse feedback, a mismatched or
  expired authorization, stop, position control or a drive fault clears the
  supplement immediately. Wheel-speed feedback is not current or torque
  feedback, so ground traction and actual turning improvement remain untested.

## Current position-control handoff

- Commit `b424189` fixes the shared DriveBase transition from continuous
  position-control PWM to the short-pulse region used near a target.
- When an individual wheel enters that low-speed region, its previous
  continuous PWM is first set to zero and a fresh stop-settle window is
  established. The existing 36 ms pulse gap and 25 ms per-wheel staggering are
  then retained instead of issuing a pulse while the wheel is still driven.
- Old pulse-response state is cancelled when switching between pulse and
  continuous control. Four-wheel distance targets, tolerances, synchronization,
  fault thresholds and fault latching are unchanged.
- This affects every DriveBase position move, including lost-line retrace and
  failed-probe rollback. Host tests reproduce both forward and reverse handoff,
  but actual wheel inertia, alarm behavior and ground recovery remain untested.

## Current infrared-remote audio behavior

- Latest physical observation before this fix: pressing the intended sound
  button produced no audible result. Source inspection found that the active
  buzzer's PG12 output setup existed only in the unused legacy
  `MX_Experiment1_GPIO_Init()` path; the integrated startup calls
  `MX_GPIO_Init()` instead.
- Commit `0d31f10` initializes PG12 low as a push-pull output in
  `MX_GPIO_Init()` before the phrase player starts. It does not reconfigure
  the entity keys, change the NEC key map, or change motor/line behavior.
- The direction-pad centre/buzzer command `0x05` maps to a dedicated
  `IR_REMOTE_VIRTUAL_AUDIO_ONCE` event.
- One complete NEC frame starts exactly one 1.53-second preset five-attack
  phrase and does not start, stop, or change a driving mode.
- NEC repeat frames remain suppressed, so holding the key does not queue
  repeated playback. A later fresh press restarts one complete phrase.
- Existing stop/fault, encoder, ultrasonic and bypass warning arbitration can
  immediately cancel this lower-priority audio.
- The serial `b` command remains an equivalent one-shot diagnostic entry.
- After flashing `0d31f10`, the user short-pressed the intended sound button
  and explicitly confirmed audible output (`响了`). The same fix remains in
  deployed `b424189`.

## Confirmed hardware facts

- All four encoders are repaired and usable as AB quadrature inputs.
- M2 is mapped to PA15/PB3; do not restore the obsolete fallback.
- Wheel order is M1 left-front, M2 left-rear, M3 right-front, M4 right-rear.
- Motor direction compensation remains centralized in `Core/Src/motorPWM.c`.
- K210 is physically removed for now.
- OLED is the external J12 display and includes battery/status information.
- Encoder distance/angle is a wheel-motion estimate; ground yaw requires
  calibration because slip and battery/load change the result.

## Evidence ledger

| Evidence level | Current result | Scope |
|---|---|---|
| computer build/link | passed | integrated formal `b424189`; BIN is 67288 bytes; buzzer fix retained |
| host regression | passed | both recovery profiles and load-assist tests pass; real integrated control chain passes line loss, position retrace, middle hit, braking, slow capture and return to normal; forward/reverse continuous-to-pulse handoff leaves the entering wheel at zero PWM before pulsing; MSVC /W4 /WX |
| flash/readback/GO | passed | COM11 at 57600 baud; 33-page selective erase; calibration page preserved; 67288-byte write and readback; `VERIFY OK`; `GO OK` |
| physical buzzer | passed | user explicitly confirmed `响了` after the PG12 initialization fix; fix retained in current firmware |
| wheels off ground | not performed this turn | diagnostic image compilation is not a lifted-wheel test |
| ground driving | current position-handoff firmware untested | retrace stopping, pulse completion, capture, traction and any fault alarm require same-surface observation |

## Open issue and next safe step

The requested position-control handoff fix, formal build, recovery/load-assist
and integrated handoff regressions, flash, readback verification and GO are
complete. Physical buzzer output was confirmed on an earlier firmware and its
source fix is retained. Ground-test lost-line retrace in both directions and
observe whether the car now stops cleanly near the retrace target before its
short completion pulses. If it stops with warning beeps, record the group count
or serial fault mask. Use the immediate rollback point if it performs worse.

## Update protocol

After integrating code, flashing, or receiving a physical result, the
coordinator must update the applicable fields and evidence row. Keep old facts
in Git history instead of accumulating a long diary here. Never mark a physical
test passed from a successful compilation, programmer verification, OLED text,
or encoder counts alone.
