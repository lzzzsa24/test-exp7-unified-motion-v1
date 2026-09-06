# Shared project state

This is the handoff record for humans, Codex tasks, and delegated models. It is
not a substitute for Git: the checker resolves the live branch and HEAD every
time. Only the integration coordinator updates this file after a merge, flash,
or physical test.

## Machine-readable snapshot

```text
state_schema_version: 1
state_updated_at: 2026-09-06
integration_branch: feature/line-reacquire-lock
repository_head_at_update: e630552
latest_code_commit: 9ac30b0
flashed_source_commit: 9ac30b0
flash_record_commit: e630552
deployed_tag: deployed/2026-09-06-persistent-search-fault-log
formal_bin_path: manual-build-unified-motion/exp7_unified_motion.bin
formal_hex_path: manual-build-unified-motion/exp7_unified_motion.hex
formal_bin_size_bytes: 67524
flashed_bin_sha256: 4F00813D7CA5E8232EAC0612FCAC24C2E0CA588A4DBD93810B69B1624A75B6A6
flashed_hex_sha256: 7FEF2FEB83960E78FFC3AC5B7B1F2A9CC14150E5EC6DF36196DEDCB367F67B87
ground_test_status: persistent_search_fault_log_flashed_ground_test_pending_buzzer_passed
k210_status: removed
candidate_source_commit: 9ac30b0
candidate_bin_size_bytes: 67524
candidate_bin_sha256: 4F00813D7CA5E8232EAC0612FCAC24C2E0CA588A4DBD93810B69B1624A75B6A6
candidate_hex_sha256: 7FEF2FEB83960E78FFC3AC5B7B1F2A9CC14150E5EC6DF36196DEDCB367F67B87
user_reported_flash: tool_verified_current_candidate
```

`repository_head_at_update` is the source/history anchor present when this
snapshot was written. Documentation-only governance commits may be newer; the
checker requires the anchor to remain an ancestor and prints the live HEAD.

## Active project and deployed firmware

- Repository: `F:\myproject\jidian\project\test-exp7-unified-motion-v1`
- Integration branch: `feature/line-reacquire-lock`
- Latest firmware source commit: `9ac30b0` (`Log line drive faults in RAM and continue with bounded wheel feedforward`), now flashed. It integrates the requested `eba56d0` plus prerequisite `3fcf4de` on top of the current integration history, retaining the position handoff fix, buzzer GPIO fix and IR centre-key audio.
- Flash/readback record: `e630552` (`Record persistent-search fault-log firmware flash`).
- The formal BIN above was rebuilt from the clean integration checkout, then
  written through the STM32 ROM bootloader on USB-SERIAL CH340K COM11 at
  57600 baud. Selective erase covered 33 firmware pages, preserved the final
  calibration page, wrote and read back 67524 bytes with `VERIFY OK`, and
  completed `GO OK: 0x08000000`.
- Build products under `manual-build-*` are intentionally ignored by Git. A
  different computer must rebuild the named source commit rather than assume
  the artifact was transferred.
- Current formal BIN/HEX were built in this integration checkout from
  `9ac30b0`. The preceding adaptive-search build remains under
  `manual-build-adaptive-line-search`.
  Previous isolated candidates remain under the validation directory.
- Immediate rollback tag: `rollback/2026-09-06-before-persistent-search-fault-observation`
  points to `7a79682`. The buzzer GPIO fix and earlier adaptive-search rollback
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

Latest user observations before this deployment (2026-09-05): KEY2 could emit
1-, 5- or 8-beep drive alarms, alternate very small rotations, or stop silently.
The user then explicitly requested continued searching and fault observation
instead of stopping the line mode.

The deployed `9ac30b0` recovery changes KEY1/KEY2 as follows:

- On line loss it briefly brakes, then continuously rotates in the most recent
  reliable direction; without a hint it defaults left. Search uses equal and
  opposite 2493-CPS wheel groups. It no longer retreats, reverses search side,
  returns to encoder start counts, or stops for distance, attempt or time
  budgets.
- While searching it continuously repeats the existing 1.53-second preset
  buzzer phrase. A middle X1/X3 line hit, excluding the both-outer wide-line
  case, brakes and must confirm for 20 ms before the phrase is stopped.
- Confirmed capture enters at least 500 ms of low-speed line following and
  requires stable middle-line evidence for 80 ms before normal speed resumes.
  Losing the line during capture restarts same-direction rotation and audio.
- Outer-only and all-four-black input cannot directly complete capture. Manual
  STOP, mode change or zero base-speed still stop motion and cancel the phrase;
  power-on still enters STOP.
- Persistent rotation is intentionally unbounded at the recovery layer and can
  drift or heat a physically stalled motor. Encoder wheel speed does not prove
  chassis rotation or guarantee that the line will be found.

## Current line-turn load assistance

- Commit `63dbfe6`, retained in `9ac30b0`, keeps the requested four-wheel CPS targets and adds a
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
  expired authorization, stop or position control clears the supplement. A
  wheel degraded by the line-only observation policy also loses PI and load
  assistance and uses bounded feedforward instead. Wheel-speed feedback is not
  current or torque feedback, so ground traction remains untested.

## Current line-drive fault observation

- Normal line tracking and persistent search enable a line-only observation
  policy. Existing no-motion, wrong-direction and illegal-encoder thresholds
  are retained, but reaching them records an event instead of setting the
  blocking DriveBase fault mask or commanding a fault stop.
- An affected wheel uses the existing voltage-compensated CPS-to-PWM mapping,
  its continuous-drive PWM floor and the 3599 hardware ceiling. Its suspect
  feedback no longer adds PI, load assistance or repeated startup boost. Other
  wheels retain closed-loop control. This can keep a truly stalled motor
  energized and is not torque or current control.
- Other modes and position actions retain blocking drive faults. STOP or mode
  change clears the active degraded-wheel state but does not erase the log.
- A 32-entry RAM ring stores event time, sensor/recovery state, battery value
  and four-wheel requested/controlled/measured speed, PWM, encoder delta,
  illegal transition count and zero-motion time. It survives repeated reads
  and operator STOP but is lost on reset, power loss, reflash or DriveBase
  reinitialization; it is not stored in the calibration Flash page.
- After a test, press remote `0`, keep power connected, open the application
  serial port at 115200 8N1 without resetting DTR/RTS, and send `f` or `F`.
  Preserve the final complete block from `LFAULT BEGIN` through `LFAULT END`.

## Current position-control handoff

- Commit `b424189`, retained in `9ac30b0`, fixes the shared DriveBase transition from continuous
  position-control PWM to the short-pulse region used near a target.
- When an individual wheel enters that low-speed region, its previous
  continuous PWM is first set to zero and a fresh stop-settle window is
  established. The existing 36 ms pulse gap and 25 ms per-wheel staggering are
  then retained instead of issuing a pulse while the wheel is still driven.
- Old pulse-response state is cancelled when switching between pulse and
  continuous control. Four-wheel distance targets, tolerances, synchronization,
  fault thresholds and fault latching are unchanged.
- This still affects other DriveBase position moves, but the current lost-line
  recovery no longer invokes position mode. Host tests reproduce both forward
  and reverse handoff; actual wheel inertia and alarm behavior remain untested.

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
  deployed `9ac30b0`.

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
| computer build/link | passed | integrated formal `9ac30b0`; BIN is 67524 bytes; buzzer fix retained |
| host regression | passed | 2493/1870-CPS profiles pass persistent rotation, actual repeated phrase, capture/re-loss, STOP and wrap; real DriveBase tests pass 90-second search, line-only fault logging/degradation, bounded fallback, capture and retained blocking faults for other owners; 32-entry RAM log/coalescing/export tests pass; geometry self-test passes; MSVC /W4 /WX |
| flash/readback/GO | passed | COM11 at 57600 baud; 33-page selective erase; calibration page preserved; 67524-byte write and readback; `VERIFY OK`; `GO OK` |
| physical buzzer | passed | user explicitly confirmed `响了` after the PG12 initialization fix; fix retained in current firmware |
| wheels off ground | not performed this turn | diagnostic image compilation is not a lifted-wheel test |
| ground driving | current persistent-search/fault-log firmware untested | indefinite rotation, repeated audio, degraded feedforward, capture, drift and motor heating require controlled observation |

## Open issue and next safe step

The requested persistent-search and RAM fault-observation integration, formal
build, all host regressions, flash, readback verification and GO are complete.
Physical buzzer output was confirmed on an earlier firmware; continuous repeat
and ground motion are not yet physically verified. Test with room to rotate and
the remote STOP ready. After any abnormal wheel behavior, press `0` and keep
power connected so the RAM log can be exported with `f`. Do not leave a stalled
motor energized. Use the immediate rollback point if behavior is unsafe.

## Update protocol

After integrating code, flashing, or receiving a physical result, the
coordinator must update the applicable fields and evidence row. Keep old facts
in Git history instead of accumulating a long diary here. Never mark a physical
test passed from a successful compilation, programmer verification, OLED text,
or encoder counts alone.
