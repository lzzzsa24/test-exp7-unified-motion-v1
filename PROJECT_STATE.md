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
repository_head_at_update: c018174
latest_code_commit: 1bf6fe6
flashed_source_commit: 1bf6fe6
flash_record_commit: c018174
deployed_tag: deployed/2026-09-05-continuous-recovery
formal_bin_path: manual-build-unified-motion/exp7_unified_motion.bin
formal_hex_path: manual-build-unified-motion/exp7_unified_motion.hex
formal_bin_size_bytes: 67800
flashed_bin_sha256: 9A7D5A54585CD6093E65630BE5F5E835B53AEEDDAAF46E3CCCAB13C11E580789
flashed_hex_sha256: BD157BEF956446E1633C6FE445CF5C5CB78982FA6017B6CA29AB3ADE89FCCC27
ground_test_status: continuous_recovery_flashed_ground_test_pending_buzzer_passed
k210_status: removed
candidate_source_commit: 1bf6fe6
candidate_bin_size_bytes: 67800
candidate_bin_sha256: 9A7D5A54585CD6093E65630BE5F5E835B53AEEDDAAF46E3CCCAB13C11E580789
candidate_hex_sha256: BD157BEF956446E1633C6FE445CF5C5CB78982FA6017B6CA29AB3ADE89FCCC27
user_reported_flash: tool_verified_current_candidate
```

`repository_head_at_update` is the source/history anchor present when this
snapshot was written. Documentation-only governance commits may be newer; the
checker requires the anchor to remain an ancestor and prints the live HEAD.

## Active project and deployed firmware

- Repository: `F:\myproject\jidian\project\test-exp7-unified-motion-v1`
- Integration branch: `feature/line-reacquire-lock`
- Latest firmware source commit: `1bf6fe6` (`Recover lines with continuous group motion and coarse returns`), now flashed. It applies the requested `ad0a3ee` change on top of the current integration history, retaining per-wheel line-turn assistance, the position handoff fix, buzzer GPIO fix and IR centre-key audio.
- Flash/readback record: `c018174` (`Record continuous grouped line recovery firmware flash`).
- The formal BIN above was rebuilt from the clean integration checkout, then
  written through the STM32 ROM bootloader on USB-SERIAL CH340K COM11 at
  57600 baud. Selective erase covered 34 firmware pages, preserved the final
  calibration page, wrote and read back 67800 bytes with `VERIFY OK`, and
  completed `GO OK: 0x08000000`.
- Build products under `manual-build-*` are intentionally ignored by Git. A
  different computer must rebuild the named source commit rather than assume
  the artifact was transferred.
- Current formal BIN/HEX were built in this integration checkout from
  `1bf6fe6`. The preceding adaptive-search build remains under
  `manual-build-adaptive-line-search`.
  Previous isolated candidates remain under the validation directory.
- Immediate rollback tag: `rollback/2026-09-05-before-continuous-recovery`
  points to `307a2b7`. The buzzer GPIO fix and earlier adaptive-search rollback
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

Latest user ground observation before this deployment (2026-09-05): KEY2 could
produce 1-, 5- or 8-beep drive alarms, and all-white recovery could alternate
only very small opposite rotations. The new recovery avoids DriveBase position
mode and its fine per-wheel completion during line reacquisition.

The deployed `1bf6fe6` recovery changes KEY1/KEY2 as follows:

- Normal tracking retains up to 16 middle-line encoder snapshots at 20 ms
  intervals. On loss it actively brakes; a line seen while stopping is
  confirmed before any reverse move.
- With reliable recent forward history, all four wheels reverse together at
  1870 CPS toward a 160-600 ms-old reference. The group stops when any wheel
  reaches the shared budget, limited to 45 mm / 600 ms with an 8 mm margin.
  At most two such retreats are allowed; absent history does not cause a blind
  reverse move.
- Search uses equal-and-opposite 2493-CPS wheel groups and the existing bounded
  lagging-wheel assistance. Three all-white scans widen to 140, 280 and 420 mm
  budgets with corresponding 700, 1400 and 2100 ms limits. Matching outer-line
  guidance may extend only to 420 mm / 2800 ms.
- After a failed scan, all four wheels reverse as a group toward its starting
  counts. The first wheel to reach its coarse return budget stops the group;
  individual wheels are not micro-positioned or repeatedly corrected. Very
  small, badly imbalanced or above-450-mm returns are rejected.
- Middle or single-outer line evidence brakes the group immediately. A middle
  hit is stationary-confirmed before at least 500 ms of low-speed capture;
  stable outer evidence selects the next search direction but cannot directly
  restore normal-speed travel.
- Any segment with a wheel moving less than 5 mm, a DriveBase fault, failed
  return, three exhausted scans or the shared 8-second deadline stops recovery.
  The recovery never clears a latched drive fault and does not call the
  position-motion API. Wheel travel still cannot prove chassis displacement.

## Current line-turn load assistance

- Commit `63dbfe6`, retained in `1bf6fe6`, keeps the requested four-wheel CPS targets and adds a
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

- Commit `b424189`, retained in `1bf6fe6`, fixes the shared DriveBase transition from continuous
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
  deployed `1bf6fe6`.

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
| computer build/link | passed | integrated formal `1bf6fe6`; BIN is 67800 bytes; buzzer fix retained |
| host regression | passed | 2493/1870-CPS recovery profiles pass continuous retreat, widening scans, coarse returns, capture, faults and watchdog; real DriveBase integration remains in speed mode without position timeout/sync faults; load-assist and position-handoff regressions also pass; MSVC /W4 /WX |
| flash/readback/GO | passed | COM11 at 57600 baud; 34-page selective erase; calibration page preserved; 67800-byte write and readback; `VERIFY OK`; `GO OK` |
| physical buzzer | passed | user explicitly confirmed `响了` after the PG12 initialization fix; fix retained in current firmware |
| wheels off ground | not performed this turn | diagnostic image compilation is not a lifted-wheel test |
| ground driving | current continuous-recovery firmware untested | grouped retreat/search/return, capture, drift and any fault alarm require same-surface observation |

## Open issue and next safe step

The requested continuous grouped recovery integration, formal build, all host
regressions, flash, readback verification and GO are complete. Physical buzzer
output was confirmed on an earlier firmware and its source fix is retained.
Ground-test loss after straight travel and after left/right curves. Observe
whether it now makes meaningful grouped motion instead of alternating tiny
pulses, whether failed scans return approximately, and whether capture holds.
If it stops with warning beeps, record the group count or serial fault mask.
Use the immediate rollback point if it performs worse.

## Update protocol

After integrating code, flashing, or receiving a physical result, the
coordinator must update the applicable fields and evidence row. Keep old facts
in Git history instead of accumulating a long diary here. Never mark a physical
test passed from a successful compilation, programmer verification, OLED text,
or encoder counts alone.
