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
repository_head_at_update: b35b30d
latest_code_commit: 6f32f48
flashed_source_commit: 6f32f48
flash_record_commit: b35b30d
deployed_tag: deployed/2026-09-05-adaptive-line-search
formal_bin_path: manual-build-unified-motion/exp7_unified_motion.bin
formal_hex_path: manual-build-unified-motion/exp7_unified_motion.hex
formal_bin_size_bytes: 63772
flashed_bin_sha256: 4F4E186E8D841BC7B12D74087E7F7D77D017BDCD969E1BFC5378B8E0A856C972
flashed_hex_sha256: 31B5C81AF07CF64EE05E0E997127A3D81D8B6CF1F50EFC00B3C5F69176E5819A
ground_test_status: adaptive_search_flashed_physical_tests_pending
k210_status: removed
candidate_source_commit: 6f32f48
candidate_bin_size_bytes: 63772
candidate_bin_sha256: 4F4E186E8D841BC7B12D74087E7F7D77D017BDCD969E1BFC5378B8E0A856C972
candidate_hex_sha256: 31B5C81AF07CF64EE05E0E997127A3D81D8B6CF1F50EFC00B3C5F69176E5819A
user_reported_flash: tool_verified_current_candidate
```

`repository_head_at_update` is the source/history anchor present when this
snapshot was written. Documentation-only governance commits may be newer; the
checker requires the anchor to remain an ancestor and prints the live HEAD.

## Active project and deployed firmware

- Repository: `F:\myproject\jidian\project\test-exp7-unified-motion-v1`
- Integration branch: `feature/line-reacquire-lock`
- Latest firmware source commit: `6f32f48` (`Recover lost lines with sensor-guided widening sweeps`), now flashed. It retains the IR centre-key audio change.
- Flash/readback record: `b35b30d` (`Record adaptive line search firmware flash`).
- The formal BIN above was rebuilt from the clean integration checkout, then
  written through the STM32 ROM bootloader on USB-SERIAL CH340K COM11 at
  57600 baud. Selective erase covered 32 firmware pages, preserved the final
  calibration page, wrote and read back 63772 bytes with `VERIFY OK`, and
  completed `GO OK: 0x08000000`.
- Build products under `manual-build-*` are intentionally ignored by Git. A
  different computer must rebuild the named source commit rather than assume
  the artifact was transferred.
- Current formal BIN/HEX were built in this integration checkout from
  `6f32f48`. A named copy is under `manual-build-adaptive-line-search`.
  Previous isolated candidates remain under the validation directory.
- Rollback source tag: `rollback/2026-09-05-before-adaptive-line-search`
  points to `9f36f46`. Pre-change combined BIN/HEX were copied into
  `manual-build-rollback-bounded-search`; the earlier deployed image remains
  in `manual-build-rollback-forward-search`.

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

Latest user ground observation (2026-09-05): after self-flashing the previous
turn's new firmware, the car sometimes does not move after losing the line.
The user also rejects a fixed angular cutoff because the course has acute
corners. This supersedes the previous assumption that their observation
necessarily refers to the old rear-pivot image.

The deployed `6f32f48` implementation changes KEY1/KEY2 recovery as follows:

- Search directly commands four-wheel speed feedback at equal and opposite
  left/right targets of 3600 CPS. It no longer requests a position move, and
  neither encoder travel nor an estimated chassis angle ends recovery.
- Recent stable sensor-position hints choose the initial direction. If no
  reliable hint exists, start a short left exploratory leg rather than
  latching STOP. This default is a probe, not a claim about route direction.
- With a reliable hint the first leg is 900 ms; without one it is 250 ms.
  If a leg finds no useful sensor evidence, brake, wait a 70 ms guard, reverse,
  and double its duration up to 2400 ms. A matching outer sensor allows the
  leg to continue past that duration while approaching the line.
- An opposite outer-only hit confirmed for 20 ms corrects the search side via
  the same brake/guard transition. Outer-only hits retain recovery ownership.
  A middle hit requires 12 ms confirmation before active braking and 250 ms
  of low-speed capture. Centred capture commands straight travel.
- Failed captures keep the original episode watchdog. Unknown direction or a
  completed leg no longer latches STOP. An 8-second whole-episode watchdog
  or a DriveBase fault still stops the car; stop/reselect a mode to retry.
- Mode reset and a zero base-speed command cancel directly owned search/brake.
  KEY1's never-seen-line forward policy and normal on-line steering remain.

## Current infrared-remote audio behavior

- The direction-pad centre/buzzer command `0x05` maps to a dedicated
  `IR_REMOTE_VIRTUAL_AUDIO_ONCE` event.
- One complete NEC frame starts exactly one 1.53-second preset five-attack
  phrase and does not start, stop, or change a driving mode.
- NEC repeat frames remain suppressed, so holding the key does not queue
  repeated playback. A later fresh press restarts one complete phrase.
- Existing stop/fault, encoder, ultrasonic and bypass warning arbitration can
  immediately cancel this lower-priority audio.
- The serial `b` command remains an equivalent one-shot diagnostic entry.

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
| computer build/link | passed | formal `6f32f48`; BIN is 63772 bytes; hashes match the recorded candidate and flashed fields |
| host regression | passed | real line controller with mocked HAL/DriveBase; fresh/unknown direction, widening reversal, opposite-sensor correction, continuation past old limits, capture/brake ownership, watchdog across retries, faults, mode reset and tick rollover; MSVC /W4 /WX |
| flash/readback/GO | passed | COM11 at 57600 baud; 32-page selective erase; calibration page preserved; 63772-byte write and readback; `VERIFY OK`; `GO OK` |
| wheels off ground | not performed this turn | diagnostic image compilation is not a lifted-wheel test |
| ground driving | current firmware untested | prior candidate sometimes stopped on line loss; do not transfer that result to deployed `6f32f48` |

## Open issue and next safe step

Adaptive-search implementation, formal build, host regression, flash, readback
verification and GO are complete. Physical behavior remains unverified. Ground
checks should include loss with no direction hint, left/right acute corners,
misleading initial direction, and capture after a reverse sweep. Verify actual
backward displacement separately from encoder motion. The sweep durations and
8-second watchdog are initial parameters, not ground-calibrated results. Four
digital probes cannot reliably distinguish an incoming segment from an outgoing
segment with the same sensor pattern; sensor-guided search reduces blind
rotation but cannot prove route identity.

## Update protocol

After integrating code, flashing, or receiving a physical result, the
coordinator must update the applicable fields and evidence row. Keep old facts
in Git history instead of accumulating a long diary here. Never mark a physical
test passed from a successful compilation, programmer verification, OLED text,
or encoder counts alone.
