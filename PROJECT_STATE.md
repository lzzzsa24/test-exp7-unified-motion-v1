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
repository_head_at_update: 045815b
latest_code_commit: 0d31f10
flashed_source_commit: 0d31f10
flash_record_commit: 045815b
deployed_tag: deployed/2026-09-05-buzzer-gpio-fix
formal_bin_path: manual-build-unified-motion/exp7_unified_motion.bin
formal_hex_path: manual-build-unified-motion/exp7_unified_motion.hex
formal_bin_size_bytes: 63816
flashed_bin_sha256: F153C4F39ACBA75A8B48C61C60BE9D1931DDB284B82AA7AA1CD8445053EFB557
flashed_hex_sha256: E20FBC7F4C5FCE9ACD42F8B0B20EBF16AB7561F1425FF79190802055D25B48D0
ground_test_status: buzzer_gpio_fix_flashed_audio_check_pending
k210_status: removed
candidate_source_commit: 0d31f10
candidate_bin_size_bytes: 63816
candidate_bin_sha256: F153C4F39ACBA75A8B48C61C60BE9D1931DDB284B82AA7AA1CD8445053EFB557
candidate_hex_sha256: E20FBC7F4C5FCE9ACD42F8B0B20EBF16AB7561F1425FF79190802055D25B48D0
user_reported_flash: tool_verified_current_candidate
```

`repository_head_at_update` is the source/history anchor present when this
snapshot was written. Documentation-only governance commits may be newer; the
checker requires the anchor to remain an ancestor and prints the live HEAD.

## Active project and deployed firmware

- Repository: `F:\myproject\jidian\project\test-exp7-unified-motion-v1`
- Integration branch: `feature/line-reacquire-lock`
- Latest firmware source commit: `0d31f10` (`Initialize buzzer GPIO in integrated startup`), now flashed. It retains adaptive line recovery and the IR centre-key audio change.
- Flash/readback record: `045815b` (`Record buzzer GPIO fix firmware flash`).
- The formal BIN above was rebuilt from the clean integration checkout, then
  written through the STM32 ROM bootloader on USB-SERIAL CH340K COM11 at
  57600 baud. Selective erase covered 32 firmware pages, preserved the final
  calibration page, wrote and read back 63816 bytes with `VERIFY OK`, and
  completed `GO OK: 0x08000000`.
- Build products under `manual-build-*` are intentionally ignored by Git. A
  different computer must rebuild the named source commit rather than assume
  the artifact was transferred.
- Current formal BIN/HEX were built in this integration checkout from
  `0d31f10`. The preceding adaptive-search build remains under
  `manual-build-adaptive-line-search`.
  Previous isolated candidates remain under the validation directory.
- Immediate rollback tag: `rollback/2026-09-05-before-buzzer-gpio-fix`
  points to `2a7a5d5`. The earlier adaptive-search rollback and build copies
  remain available.

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
- The fixed firmware is flashed and read back, but audible output from the
  physical buzzer still requires user confirmation.

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
| computer build/link | passed | formal `0d31f10`; BIN is 63816 bytes; hashes match the recorded candidate and flashed fields |
| host regression | passed | real line controller with mocked HAL/DriveBase; fresh/unknown direction, widening reversal, opposite-sensor correction, continuation past old limits, capture/brake ownership, watchdog across retries, faults, mode reset and tick rollover; MSVC /W4 /WX |
| flash/readback/GO | passed | COM11 at 57600 baud; 32-page selective erase; calibration page preserved; 63816-byte write and readback; `VERIFY OK`; `GO OK` |
| physical buzzer | pending | PG12 initialization fix is deployed; press the remote centre/sound button in STOP mode to confirm audible output |
| wheels off ground | not performed this turn | diagnostic image compilation is not a lifted-wheel test |
| ground driving | current firmware untested | prior candidate sometimes stopped on line loss; do not transfer that result to deployed `6f32f48` |

## Open issue and next safe step

The buzzer GPIO fix, formal build, host regression, flash, readback verification
and GO are complete. First short-press the remote centre/sound button while the
car remains in STOP. If it is still silent, send serial `b`: this distinguishes
an unmatched remote key from a remaining buzzer hardware/output issue. Physical
audio and ground-driving behavior remain unverified. Subsequent line checks
should still cover loss with no direction hint, acute corners, misleading
initial direction, and capture after a reverse sweep.

## Update protocol

After integrating code, flashing, or receiving a physical result, the
coordinator must update the applicable fields and evidence row. Keep old facts
in Git history instead of accumulating a long diary here. Never mark a physical
test passed from a successful compilation, programmer verification, OLED text,
or encoder counts alone.
