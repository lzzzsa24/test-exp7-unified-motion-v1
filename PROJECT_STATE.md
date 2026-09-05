# Shared project state

> Isolated historical test worktree: `test/sep3-predicted-spin`.
> Integration deployment below remains a reference snapshot, not this test's
> deployment claim. Candidate `3733303` restores the September 3 14:09 timed
> single-direction line decisions through the current DriveBase. See
> `SEP3_LINE_TEST.md` for provenance, limitations and test instructions.
> Newest user observation: current four-sensor tracking is worse than the early
> September 3 version; the remembered version did not reverse after losing line.
> This candidate is built and host-tested only; no flashing or physical test.

This is the handoff record for humans, Codex tasks, and delegated models. It is
not a substitute for Git: the checker resolves the live branch and HEAD every
time. Only the integration coordinator updates this file after a merge, flash,
or physical test.

## Machine-readable snapshot

```text
state_schema_version: 1
state_updated_at: 2026-09-05
integration_branch: feature/line-reacquire-lock
repository_head_at_update: 62d29ba
latest_code_commit: dec0a27
flashed_source_commit: dec0a27
flash_record_commit: 62d29ba
deployed_tag: deployed/2026-09-05-geometry-line-search
formal_bin_path: manual-build-unified-motion/exp7_unified_motion.bin
formal_hex_path: manual-build-unified-motion/exp7_unified_motion.hex
formal_bin_size_bytes: 63796
flashed_bin_sha256: 15BE841FDB40707F93DED4C3058B02057C9460C6ACA018B733FDFCAB8F4AEA89
flashed_hex_sha256: A9B1C95DBC80FDA656BFE14346EFD09A85BDF7983971D72402C170D52A290E3B
ground_test_status: geometry_line_search_flashed_ground_test_pending_buzzer_passed
k210_status: removed
candidate_source_commit: 3733303
candidate_bin_size_bytes: 61376
candidate_bin_sha256: F7E8173E5416C9234D887029BA2BDB1D15B166DC4DBE2D25642B83FE74837F88
candidate_hex_sha256: 85D8B425892776AED7CBF7F4F01D0DD80F3BDA7FE944D98A85C61346C29AE1CA
user_reported_flash: historical_test_candidate_not_flashed
```

`repository_head_at_update` is the source/history anchor present when this
snapshot was written. Documentation-only governance commits may be newer; the
checker requires the anchor to remain an ancestor and prints the live HEAD.

## Active project and deployed firmware

- Repository: `F:\myproject\jidian\project\test-exp7-unified-motion-v1`
- Integration branch: `feature/line-reacquire-lock`
- Latest firmware source commit: `dec0a27` (`Derive symmetric line search targets from chassis geometry`), now flashed. It applies the requested `18774a2` change on top of the current integration history, retaining the buzzer GPIO fix, adaptive line recovery and IR centre-key audio.
- Flash/readback record: `62d29ba` (`Record geometry-derived line search firmware flash`).
- The formal BIN above was rebuilt from the clean integration checkout, then
  written through the STM32 ROM bootloader on USB-SERIAL CH340K COM11 at
  57600 baud. Selective erase covered 32 firmware pages, preserved the final
  calibration page, wrote and read back 63796 bytes with `VERIFY OK`, and
  completed `GO OK: 0x08000000`.
- Build products under `manual-build-*` are intentionally ignored by Git. A
  different computer must rebuild the named source commit rather than assume
  the artifact was transferred.
- Current formal BIN/HEX were built in this integration checkout from
  `dec0a27`. The preceding adaptive-search build remains under
  `manual-build-adaptive-line-search`.
  Previous isolated candidates remain under the validation directory.
- Immediate rollback tag: `rollback/2026-09-05-before-geometry-line-search`
  points to `ab62f3f`. The buzzer GPIO fix and earlier adaptive-search rollback
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

Latest user ground observation (2026-09-05): after self-flashing the previous
turn's new firmware, the car sometimes does not move after losing the line.
The user also rejects a fixed angular cutoff because the course has acute
corners. This supersedes the previous assumption that their observation
necessarily refers to the old rear-pivot image.

The deployed recovery implementation changes KEY1/KEY2 recovery as follows:

- Search directly commands four-wheel speed feedback at equal and opposite
  left/right targets derived from the measured chassis geometry. The default
  120 deg/s nominal yaw converts to 2493 CPS on every wheel on the same axle
  side. It no longer requests a position move, and
  neither encoder travel nor an estimated chassis angle ends recovery.
- Recent stable sensor-position hints choose the initial direction. If no
  reliable hint exists, start a short left exploratory leg rather than
  latching STOP. This default is a probe, not a claim about route direction.
- With a reliable hint the geometry-scaled first leg is 1300 ms; without one
  it is 362 ms.
  If a leg finds no useful sensor evidence, brake, wait a 70 ms guard, reverse,
  and double its duration up to 3466 ms. A matching outer sensor allows the
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
- Commit `dec0a27` replaces the experimental front/rear 60-percent allocation
  with symmetric front/rear targets. Its conversion uses the measured 129 mm
  track width, 47 mm wheel diameter, 1040 counts/revolution and the existing
  skid estimate, producing a 338 mm effective track. The timers preserve the
  previous nominal search travel rather than imposing a turn-angle limit.
  Normal on-line steering and capture remain unchanged. This is still a
  wheel-speed model; encoders cannot by themselves prove ground yaw or pivot
  position.

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
  deployed `dec0a27`.

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
| computer build/link | passed | integrated formal `dec0a27`; BIN is 63796 bytes; buzzer fix retained |
| host regression | passed | default 120 deg/s / 2493 CPS and comparison 90 deg/s / 1870 CPS profiles both pass geometry conversion, symmetric wheel targets, on-line isolation, recovery, capture, watchdog, faults, reset and tick rollover; MSVC /W4 /WX |
| flash/readback/GO | passed | COM11 at 57600 baud; 32-page selective erase; calibration page preserved; 63796-byte write and readback; `VERIFY OK`; `GO OK` |
| physical buzzer | passed | user explicitly confirmed `响了` after the PG12 initialization fix; fix retained in current firmware |
| wheels off ground | not performed this turn | diagnostic image compilation is not a lifted-wheel test |
| ground driving | current geometry-derived search firmware untested | symmetric 2493-CPS search requires same-surface comparison; do not infer chassis yaw from the build |

## Open issue and next safe step

The requested geometry-derived search integration, formal build, both host
regression profiles, flash, readback verification and GO are complete.
Physical buzzer output was confirmed on the preceding firmware and its source
fix is retained. Ground-test lost-line search in both directions and observe
whether the car rotates symmetrically, advances unexpectedly, or repeatedly
misses capture. Use the immediate rollback point if it performs worse.

## Update protocol

After integrating code, flashing, or receiving a physical result, the
coordinator must update the applicable fields and evidence row. Keep old facts
in Git history instead of accumulating a long diary here. Never mark a physical
test passed from a successful compilation, programmer verification, OLED text,
or encoder counts alone.
