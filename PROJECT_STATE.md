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
repository_head_at_update: c7afd6d
latest_code_commit: c7afd6d
flashed_source_commit: 4f5bae1
flash_record_commit: fb47209
deployed_tag: deployed/2026-09-05-forward-biased-line-search
formal_bin_path: manual-build-unified-motion/exp7_unified_motion.bin
formal_hex_path: manual-build-unified-motion/exp7_unified_motion.hex
formal_bin_size_bytes: 63464
flashed_bin_sha256: A7A4E702F7E5F6EC707B0D82A0D74A3EDB85119EF610D1197FB0F73A12F2D6C6
flashed_hex_sha256: 1372B9C0864322D23589F8476B74B11A13020E670ADBA95D31DD96A874D7DABC
ground_test_status: deployed_search_failed_combined_candidate_unflashed
k210_status: removed
candidate_source_commit: c7afd6d
candidate_bin_size_bytes: 63980
candidate_bin_sha256: 1AED628A7217697162EC4570AA7B9C263E00E52177171E2B822DD9B839ED940C
candidate_hex_sha256: 0A03B47B3D6D9C181C486775200C5B0ECD47A9DBC87E19096F92C1E75AA284AA
```

`repository_head_at_update` is the source/history anchor present when this
snapshot was written. Documentation-only governance commits may be newer; the
checker requires the anchor to remain an ancestor and prints the live HEAD.

## Active project and deployed firmware

- Repository: `F:\myproject\jidian\project\test-exp7-unified-motion-v1`
- Integration branch: `feature/line-reacquire-lock`
- Latest firmware source commit: `c7afd6d` (`Bind IR center key to one-shot buzzer phrase`), not flashed. It includes the bounded line-search commit `83ca102`.
- Deployed firmware remains `4f5bae1` (forward-biased rear-pivot search).
- Flash/readback record: `fb47209` (`Record forward-biased search firmware flash`)
- The formal BIN above was written over 31 pages through the bootloader at
  57600 baud with the last calibration page preserved. Full readback reported
  `VERIFY OK`, and GO succeeded.
- Build products under `manual-build-*` are intentionally ignored by Git. A
  different computer must rebuild the named source commit rather than assume
  the artifact was transferred.
- The latest validated combined candidate is in the detached validation worktree
  `F:\myproject\jidian\validation\ir-center-audio-20260905`, at `c7afd6d`.
  BIN/HEX: `manual-build-unified-motion/exp7_unified_motion.bin` and `.hex`
  under that worktree. The earlier line-only candidate remains in
  `F:\myproject\jidian\validation\line-recovery-20260905` at `83ca102`.
- Rollback source tag: `rollback/2026-09-05-before-bounded-line-search`.
  Original deployed BIN/HEX were copied to
  `manual-build-rollback-forward-search` before rebuilding.

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

Latest user ground observation (2026-09-05): at bends the car loses the line,
search rotation moves it backward, and the predicted direction often leads it
back along the incoming route. Under the existing attribution rule this is
feedback on deployed `4f5bae1`; no new flash or instrumented measurement occurred.

The unflashed combined `c7afd6d` candidate includes the `83ca102` KEY1/KEY2
recovery changes as follows:

- Both left wheels and both right wheels counter-rotate with equal signed
  travel magnitudes and a 3600 CPS limit, using `EncoderTurn_Start(angle, 0, cps)`.
  Rear wheels are no longer left unpowered. Motor polarity is unchanged.
- Direction uses the physical X2/X1/X3/X4 weighted position after 20 ms of
  consistent evidence. Opposing evidence immediately invalidates an old hint;
  hints expire after 200 ms, clear after 80 ms centred, and clear at crossings.
  Filtered steering commands and historical middle-pair departures are not
  fallback direction sources.
- Each recovery episode has an 85-degree encoder travel estimate budget and
  a 2500 ms deadline. Brake travel is sampled directly from encoders. A failed
  capture keeps the remaining budget and original deadline instead of starting
  another full revolution. These limits do not measure actual chassis yaw.
- Outer-only hits retain search ownership; a middle hit requires 12 ms before
  braking. Capture uses 250 ms of low-speed tracking, and centred sensors command
  straight travel rather than continued steering from the old prediction.
- Unknown direction, exhausted travel, timeout or turn fault latches STOP.
  Stop and reselect a mode to retry. KEY1's never-seen-line forward policy is
  retained. Normal on-line steering is unchanged.

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
| computer build/link | passed | isolated combined candidate `c7afd6d`; BIN is 63980 bytes; hashes in candidate fields; audio symbols and centre-key diagnostic strings are present in ELF |
| host regression | passed | exact `c7afd6d` worktree: real line/turn sources with mocked HAL/DriveBase; left/right histories, four-wheel targets, capture ownership, brake travel, cumulative limit, timeout, faults, mode reset and tick rollover; MSVC /W4 /WX |
| flash/readback/GO | candidate not flashed | historical deployed `4f5bae1` verification still applies only to flashed hash fields; calibration page preserved |
| wheels off ground | not separately recorded | do not infer from flash success |
| ground driving | deployed search failed; combined candidate untested | latest user report records backward search and returning along incoming line; remote audio has not been heard on hardware |

## Open issue and next safe step

Combined candidate implementation, formal build and host regression are
complete. Flash only after fresh explicit authorization, selecting an artifact
whose source and hash are known. First verify at rest that one centre-button
press produces one phrase and leaves the mode unchanged; then check both
left/right turns, backward displacement, capture and timeout on the intended
surface. The conservative search cap may stop on tight bends and needs ground
evaluation. Four digital probes cannot identify route direction once an
indistinguishable incoming segment is under them; this change reduces
stale-direction and long-spin causes, not a proof against all route reversals.

## Update protocol

After integrating code, flashing, or receiving a physical result, the
coordinator must update the applicable fields and evidence row. Keep old facts
in Git history instead of accumulating a long diary here. Never mark a physical
test passed from a successful compilation, programmer verification, OLED text,
or encoder counts alone.
