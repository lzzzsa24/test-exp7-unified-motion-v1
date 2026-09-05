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
repository_head_at_update: 83ca102
latest_code_commit: 83ca102
flashed_source_commit: 4f5bae1
flash_record_commit: fb47209
deployed_tag: deployed/2026-09-05-forward-biased-line-search
formal_bin_path: manual-build-unified-motion/exp7_unified_motion.bin
formal_hex_path: manual-build-unified-motion/exp7_unified_motion.hex
formal_bin_size_bytes: 63464
flashed_bin_sha256: A7A4E702F7E5F6EC707B0D82A0D74A3EDB85119EF610D1197FB0F73A12F2D6C6
flashed_hex_sha256: 1372B9C0864322D23589F8476B74B11A13020E670ADBA95D31DD96A874D7DABC
ground_test_status: deployed_search_failed_candidate_unflashed
k210_status: removed
candidate_source_commit: 83ca102
candidate_bin_size_bytes: 63864
candidate_bin_sha256: 7154A25E6567C4A469A5B76BA3E4D35505794BE70430EC097EF3CF02BC3043E2
candidate_hex_sha256: 04796819AF83A53C11CAEFA40DAFC1A4C365E5DA2C33C180292EA5220401A3F9
```

`repository_head_at_update` is the source/history anchor present when this
snapshot was written. Documentation-only governance commits may be newer; the
checker requires the anchor to remain an ancestor and prints the live HEAD.

## Active project and deployed firmware

- Repository: `F:\myproject\jidian\project\test-exp7-unified-motion-v1`
- Integration branch: `feature/line-reacquire-lock`
- Latest firmware source commit: `83ca102` (`Bound lost-line search and refresh sensor direction hints`), not flashed.
- Deployed firmware remains `4f5bae1` (forward-biased rear-pivot search).
- Flash/readback record: `fb47209` (`Record forward-biased search firmware flash`)
- The formal BIN above was written over 31 pages through the bootloader at
  57600 baud with the last calibration page preserved. Full readback reported
  `VERIFY OK`, and GO succeeded.
- Build products under `manual-build-*` are intentionally ignored by Git. A
  different computer must rebuild the named source commit rather than assume
  the artifact was transferred.
- The validated candidate is in the detached validation worktree
  `F:\myproject\jidian\validation\line-recovery-20260905`, at `83ca102`.
  BIN/HEX: `manual-build-unified-motion/exp7_unified_motion.bin` and `.hex`
  under that worktree. This separates the candidate from concurrent unrelated
  IR/buzzer edits and builds in the integration directory. The integration
  directory's current BIN/HEX must not be assumed to match this candidate.
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

The infrared remote also supplies the virtual mode keys and a stop command.

## Current line-loss behavior

Latest user ground observation (2026-09-05): at bends the car loses the line,
search rotation moves it backward, and the predicted direction often leads it
back along the incoming route. Under the existing attribution rule this is
feedback on deployed `4f5bae1`; no new flash or instrumented measurement occurred.

The unflashed `83ca102` candidate changes KEY1/KEY2 recovery as follows:

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
| computer build/link | passed | isolated candidate `83ca102`; BIN is 63864 bytes; hashes in candidate fields |
| host regression | passed | real line/turn sources with mocked HAL/DriveBase; left/right histories, four-wheel targets, capture ownership, brake travel, cumulative limit, timeout, faults, mode reset and tick rollover; MSVC /W4 /WX |
| flash/readback/GO | candidate not flashed | historical deployed `4f5bae1` verification still applies only to flashed hash fields; calibration page preserved |
| wheels off ground | not separately recorded | do not infer from flash success |
| ground driving | deployed search failed; candidate untested | latest user report records backward search and returning along incoming line |

## Open issue and next safe step

Candidate implementation and host regression are complete. Flash only after
fresh explicit authorization, selecting an artifact whose source and hash are
known; do not assume a concurrent integration-directory build is this candidate.
Then check both left/right turns, backward displacement, capture and timeout on
the intended surface. The conservative search cap may stop on tight bends and
needs ground evaluation. Four digital probes cannot identify route direction
once an indistinguishable incoming segment is under them; this change reduces
stale-direction and long-spin causes, not a proof against all route reversals.

## Update protocol

After integrating code, flashing, or receiving a physical result, the
coordinator must update the applicable fields and evidence row. Keep old facts
in Git history instead of accumulating a long diary here. Never mark a physical
test passed from a successful compilation, programmer verification, OLED text,
or encoder counts alone.
