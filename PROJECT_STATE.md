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
repository_head_at_update: fb47209
latest_code_commit: 4f5bae1
flashed_source_commit: 4f5bae1
flash_record_commit: fb47209
deployed_tag: deployed/2026-09-05-forward-biased-line-search
formal_bin_path: manual-build-unified-motion/exp7_unified_motion.bin
formal_hex_path: manual-build-unified-motion/exp7_unified_motion.hex
formal_bin_size_bytes: 63464
flashed_bin_sha256: A7A4E702F7E5F6EC707B0D82A0D74A3EDB85119EF610D1197FB0F73A12F2D6C6
flashed_hex_sha256: 1372B9C0864322D23589F8476B74B11A13020E670ADBA95D31DD96A874D7DABC
ground_test_status: pending
k210_status: removed
```

`repository_head_at_update` is the source/history anchor present when this
snapshot was written. Documentation-only governance commits may be newer; the
checker requires the anchor to remain an ancestor and prints the live HEAD.

## Active project and deployed firmware

- Repository: `F:\myproject\jidian\project\test-exp7-unified-motion-v1`
- Integration branch: `feature/line-reacquire-lock`
- Latest firmware source commit: `4f5bae1` (`Add forward bias to rear-pivot line search`)
- Flash/readback record: `fb47209` (`Record forward-biased search firmware flash`)
- The formal BIN above was written over 31 pages through the bootloader at
  57600 baud with the last calibration page preserved. Full readback reported
  `VERIFY OK`, and GO succeeded.
- Build products under `manual-build-*` are intentionally ignored by Git. A
  different computer must rebuild the named source commit rather than assume
  the artifact was transferred.

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

The latest change affects the lost-line search used by KEY1 and KEY2. It keeps
the rear axle stationary so the nose sweeps for the line, but adds forward bias
to avoid the whole chassis walking backward:

- outer front wheel: forward at 3600 encoder counts/s;
- inner front wheel: reverse at 45 percent, about 1620 counts/s;
- both rear wheels: stopped;
- search direction still comes from the last reliable line-side prediction;
- normal on-line steering is unchanged.

The immediately preceding equal-front-speed rear-pivot version caused a large
backward displacement on the real car. This forward-biased version has been
built and flashed, but the user has not yet reported its ground result. Unless
the user explicitly says otherwise, their next behavior report refers to this
latest flashed version.

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
| computer build/link | passed | formal BIN/HEX derived from `4f5bae1`; BIN is 63464 bytes |
| flash/readback/GO | passed | exact BIN hash above; calibration page preserved |
| wheels off ground | not separately recorded | do not infer from flash success |
| ground driving | pending | forward-biased lost-line search awaits user feedback |

## Open issue and next safe step

The open question is whether the forward-biased nose sweep reacquires the line
without large backward motion or repeated local crawling. Obtain one controlled
ground observation before changing several gains at once. If it still fails,
record the detected sensor pattern, predicted search side, and which wheels
actually turn, then tune one search parameter or fix one ownership transition.

## Update protocol

After integrating code, flashing, or receiving a physical result, the
coordinator must update the applicable fields and evidence row. Keep old facts
in Git history instead of accumulating a long diary here. Never mark a physical
test passed from a successful compilation, programmer verification, OLED text,
or encoder counts alone.
