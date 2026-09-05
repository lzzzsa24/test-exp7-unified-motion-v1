# Shared project state

This checkout now contains the independent simple four-sensor candidate.
Source state is live Git. Deployment facts below are explicitly separate from
this unflashed candidate; they were refreshed from the other integration
worktree's committed record during this task.

## Machine-readable snapshot

```text
state_schema_version: 1
state_updated_at: 2026-09-05
integration_branch: feature/simple-four-line
repository_head_at_update: 6f6f014
latest_code_commit: 6f6f014
flashed_source_commit: 8c4af2b
flash_record_commit: 6c528d8
deployed_tag: deployed/2026-09-05-confirmed-line-resume
formal_bin_path: manual-build-simple-line/simple_four_line.bin
formal_hex_path: manual-build-simple-line/simple_four_line.hex
formal_bin_size_bytes: 68172
flashed_bin_sha256: BE3C5461CAB290E6DD65EF1A5D50F839289DB592880F200034CB1CDFCD654F0D
flashed_hex_sha256: 4E6D00F7A8CD3A9A11F13B88259E9C0C87A6033A8330E26AB3EE9E8C711BB27D
ground_test_status: simple_four_line_unflashed_and_untested
k210_status: removed
candidate_source_commit: 6f6f014
candidate_bin_size_bytes: 10048
candidate_bin_sha256: 8720FB0CE906FC4AF3BC7B97E05F64712DE9368495DC7A7DA99F2B498F0177BF
candidate_hex_sha256: EDD564E8033E133787AD543869097DC438A6D15E2C4D698E063F1186678B52F9
user_reported_flash: no_new_simple_line_flash
```

The schema-1 checker uses `formal_bin_size_bytes` as the last flashed
snapshot size, despite the field's legacy name. The active build paths now
point to the new candidate, whose size is `candidate_bin_size_bytes`.
Do not infer deployment from a matching candidate file. The repository anchor
and candidate identify source commit `6f6f014`; state-only commits may follow.

## Active candidate

- Checkout: `F:myprojectjidianproject	est-exp7-unified-motion-v1`.
- Branch: `feature/simple-four-line`, created from `d2efe0e` at the user's request.
- Immutable starting-point tag: `rollback/2026-09-05-before-simple-four-line` -> `d2efe0e`.
- Source: `6f6f014` (`Implement standalone four-sensor line follower`).
- The application was written afresh in `SimpleLine`. Historical projects
  supplied hardware facts, interface mappings and measured motor calibration
  points, not the old driving/recovery state machines.
- The rewritten `Core/Src/motorPWM.c` retains the exact wheel order,
  forward/reverse polarity, TIM mapping and measured calibration points.
  It adds a 10 ms zero-output interval before reversing an energized side.
- The build explicitly selects the new application plus MCU/HAL/runtime
  support. No old DriveBase, recovery, encoder, obstacle or vision controller
  appears in the ELF. CubeIDE Debug/Release selections are synchronized.
- `build_unified_motion.ps1` is the standard entrypoint and delegates to
  `build_simple_line.ps1`. The linker reserves 0x0807F800..0x0807FFFF.
- Build products remain ignored by Git. Rebuild the named source commit on
  another machine. No remote push, serial-port access or flash was performed
  by this task.

## Candidate behavior and controls

All definitions in this section apply after the simple candidate is flashed,
not to the currently recorded integrated firmware.

| Input | Action |
|---|---|
| power-on / reset | STOP; start keys held at boot require release |
| physical KEY1 or KEY2; remote/serial 1 or 2 | start the single line-following mode |
| physical KEY3; remote/serial 0 or 3 | STOP; physical KEY3 wins while held |
| serial s / S / space | STOP |
| serial ? | request current raw/filtered sensors and applied logical PWM |

- Sensor order left-to-right is X2/PF14, X1/PF13, X3/PF15, X4/PG0.
  Black is electrically low and displayed as 1.
- Every 5 ms, two identical samples accept a new mask. Centre readings drive
  straight or apply a two-level differential correction. Outer-only or the
  two same-side readings counter-rotate the sides.
- White after a recent turn continues that direction. Without a recent
  direction, moving over a short gap is limited to 60 ms. Starting on all
  white leaves the motors stopped.
- Sharp turns and searches share a 900 ms deadline. Only 30 ms of centre
  evidence clears it; transient centre hits and repeated START cannot renew it.
- Wide/junction/disjoint masks travel straight slowly for at most 300 ms.
  Continuous unstable input for 50 ms, a control-loop gap above 50 ms,
  or exhausted recovery causes a latched STOP requiring explicit restart.
- An independent watchdog is fed only after each completed control cycle.
  Its nominal interval is 200 ms at 40 kHz LSI; physical timing is unmeasured.
- USART1 RX/TX use interrupts; busy telemetry is dropped. Raw sensors also
  drive four RGB components. LED1 indicates running; LED2 indicates automatic
  fault stop. Status changes chirp the active buzzer briefly.
- This application does not sample encoders, battery, obstacles or K210, and
  does not refresh OLED. PWM zero is coasting rather than mechanical braking.
  It supplies no wheel-stall or low-battery protection.

## Latest recorded deployment, from the other task

During this task, `feature/line-reacquire-lock` independently advanced in
`F:myprojectjidianworktreesline-reacquire-integration`. Its state at
`7a79682` records source `8c4af2b` as flashed, with record `6c528d8`
and tag `deployed/2026-09-05-confirmed-line-resume`.
That record says COM11, 57600 baud, 34 selectively erased firmware pages,
calibration page retained, 68172-byte write/readback, VERIFY OK and GO OK.

These are imported committed deployment facts, not a flash performed or a
physical test witnessed by this simple-line task. The source changes from
that other branch were not merged here. If that branch advances again,
refresh its deployment record before the next hardware operation.

Its integrated KEY3/3 still selects the encoder figure eight. Do not apply the
candidate's new KEY3 STOP meaning until the candidate is actually deployed.

## Hardware facts

- Board: YB-DSF01-V1.1 / STM32F103ZETx, 8 MHz HSE / 72 MHz system clock.
- M1 left-front and M2 left-rear use TIM8 PC6/7/8/9; M3 right-front and M4
  right-rear use fully remapped TIM1 PE9/11/13/14. PWM is 20 kHz, ARR 3599.
- The four AB encoders are recorded repaired. M2 is PA15/PB3; the old single-edge
  fallback is obsolete. They are not used by this candidate.
- K210 is recorded physically removed. The OLED is the separate J12 device.
- Historical PWM calibration was wheels-off-ground evidence, not loaded
  vehicle speed or yaw evidence. No new physical observations were supplied.
- Full mapping and evidence sources: `SimpleLine/HARDWARE.md`.

## Evidence ledger for the simple candidate

| Evidence | Result | Scope |
|---|---|---|
| computer compile/link | passed | GNU Arm GCC, application -Wall -Wextra -Werror; BIN 10048 bytes |
| controller/operator host tests | passed | 237 assertions; all 16 masks, filtering, timeout bounds, repeat/held keys, NEC and tick wrap |
| board/motor host tests | passed | 18079 assertions including full PWM sweep, GPIO order, direction mapping, reversal interval, UART; HAL/register stubs |
| artifact / IDE static checks | passed | 22 compiled C source hashes match manifest; old control symbols absent; Debug/Release source exclusions checked |
| flash/readback/GO | not performed | this simple candidate has never been deployed by this task |
| wheels off ground | not performed | host register tests do not prove motor motion |
| ground driving | not performed | PWM and timeout values remain initial settings for actual testing |

The old firmware's buzzer pass or flash record does not validate the new
candidate's motor or sensor behavior. Details and use instructions are in
`SimpleLine/README.md`.

## Next step and update protocol

The requested branch implementation, local source commit, build and host
verification are complete. No hardware operation is currently authorized.
A later explicit flash request should first enumerate ports and refresh the
other branch's deployment state, retain the calibration page, then verify the
exact candidate. Afterwards separately check sensor order, lifted-wheel
directions/STOP, and ground straight/left/right/loss/wide-line behavior.

Only record physical conclusions actually observed. Keep source, candidate
artifacts and last flashed facts separate; never call host tests or byte
readback a successful ground test.
