# Shared project state

This checkout contains the continuous four-sensor line follower, SL2.
Live Git identifies source. The new candidate has not been flashed by this
task. The user's latest driving observation is recorded separately from the
older programmer-verified deployment record.

## Machine-readable snapshot

```text
state_schema_version: 1
state_updated_at: 2026-09-05
integration_branch: feature/simple-four-line
repository_head_at_update: 1476354
latest_code_commit: 1476354
flashed_source_commit: 8c4af2b
flash_record_commit: 6c528d8
deployed_tag: deployed/2026-09-05-confirmed-line-resume
formal_bin_path: manual-build-simple-line/simple_four_line.bin
formal_hex_path: manual-build-simple-line/simple_four_line.hex
formal_bin_size_bytes: 68172
flashed_bin_sha256: BE3C5461CAB290E6DD65EF1A5D50F839289DB592880F200034CB1CDFCD654F0D
flashed_hex_sha256: 4E6D00F7A8CD3A9A11F13B88259E9C0C87A6033A8330E26AB3EE9E8C711BB27D
ground_test_status: earlier_image_user_reports_repeated_stops_SL2_unflashed
k210_status: removed
candidate_source_commit: 1476354
candidate_bin_size_bytes: 9616
candidate_bin_sha256: 33B3EA47F0B03B1E415B91D5C58B089CA22CEEB539895953C9B6E1E782BBC781
candidate_hex_sha256: 3C742B22ABB4A7CD4E6B00026B422C2763C2929A8DB852A4AA32E353116840FC
user_reported_flash: current_driving_image_hash_unverified
```

Schema 1 uses `formal_bin_size_bytes` as the last programmer-verified
snapshot size. The active paths point to the new candidate, whose size is
`candidate_bin_size_bytes`. The flashed fields and deployed tag are an
older verified record; they are not proof of which image the user just drove.
The anchor is source commit `1476354`; documentation-only commits may follow.

## Latest user observation and requested change

On 2026-09-05 the user reported that the car stops immediately on line loss,
also moves briefly then stops while still on the line, and requires repeated
button presses to make slow forward progress. The user explicitly requested
removing timeout stops and searching continuously after loss until a remote
stop command.

The previous simple source had several possible automatic-stop triggers:
900 ms turn/search, 300 ms wide line, 60 ms all-white gap with no recent
direction, 50 ms unstable samples, 50 ms control-loop delay and a watchdog
reset path. UART errors also generated STOP. No contemporaneous serial reason
or current Flash hash was supplied, so the exact trigger is not diagnosed.

The revision removes these automatic-stop paths together. The reported
behavior belongs to the earlier image the user drove; SL2 has no new physical
result yet.

## Active candidate and rollback

- Checkout: `F:/myproject/jidian/project/test-exp7-unified-motion-v1`.
- Branch: `feature/simple-four-line`, originally created from `d2efe0e`.
- Source: `1476354` (`Keep simple line tracking active until an operator stop`).
- Current revision identifier in UART status: `SL2`.
- Immediate rollback tag:
  `rollback/2026-09-05-before-continuous-simple-line` -> `1747620`.
- Previous BIN/HEX/ELF/MAP and source manifest were copied to
  `manual-build-simple-line-before-continuous-search`. The saved BIN is
  10048 bytes with SHA-256
  `8720FB0CE906FC4AF3BC7B97E05F64712DE9368495DC7A7DA99F2B498F0177BF`.
- Original project rollback:
  `rollback/2026-09-05-before-simple-four-line` -> `d2efe0e`.
- Build entrypoint remains `build_unified_motion.ps1`, delegating to
  `build_simple_line.ps1`. Build artifacts are ignored by Git.
- Motor PWM, polarity, per-wheel calibration and the 10 ms reversal-coast
  interval are unchanged. This revision does not retune driving speeds.
- Only this checkout was modified. No serial access, flash or remote push
  was performed.

## SL2 behavior and controls

| Input | Action |
|---|---|
| power-on / reset | STOP; held start keys must be released |
| physical KEY1 / KEY2; remote or serial 1 / 2 | start continuous tracking/search |
| physical KEY3; remote or serial 0 / 3 | STOP; held KEY3 wins over START |
| serial s / S / space | STOP |
| serial ? | report raw/filtered sensors and applied logical PWM |

- Each 5 ms control pass uses physical order X2/PF14, X1/PF13, X3/PF15, X4/PG0.
  Black is electrically low and represented by 1.
- The first sample after explicit START decides immediately; subsequent
  changes require two identical samples. Persistent noise keeps the last
  accepted action rather than stopping.
- Centre 0110 drives both sides at 2400. Single inner readings apply
  2200/2600 differential correction. Outer-only or same-side pairs
  counter-rotate at 2700, without a time limit.
- All-white starts or continues an uninterrupted spin toward the last
  observed left/right direction. With no direction in the current run,
  default is left (-1), configurable in `line_config.h`.
- Direction history never expires with elapsed time. Search does not alternate
  just because time passes. A confirmed middle reading restores tracking
  immediately, with no need to reselect a mode.
- Wide/junction/disjoint masks continue slowly at 2200/2200 without a deadline.
  There are no gap, turn, search, wide, noisy-input or loop-delay stop timers.
  No independent watchdog is started by this firmware.
- During ordinary operation, STOP is issued only by operator input. Initial
  boot/reset remains stopped, and CPU/clock faults still disable PWM in the
  fault handler. The 10 ms zero-output reversal interval is not a latched stop.
- UART RX/TX remain interrupt-driven. Damaged input bytes are discarded and
  error flags cleared; a UART error does not invent a STOP event. Valid remote,
  key and serial STOP events retain priority.
- LED1 indicates running; LED2 now indicates SEARCH. Raw sensors still drive
  four RGB components. Status reasons are USER/OK, with prefix SL2.
- Encoders, battery, obstacle sensing, K210 and OLED remain unused by this
  application. PWM zero means coasting, not active mechanical braking.

## Older programmer-verified deployment record

The other worktree is
`F:/myproject/jidian/worktrees/line-reacquire-integration`.
Its branch `feature/line-reacquire-lock` remained at `7a79682` when
checked this turn. That committed record identifies `8c4af2b` as flashed
with record `6c528d8` and tag
`deployed/2026-09-05-confirmed-line-resume`.

The older record says COM11 at 57600 baud, 34 selectively erased pages,
calibration page preserved, 68172-byte write/readback, VERIFY OK and GO OK.
This task did not perform or revalidate that flash. The user's newer driving
report takes precedence for observed behavior, but does not establish the
current image's exact commit/hash. Refresh deployment state before the next
hardware operation. The older integrated image uses KEY3/3 for figure eight;
SL2 uses it for STOP.

## Stable hardware facts

- YB-DSF01-V1.1 / STM32F103ZETx; HSE 8 MHz and system clock 72 MHz.
- M1 left-front/M2 left-rear: TIM8 PC6/7/8/9.
  M3 right-front/M4 right-rear: fully remapped TIM1 PE9/11/13/14.
  PWM is 20 kHz, ARR 3599; polarity remains centralized in motorPWM.c.
- Four repaired AB encoders, M2 on PA15/PB3; no obsolete fallback restored.
- K210 is recorded removed. OLED is the separate J12 display.
- Last 2 KiB Flash page, 0x0807F800..0x0807FFFF, remains reserved and unwritten.
- Complete mapping and historical hardware evidence: `SimpleLine/HARDWARE.md`.

## Evidence for source 1476354 / SL2

| Evidence | Result | Scope |
|---|---|---|
| computer compile/link | passed | GNU Arm GCC; application -Wall -Wextra -Werror; BIN 9616 bytes |
| controller/operator host tests | passed | 169790 assertions; all 16 masks stay active for 10 simulated seconds each, left/right search for 5 simulated minutes each, noise, reacquisition, delayed steps, wraparound and manual STOP |
| board/motor host tests | passed | 18105 assertions; hardware mappings and PWM sweep, reversal timing, busy UART, corrupt bytes ignored, remote 0 decoded by real IRQ code then controller/PWM stop |
| artifact/static checks | passed | all 22 C source hashes match manifest; old controller/watchdog symbols absent; active Line_Stop callers are initialization and operator events |
| flash/readback/GO | not performed this turn | no SL2 deployment proof |
| wheels off ground | not performed this turn | HAL/register stubs do not prove physical motion |
| ground driving | SL2 untested | user's repeated-stop report concerns the earlier running image |

Simulated minutes are host test inputs, not wall-clock vehicle driving.
The previous source's physical failure report does not prove that the new
revision has fixed the real car. No current firmware hash was read back.

## Handoff

Implementation, local source commit, build and relevant host checks are done.
A later explicit flash request must enumerate the current serial port and
preserve the calibration page. After deployment, check the SL2 prefix, an
all-white start and continuous search, reacquisition, and remote 0/3 STOP.
Record flash verification, lifted-wheel behavior and ground driving separately.
