# Shared-state and multi-model workflow

This repository uses committed files, not chat memory, as the durable handoff
channel. A fresh task opened at the repository reads `AGENTS.md`, then validates
`PROJECT_STATE.md` against live Git before doing work.

## Roles

| Role | Model / effort | Use it for | Do not use it for |
|---|---|---|---|
| integration coordinator | current task; normally GPT-5.6 Sol High, or GPT-6 Astra Medium for hard redesigns | user dialogue, decisions, merges, shared state, final build, hardware | parallel ownership of the serial port |
| `architect` | GPT-6 Astra Medium, read-only | cross-module state machines, safety review, difficult root-cause analysis | routine edits or repetitive builds |
| `firmware_worker` | GPT-5.6 Terra Medium | one bounded implementation in a separate worktree | merging, flashing, shared-state edits |
| `fast_checker` | GPT-5.3 Codex Spark Medium | independent build/static/log checks | architecture decisions or hardware claims |

The high-cost model is intentionally reserved for decisions where deeper
reasoning changes the design. Routine implementation and verification use less
expensive models. The project limit is two concurrent delegated threads.

## Choose serial or parallel work

Keep work in one task when changes share a state machine, touch the same files,
depend on a physical observation, or require the same COM port. Parallelize only
independent work after the user explicitly asks for multi-agent/multi-model
execution. Good pairs are read-only architecture plus repository inventory, or
a bounded source patch plus an independent build/review after the patch has a
stable commit.

Never ask two agents to implement competing fixes in the same checkout. Use a
separate branch/worktree for each modifying worker, and give it the exact
`start_commit`. A managed worktree starts from the selected branch HEAD and does
not automatically follow later commits from the coordinator.

## Starting a new task

1. Commit or intentionally shelve the coordinator's current work.
2. Start the new task from the integration branch at its current HEAD.
3. In the prompt, name the repository, role, start commit, allowed files,
   acceptance criteria, and forbidden actions.
4. Require the completion packet below.
5. The coordinator reviews and integrates the returned commit, reruns the formal
   build, and updates `PROJECT_STATE.md`.

For another computer or a cloud task, local commits are not enough: explicitly
push the integration/worker branch when authorized. Local worktrees share Git
objects and refs, but their working files remain isolated.

## Prompt patterns

Architecture review:

```text
Use the architect role read-only. Start from commit <sha>. Read AGENTS.md and
PROJECT_STATE.md. Analyze <bounded question> in <files>. Return a recommended
design, risks, exact integration points, and acceptance tests. Do not edit,
flash, merge, push, or update shared state.
```

Bounded implementation:

```text
Use the firmware_worker role in a separate worktree from commit <sha>. Implement
<one change> only in <allowed files>. Preserve motor polarity and all unrelated
modes. Build and commit the result. Do not use COM, flash, merge, push, or edit
PROJECT_STATE.md. Return the required completion packet.
```

Quota-aware two-model request:

```text
Delegate at most two independent tasks: architect for the read-only design, then
firmware_worker for the bounded implementation after the design is accepted.
Use fast_checker only as a replacement for, not an addition to, a redundant
verification pass. Wait for results. Do not flash.
```

## Worker completion packet

```text
role: architect | firmware_worker | fast_checker
start_commit: <sha>
result_commit: <sha or N/A for read-only work>
files_changed: <paths or none>
verification_completed: <commands and results>
not_verified: <especially physical tests>
risks_or_assumptions: <short list>
integration_notes: <dependencies, conflicts, migration steps>
```

## Coordinator integration checklist

1. Confirm the worker started from the promised commit and changed only its
   assigned files.
2. Review the diff and resolve interfaces in the coordinator checkout.
3. Run `tools/check_project_state.ps1`, the formal build, and
   `git diff --check`.
4. Commit the integrated source.
5. Flash only under current explicit authorization, after enumerating the port
   and preserving the calibration page.
6. Record build, flash/readback, lifted, and ground evidence separately in
   `PROJECT_STATE.md`.
7. Commit the state update. Push only when explicitly requested.
