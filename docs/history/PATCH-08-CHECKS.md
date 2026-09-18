# Patch 8 (Phase 1) — what changed, and how to check it

Comms realism, one real bug found live-testing patch 7, and the two tree
headers pulled forward from the Phase 2 design. Custom mode, SPAWN/
RELOCATE/RENAME/DESPAWN, NETWORK/SQUAD, multi-terminal and SAVE SCENARIO
are designed and approved but **not** in this patch — see
`docs/history/BACKLOG.md` / the saved plan for that design.

---

## 1. Bug fix: the retask race (typing a command twice)

### What was actually happening

Live-testing patch 7 in the Console, several commands needed to be typed
twice before taking effect (`blue launch`, `blue halt`, `REOBJECTIVE car1
pursue car3`, `REOBJECTIVE car1 shuttle (...)`, `REOBJECTIVE car1 stop`),
and a mistyped command (`blut halt`) sometimes rode along with its
correction, applied together in one batch.

Root cause: the retask channel was one shared file, `runs/retask/queue`,
that the Console appended to and `drain_retasks()` read then cleared each
poll. A previous patch (7) had already fixed the most obvious version of
that race (read-then-delete racing a concurrent append) by renaming the
file before reading it — but the rename itself has the exact same
exposure: on Windows, a second process cannot rename or delete a file the
first still has open, because Python's `open()` doesn't request
`FILE_SHARE_DELETE`. When the Console's (brief) append handle and the
stub's poll happened to overlap, `queue.rename(...)` raised, and
`except OSError: pending = None` swallowed it silently — skipping the
**entire** read for that poll, not just the contended part. Because the
Console only ever appends, commands already sitting in the file stayed
there, unprocessed, while more got appended on top, until some later poll
finally got a clean rename and processed everything accumulated in one go.
That is precisely why retyping "worked" (it wasn't a second attempt
succeeding, it was the first attempt finally getting through) and why a
typo could ride along with its correction (both were stuck in the same
unclaimed file).

### The fix

Stop sharing one file. Each command is now its own brand-new file,
`cmd_<monotonic-nanoseconds>.txt`, written by the Console to a temp name
and atomically renamed into place (`console/app.py`,
`ShellPanel._send_queue_line`). `drain_retasks()`
(`tools/stub_telemetry.py`) globs `cmd_*.txt`, claims each one it finds by
renaming it to a per-process `.reading` suffix, reads it, deletes it — and
now **logs any failure at every step** rather than swallowing it (`print(
..., file=sys.stderr)`, visible in the Console's Log). Since nothing ever
reopens an existing `cmd_*.txt` path (the Console only ever creates new
ones), there is no longer a writer for the reader to contend with — a
claim failure should now be genuinely rare, not routine, and even if one
happens the file is left in place for the next poll rather than lost.
`start_run()`'s pre-run cleanup now clears the whole spool (`cmd_*` plus
any stray legacy `queue` file) instead of just the old single path.

### Check

1. `python3 tests/test_all.py` — expect **94 passed, 0 failed**. Two new
   tests: `test_retask_spool_is_one_file_per_command` (order preserved,
   stale `.reading` leftovers are inert) and
   `test_retask_claim_failure_is_never_silent_and_not_lost` — this one
   simulates the actual bug (a claim that raises `OSError`, as Windows
   did) and asserts the failure is printed, nothing is processed on that
   attempt, the file survives, and the very next poll picks it up cleanly.
2. A concurrency stress test (writer thread + polling reader, 200 commands,
   run outside the suite) delivered 200/200 with zero stuck files — not a
   substitute for Will confirming it live on Windows, but real evidence
   the new design has no logic bug under genuine concurrent pressure.
3. **Live, in the Console**: `blue launch`, then `blue halt`, then
   `REOBJECTIVE car1 pursue car3`, then `REOBJECTIVE car1 shuttle
   (-3,2,0) (3,2,0)`, then `REOBJECTIVE car1 stop` — each should take
   effect on the first attempt. If any command still needs retyping, that
   is new information (the fix removes the *known* contention; report
   back exactly what didn't take first time).

---

## 2. Comms Quality column

`console/app.py`'s link table header changed "PDR" → "Quality", and
`fill_comms()` now formats it as a percentage (`f"{pdr*100:.2f}%"`)
instead of a bare 2-decimal fraction (`f"{pdr:.2f}"`, which flattened a
genuinely-computed 0.998 down to "1.00"). The data was always there
(`rf_link()` already computes `pdr` to 3dp); this was a display-precision
bug, not a missing-data one. At typical lab distances it will still
honestly read "100.00%" — the fix makes the real number visible so it's
watchable as distance/jamming/interference actually change it, not
manufactures loss that isn't there.

### Check
4. Comms tab, any open scenario with links: header reads "Quality", values
   are percentages. `python3 -c "..."` against `rf_link()` directly (see
   session transcript) confirms the percentage genuinely moves at
   realistic distances (100.00% → 99.90% → 96.20% → 80.90% → 60.40% from
   1 m to 200 m) even though it saturates to 100.00% at lab-room scale.

## 3. Self-jamming seam (comment only)

One comment added at `apply_routing()`'s `rf_link(poses[a], poses[b])`
call (`tools/stub_telemetry.py`), marking it as the future hook for real
self-interference (sum received power from every other active
same-spectrum transmitter, pass as `interference_mw` — the parameter
already exists on `rf_link()`, unused). No behaviour change; nothing to
check beyond confirming the comment is there and the routing loop's output
is identical.

---

## 4. Tree headers — Overview vs. Mission, decoupled

Pulled forward from the Phase 2 design since it's small and standalone.

- **Overview**'s root now reads `Scenario: <name>` (or `Scenario: Custom`
  with nothing named) instead of `Mission: <name>` — Overview is
  equipment; it no longer gets headed by tasking.
- **Mission**'s root now reads `Mission: UNASSIGNED` until something is
  genuinely **assigned at runtime** — REOBJECTIVE, REMISSION, or
  LOADMISSION taking effect. Critically, this is true even for a legacy
  `scenarios/` file whose agents already carry baked-in objectives (every
  one of them does): opening `three_car_fleet.yaml` reads `Mission:
  UNASSIGNED` until a live command actually changes something, because the
  file's own baked-in tasking was never *assigned*, it was just *loaded*.
  Once any agent's live objective first differs from what the file said,
  the header flips to `Mission: assigned (live)` and stays there for the
  rest of the run (reset back to `UNASSIGNED` on the next fresh
  `load_scenario()` call). Doesn't yet distinguish "assigned via
  LOADMISSION <file>" with the file's own name — everything live-assigned
  reads the same generic marker; naming it more specifically is a small
  follow-up, not done here.

### Check
5. Open `scenarios/three_car_fleet.yaml`. Overview tree root: `Scenario:
   three_car_fleet`. Mission tree root: `Mission: UNASSIGNED`, even though
   the objective rows underneath already show each car's baked-in shuttle
   lane. `blue launch` then `REOBJECTIVE car1 pursue car3` — Mission root
   flips to `Mission: assigned (live)`. Opening a different file resets
   both headers.

---

## Design questions answered this session, not built

- **A network/authority view — its own tab, not folded into Comms.**
  Agreed with Will's instinct: Comms is RF measurement (SINR/PDR/state per
  link); a network view is command structure — declared config
  (authority/routing/coordinator/squads) *and* its live computed
  consequence (`command_authority()`'s decider/tier/reachable per agent),
  which is a different axis entirely, not a subset of link quality.
  Structurally the same split Overview/Sensor-panel already draws
  (declared equipment vs. live sensor readings) — a third tab is the
  consistent extension, not scope creep. This is also the item
  `docs/history/PATCH-06-CHECKS.md` already flagged as the top of the "still not
  visible" queue: `netcheck.py`'s COMMAND AUTHORITY table, live, in the
  GUI. Not built - queued.
- **RESYNC (queued, not built)** — sound. Patch 7's `phase_t0` fix (each
  agent's shuttle/patrol/orbit phase measured from its own "became active"
  time, not the run's absolute clock) is exactly why retasked agents drift
  out of phase with each other: agents (re)armed or (re)assigned at
  different moments get different `phase_t0` references. A `RESYNC`
  command that sets a group's `phase_t0` to one common value, touching
  neither `mission` nor pose, is cheap, low-risk (same "picked up fresh,
  safe live" category as `NETWORK`/`SQUAD`, not the RENAME/DESPAWN
  reference-danger category), and doesn't compromise "absolute commanded
  position" - it only changes where in an unchanged cycle an agent
  currently sits, never the cycle's endpoints. One real design question
  for whenever it's queued for real: phase only has a stable shared
  meaning for agents on the same period (same leg length ÷ speed) - worth
  deciding then whether RESYNC checks that and warns, or just trusts the
  operator.

## Still not visible anywhere in the Console

Unchanged from patch 7, now with the network/authority view queued
specifically to close it:
- Command authority (who decides, reachable, tier).
- Measured topology (shape, betweenness, hub).
- `active` vs `spare` — the viewport still draws both identically.
