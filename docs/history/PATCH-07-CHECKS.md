# Patch 7 — what changed, and how to check it

Separates spawning a robot from tasking it from launching it, adds a real
military-style order layer (REMISSION) that exercises the existing
authority model instead of just labelling it, and fixes a live bug where an
unresolvable shuttle waypoint silently substituted the arena half-width.

**This version also fixes nine bugs found testing the first cut of this
patch live in the Console** — see section 0 below. Everything past that
section describes the patch as it now stands, bug fixes included; where a
behaviour changed as a direct result of a bug fix (ground stations no
longer arming under a network-wide LAUNCH, in particular), the description
already reflects the fix, not the original design.

---

## 0. Bugs found testing this patch live, fixed

**1–3: a real command sometimes silently did nothing** (`blue halt`,
`REOBJECTIVE car1 pursue car3`, and a 2-value `(x,y)` coordinate all
reported as "does nothing," while the identical shape with a different
agent/verb/value worked). None of these reproduced in isolated, single-
process testing — parsing, validation and assignment all succeeded
identically to the exact failing commands. The pattern (same code path,
intermittent failure, no error) pointed at a race rather than a logic bug:
`drain_retasks()` used to **read, then delete** the queue file the Console
appends to. A write landing in the gap between those two steps is silently
lost, and on Windows the delete can outright *fail* while the Console's
file handle is still open (POSIX allows deleting an open file; Windows
generally does not) — either way, a command vanishes with no error, exactly
matching what was reported. Fixed by making the handoff atomic: the reader
now **renames** the queue file to a process-exclusive name before reading
it. A rename is atomic on both platforms, so a write landing before it is
safely included, and one landing after it just starts a fresh `queue` file,
picked up next poll (0.1 s later) — never lost, at worst one tick late.
- **Check:** `test_retask_queue_atomic_handoff` covers the defensive half
  (a leftover `.reading` file from a crashed read doesn't block the next
  real queue). The race itself isn't unit-testable deterministically; the
  fix is a standard atomic-handoff pattern for exactly this failure mode.
  Retest bugs 1–3's exact repros live — they should no longer be
  intermittent.

**4: a pasted `$ ` prompt went to bash.** A shell transcript pasted into
the Terminal (e.g. `$ REOBJECTIVE car1 shuttle (-3,3,0) (3,3,0)`) was sent
as a literal command, and the unmatched parentheses threw a bash syntax
error. `ShellPanel.run()` now strips one leading `$` before doing anything
else with the line.
- **Check:** paste `$ car1 launch` into the Terminal — it arms car1, no
  bash error.

**5: launching hiccuped — cars jumped, then settled.** Shuttle/patrol/orbit
phase was computed from the run's absolute `t`. An agent armed at t=30 s
computed its phase as if it had been shuttling since t=0, so its target
snapped to wherever a 30-second-old cycle happened to be — often nowhere
near the agent's actual position — and it lurched off to catch up before
settling into the real oscillation. Fixed with a per-agent `phase_t0`,
reset to "now" on every (re)launch and every (re)assignment; all three
cyclic objectives now measure phase from `t - phase_t0`, not raw `t`.
- **Check:** `test_phase_t0_clean_launch` — launching at t=30 s now targets
  the agent's own position exactly, not a point 3 m away. Live: arm an
  agent well after Play (wait 20–30 s before `blue launch`) and watch it
  pull away smoothly instead of snapping toward a distant point first.

**6: `LAUNCH: blue (7 agents)` armed the ground station; REMISSION
correctly excludes it.** A network-wide `LAUNCH`/`HALT` used a different
eligibility rule than REMISSION's `platform != "ground_station"`. Now the
same rule applies to both — a network-wide LAUNCH/HALT skips ground
stations; naming one explicitly (`gcs launch`) still works, since that's a
deliberate per-agent action.
- **Check:** `blue launch` on a scene with a `gcs` reports one fewer agent
  than the network's total membership; `gcs launch` still arms it.

**7: `blue launch` on `missions/squad_patrol.yaml` crashed all six cars
together.** Confirmed as a **data** problem, not a code bug: the file tasks
car1/car2/car3 all `shuttle between A B` — the identical line, staggered
only by a *time* offset, not a spatial one, so all three genuinely share
one track and collide once their phases catch up to each other (same for
car4/5/6 on C/D). A REMISSION after Play fixes it because REMISSION
computes a real spatial lane per agent; the baked-in file never did. Not
patched here — `squad_patrol.yaml` is exactly the file the restructure
discussion (below) proposes converting from a mission-with-baked-objectives
into a scenario with none, which removes the wrong data rather than
papering over it with six hand-picked offsets that would just be thrown
away.

**8: stale Terminal/Log output from a previous run was still on screen.**
Opening a new scenario now clears the Log panel and every open Terminal
tab, so old retask/launch/error output can't be mistaken for the currently
open run's.
- **Check:** run something, generate some Log/Terminal output, then File →
  Open a different scenario — both panels are empty immediately after load.

**9: diagonal shuttle (`shuttle (-3,2,0) (3,3,0)`), unconfirmed because of
bug 4.** No code issue — `mission_target`'s shuttle math is a plain linear
interpolation between two arbitrary points, never assumed axis-aligned.
Confirmed directly: an agent shuttling between those two points travels the
full diagonal (`y` moves with `x`, not flat).

---

## 1. Assign, inspect, launch — the gate before anything moves

### Changes
- Every agent gets a new `armed` flag, default `False`. `step()` now holds an
  agent still whenever it is unarmed, regardless of its objective — the same
  code path that already held a `static` agent still.
- Assigning an objective (initial load, `REOBJECTIVE`, `REMISSION`,
  `LOADMISSION`) never touches `armed`. Only two new commands do:
  `<network> launch` / `<agent> launch` (arm) and `<network> halt` /
  `<agent> halt` (un-arm, freezes at current pose, objective untouched).
  Typed straight into the Console's Terminal tab, e.g. `blue launch`.
- **This applies to every file, uniformly** — a bare scene, a split mission,
  or a legacy self-contained `scenarios/` file. Pressing Play never starts
  anything moving any more; it starts the sim idle. This is a deliberate,
  confirmed decision (see `docs/history/maps-missions-and-retasking.md`, "Assign,
  inspect, launch") — the three existing demo scenarios
  (`three_car_fleet.yaml`, `formation_demo.yaml`, `wall_follow_demo.yaml`)
  now need `blue launch` after Play where they didn't before. Their header
  comments say so now.
- Retasking an already-armed agent still takes effect immediately, exactly
  as before — the gate is only about the *first* time an objective takes
  effect, never about every change after that. No halt is required before a
  live retask.
- The Mission tree and the single-agent viewport panel both show the armed
  state now: `shuttle A-B  [assigned, not launched]` vs `shuttle A-B
  [armed]`. A LAUNCH/HALT toggle repaints the row immediately but does not
  write a spurious history entry — only an actual objective change does.

### Check
1. `python3 tests/test_all.py` — expect **88 passed, 0 failed**.
2. Console: open `scenes/lab_box.yaml`, press Play. Every car sits still.
3. Terminal: `REOBJECTIVE car1 shuttle A B`. Mission tree shows car1's row
   tagged `[assigned, not launched]`. Car1 does not move.
4. Terminal: `blue launch`. Car1 starts shuttling; the tag flips to
   `[armed]`.
5. Terminal: `car1 halt`. Car1 freezes exactly where it is (not at a
   waypoint) with its objective still shown. `car1 launch` resumes it.
6. Open `scenarios/three_car_fleet.yaml`, Play — nothing moves (this is the
   behaviour change from before this patch). `blue launch` — all three cars
   start shuttling, same as pre-patch Play used to do on its own.

---

## 2. REMISSION — a system order, decomposed by authority

### Changes
- `REMISSION <network> <verb> <args>` is a mission given to the *system*.
  The system decomposes it into per-agent objectives, gated by that
  network's `authority`:
  - `centralized` — the coordinator decomposes for every reachable member;
    an unreachable one is skipped.
  - `decentralized` — every agent self-allocates; nothing is ever skipped.
  - `hierarchical` — the coordinator broadcasts the same order to every
    squad leader; each reachable leader decomposes it across its own
    squad. **A squad whose leader is unreachable at issue time is skipped
    wholesale** — its members keep their prior objective. Deliberate: the
    coordinator does not reach around a dead leader, because that would
    make hierarchical behave exactly like centralized at the one moment
    you're trying to measure the difference.
  - Ground stations (`platform: ground_station`) never receive an
    objective from a REMISSION.
- `shuttle` decomposition gives each eligible agent its own parallel lane,
  offset along the line's perpendicular, using **one flat index across the
  whole tasked roster** — not restarted per squad. A first pass that
  restarted per squad put two squads' member 0 in the same lane; caught
  before shipping, regression test added
  (`test_remission_hierarchical_no_collision`).
- **Out-of-bounds is rejected wholesale, never clamped or recentred.** If
  any agent's computed lane would land outside the arena, nothing is
  retasked — not even the agents whose lane would have fit — and the Log
  names every offending agent and coordinate. Also caught before shipping:
  the first draft's own worked example put a lane a metre outside an 8×8
  room. Clamping the spacing or shifting the line was considered and
  rejected — either would mean an agent's real commanded position silently
  differs from what was typed, which is exactly what this framework exists
  to prevent everywhere else.
- Other verbs (`pursue`, `orbit`, `stop`) decompose by broadcasting the
  same objective to every eligible agent verbatim — no bespoke geometry for
  them yet.

### Known limitation
- The eligibility filter only excludes `platform == "ground_station"`. A
  `quadcopter_iris` on the same network as ground vehicles is **not**
  excluded and would currently be handed a ground shuttle lane like a car —
  wrong, but left as-is deliberately. Air decomposition (altitude layers, a
  hold-station or relay objective instead of a ground lane) is a separate
  job.

### Check
7. Console: `scenes/lab_box.yaml`, Play, then
   `REMISSION blue shuttle (-3,0,0) (3,0,0)`. Mission tree shows three
   *different* parallel lanes (y = −2, 0, 2), tagged `[assigned,
   not launched]`; gcs untouched. `blue launch` — all three shuttle their
   own lane.
8. Same file: `REMISSION blue shuttle (-3,3,0) (3,3,0)` — a lane at y=+5 is
   outside the 8×8 room. Confirm it is rejected: Log names every offending
   agent/coordinate, and none of the three agents' objectives changed.
9. `scenes/squad_hall.yaml` (or `missions/squad_patrol.yaml`), Play, both
   squads reachable: `REMISSION blue shuttle (-10,0,0) (10,0,0)` gives six
   distinct lanes, not three collisions. Move/jam alpha's leader (car1) out
   of range, reissue — alpha's members show no new assignment, bravo's three
   collapse back to bravo's own contiguous three lanes, `alpha SKIPPED —
   leader car1 unreachable` in the Log.

---

## 3. LOADMISSION — file-based per-agent distribution, kept separate

### Changes
- `LOADMISSION <mission-file>` reads a `missions/*.yaml` file's
  `objectives:` block and applies each named agent's objective **verbatim**
  to the running agent set — no decomposition, no authority gating, a
  direct operator action like `REOBJECTIVE`, just file-sourced and
  multi-agent. Different job from REMISSION on purpose — see
  `docs/history/maps-missions-and-retasking.md`.
- An agent the file names that isn't in the running scene is warned about,
  not silently dropped or crashed on.

### Check
10. Console, any running scene: `LOADMISSION missions/three_lane_shuttle.yaml`
    — car1/car2/car3 get that file's own objectives (named points A–F, not
    a decomposed lane), `armed` untouched.

---

## 4. Coordinate grammar: `(x,y[,z])`

### Changes
- Both `REOBJECTIVE ... shuttle <A> <B>` and `REMISSION <net> shuttle <A>
  <B>` accept `(x,y)` or `(x,y,z)` literals anywhere a position is
  expected, parentheses required, `z` defaulting to `0`, internal spaces
  tolerated (`(-3, 3, 0)` parses). A bare word is still a point name.
  Mixing is fine: `shuttle A (3,2,0)`.
- Scope: the terminal/retask grammar only. Mission YAML files keep using
  explicit `{x: ..., y: ...}` dicts, unchanged.

### Check
11. Terminal, any running scene: `REOBJECTIVE car1 shuttle (-3,3,0) (3,3,0)`
    — car1 adopts the literal lane immediately, Mission tree shows
    `shuttle -3,3-3,3` (point rendering, not a dict repr).

---

## 5. Bug fix: unresolvable waypoints no longer fall back to the arena half-width

### Changes
- Root cause: an unresolved shuttle endpoint used to fall back to
  `(-hx, ...)`/`(hx, ...)` silently. `REOBJECTIVE car1 shuttle A B` on
  `three_car_fleet.yaml` (which defines no named points) hit this on both
  ends and quietly swept −4..+4 instead of car1's real −3..+3 lane — looked
  like it worked, wasn't.
- Fixed at the point an objective is **accepted**, not at tick time:
  `validate_objective()` resolves both endpoints and checks they're in
  bounds before the objective is ever written onto an agent. On failure,
  the agent's existing objective is left completely alone — "leaves the
  agent doing what it was doing" — and the rejection is reported on stderr
  and, persistently, in the agent's `health.warnings` (visible in the
  Console, not just a terminal you might not be watching).
- Tick-time fallback (for any path that somehow skips validation) now holds
  the agent at its **current pose**, never a fabricated point, with a
  one-shot-per-agent warning instead of spamming stderr every tick.

### Check
12. `three_car_fleet.yaml`, Play, `blue launch`, then `REOBJECTIVE car1
    shuttle A B` (A/B undefined on this file) — Log shows the rejection,
    car1 keeps its original −3..+3 lane. Check the Results plot, not just
    the viewport — the old drift was subtle enough to look right at a
    glance.

---

## 6. Geodetic origin — schema seam only, no conversion

### Changes
- A scene may optionally declare `arena.origin: {lat, lon, alt, heading}`
  (each a provenance quantity). `load_scenario()` reads it into
  `world["origin"]` if present, else `None`. Nothing else touches it — no
  conversion function, no behaviour change.
- The schema comment is explicit that this anchors whatever point is
  currently local `(0,0,0)` — **not assumed to be the arena centre.** A
  later, separate patch is expected to move local `(0,0)` to a corner of
  the room (tracked, not built here — touches ~44 coordinates across 7
  files and ~54 places in the code that assume half-extent maths;
  deliberately its own patch so failures stay attributable). This schema's
  meaning does not change when that happens.

### Check
13. `python3 tests/test_all.py` covers this (`test_origin_passthrough`) —
    no manual Console check needed, it's inert by design.

---

## Corrections to earlier checklists

- **PATCH-05/06's "press Play" checks are one step longer now.** Any check
  in those documents that says "press Play" and expects motion now also
  needs `<network> launch` (usually `blue launch`) first. Not rewritten in
  those files, per standing instruction — noted here instead.

## Still not visible anywhere in the Console

Unchanged from patch 6, still the top of the queue:
- Command authority (who decides, reachable, tier) — REMISSION now *uses*
  this every time it decomposes, but nothing paints it in the tree.
- Measured topology (shape, betweenness, hub).
- SINR / rx power / PDR per link.
- `active` vs `spare` — the viewport still draws both identically.

New to this patch:
- **Armed state is only visible in the Mission tree's text tag** — no
  colour, no icon. Fine for now, but "6 agents armed, 2 not" at a glance
  would read faster once there's more than a handful of agents on screen.
- **The `quadcopter_iris` / air-decomposition gap** (see section 2) has no
  UI warning either — REMISSION will happily hand a drone a ground lane and
  say nothing extra about it being a bad idea.
