# Patch 9 — what changed, and how to check it

The refactor-and-two-tabs run, done in steps so no single change is big enough
to break the simulator. Each step is applied, tested, and checked before the
next. This doc grows one section per step.

---

## Step 1+2 — Three layers: scene + fleet, composed by a mission

(Step 1 first shipped with a middle layer called "scenario"; Will cut that
word the same day — it was the word causing the confusion — and the layer is
now the **fleet**. This section describes the result as it stands.)

### What changed
- **`docs/vocabulary.md`** (new) — locks the model: **scene** (world only),
  **fleet** (agents + networks, no world, no tasking), **mission** (names a
  scene AND a fleet, then commands the fleet). "Scenario" is retired.
  Supersedes the old "map + mission" decision in `docs/BACKLOG.md`.
- **`tools/stub_telemetry.py`** — `resolve_mission()` rewritten: resolves
  `scene:` (alias `map:`) and `fleet:` references recursively and overlays the
  file on top; factored into `_base_path` / `_overlay`. A file naming no base
  is self-contained (the old shape) and loads unchanged.
- **`deadband/spec.py`** — validation is now layer-aware: a scene owes an
  arena, a fleet owes agents, a mission owes neither (they arrive through the
  chain); a file with no `kind` and no reference still owes everything.
- **The canonical trio** — the working set is now exactly one of each:
  - `scenes/lab_box.yaml` — the 8x8 lab, world only (agents stripped out).
  - `fleets/3_roboracer.yaml` — gcs + three cars, networks, radios; carries
    every measured dimension byte-for-byte from the old file.
  - `missions/test.yaml` — `scene: lab_box` + `fleet: 3_roboracer`, three
    shuttle lanes. **This is the new benchmark and the new default** in the
    Console, netcheck, the stub and the ROS launch.
- **`attic/`** (new) — every removed file, moved not deleted: the whole old
  `scenarios/` folder, the yaml missions (`squad_patrol`, `three_lane_shuttle`,
  `demo_*`), `my_first.py`, and the old fleet-embedding scenes (`big_hall`,
  `squad_hall`, `demo_field`). Nothing reads it; delete it whenever.
  (The now-empty `scenarios/` folder itself needs deleting by hand — the
  sandbox can move files but not remove directories.)
- **Kept, deliberately** (pushed back on deleting): `missions/wall_follow.py`
  and `missions/return_on_link_loss.py` — behaviour scripts, i.e. verbs an
  objective invokes, embodying the two-beam follower and the link-loss work;
  and `missions/example_pursuit.py`, which the ROS controller node names as
  its default parameter.
- **`tests/test_all.py`** — behavioural tests now load `missions/test.yaml`;
  the two-squad REMISSION tests generate their own two-squad fixture (so the
  attic stays optional); the waypoint-rejection repro generates a point-less
  fixture; new chain test proves scene/fleet/mission each stay honest.

### Check
1. `python3 tests/test_all.py` — expect **101 passed, 0 failed**.
2. `python3 tools/netcheck.py` (no argument) — runs `missions/test.yaml`:
   four agents, star on gcs, three ACTIVE links, three spare.
3. **In the Console:** File → Open → `missions/test.yaml`. Expect the familiar
   three-lane lab picture: Overview root `Scenario: test`, gcs + car1-3 on
   blue, points A-F, Environment showing `Scene: lab_box`. `blue launch` then
   `REOBJECTIVE car1 pursue car3` should behave exactly as before.
4. **In the Console:** open `scenes/lab_box.yaml` alone — an empty room, no
   agents (that is correct: a scene is the world only), no validation errors.
5. `python3 -m deadband validate missions/test.yaml` — OK; then validate
   `fleets/3_roboracer.yaml` — OK (a fleet owes agents, not an arena).

### Known gaps (closed in later steps)
- The Console's File dialog and tree headers still say "scenario" in places;
  Step 3 renames the surfaces to match the vocabulary.
- Fleet spawn poses are raw coordinates (scene-coupled) — spawning at named
  points is queued.
- The two-squad rig now lives only as a test fixture; when a second fleet is
  wanted for hierarchy work in the Console, write `fleets/2_squads.yaml`.

---

## Step 3 — Compose in Setup, command from the terminal

### What changed
- **A mission is now the command, and nothing else.** `missions/*.yaml` carry
  `objectives:` only — no `scene:`, no `fleet:` (enforced by the test suite).
  You do not OPEN a mission; you ISSUE one, with the run up:
  `SETMISSION <name>` then `blue launch`. SETMISSION applies
  `missions/<name>.yaml`, gated by command authority — an agent whose decider
  cannot reach it is NOT retasked (the property REMISSION existed for, kept in
  the one remaining order verb) — and titles the run with the mission's name.
  **REMISSION and LOADMISSION are removed** (`decompose_order` and its lane-
  splitting helpers deleted with it).
- **The Console opens BLANK.** No auto-loaded scenario. The **Setup tab**
  (first tab) composes the run: pick a scene → the world appears; pick a
  fleet → a spawn dialog asks where each agent starts (fleet's own poses as
  defaults, z carried through so a suspended gcs keeps its height). Setup
  writes `runs/current_setup.yaml` and loads it — a real file, so Play, the
  ROS launch, save and reload all work unchanged.
- **Tabs:** `Setup, Overview, Mission, Comms, Contested, Results`.
  Environment is gone as a tab — the scene tree (arena + background) now
  lives inside Setup, under the pickers. **Contested replaces Cyber** and
  will hold everything that degrades the spectrum: scene background, jammers,
  spoofing, attack injection. File → Open is relabelled "Open file
  (advanced)..." — a legacy escape hatch, not the workflow.
- **Results titled by mission:** the frame contract gains a `mission` field
  (set by SETMISSION, `None` before); kept bags are renamed
  `runs/<mission>_<timestamp>`; CSV export defaults to `<mission>.csv`.
- **`default_run.yaml`** (new, repo root) — the headless composition (lab
  scene + lab fleet, untasked); the default for the stub, netcheck and the
  ROS launch.
- **Tests: 101 passed.** REMISSION's five tests replaced by four SETMISSION
  tests (applies-and-names, bare-name resolution, reachability gating on an
  orphaned agent, per-agent out-of-bounds rejection). NOTE one deliberate
  semantic: a squad leader losing its uplink does NOT strand its squad —
  command_authority falls back to the coordinator ("degraded, not
  decapitated"); only a genuinely orphaned agent (no decider reachable) is
  skipped by SETMISSION.

### Check
1. `python3 tests/test_all.py` — **101 passed, 0 failed**.
2. **Open the Console — the screen is blank.** Setup tab: choose scene
   `lab_box` → the room appears, empty. Choose fleet `3_roboracer` → spawn
   dialog with gcs + car1-3 at their default poses → Spawn here → four
   agents appear.
3. Press **Play**, then in the terminal: `SETMISSION test` → Mission tree
   header flips to `Mission: test`, Setup tab shows `Mission: test`. Then
   `blue launch` — the three cars run their lanes. `REOBJECTIVE car1 pursue
   car3` still works; `REMISSION ...` is (correctly) no longer a command.
4. Press Play with nothing set up (fresh Console): refused with a message,
   not a crash.
5. ROS path: run the same setup with the ROS source; on Stop → Keep, the bag
   is `runs/test_<timestamp>` (titled by mission), not `bag_<timestamp>`.
6. Export CSV after a stub run — the suggested filename is `test.csv`.

### Known gaps
- The spawn dialog is a table, not click-to-place on the map. Queued.
- SETMISSION must be typed AFTER Play (the pre-run spool clean wipes earlier
  commands — deliberate, stale-command protection). The Setup tab's hint says
  the order; a queued nicety is buffering it.
- Terminals are still plain; blue/red/white cells are the next-but-one step.

---

## Step 3b — Spawn z, Setup lock, timestamped titles, the background audit

### What changed
- **Spawn dialog gains a real z column** (editable, fleet default offered).
  An aerial fleet spawns AT altitude, and the suspended-gcs trick is a z
  decision - it was wrong to carry it silently.
- **Setup locks during a run.** Play (stub or ROS) disables the Setup tab -
  recomposing the world under a live sim is a crash waiting to happen - and
  steps you off it if you are on it; Stop re-enables it. Everything else
  stays live.
- **Every kept result is mission + date-time.** Bags already were
  (`runs/<mission>_<YYYYMMDD_HHMMSS>`); CSV export now suggests
  `<mission>_<YYYYMMDD_HHMMSS>.csv` too.
- **Background audit** - `docs/contested-background.md` (new): what the scene
  declares and WHY (the test: true of the world with no adversary in it),
  what should join (terrain, per-band conditions, C/N0-grade GNSS quality),
  what was considered and excluded (atmospheric attenuation below 10 GHz),
  and what must never creep in (anything with intent - that is an agent or
  an attack). `scenes/lab_box.yaml` now declares `spectrum.noise_floor`
  (-95 dBm, unsourced, measurement flagged) - the resting SINR denominator
  belongs to the room, not to `rf_link()`'s defaults. The Setup scene tree
  shows the full background vocabulary, greying undeclared blocks.

### Check
1. Setup -> fleet `3_roboracer` -> the spawn dialog now has x, y, **z**, yaw.
   Set gcs z to 1.5 and spawn: the gcs floats (front/side view shows it).
2. Press Play -> the Setup tab greys out (tooltip says why); if you were ON
   Setup you land on Overview. Stop -> Setup is back.
3. Export CSV after a run with `SETMISSION test` -> suggested name is
   `test_<date>_<time>.csv`.
4. Setup scene tree for lab_box -> Background shows propagation, spectrum,
   gnss, wind (none greyed - lab_box declares all four).
5. `python3 tests/test_all.py` - 101 passed (nothing behavioural changed).

---

## Step 3c — Result titles carry the whole run; scene tree slimmed

### What changed
- **One titling rule, `_run_stem()`:** every kept result is now
  `<scene>_<fleet>_<mission>_<YYYYMMDD_HHMMSS>` — CSV export suggestion AND
  the kept-bag rename both use it, so a results folder reads without a
  decoder. Missing fields are simply absent (no mission set → scene_fleet_ts);
  filename-illegal characters are replaced (which is why the separator is
  `_`, not `|`).
- **Map builder / Import topography stub rows removed** from the Setup scene
  tree — Will is planning separate companion apps for those; dead greyed rows
  were noise.
- Stale `.git/index.lock` (created by a sandboxed git status that could not
  delete it) moved to `attic/stale_git_index.lock` — delete with the rest of
  the attic.

### Check
1. Stub run with `SETMISSION test` → Export CSV → suggested:
   `lab_box_3_roboracer_test_<stamp>.csv`.
2. ROS run, Stop → Keep → bag lands as
   `runs/lab_box_3_roboracer_test_<stamp>`.
3. Setup scene tree: the Scene node shows the room only — no greyed
   Map builder / Import topography rows.
