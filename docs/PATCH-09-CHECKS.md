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

---

## Step 4 — Contested: jamming is physics, not a flag

### What changed
- **A jammer is an ordinary agent.** A `jammer: {tx_power, band}` block on any
  agent (put it on the **red** network) makes it an emitter; the LAUNCH/HALT
  state machine arms it, so `red launch` turns the jamming on. Its power
  travels the same log-distance path loss as any signal and lands in every
  same-band receiver's SINR denominator (`rf_link`'s `interference_mw`, which
  already existed and was unused). No special-case jamming code, no global
  flag — exactly the design the docstrings promised.
- **The scene owns the baseline.** `apply_routing` and `frame` now read the
  scene's declared `path_loss_exponent` and `noise_floor` (`scene_rf()`)
  instead of `rf_link`'s function defaults. The floor is the reference the
  jamming delta is measured from.
- **Authority degrades with the spectrum.** `frame` scores the links FIRST
  (with jamming), then `command_authority` judges reachability from those
  scored links — so a jammer that kills a hub link strips the command
  authority that flowed over it. `command_authority` gained an optional
  `link_states` arg for this; its old geometry fallback is unchanged.
- **Per-agent experienced spectrum** in every frame: each agent reports the
  noise floor it actually sees (`rf.noise_floor_dbm`), the scene baseline
  (`rf.baseline_dbm`), and a `jammed` flag (>3 dB above baseline).
  `attacks_active` lists the emitters transmitting right now.
- **The Contested tab is live** (replaces the placeholder): a tree of the
  scene baseline (noise floor, path-loss exponent, GNSS), the active emitters,
  and each agent's experienced floor — jammed agents in red.
- **`fleets/3_roboracer_jammed.yaml`** (new): the lab fleet + `jam1`, a red
  jammer at room centre, 10 dBm (tuned so blue links show a MIX — some hold,
  some degrade, some drop — rather than a uniform wall; raise to 30 dBm / 1 W
  for total denial).
- **Band separation is a real defence:** an off-band jammer contributes
  nothing (the frequency-hopping work later depends on this).
- **Tests: 116 passed** (+3 sections: jamming raises the floor / drops links /
  strips authority; off-band does nothing; the scene baseline feeds the RF
  model — path-loss exponent shortens clean-link range, noise floor scales the
  jamming delta).

### Check
1. `python3 tests/test_all.py` → **116 passed, 0 failed**.
2. **Console:** Setup → scene `lab_box` → fleet `3_roboracer_jammed` (spawn
   dialog now lists jam1 too) → Play → `SETMISSION test` → `blue launch`.
   Cars shuttle, Comms all green, Contested tab shows every agent at −95 dBm.
3. **`red launch`** → the Comms link table goes amber/red (some blue links
   degrade, some drop); the Contested tab lists `jam1` as an active emitter
   and shows car1/car2 with a raised floor in red; on the map, jammed cars'
   command authority drops (car1/car2 lose their decider, car3 hangs on).
   `red halt` → recovers.
4. `python3 tools/netcheck.py` on a jammed-fleet composition shows jam1 as a
   red self-network agent; links read clean because netcheck never arms it
   (it's a static inspector — arming is a Console/live action).

### Known gaps
- Jamming is continuous-only. Reactive / random / deceptive jammers (the
  README's ladder) are behaviours on the jammer agent — a later step.
- Self-jamming (the fleet's own transmitters raising each other's floor) is
  still unmodelled; the seam is the same `interference_mw` path.
- GNSS is still binary available/denied; C/N0-grade quality (needed for
  spoofing) is queued in docs/contested-background.md.
- The jammer is static; a mobile jammer (`REOBJECTIVE jam1 pursue car1` from
  the red cell) arrives with the cell terminals.

---

## Step 4b — Red side: separate fleets, console-editable jamming, range ring

Answers Will's four asks (1 Sep), grounded in the project library — see
`docs/jamming-model-justification.md` for the honest, sourced verdict on
whether the jamming model is useful (short: yes as a network/command-
resilience testbed, no as a physical-layer RF simulator; do not quote exact
dB/PDR figures as truth).

### What changed
- **Blue and red are separate fleets.** The combined `3_roboracer_jammed`
  fleet was the mistake — moved to `attic/`. Blue stays `3_roboracer`
  (friendly only); red is `red_jammer` (one jammer, its own red network,
  spawned OPPOSITE the gcs at y = +3.0). The loader gained a `fleets:` list
  (blue + red overlaid); `_overlay` now UNIONS the `networks`/`radios`/
  `points` sections so a second fleet cannot erase the first's network.
- **Setup has two pickers — Blue fleet and Red fleet** — each with its own
  spawn dialog; they must be different files. Red is optional. Composition
  writes `runs/current_setup.yaml` with `fleets: [blue, red]`.
- **Jamming is editable in the Console, not baked in.** `red launch`/`red
  halt` arm/disarm (the existing state machine); a new **`JAM <id> power
  <dBm>`** / **`JAM <id> band <MHz>`** command tunes a running jammer live,
  and the Contested tab's emitter rows are **double-click editable** (prompts
  for transmit power, sends JAM). The fleet file only carries a sensible
  default.
- **Range ring on a selected jammer** (TOP view): a dashed **influence
  boundary** (J/N = 0 dB, the classic jammed-area boundary, Tedeschi & Di
  Pietro 2021) and a filled **denial core** (J/N = 20 dB), labelled
  "nominal" because a real jammed area is ragged, not a circle (sensors 2024,
  Baltic trial). Radius from the shared, tested `jammer_range_m()`.
- **Interception designed** (`docs/interception-design.md`): a cell "hears"
  commands on its band — plaintext, or encrypted-traffic-only — reusing the
  `rf_link()` verdict; built with the cell terminals (it needs a red cell to
  deliver to). This is the sensing half of a reactive jammer.
- **Tests: 129 passed** (+3 sections: two fleets compose and stay separate;
  the JAM command tunes live and rejects non-jammers; the range helper grows
  with power / shrinks with J/N threshold).

### Check
1. `python3 tests/test_all.py` → **129 passed, 0 failed**.
2. **Console Setup now has Blue fleet AND Red fleet pickers.** Scene
   `lab_box` → Blue `3_roboracer` (spawn) → Red `red_jammer` (spawn) →
   jam1 appears on the far side from the gcs. Choosing the same file for red
   as blue is refused.
3. **Click jam1** (TOP view) → a red dashed influence ring + filled denial
   core, labelled with the nominal radius. Front/side views show no ring
   (a 2-D contour only reads on the plan).
4. Play → `SETMISSION test` → `blue launch` → `red launch`: blue links
   degrade/drop, Contested lists jam1. **Double-click jam1 in Contested** →
   enter 30 → denial spreads; enter -20 → it recedes. Or type
   `JAM jam1 power 30` in the terminal. `red halt` clears it.
5. Read `docs/jamming-model-justification.md` — the honest account of what the
   model can and cannot claim.

### Known gaps
- Range ring is a circle (omni only); directional jammers/antennas are a
  later axis (the model is omni — justification doc §weaknesses).
- Interception is designed, not built (lands with the cell terminals).
- Still constant-jammer only; reactive/random/deceptive is the levels ladder.
- No processing gain / spread spectrum — the model understates a
  spread-spectrum radio's resilience (justification doc §weaknesses).

---

## Step 5 — Network tab + three cells (built together), blue-only missions

### What changed
- **gcs is blue** (was drawn grey) — it was always on the blue network; now
  it reads blue too.
- **Missions are blue-only**, enforced in the sim (`_is_taskable`): SETMISSION
  skips any adversary-network agent or jammer with a printed reason, and a
  live REOBJECTIVE on one is ignored. Red is jamming, driven by
  launch/halt/JAM, never by objectives.
- **Three cell terminals** replace the single plain terminal: **Blue cell**
  (commands blue: SETMISSION, blue launch/halt, REOBJECTIVE blue), **Red cell**
  (red launch/halt, JAM — no SETMISSION, cannot task blue), **White cell**
  (umpire: anything + shell; extra "New terminal" tabs are white). Scope is
  enforced by `Console.side_of()` (network `system` field); a cross-side order
  prints why it was blocked and is not sent. Bash works in every cell.
- **Network tab** (new, after Mission): `netcheck`'s tables live in the GUI —
  Declared (authority/routing/coordinator/squads/doctrine per network), Live
  (each agent's decider/tier/reachable, unreachable ones in red), Measured
  topology (shape/hub/betweenness, active/spare/down links, and a ⚠ when the
  measured shape ≠ the declared routing).
- **Editable leader-loss doctrine** (`leader_loss: fallback|strand`): a squad
  whose leader drops either reports up to the coordinator (fallback, default,
  "degraded not decapitated") or is stranded. Declarable per network, shown in
  the Network tab. Discussion + open question in docs/cells-and-network.md.
- **Tests: 139 passed** (+2 sections: missions-are-blue-only; leader-loss
  doctrine fallback vs strand).

### Check (all verified headless; GUI ones need your eyes)
1. `python3 tests/test_all.py` → 139 passed, 0 failed.
2. Console Terminals area shows three tabs — Blue / Red / White cell —
   colour-coded. In the **Red cell**, `SETMISSION test` is blocked
   ("missions are a BLUE-cell action"); in the **Blue cell**, `JAM jam1
   power 20` is blocked ("JAM is a RED-cell action"); White does both.
3. Network tab: Declared shows blue (star, coordinator gcs) and red;
   after `red launch` + jamming, Live shows car1/car2 decider gcs
   UNREACHABLE (red), car3 reachable.
4. Blue cell: `blue launch`, `SETMISSION test`; Red cell: `red launch`,
   `JAM jam1 power 30`. Behaviour matches CHECK 4 numbers above.

---

## Step 5b — Jamming now CHANGES BEHAVIOUR (the missing causal link)

Will spotted it live: under jamming the Network tab showed every car
UNREACHABLE and links down, but the cars kept driving. Because `step()` only
ever checked `armed`, never whether an agent could still reach its commander —
so `on_link_loss` was declared and never read. Fixed.

### What changed
- `frame()` computes reachability from the PRE-step link state (honest
  one-tick lag) and passes the set of cut-off agents to `step()`.
- `step()` honours each agent's `on_link_loss` doctrine: **hold** (default —
  freeze until the link returns; a centralized agent with no coordinator has
  no orders) or **continue** (ignore, the old behaviour / a control). A
  **decentralized** agent is never cut off (it commands itself), so it keeps
  moving — the robustness result, now physical, not just a readout.
- The frame carries `link_loss_hold` per agent; the map draws "⊘ held (link
  loss)" on a frozen car so it reads as "cut off", not "arrived".
- Tests: **143 passed** (+1 section: a centralized fleet freezes under
  jamming, a decentralized one keeps going).

### Check
1. `python3 tests/test_all.py` → 143 passed.
2. Run the jammed setup, `blue launch`, `SETMISSION test`, cars shuttle.
   `red launch` + `JAM jam1 power 30` → the cars STOP and show "⊘ held";
   `red halt` → they resume. (Before this, they drove on regardless.)

---

## Step 6 — GNSS-denial drift (belief vs truth), no-LiDAR rig, orange down line

The headline correctness feature: jamming now produces the RIGHT effect for the
RIGHT band. Full model in docs/gnss-drift-model.md; research in
docs/jamming-effects-research.md.

### What changed
- **Two poses per agent:** true (physics) and belief (estimate). A car with a
  fix (GNSS or lidar/vision) has belief == truth. GNSS-denied and unaided, it
  dead-reckons and DRIFTS at ~4% of distance (UAV Navigation figure), the drift
  emerging from integrating a per-tick error, never hardcoded. The controller
  steers on belief, so a drifted car physically misses. A regained fix
  recovers it.
- **Two jammings, cleanly separated by band:** a GNSS-band jammer (1575 MHz)
  denies the position fix -> drift; a comms-band jammer (2400 MHz) denies the
  link -> failsafe/hold. Retune the one jammer live: `JAM jam1 band 1575`.
- **Sensors matter:** a lidar/vision car resists GNSS jamming (localises
  without GNSS); a no-lidar car has only GNSS to lose.
- **`fleets/3_roboracer_no_lidar`** (built earlier) and **`scenes/open_field`**
  (GNSS available) are the clean rig.
- **UI:** the map draws a belief "ghost" for any drifting car ("thinks: X m
  off"); the Sensor panel is per-agent (GNSS status, estimated error, the
  agent's real sensor list, lidar plot only if it has one).
- **Orange down line:** a DOWN link is now drawn orange, unmistakable.
- **Tests: 151 passed** (+6: drift grows under GNSS jamming, recovers on fix,
  lidar resists, comms-band jammer causes no drift).

### Check (headless verified)
1. `python3 tests/test_all.py` -> 151 passed.
2. Setup: scene `open_field`, Blue `3_roboracer_no_lidar`, Red `red_jammer`.
   Play; White/Blue cell: `SETMISSION test`, `blue launch` (cars shuttle,
   Sensor panel "GNSS ok"). Red cell: `JAM jam1 band 1575`, `red launch` ->
   ghosts appear and widen, Sensor panel "GNSS DENIED - est error X m". `red
   halt` -> ghosts snap back.
3. Same with Blue `3_roboracer` (has lidar): GNSS jamming does nothing -
   the lidar localises the car. That contrast is the result.
4. Any run with a down link: the down link line is orange.

### Known gaps (in docs/gnss-drift-model.md)
- Lidar aiding is a scalar proxy (has-lidar => no drift); real odometry needs
  map features and has its own error. The Kalman/fusion step refines this.
- Drift direction is a random walk (growth rate sourced; exact path
  illustrative).

---

## Step 7 — Band strip (frequency filter above the timeline)

### What changed
- A **band strip** sits above the timeline: one button per frequency in the
  scene (All, each comms band, GNSS 1575). Selecting a comms band shows only
  that band's links; "All" shows everything; "GNSS" hides the comms lines so
  the positioning drift (ghosts) is the story on that band.
- A band button turns **orange, bold, live** while a jammer is emitting on it,
  so what is being jammed — and when — is evident at a glance. GNSS counts a
  jammer within 5 MHz of L1.
- Each link now carries `band_mhz` (from its network) so the viewport can
  filter by band; nothing else changed in the sim.
- Tests: 152 passed (link band tag added; no behaviour change).

### Check
1. `python3 tests/test_all.py` -> 152 passed.
2. Run any scene: the Band strip shows All / the comms band / GNSS. Click
   the comms band -> only comms links draw; click GNSS -> comms lines vanish.
3. `red launch` on the comms band -> that band's button goes orange live.
   `JAM jam1 band 1575` -> the GNSS button goes orange instead.
