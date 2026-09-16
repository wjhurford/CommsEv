# Backlog

Everything agreed, proposed or deferred, so it stops living in a chat thread.
Newest decisions at the top of each section. When something is done it moves to
the bottom with a date.

---

## Resuming in a new session

This file plus `docs/writing-a-mission.md` and `docs/ros2-and-missions.md` are
the handover. Nothing important should live only in a chat.

**Where things stand (26 Aug 2026).** The Console runs, drives a real ROS 2
stack, records readable MCAP bags, and PlotJuggler opens them. Scenarios are
YAML with provenance on every physical quantity. Missions are single Python
files with a frozen `target(agent, world)` contract, and three worked examples
exist. Nothing about the radio, the spectrum or attacks is modelled yet - link
quality is a distance falloff and says so in its docstring.

**How to run it.** Open `console/app.py` (or the desktop shortcut). File → Open
a scenario. Press Play: that starts the ROS nodes, records a bag, and connects.
Press Stop: it closes the recorder cleanly and asks whether to keep the bag.
The embedded Terminals tab starts in the repo root.

**The three scenarios, and which to open for what.**
  * `three_car_fleet.yaml` - the untouched benchmark. Three cars shuttling.
  * `wall_follow_demo.yaml` - the wall-follow rig. Cars 1/2 parked as static
    obstacles, gcs suspended at z 1.5 (out of the scan plane, so genuinely
    invisible to lidar - verified in the sim, not just asserted). Car 3 runs
    missions/wall_follow.py.
  * `formation_demo.yaml` - the FIRST hand-written-mission rig. Cars 1/2
    shuttle, car 3 runs missions/my_first.py holding station off car 1's flank.
    Change one number in car 3's mission block and re-run to feel the edit-run-
    see loop. Verified headless: car 3 tracks car 1 on the correct side; the
    held gap runs tighter than the 2.0 m target because car 3 chases a moving
    slot at its own speed cap - that is honest physics, not a bug.

The left panel's first tab is now **Overview** (was "Scenario") - it shows the
networks/radios tree, i.e. what is in the world, distinct from the loadable
task a mission represents.

**What Will does and does not do.** He is the manager on this: conceptual
direction, decisions, and judgement about what is worth building. He is new to
robotics and does not want to write the plumbing. He has asked, explicitly, to
be pushed back on when an idea is weak, and for bulleted next steps at the end
of every reply, carried forward rather than reset.

**The standing constraint.** Open-sourcing must be cleared in writing with
Cheng and Loughborough IP before anything is published. The repo stays private
until then. This is irreversible and it gates the whole point of the project.

---

## Decisions taken (do not relitigate)

- **SUPERSEDED (1 Sep 2026) by the three-layer model** - see
  `docs/vocabulary.md`. A run is now **scene + fleet, composed by a mission**:
  **scene** (the world only - arena, radio medium, contested background,
  points; what the File dropdown picks), **fleet** (the agents and their
  wiring - bodies, sensors, radios, networks, authority/routing; no world, no
  tasking), **mission** (names a scene AND a fleet, then issues the command -
  per-agent objectives or a fleet-wide order). The word "scenario" is retired
  from the framework - it was the word causing the confusion. The loader
  resolves the chain recursively; a file naming no base is self-contained (the
  old shape) and still loads. Old files are preserved in `attic/`, which
  nothing reads.
  - *Previously:* "A run is a MAP plus a MISSION." A map was the world; a
    mission was tasking; `scenario` meant a legacy combined file. Kept here for
    provenance - do not follow it, follow `docs/vocabulary.md`.
- **An objective is a verb, not a destination.** `shuttle between A B`,
  `pursuit`, `wall_follow`, `orbit`, `static`, `script`. Portable because it
  refers to points the map resolves, not coordinates. `docs/maps-missions-and-
  retasking.md`.
- **Objectives can be retasked live.** A command channel (`--retask DIR`, one
  line into `DIR/queue`) swaps a running agent's objective on the next tick.
  File-based so the same channel serves a terminal, the Console, and later a
  ROS service. This is why they are missions and not configs.
- **The GUI edits the scenario file; the file stays the source of truth.** Not
  a GUI-only configurator. Keeps runs diffable in git, sweepable from a script,
  and citable in a paper.
- **The Console is a client.** It reads one stream of telemetry frames and knows
  nothing about what produces them. That boundary is why swapping the stub for
  real ROS 2 took one afternoon.
- **The mission signature is `target(agent, world)` and it is frozen.** New
  senses are added to `world`, never to the argument list. Changing the
  signature later invalidates every mission anyone has written, including a
  future student's; growing the world costs nothing.
- **A mission sees only what ITS OWN agent senses.** The ROS controller
  subscribes to one `/scan` - its own - even though every scan is on the wire.
  A mission that can read other agents' sensors is cheating in a way no real
  vehicle can, and cheating quietly is worse than not working.
- **Algorithms never import `rclpy`.** A mission, controller or detector takes
  plain numbers and returns plain numbers; the ROS node is a wrapper. This is
  the whole sim-to-real argument.
- **Networks are plural from day one**, and the RF medium is shared and global.
  Adding adversarial agents on their own network is then additive.
- **Every physical parameter carries `{value, unit, source}`.** Unsourced values
  are counted and flagged, never auto-filled.
- **Gazebo Harmonic as the primary world** when the 3D layer arrives; ROS 2
  Humble throughout, pinned by AP_DDS.
- **Don't rebuild PlotJuggler.** Record in formats it reads; build only the
  domain-aware views it cannot do.
- **No 3D engine in the Console.** Four orthographic projections of boxes.

---

## Next up

- [ ] **Test the wall follower under ROS.** Headless it holds 0.85 m against a
      0.80 m standoff, sd 0.08 m, both sides, no contacts. Not yet watched in
      the Console since the two-beam rewrite
- [ ] **Will watches his first mission drive** — `missions/my_first.py` is now
      written and points at `scenarios/formation_demo.yaml`; it passes headless.
      The remaining half is Will opening that scenario in the Console, pressing
      Play, and changing one number (offset / side / follow) to feel the edit-
      run-see loop. This is the gate before the mission dropdown gets built -
      the dropdown must not hide a UX problem the by-hand path would reveal.
- [ ] **Pick a mission from the Console** — right now swapping one means
      editing YAML. A dropdown on the agent, writing back to the scenario file,
      would make it clickable like everything else
- [ ] **PACE plan** (Primary / Alternate / Contingency / Emergency per network).
      Declare a fallback ladder, then measure whether the fallback actually
      works under attack. Best paper-shaped idea on this list.
- [ ] **Real-test overlay** — read a rosbag from the real cars and draw their
      trajectory over the simulated one. Sim-to-real validation.
- [ ] **VESC model** — commands pass through real limits (current, ERPM, servo
      travel) and return odometry with real quantisation. Gives an attack
      surface *below* ROS: spoof the VESC and the ROS layer sees nothing wrong.
- [ ] **Edge compute budget** — per-agent CPU/memory as a modelled, limited
      resource. Without it, "does this detector run in time on a Jetson" is a
      question the simulator cannot answer, which quietly invalidates any
      ML-on-edge result.

## In-app workflow — everything set up in the Console (Will's vision, 26 Aug)

The direction: a researcher who is not Will can open the Console and build a
whole run without touching YAML. Staged so no single step is big enough to
break the simulator. Order is roughly dependency order.

- [ ] **Objective visible on a selected agent, live** — when an agent is
      selected during a run, show its current objective next to its speed, so
      "what is car3 doing right now" is answerable at a glance. (Pre-run
      display of the objective in the Overview tree is DONE; this is the live,
      running-view version.)
- [ ] **Per-agent bespoke sensor panel** — the right-hand panel shows the
      sensors THIS agent has, not a fixed lidar view. A car with lidar shows
      lidar (the current view, kept as the default when lidar is present); an
      agent with a camera shows a camera view; one with neither shows nothing.
      NB: do not simply hide the lidar - it is the only window into what a
      sensor-driven mission perceives, and it is how you would catch the
      /car3/scan QoS bug. Generalise it, don't remove it.
- [ ] **Pre-run point editing for split missions** — a mission's objective
      names points the MAP owns (shuttle between A and B). Editing A and B
      before a run currently needs the map file; the Console refuses the edit
      with a note. Wire an in-app map-point editor so the points are adjustable
      pre-run from the Console. (Inline scenarios with raw from/to coordinates
      are already editable in the property panel.)
- [ ] **Simple retask grammar + a retask mode in the Console** — a command box
      that puts you in "retask mode": `retask` then `car1 pursue car3`, or
      `shuttle 1 3` (between points 1 and 3). Writes the same line the file
      channel already consumes (docs/maps-missions-and-retasking.md). The stub
      channel exists; this is the friendly front end and the grammar polish.
- [ ] **Retask over ROS** — the `--retask` file channel is stub-only. A real
      ROS 2 service is needed to retask a running ROS controller. Gates live
      retasking of real cars.
- [ ] **The setup workflow / startup page** — one page, walked top to bottom:
      pick a map; set environmental conditions; pick or name a mission; how
      many networks; how many agents per network; define each agent; set each
      agent's objective and its points of interest; choose the comms channel;
      choose the system architecture. Then a **Simulate** button drops into the
      current home screen, where starting coordinates and objectives can be
      tweaked pre-run, and Play begins the run (with retasking available during
      it). Explicitly NOT needed for the current research sandbox - this is the
      hand-over-to-others milestone, built last of this group.

- [x] 2026-09-01 no-LiDAR fleet `3_roboracer_no_lidar` - the clean rig for
      measuring positioning faults (cars follow GCS waypoints open-loop, no
      sensor loop to mask drift).
- [ ] **GNSS-denial drift model** (Will, 1 Sep) - the headline effect. Two
      poses per agent (true/believed); GNSS jammed -> dead-reckoning drift
      that emerges from integrating IMU error without GPS correction; steer on
      belief so the car physically misses. docs/jamming-effects-research.md.
- [ ] **More jamming targets & techniques** (Will, 1 Sep) - beyond comms +
      GNSS: radar/EO-IR sensor jamming; reactive/random/deceptive/follow-on
      techniques; barrage vs spot vs sweep; directional jamming. See
      docs/spectrum-and-sensors-reference.md.
- [ ] **Real frequency bands** (Will, 1 Sep) - GPS L1/L2/L5, GLONASS, Galileo;
      900/1300/2400/5800 ISM; Link 16 (hopping); UHF/VHF; SATCOM; X-band
      radar. Additive - band separation already modelled. Reference doc above.
- [ ] **More vehicle sensors** (Will, 1 Sep) - EO/IR camera + optical flow
      (works in open field where lidar can't), magnetometer, radar altimeter,
      wheel odometry, terrain-referenced nav, UWB/beacon ranging. Each feeds
      the position estimate; the Kalman step fuses survivors. Reference doc.
- [ ] **GNSS-denial grace period** (Will, 1 Sep) - real GPS+INS blends, so
      position stays accurate ~10 s (ArduPilot) before diverging, rather than
      drifting linearly from t=0 as v1 does. Add a per-agent denied-duration
      before drift onset / an EKF-style ride-out. Directly answers "why so
      quick".
- [ ] **Camera/optical-flow aiding in open field** - lidar needs structure
      (now enforced), but optical flow localises off ground texture with no
      3D features. A camera sensor should aid where a lidar cannot.
- [ ] **Formation as a variable** (Will, 1 Sep) - cluster / line-relay /
      spread / adaptive-relay. Splitting one long link into two short hops is
      a large SINR gain; optimising a relay's position to maximise SINR is a
      published anti-jam technique (Critical Analysis [29]; Drones 2025 [9]).
      Experiment 6. See docs/experiments-design.md round 3.
- [ ] **Antenna orientation / gain pattern** - Chen 2018 measured that antenna
      alignment materially changes received power on UAVs. A second geometric
      lever, after formation.
- [ ] **Terrain masking** - the radio horizon is now modelled; terrain
      blocking (hills, buildings) is the next realism step for both comms and
      GNSS denial.
- [ ] **Failsafe doctrines** - expand on_link_loss to hold/rtl/land/continue
      with a realistic timeout (ArduPilot's real menu). Comms jamming's full
      behaviour set.
- [ ] **Per-agent beliefs** (Will, 1 Sep) - each agent's own estimate of
      world state (positions of others, link health, its own pose) built only
      from what ITS sensors and network deliver, displayed beside ground
      truth. The gap between belief and truth IS the cost of jamming, and the
      thing Kalman filtering / ML later defend. Groundwork for both the
      Contested work and the estimation layer.
- [ ] **Editable doctrines** (Will, 1 Sep) - authority behaviours (e.g. does
      a squad fall back to the coordinator when its leader drops, or strand?)
      should be declarable per network rather than hard-coded. To be designed
      with the Network tab, where doctrine becomes visible.
- [ ] **Fleet-wide REOBJECTIVE** (Will, 1 Sep) - `REOBJECTIVE blue <verb>`
      / `REOBJECTIVE alpha <verb>`: scope tokens (network / squad / agent)
      resolved exactly like LAUNCH/HALT's, one objective applied to every
      resolved agent, gated by reachability like SETMISSION. An order-grammar
      extension, NOT a new file layer - missions stay the named overall goal,
      objectives stay per-agent verbs. Natural fit for the blue-cell
      terminal step.
- [ ] **SETMISSION before Play** - buffer a pre-run SETMISSION in the Console
      so the order can be staged before the sim starts, instead of the spool
      wipe eating it. Until then the order is: Play, SETMISSION, blue launch.

- [ ] **Processing gain / spread spectrum** (1 Sep) - the jamming model has
      no DSSS/FHSS margin, so it UNDERSTATES a spread-spectrum radio's
      resilience. Biggest gap before any "anti-jam radio" claim. See
      docs/jamming-model-justification.md.
- [ ] **Command interception** (1 Sep) - a cell hears commands on its band
      (plaintext / encrypted-traffic-only), reusing the rf_link verdict; the
      sensing half of a reactive jammer. Designed in
      docs/interception-design.md; builds with the cell terminals.
- [ ] **Jammer levels ladder** - constant (done) -> reactive -> random ->
      deceptive -> follow-on/smart, the survey's taxonomy and the README's
      ladder. Behaviours on the jammer agent.

## Later

- [ ] Spectrum waterfall — frequency across, time down, power as colour. Makes
      the difference between jamming and congestion visible rather than inferred
- [ ] Emitter detectability / direction finding — the cost of transmitting
- [ ] Map builder and topography import (greyed into the Scene tab already)
- [ ] Attack injection panel — the Cyber tab. Deliberately last: an attack panel
      over a simulator with no radio model can only fake its own results
- [ ] Adversarial agents on a second network — architecture is ready, behaviour
      is not
- [ ] Link matrix view — who can hear whom, as a grid. The map gets unreadable
      past about five agents
- [ ] Fiber tether link type — unjammable, spool-bounded, severable. The most
      distinctive single feature available
- [ ] Custom X-vs-Y plotter
- [ ] Multi-drop rosbag writer so runs open directly in PlotJuggler
- [ ] New-scenario wizard (blank-page case only — not a forced flow)

---

## Measurements Will needs to take

- [ ] **Wall reflectivity** — point a car at a brick wall, note where returns
      stop. Replaces the interpolated estimate in `effective_range()`
- [ ] **Odometry drift** — a known straight run and a known turn, to replace the
      flagged `ODOM_VARIANCE` guesses in `world_node.py`
- [ ] **Car masses** — three numbers, currently empty in the scenario
- [ ] **Radio tx power / sensitivity / data rate** — from whichever modules the
      fleet actually flies
- [ ] Rosbag from the real cars, for the overlay

## Known gaps and honest caveats

- Lidar is modelled at 271 rays on the wire; the real sensor takes 1081 samples
  per scan. Display resolution, deliberately separate from the sensor spec
- Collision is a kinematic constraint, not physics — no momentum, restitution
  or contact forces
- `link_state()` is a placeholder that falls off with distance. It is **not** a
  path loss model and must not be reported as one
- Fiber tether mechanics and emitter time-to-geolocate have **no good academic
  source**. Model them parametrically and declare the parameter free
- The embedded terminal uses pipes, not a pseudo-terminal: no sudo prompts,
  no vim
- The sim core still lives in `tools/stub_telemetry.py` and is imported by the
  ROS nodes. It should move into the `commsev` package once interfaces settle

---

## Done

- 2026-08-26 — Two-beam wall follower. Steering on one side distance oscillates
  by construction; a second beam angled forward gives the wall's angle in the
  same instant, so the correction is computed from where the car will be. sd
  fell from 0.17 m to 0.08 m and the excursions from 2.6 m to 1.4 m
- 2026-08-26 — `scenarios/wall_follow_demo.yaml`; the ground station suspended
  at 1.5 m is a real demonstration that the scan plane model works
- 2026-08-26 — Console passes the open scenario to `ros2 launch`. It had been
  hardcoded to the launch default, so the picture could silently disagree with
  the file
- 2026-08-26 — Bags close cleanly. The recorder is now SIGINT'd and waited for
  instead of killed, so MCAP writes its index and PlotJuggler stops reporting
  "corrupted / recovered partially". The data was always there; the footer
  saying where it was, was not
- 2026-08-26 — Mission contract v2: `target(agent, world)`, with lidar and link
  state reachable from inside a mission. Old four-argument form still runs,
  deprecated. `docs/writing-a-mission.md`, plus `wall_follow.py` (sensor loop)
  and `return_on_link_loss.py` (network-reactive)
- 2026-08-26 — PlotJuggler instructions rewritten for the MCAP flow
- 2026-08-26 — ROS 2 nodes: world, bridge, controller. Real `LaserScan`,
  `Odometry`, `AckermannDriveStamped` on 14 topics; Console reads them
- 2026-08-26 — Odometry covariance published instead of zeros
- 2026-08-26 — Embedded WSL terminal in the Console
- 2026-08-26 — Comms tab: emitter list and link table
- 2026-08-26 — Splittable drag-and-drop plot canvas; PlotJuggler-shaped tree
- 2026-08-26 — Scriptable missions (`missions/*.py`)
- 2026-08-26 — Reflectivity model from the datasheet's two points
- 2026-08-26 — Lidar as a plane at mount height; finite range; point-cloud draw
- 2026-08-26 — Stateful motion with blocking collision
- 2026-08-26 — Timeline scrubbing driving both plots and the world view
- 2026-08-26 — Provenance rule enforced by the spec loader
