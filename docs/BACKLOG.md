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

- **A run is a MAP plus a MISSION, combined at run time.** A map is the world
  (arena, radios, agent bodies, named points); a mission is tasking (one
  objective per agent). One mission runs on any map that defines the points it
  names; proven on `lab_box` and `big_hall`. `scenarios/` files stay valid as
  missions with the map inline - backward compatible, nothing forced to split.
  Terminology: the thing you write and run is a **mission**, not a scenario.
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
  ROS nodes. It should move into the `deadband` package once interfaces settle

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
