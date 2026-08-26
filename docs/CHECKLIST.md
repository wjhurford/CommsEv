# Deadband — project checklist

The running record of what is built and what is next, kept current every session
so nothing lives only in a chat thread. `BACKLOG.md` holds the reasoning and the
decisions; this is the flat tick-list. Newest done items at the top of Done.

The aim, in one paragraph, for when it is needed (papers, a LinkedIn post, a
hand-over): **Deadband is a pre-test simulation framework for multi-robot
missions under real communication constraints. You describe a world (a map) and
a task (a mission of per-agent objectives), combine them, and watch the fleet
run — with the network modelled as a first-class object, so a mission can react
to link loss the way a real vehicle must. It is built to blend with ROS 2, the
RoboRacer stack, and later ArduPilot for drones, and to be driven mostly from
one app.**

---

## Done

- [x] **Map / mission split** — a run is a MAP (world) plus a MISSION (tasking),
      merged at run time. One mission runs on any map that defines the points it
      names. Proven on `lab_box` and `big_hall`.
- [x] **Named points of interest** — a map defines A, B, HOME…; an objective
      says "shuttle between A and B"; the map resolves them. Portable across maps.
- [x] **Objectives** — `shuttle`, `pursuit`, `patrol`, `orbit`, `static`,
      `script`. A verb with parameters, never a fixed destination.
- [x] **Live retasking (stub)** — swap an agent's objective mid-run via a file
      channel; the next tick it is doing the new thing. Proven: shuttle→pursue.
- [x] **Overview tree shows mission name + per-agent objective** — with a
      double-click popup describing what each agent is doing.
- [x] **Console resolves map+mission for display** — a split mission shows its
      agents (from the map) instead of an empty tree.
- [x] **First hand-written mission** — `missions/my_first.py` + `formation_demo`
      rig. (Written and tested headless; Will to watch it drive in the Console.)
- [x] **Scenario tab renamed to Overview.**
- [x] **Bags close cleanly** — MCAP, readable in PlotJuggler.
- [x] **Mission contract v2** — `target(agent, world)`, lidar + link reachable.
- [x] **ROS 2 nodes** — world, bridge, controller on real message types.
- [x] **Delivery via patch** — every chat-interface session ends with a
      `git am` patch, since the sandbox cannot push to the remote.

## In progress (this session)

- [ ] **Two tabs: Overview (equipment) and Mission (tasking)** — Overview =
      system → network → agents → their sensors/equipment; Mission = system →
      network → agents → objectives. Tab order: Results, Cyber, Comms, Overview,
      Environment, Mission.
- [ ] **System tier in the tree** — a system groups one or more networks (e.g.
      blue+green = friendly, red = adversary). Real from the start, even with one.
- [ ] **`REOBJECTIVE` retask grammar** — terminal line
      `REOBJECTIVE <agent> <objective> <args>`, e.g. `REOBJECTIVE car1 pursue
      car3`. Later `REMISSION` to retask a whole system.

## Next up

- [ ] **Objective + speed on a selected agent, live** — visible while running.
- [ ] **Per-agent bespoke sensor panel** — shows the sensors THIS agent has;
      lidar stays as default when present, is not removed.
- [ ] **Pre-run point editing for split missions** — in-app map-point editor;
      right now the Console says the field is map-owned.
- [ ] **Retask over ROS** — a ROS 2 service; the file channel is stub-only.
- [ ] **Mission dropdown / picker in the Console** — set an objective the
      clickable way, writing back to the file.
- [ ] **File picker shows maps/ and missions/**, not just scenarios/.

## The in-app setup workflow (hand-over milestone, built last of its group)

- [ ] One page: pick map → conditions → mission → networks → agents per
      network → define agents → objectives + points → comms channel → system
      architecture → **Simulate** → drops into the run view.

## Research capabilities (the paper-shaped work)

- [ ] **PACE plan** (Primary/Alternate/Contingency/Emergency per network) —
      declare a fallback ladder, measure whether it holds under attack.
- [ ] **Real-test overlay** — draw a real-car rosbag over the simulated run.
- [ ] **VESC model** — real actuator limits; an attack surface below ROS.
- [ ] **Edge compute budget** — per-agent CPU/memory as a modelled resource.

## Later

- [ ] Spectrum waterfall; emitter detectability / direction finding; map builder
      + topography import; attack-injection (Cyber) panel; adversarial agents on
      a second network; link-matrix view; fiber tether link type; ArduPilot +
      Gazebo Harmonic air layer.

## Gating / external (not code)

- [ ] **Open-sourcing cleared in writing** with Cheng and Loughborough IP —
      irreversible, gates publication.
- [ ] **Measurements only Will can take** — car masses, wall reflectivity, radio
      tx/sensitivity/data-rate, odometry drift, a real-car rosbag.

## Deferred by decision

- ROS is paused — running stubs only until the stub side is perfect.
