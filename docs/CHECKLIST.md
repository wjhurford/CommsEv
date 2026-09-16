> **SUPERSEDED (1 Sep 2026).** Written against the old map/mission vocabulary and the retired `scenarios/` files.
> The live model is **scene + fleet, composed by a mission** - see `docs/vocabulary.md`.
> Kept for provenance; do not follow the file names or commands here.

# CommsEv — project checklist

The running record of what is built and what is next, kept current every session
so nothing lives only in a chat thread. `BACKLOG.md` holds the reasoning and the
decisions; this is the flat tick-list. Newest done items at the top of Done.

The aim, in one paragraph, for when it is needed (papers, a LinkedIn post, a
hand-over): **CommsEv is a pre-test simulation framework for multi-robot
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

## Done — this session (verified)

- [x] **Two tabs: Overview (equipment) and Mission (tasking)** — Overview =
      system → network → agents → their sensors/equipment; Mission = system →
      network → agents → objectives. No hardware in the objective tree, no
      objective in the equipment tree. Tab order: Results, Cyber, Comms,
      Overview, Environment, Mission.
- [x] **System tier in the tree** — a system groups one or more networks
      (blue+green = friendly, red = adversary). A network opts in with
      `system: friendly`; until maps say so everything lands in one system, so
      the tier is correct now and simply gains siblings when red arrives.
- [x] **`REOBJECTIVE` retask grammar** — terminal line
      `REOBJECTIVE <agent> <objective> <args>`, e.g. `REOBJECTIVE car1 pursue
      car3`. Writes the queue line the running sim consumes next tick. Verified
      end to end: car1 held its lane, then broke formation and closed on car3
      from 4.00 m to 0.55 m. `REMISSION` is recognised and reports honestly
      that it is not built yet rather than failing silently.
- [x] **Publications tree keeps collapsed folders** across live refresh.

## Next up

- [ ] **Objective + speed on a selected agent, live** — DONE for objective and
      speed; the panel still needs to feel bespoke rather than generic.
- [ ] **Bespoke Properties panel** — per-agent, shaped to what that agent is and
      is doing, rather than a flat table of file keys.
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

The gap to close, stated plainly: right now nothing BREAKS. There is a control
condition and no contested condition. Everything below follows from that, and
in this order it tells one complete story — here is a fleet, here is the attack,
here is how it degrades, here is what each robot THOUGHT was happening, here is
whether the fallback held.

- [ ] **Attacks that actually fire (Cyber tab)** — jamming (a region or emitter
      raising the noise floor), spoofing (an agent fed false positions), replay.
      The single highest-value addition: it turns CommsEv from a fleet sim into
      a testbed for autonomy under attack, and every other feature below is more
      impressive once it exists.
- [ ] **PACE plan** (Primary/Alternate/Contingency/Emergency per network) —
      declare a fallback ladder, jam the primary, and MEASURE it: time to
      detect, time to switch, what was lost in between. A result, not a demo.
- [ ] **Degraded-comms autonomy behaviours** — return-to-comms, continue-on-
      last-order, hold-and-wait, autonomous regroup. `return_on_link_loss.py`
      is written and has never been run. Jam the net and watch the fleet behave
      sensibly without being told: that is the demo that lands.
- [ ] **Per-agent belief, not just ground truth** — each agent holds a stale,
      partial picture with age on every track ("last seen 4 s ago"). Let the
      operator view switch between ground truth and any agent's belief. The
      divergence between them under jamming is the whole point.
- [ ] **Multi-domain: the air layer** — ArduPilot + Gazebo. Ground and air on
      one network, with the air asset as a comms relay when the ground link
      breaks. Uses the z-axis already modelled.
- [ ] **Replay / after-action review** — scrub a recorded run with link drops,
      objective changes and attack events on one timeline. Makes it a tool
      rather than a script.
- [ ] **Seeded batch runs → results table** — same mission across N jamming
      intensities, headless, producing a curve. The difference between a demo
      and a paper. `seed` already exists.
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

---

## System architecture (internship scope — confined to the lab room)

The distinction that makes this research rather than a label: **decision
authority is not network topology.** A mesh can carry centralized decision-
making, and then the mesh survives a hub loss while the decision-making does
not. CommsEv can show that gap because it models comms as a first-class object.

- [x] **Three architectures with real consequences** — `architecture:` on a
      network, read by `command_authority()`, reported per agent every frame as
      {decider, reachable, tier}:
      * `centralized` — one coordinator decides; unreachable = no authority.
        Optimal while the link holds, brittle when it does not.
      * `decentralized` — every agent is its own authority; a lost link costs
        information, never authority. Robust, globally mediocre decisions.
      * `hierarchical` — squad leader, then coordinator, then `orphaned`.
        Degraded rather than decapitated. The one worth measuring.
- [ ] **`on_link_loss` driven by authority** — an agent that has lost authority
      should act on it (hold / continue / return), not carry on regardless.
      The state is reported; nothing consumes it yet. NEXT.
- [ ] **Architecture visible in the Console** — show each agent's decider and
      whether it is reachable; colour the tree row when authority is lost.
- [ ] **Measure the three against each other** — same mission, same jamming,
      three architectures; time-to-detect, time-to-recover, task completion.
      This is the internship result.

**A finding already, from testing this:** nothing in an 8x8 room can drive out
of radio range — the link model only dies past 12 m. So architecture has no
observable consequence in the lab *unless something takes the link away*. That
is the argument for the Cyber/jamming work: in a room this size, attack is the
only realistic route to link loss.
