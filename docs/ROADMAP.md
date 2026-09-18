# Roadmap — agreed with Will, 1 Sep 2026

The build order from here, one step per patch section in history/PATCH-09-CHECKS (and
onward), each applied + tested + click-checked before the next. Newest
decisions supersede older text in history/BACKLOG.md; this file is the order.

## Done

**1. Contested tree — DONE (1 Sep, Step 4 in history/PATCH-09-CHECKS).** Jamming becomes physics, not a word:
- the scene's background (noise floor, path-loss exponent) feeds `rf_link()`
  instead of function defaults — the baseline is the scene's;
- a **jammer is an ordinary agent** (`jammer:` block: tx power, band) on the
  red network; `red launch` arms it and its power lands in every same-band
  receiver's SINR denominator; PDR falls, links degrade or drop, and command
  authority in the frame degrades with them;
- the Contested tab becomes a live tree: scene baseline, jammers (position,
  power, band, armed), and the noise floor each agent actually experiences;
- `fleets/3_roboracer_jammed.yaml`: the lab fleet plus one red jammer — the
  first genuinely contested run.

## Done

**2. Network tab — DONE (1 Sep, Step 5).** Command authority and measured topology, live in the GUI
(netcheck's tables, finally visible): who decides for whom, reachable or not,
declared routing vs measured shape, active vs spare links. This is where the
**editable doctrines** discussion happens (does a squad strand or fall back
when its leader drops — declarable per network, not hard-coded).

## Now (correctness focus)

**PRIORITY (Will, 1 Sep) — model the ACTUAL effects of jamming, emergent not
hardcoded.** See docs/jamming-effects-research.md. Two jammings, two effects:
comms jamming -> failsafe doctrine (hold/rtl/land/continue, real timeout);
GNSS jamming -> belief-vs-truth position drift (dead reckoning, error grows
~10 s then diverges). No-LiDAR fleet (3_roboracer_no_lidar, DONE) is the clean
rig. Build order: (1) belief/truth + GNSS drift [DONE 1 Sep, Step 6], (2) failsafe doctrines,
(3) mitigations (LiDAR/vision/Kalman close the gap). Reviewed by Will first.


**3. Cell terminals — DONE with the Network tab (Step 5).** Remaining on the red side: build interception, then reactive jammer. Deferred by Will until the ONE jammer is verified correct. Scoped by side: blue cell commands
blue, red cell commands red (the jammer!), white cell is the umpire's full
shell. **Fleet-wide REOBJECTIVE** (`REOBJECTIVE blue <verb>`, scope tokens
like LAUNCH/HALT's, reachability-gated) lands here. **Command interception**
lands here too (docs/interception-design.md): a cell hears commands sent over
its band, plaintext or encrypted-traffic-only, and can act — a red cell
driving a mobile jammer toward an overheard sender is a reactive jammer, the
first adversarial exercise.

**4. Per-agent beliefs.** Each agent's own estimate of world state, built
only from what its sensors and network deliver, shown beside ground truth.
The belief-truth gap is the measured cost of jamming — and the interface the
estimation layer defends.

**5. Kalman filters + ML (design doc, then build).** Estimation (EKF on
odometry+GNSS+links) closing the belief gap under jamming; learned detectors
for jamming/spoofing classification; both graded by the belief-truth gap and
mission outcome, per the README's levels (continuous → reactive → random →
deceptive).

**6. Gamified levels.** The README's ladder, built on all of the above:
scripted jamming levels, graded on comms quality + mission outcome + time.

## Standing constraints
- Every step ships tests + a history/PATCH-09-CHECKS section; commit after each.
- Provenance rule everywhere; free parameters declared, never hidden.
- Spawn dialog stays simple (x, y, z, yaw) — click-to-place is not queued
  until the companion apps exist.
- SETMISSION order is Play → SETMISSION → blue launch (pre-Play buffering is
  a nicety, queued).

## Major milestone (Will, 1 Sep) — real drones via ArduPilot + Gazebo

Integrate ArduPilot SITL flight stacks flying in Gazebo Harmonic as the agents,
in place of (or alongside) the kinematic stub. The mission contract and the
whole Console/telemetry boundary are already designed for this: algorithms take
plain numbers, the ROS node is a wrapper, Gazebo Harmonic is the named primary
world. This is the sim-to-real step and the big one; scoped after the jamming
effects, jammer types and experiments are solid on the fast stub. ArduPilot is
GPL-3.0 - run as a SEPARATE process, never linked or vendored (LICENSE).
