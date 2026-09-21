# Roadmap — decisions of 1 September 2026

The build order from this point, one step per patch section in
`history/PATCH-09-CHECKS` (and onward), each applied, tested and click-checked
before the next. Newer decisions supersede older text in `history/BACKLOG.md`;
this file is the order.

## Done

**1. Contested tree — DONE (1 September 2026, Step 4 in `history/PATCH-09-CHECKS`).**
Jamming becomes physics rather than a label:

- the scene's background (noise floor, path-loss exponent) feeds `rf_link()`
  instead of function defaults — the baseline is the scene's;
- a **jammer is an ordinary agent** (`jammer:` block: tx power, band) on the
  red network; `red launch` arms it and its power enters every same-band
  receiver's SINR denominator; PDR falls, links degrade or drop, and command
  authority in the frame degrades with them;
- the Contested tab becomes a live tree: scene baseline, jammers (position,
  power, band, armed), and the noise floor each agent actually experiences;
- `fleets/3_roboracer_jammed.yaml`: the lab fleet plus one red jammer — the
  first genuinely contested run.

## Done

**2. Network tab — DONE (1 September 2026, Step 5).** Command authority and
measured topology, live in the GUI (`netcheck`'s tables, now visible): who
decides for whom, reachable or not, declared routing against measured shape,
active against spare links. This is where the **editable doctrines** question
is addressed (whether a squad strands or falls back when its leader drops —
declarable per network, not hard-coded).

## Now (correctness focus)

**PRIORITY (decision, 1 September 2026) — model the actual effects of jamming,
emergent rather than hard-coded.** See `docs/jamming-effects-research.md`. Two
kinds of jamming, two effects: communications jamming → failsafe doctrine
(hold/rtl/land/continue, real timeout); GNSS jamming → belief-against-truth
position drift (dead reckoning, error grows for ~10 s then diverges). The
no-LiDAR fleet (`3_roboracer_no_lidar`, DONE) is the clean rig. Build order:
(1) belief/truth + GNSS drift [DONE 1 September 2026, Step 6], (2) failsafe
doctrines, (3) mitigations (LiDAR/vision/Kalman close the gap). Each step is
reviewed by the project lead before the next.

**3. Cell terminals — DONE with the Network tab (Step 5).** Remaining on the
red side: build interception, then the reactive jammer. Deferred by decision of
1 September 2026 until the single jammer is verified correct. Scoped by side:
the blue cell commands blue, the red cell commands red (including the jammer),
and the white cell is the umpire's full shell. **Fleet-wide REOBJECTIVE**
(`REOBJECTIVE blue <verb>`, scope tokens like LAUNCH/HALT's,
reachability-gated) lands here. **Command interception** lands here too
(`docs/interception-design.md`): a cell hears commands sent over its band,
plaintext or encrypted-traffic-only, and can act — a red cell driving a mobile
jammer toward an overheard sender is a reactive jammer, the first adversarial
exercise.

**4. Per-agent beliefs.** Each agent's own estimate of world state, built only
from what its sensors and network deliver, shown beside ground truth. The
belief–truth gap is the measured cost of jamming and the interface the
estimation layer defends.

**5. Kalman filters + ML (design doc, then build).** Estimation (EKF on
odometry+GNSS+links) closing the belief gap under jamming; learned detectors
for jamming/spoofing classification; both graded by the belief–truth gap and
mission outcome, per the README's levels (continuous → reactive → random →
deceptive).

**6. Gamified levels.** The README's ladder, built on all of the above:
scripted jamming levels, graded on communications quality + mission outcome +
time.

## Standing constraints

- Every step ships tests + a `history/PATCH-09-CHECKS` section; commit after
  each.
- Provenance rule everywhere; free parameters declared, never hidden.
- Spawn dialog stays simple (x, y, z, yaw) — click-to-place is not queued
  until the companion apps exist.
- SETMISSION order is Play → SETMISSION → blue launch (pre-Play buffering is
  a convenience, queued).

## Major milestone (decision, 1 September 2026) — real drones via ArduPilot + Gazebo

Integrate ArduPilot SITL flight stacks flying in Gazebo Harmonic as the agents,
in place of (or alongside) the kinematic stub. The mission contract and the
whole Console/telemetry boundary are already designed for this: algorithms take
plain numbers, the ROS node is a wrapper, Gazebo Harmonic is the named primary
world. This is the sim-to-real step and the largest one; it is scoped after the
jamming effects, jammer types and experiments are solid on the fast stub.
ArduPilot is GPL-3.0 and is run as a SEPARATE process, never linked or vendored
(see `LICENSE`).
