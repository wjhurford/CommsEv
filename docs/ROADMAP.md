# Roadmap — agreed with Will, 1 Sep 2026

The build order from here, one step per patch section in PATCH-09-CHECKS (and
onward), each applied + tested + click-checked before the next. Newest
decisions supersede older text in BACKLOG.md; this file is the order.

## Done

**1. Contested tree — DONE (1 Sep, Step 4 in PATCH-09-CHECKS).** Jamming becomes physics, not a word:
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

## Now

**2. Network tab.** Command authority and measured topology, live in the GUI
(netcheck's tables, finally visible): who decides for whom, reachable or not,
declared routing vs measured shape, active vs spare links. This is where the
**editable doctrines** discussion happens (does a squad strand or fall back
when its leader drops — declarable per network, not hard-coded).

**3. Blue / red / white cell terminals.** Scoped by side: blue cell commands
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
- Every step ships tests + a PATCH-09-CHECKS section; commit after each.
- Provenance rule everywhere; free parameters declared, never hidden.
- Spawn dialog stays simple (x, y, z, yaw) — click-to-place is not queued
  until the companion apps exist.
- SETMISSION order is Play → SETMISSION → blue launch (pre-Play buffering is
  a nicety, queued).
