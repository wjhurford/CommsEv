# GNSS-denial drift — the model, as built

The visible effect of GNSS jamming: a vehicle that loses its position fix
dead-reckons and drifts. The drift is built as an emergent effect, not a
hardcoded value, and every number is a declared free parameter with a source.
See `docs/jamming-effects-research.md` for the background and
`docs/jamming-model-justification.md` for the scope line (trust trends, not
exact metres).

## Two poses per agent

Each agent has a **true** pose (the physics, in `poses`) and a **belief**
(`agent["belief"]`): its own estimate of where it is. `drift = belief - truth`,
reported per frame as `position_error_m`.

## The rule

Each tick, an agent has a position fix if either:

- GNSS is available (the scene provides it) and is not denied by a GNSS-band
  jammer, or
- it carries an RF-immune aiding sensor (lidar / camera), so it can localise
  against the world without GNSS or the radio.

**With a fix:** the belief tracks truth exactly (GNSS/lidar give an absolute
position each tick). Error ~ 0. A regained fix therefore corrects the drift.

**Without a fix (GNSS denied, no aiding sensor):** dead reckoning. The belief
follows the vehicle's real motion, but an error accumulates each tick at
`drift_rate x distance moved`, in a slowly random-walking direction. Because
the drift is the integral of that per-tick error, it grows with distance — the
percentage-of-distance behaviour the literature reports — and it emerges rather
than being a scripted wander. **The controller steers from the belief** (a
vehicle has no other position), so a drifted vehicle aims from a wrong origin
and its true path bends off-target: it "arrives" at its waypoint in its own
estimate while physically ending up `position_error_m` away, and it does not
know it.

## The numbers, and their sources

| parameter | value | source |
|---|---|---|
| `DRIFT_RATE_UNAIDED` | 0.04 (4% of distance) | UAV Navigation VECTOR autopilot, MEMS inertial, ~33 m/min (Dead Reckoning Operations) |
| `DRIFT_RATE_AIDED` (vision) | 0.01 (1%) | same source, with Visual Navigation System |
| `DRIFT_RATE_LIDAR` | 0.0 | lidar/vision localisation in GNSS-denied nav (reserved; lidar cars resist) |
| GNSS usable window | ~10 s before divergence | ArduPilot GPS Failsafe: "~10 s of accurate position", then "cannot be maintained" |
| `GNSS_BAND_MHZ` | 1575.42 (GPS L1) | standard |
| `GNSS_DENIAL_DBM` | -120 dBm received | free parameter; GNSS signals ~-128 dBm are "weak and easily jammed" (Critical Analysis of Spoofing and Jamming) |
| `DRIFT_TURN_SIGMA` | 0.15 rad/tick | free parameter (how the error meanders, not how fast it grows) |

## How to see it

Use scene `open_field` (GNSS available), fleet `3_roboracer_no_lidar` (no
aiding sensor), plus `red_jammer`. Retune the jammer to the GNSS band with
`JAM jam1 band 1575`, then `red launch`. Each car's belief ghost separates from
its true position and the gap widens; the Sensor panel shows "GNSS DENIED -
dead reckoning, est. position error X m". Issue `red halt` (or
`JAM jam1 band 2400`) and the fix returns; the ghosts snap back.

Swap the fleet for `3_roboracer` (which has lidar) and the same GNSS jamming
has no effect: the lidar localises the car without GNSS. That contrast is the
GNSS-denied-navigation result, and the reason sensor fit matters.

## Limits (v1)

- Lidar aiding is modelled as "has a lidar => localises => no drift", a scalar
  proxy. Real visual/lidar odometry is a full estimator with its own error and
  needs map features (a lidar in a featureless field would not in fact help).
  Flagged; the richer version is the sensor-fusion / Kalman step.
- Drift direction is a random walk; a real INS has structured bias. The growth
  rate is sourced; the exact path is illustrative.
- One vehicle, one fix source at a time. GNSS+INS blending (an EKF that rides
  out a short outage on inertial before diverging) is the mitigation step.

### Sources

- UAV Navigation, *Dead Reckoning Operations* — 4%/distance MEMS, 1% with VNS.
- ArduPilot Copter, *GPS Failsafe & Glitch Protection* — ~10 s usable inertial.
- *A Critical Analysis of Spoofing and Jamming Approaches* (project library) —
  GNSS-denied navigation, error accumulation and divergence.
- Tedeschi & Di Pietro, *SpaCCS* 2021 — dead reckoning in GNSS outage.
