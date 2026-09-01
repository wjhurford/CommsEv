# What actually happens when an agent is jammed — research, and a design to review

Will's question: don't hardcode a reaction; model the mechanism so the effects
(drift, or whatever) are *visible and emergent*. Researched against the project
library and current flight-controller documentation. **Nothing here is built
yet — this is for review.**

## The key finding: there are TWO jammings, with TWO different effects

We have been modelling one band (comms, 2.4 GHz) and one effect (the fleet
holds). Real jamming splits cleanly in two, on two different frequencies, with
two genuinely different consequences. Conflating them is why "freeze" felt
incomplete to you.

### 1. Comms jamming — the vehicle loses its ORDERS, not its position

Jam the control/telemetry link (the datalink between GCS and vehicle). The
vehicle still knows exactly where it is (GPS is fine) — it just stops hearing
its commander. After a timeout it runs a **failsafe**, and the failsafe action
is a *configured choice*, not an accident. ArduPilot's GCS-failsafe options are
exactly: **RTL** (return to launch), **SmartRTL**, **Land**, **Brake/Loiter**,
or **continue the current auto mission** (ArduPilot Copter GCS Failsafe;
default trigger 5 s with no heartbeat). The project library confirms the same
doctrine: on link loss a UAV will "try to reconnect to the control station or
locate nearby UAVs to return home safely" (Critical Analysis of Spoofing and
Jamming, ref [53]).

**So our current `on_link_loss: hold` is CORRECT for comms jamming — it is one
real failsafe among several, and declaring it per fleet is honest, not a
hardcode.** What it is missing is the other options (rtl / land / continue) and
the realistic **timeout** (a vehicle does not failsafe the instant one packet
drops; it waits a few seconds). This is a small, faithful extension.

Effect you SEE: a comms-jammed centralized fleet, after a short delay, does its
declared failsafe — holds, or turns for home, or (if `continue`) drives its
mission blind. A *decentralized* fleet keeps coordinating locally — the
resilience result we already show.

### 2. GNSS jamming — the vehicle loses its POSITION, and DRIFTS

Jam the GNSS band (~1.5 GHz, a *different* frequency from comms). Now the
vehicle cannot fix its position and falls back to **dead reckoning** —
estimating where it is by integrating its last known position with speed and
heading from the IMU (Tedeschi & Di Pietro 2021; the technique the whole
GNSS-denied literature is about). Dead reckoning **drifts**: the estimate is
good briefly, then the error grows without bound. Concretely, ArduPilot states
"inertial sensors allow approximately 10 seconds of accurate position
information," after which "the horizontal position cannot be maintained at
all"; the survey literature calls it "accuracy loss, error accumulation, and
divergence" (Critical Analysis, ref [35]).

This is the effect you were reaching for. The vehicle does not freeze — it
keeps trying to reach its waypoint, but using a position estimate that is
sliding away from the truth, so it **physically ends up in the wrong place**,
and it does not know it. That is the deep-strike-recce failure in your README:
the drone relays coordinates it *believes* are right, the missile is sent
there, and the location is wrong. GNSS jamming is how that failure is
manufactured.

Effect you SEE: the car's TRUE track (where it really is) and its BELIEVED
track (where it thinks it is) separate and drift apart, the gap widening the
longer GNSS is denied — and if it is navigating on belief, its true path bends
off-target. That gap, in metres, is the measured cost of the jamming.

## The design to build this — emergent, not hardcoded

The drift must *emerge* from integrating motion without correction, not be a
scripted wander. The mechanism:

1. **Every agent gets two poses:** `true` (the physics — where it actually is,
   what the sim already computes) and `believed` (its own estimate — new).
2. **GNSS available (not jammed):** each tick the belief is corrected toward
   truth — `believed = true + small GPS noise`. Belief tracks truth. This is
   the normal case and looks like today.
3. **GNSS jammed:** no correction. The belief advances by *dead reckoning* —
   it integrates the commanded velocity plus a small per-tick IMU error
   (bias + random walk). With no GPS to reset it, that error accumulates, so
   `believed` slides away from `true`. Drift is the *integral of the error*,
   which is why it grows with time exactly as the sources describe — no
   hardcoded drift value, it falls out of integrating noise.
4. **The controller steers on BELIEF** (a real vehicle only has its estimate),
   so a car aiming for point B by its drifted belief physically misses B. The
   miss is emergent.
5. **Parameters are declared, per the provenance rule:** IMU bias, random-walk
   sigma, GPS noise. Free parameters, flagged, tuned to give the ~10 s honest
   window; never invented silently.

The **no-LiDAR fleet** (`3_roboracer_no_lidar`, built now) is the clean rig for
this: with no LiDAR closing a loop on the walls, nothing masks the drift — the
belief-vs-truth gap is the pure positioning fault. (A LiDAR/vision car could
partly correct itself — that is the *mitigation* story for later, and it needs
the drift to exist first to be worth anything.)

## What to add so the two jammings are separable

- Jammers already carry a `band`. Give the fleet's radios a comms band and add
  a **GNSS band** (~1575 MHz) as a thing that can be jammed independently. A
  jammer on the comms band triggers failsafes; a jammer on the GNSS band
  causes drift; a broadband jammer does both. Band separation (already
  modelled) is what makes them distinct — and what makes anti-jam (hopping,
  a separate protected GNSS antenna) meaningful later.

## Proposed build order (your call)

1. **Belief-vs-truth + GNSS-denial drift** on the no-LiDAR fleet — the headline
   effect. Two tracks on the map, a live "position error (m)" readout, drift
   that grows under GNSS jamming and recovers when it lifts.
2. **Failsafe doctrines** — expand `on_link_loss` to hold / rtl / land /
   continue with a realistic timeout, so comms jamming has its full, real
   menu of behaviours.
3. Only then the mitigations (LiDAR/vision correction, sensor fusion, the
   Kalman filter) — which are exactly "close the belief-truth gap under
   denial," and now have something real to close.

### Sources
- Tedeschi & Di Pietro, *SpaCCS* 2021 — dead-reckoning in GNSS outage; jammer
  as an ordinary emitter.
- *A Critical Analysis of Spoofing and Jamming Approaches* (project library) —
  GNSS-denied navigation, dead-reckoning error accumulation and divergence
  [35]; link-loss return-home behaviour [53].
- ArduPilot Copter documentation — [GCS Failsafe](https://ardupilot.org/copter/docs/gcs-failsafe.html)
  (RTL/SmartRTL/Land/Brake/continue, 5 s timeout) and
  [GPS Failsafe & Glitch Protection](https://ardupilot.org/copter/docs/gps-failsafe-glitch-protection.html)
  (~10 s of usable inertial position, then horizontal position "cannot be
  maintained at all"; EKF rejects GPS and grows an uncertainty radius).
