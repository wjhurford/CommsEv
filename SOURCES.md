# Sourcing register

Every physical parameter in Deadband carries a source. This file tracks what
still needs one.

**Rules.** No number enters the codebase without a source. A source is a
datasheet, a paper, or a measurement we took ourselves and can describe. Where
no good source exists, say so and make the parameter explicitly free rather than
inventing a value that looks researched.

Status: `TODO` · `FOUND` · `MEASURED` · `NO SOURCE` (declared free parameter)

### Correction, 2026-09: the unaided drift rate was invented

It was `0.04` — 4% of distance travelled — attributed to a MEMS inertial
figure for an autopilot. It is an **order of magnitude worse than anything
measured** for the vehicle class actually being simulated, and the consequence
was not subtle: on any scene declaring GNSS denied (`lab_box`, indoors, no sky
view) a fleet accumulated a quarter of a metre of error over a 6 m lane, so
every mission failed on drift within seconds and the scene was unusable.

Replaced with a measured figure from a paper on exactly this vehicle class.
The result changed a headline finding: under GNSS denial on the 200 m corridor
the fleet now scores a **partial** pass — some vehicles report a false arrival
and some do not — where the invented rate failed every vehicle every time.
That looked decisive and was an artefact.


## Datasheet lookups

| Parameter | Status | Source |
| --- | --- | --- |
| Lidar range, FOV, scan rate, accuracy (UST-10LX) | TODO | Datasheet in project files |
| Radio tx power, sensitivity, data rate | TODO | Depends on module choice — pick hardware first |
| Rover mass, wheelbase, CoG, steering limits | TODO | Traxxas spec + our own measurements |
| Rover dimensions | MEASURED | Car #1, 2026-07-21 |

## Needs literature

| Parameter | Status | Notes |
| --- | --- | --- |
| Lidar noise model beyond datasheet accuracy | TODO | Hokuyo characterisation papers exist |
| IMU bias instability, random walk | TODO | Allan variance characterisation |
| Indoor path loss exponent, confined room | TODO | Matters most for the 8 m box |
| Multipath fading model, K-factor, walled arena | TODO | Depends on wall material |
| Outdoor propagation, short ground strip | TODO | Ground reflection is not negligible at 80 m |
| Background spectrum occupancy per band | TODO | Also replaceable by our own field recordings |
| Quadcopter thrust and drag coefficients | TODO | Rotor characterisation, or a thrust stand |
| GNSS receiver behaviour under spoofing | TODO | Well covered in GNSS security literature |
| Estimator innovation gate thresholds | TODO | ArduPilot defaults are in source; rationale is in the literature |

## Probably no clean source

| Parameter | Status | Notes |
| --- | --- | --- |
| Fiber tether mass/m, drag, bend radius, spool | NO SOURCE | Manufacturer data plus our own measurement. Model should say so. |
| Emissions detectability, time-to-geolocate | NO SOURCE | Open literature thins out fast. Model parametrically, declare free. |

---

## Model constants (added Aug–Sep 2026)

The RF / jamming / navigation model. Status per the rules above.

| Parameter | Value | Status | Source |
| --- | --- | --- | --- |
| Path-loss form `20lg(f)+20lg(d)+32.44` | — | FOUND | Friis; independently matched by Zhou et al. 2020 (arXiv:2008.08212) |
| Path-loss exponent (indoor) | 2.8 | NO SOURCE | Defensible indoor-multipath default; scene declares it |
| Path-loss exponent (open field) | 2.2 | NO SOURCE | Near-free-space default; scene declares it |
| Noise floor | −95 dBm | NO SOURCE | Defensible default; measure with an SDR to upgrade |
| Receiver sensitivity | −85 dBm | NO SOURCE | Commodity 2.4 GHz figure; module datasheet would upgrade |
| PDR-vs-SINR curve | logistic, k=0.8 | NO SOURCE | **Stand-in for a modulation/coding curve. Shape is right, values are not a real radio.** |
| Radio horizon `4120(√h₁+√h₂)` | — | FOUND | Standard 4/3-earth radio horizon |
| GNSS band | 1575.42 MHz | FOUND | GPS L1, standard |
| GNSS denial threshold | −120 dBm | NO SOURCE | GNSS arrives ~−128 dBm ("weak and easily jammed", Critical Analysis of Spoofing & Jamming); −120 is a declared free parameter |
| Dead-reckoning drift, unaided, **wheeled** | 0.49% of distance | **MEASURED** | Papadopoulos & Misailidis, *On Differential Drive Robot Odometry with Application to Path Planning*, ECC 2007, Table I: Pioneer 3-DX over ~125 m paths, uncalibrated error 16.1–60.9 cm = 0.13–0.49% of distance. Worst case used — an uncalibrated vehicle is the honest default |
| Dead-reckoning drift, vision/lidar aided | **UNSOURCED** | **OPEN** | Held equal to the unaided figure as a floor. The previous 1% (UAV Navigation VNS vendor claim) is now *worse* than the measured unaided rate, which is incoherent — an aiding sensor cannot make a vehicle drift faster. Needs a measured lidar- or vision-aided figure. Do not guess |
| Dead-reckoning drift, **airborne** | — | **OPEN** | A quadcopter has no wheel odometry and drifts on inertial integration. A different number entirely; must not be assumed equal to the wheeled figure. Blocks the UAV transition |
| GNSS usable window before divergence | ~10 s | FOUND | ArduPilot Copter, GPS Failsafe & Glitch Protection |
| Drift heading random-walk σ | 0.15 rad/tick | NO SOURCE | Free parameter; sets how error meanders, not how fast it grows |
| ACI correlation coefficient μ(Δf) | — | FOUND (structure) | Zhou et al. 2020 — properties (μ=1 at Δf=0, →0 far apart, symmetric, measurable) |
| Adjacent-channel rejection | 30 dB/channel | NO SOURCE | Commodity receiver figure; declared free parameter |
| Channel bandwidth | 20 MHz | NO SOURCE | Nominal |
| Contention airtime share `1/(n+1)` | — | FOUND | Channel-sharing arithmetic, not a fitted constant |
| Contention deadline model `1−exp(−deadline/delay)` | — | FOUND | Standard M/M/1 exponential service assumption; deadline comes from the network's own declared QoS |
| Contention audibility threshold | −85 dBm | NO SOURCE | Same as receiver sensitivity; declared free parameter |
| Vehicle max accel (default) | 2.0 m/s² | NO SOURCE | Nominal for a 1:10 car; **measure the real cars** |
| Vehicle min turn radius (default) | 0.6 m | NO SOURCE | Nominal; **measure the real cars** |
| LoS / NLoS excess path loss | 3 dB / 23 dB | FOUND, NOT YET USED | Zhou et al. 2020 — we do not model an LoS/NLoS split yet |

**Honest total (live files, Sep 2026): 35 of 57 declared quantities carry a
source. 22 do not.** Every unsourced one above is a declared free parameter,
not a disguised guess — but a result that turns on any of them must say so.
