# Reference — jamming targets, real frequencies, and vehicle sensors

Raised by Will (1 Sep). A working reference for what to build next, so the
model's bands and sensors match reality rather than two placeholders.

## There are more than two jammings

We model two TARGETS (what is denied): the comms link and GNSS. That is the
right starting pair, but not the whole picture. Jamming is better thought of as
target x technique.

**Targets (what an emitter denies):**
- Comms / control link (uplink command, downlink telemetry, video) — modelled.
- GNSS / positioning — modelled.
- Radar (a vehicle's own radar, or a seeker's) — not modelled.
- EO/IR and other sensors (dazzling, decoys, IR flares) — not modelled.
- Data links specifically (e.g. Link 16, MAVLink) as distinct from generic RF.

**Techniques (how) — the survey's taxonomy (Priyadarshani et al. 2025):**
- Proactive: constant (modelled), random, deceptive.
- Reactive (fires only when it hears you) — the sensing half needs interception.
- Advanced: follow-on (chases a frequency-hopper), smart/adaptive.
- Spatial: barrage (wide band) vs spot (one channel) vs sweep; directional /
  spatial-nulling (needs antenna directionality, not yet modelled).

## Real frequencies (what a small UAS / ground robot actually uses)

Our 2400 MHz (comms) and 1575.42 MHz (GPS L1) are both real and common, but a
military platform spans far more. To build next:

| Use | Band(s) | Note |
|---|---|---|
| GNSS | GPS L1 1575.42, L2 1227.6, L5 1176.45; GLONASS ~1602; Galileo E1 1575.42 | multi-constellation = harder to deny all at once |
| Small-drone C2 / video | 900 MHz, 1.3 GHz, 2.4 GHz, 5.8 GHz (ISM) | cheap COTS drones live here; easily jammed |
| Military tactical datalink | Link 16 960–1215 MHz (freq-hopping) | hopping is the anti-jam |
| UHF/VHF C2 | 30–300 MHz (VHF), 300 MHz–1 GHz (UHF) | long range, low rate |
| SATCOM | L-band ~1.5 GHz, C-band 4–8, Ku 12–18, Ka 26.5–40 GHz | BLOS control |
| Radar / seekers | X-band 8–12 GHz, Ku, mmWave | sensor jamming target |

The model already honours band separation, so adding bands is additive: a
jammer on L1 denies GPS but not a Link-16 datalink; hopping across a band
defeats a spot jammer but not a follow-on one.

## Sensors on real military vehicles — the localisation menu

Why it matters: GNSS denial only bites if GNSS is the ONLY position source.
Real platforms layer several, and which ones survive jamming is the whole
resilience question. To add (todo), roughly in order of value here:

- **IMU** (have) — dead-reckoning source; drifts. Always present.
- **GNSS** (have) — absolute fix; jammable/spoofable.
- **LiDAR** (have, now scene-aware) — localises against STRUCTURE (walls,
  terrain); useless in open space; RF-immune.
- **EO/IR camera + visual odometry / optical flow** — localises against ground
  texture and features; works in open field where lidar cannot (optical flow
  needs only texture), degraded at night/low-texture; RF-immune.
- **Magnetometer / compass** — heading (not position); degrades drift by
  bounding the heading error. Cheap, worth modelling.
- **Barometer / radar altimeter** — height; radar altimeter is a jamming
  target itself.
- **Wheel odometry (ground)** — distance travelled; better than IMU alone for
  a car; slips.
- **Terrain-referenced navigation (TERCOM/TERPROM)** — match a radar/baro
  terrain profile to a stored map; classic cruise-missile GNSS-free nav.
- **UWB / radio beacons / signals-of-opportunity** — range off known emitters;
  the paper's collaborative-positioning idea (cars range off each other).
- **Celestial / vision-of-stars** — high-end, GNSS-free absolute.

The design pattern is already right: each sensor is an input to the position
ESTIMATE, and the estimator (a Kalman filter, the queued step) fuses whichever
survive. GNSS jamming then degrades but does not blind a well-sensored vehicle,
and that difference is measurable — which is the project.
