# Sourcing register

Every physical parameter in Deadband carries a source. This file tracks what
still needs one.

**Rules.** No number enters the codebase without a source. A source is a
datasheet, a paper, or a measurement we took ourselves and can describe. Where
no good source exists, say so and make the parameter explicitly free rather than
inventing a value that looks researched.

Status: `TODO` · `FOUND` · `MEASURED` · `NO SOURCE` (declared free parameter)

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
