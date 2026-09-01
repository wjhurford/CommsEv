# The contested background — what a scene declares, and why

The **scene owns the background**: every condition that exists before anyone
hostile does. The test for whether something belongs here is one question —
*would this be true of the world if no adversary existed?* If yes, it is
background and the scene declares its baseline. If no — a jammer, a spoofer,
a deliberate emitter — it is an **agent** (or an attack), and it belongs to a
fleet or to the Contested tab, acting ON this baseline.

That split is what keeps the physics honest: `rf_link()` computes SINR with a
jammer as *just another term in the noise denominator*. The denominator's
resting value is the scene's; the extra terms are somebody's fault. Results
then measure attacks as departures from a declared baseline rather than from
an assumption nobody wrote down.

The provenance rule applies to every number here: `{value, unit, source}`,
unsourced values counted and flagged, never invented.

## What the scene declares today (`arena:` in scenes/*.yaml)

**`propagation`** — path loss exponent, shadowing sigma. Why here: the walls
impose it. It is the exponent that turns distance into received power
(`PL(d) = PL(d0) + 10·n·log10(d/d0)`); indoor multipath sits around n=2.7–3.5
against free space's 2.0. Every link budget in the run flows through it, so it
cannot belong to any one radio or agent. Currently declared with empty
sources — it is on the measurements list, and any result that turns on its
exact value has to say so.

**`spectrum`** — resting model (`clean`), `background_emitters`, and now
`noise_floor` (dBm). Why here: the noise floor is the denominator of every
SINR *before anyone transmits* — jamming and congestion are both measured as
departures from it, so the baseline must be a property of the room, not a
default buried in `rf_link()`'s signature (where -95 dBm currently lives as a
stand-in). `background_emitters` is the seam for the "noise floor raised by
emitters nobody controls" condition in the README — congestion, which the
spectrum-waterfall backlog item exists to make visually distinct from jamming.

**`gnss`** — availability (`denied` indoors: no sky view). Why here: the sky
is part of the world. This is deliberately binary today; the honest next
refinement (needed before GNSS *spoofing* can mean anything in the Contested
tab) is a received-quality figure — nominal C/N0 per constellation — because
real-world jamming presents as C/N0 depression long before outright denial,
which is exactly what the AIS/maritime GNSS RFI monitoring paper in the
project library detects fleet-wide. A spoofing attack then has a baseline to
be measured against.

**`wind`** — model (`none` indoors). Why here: the air is the world's. It is
irrelevant to the ground cars and lidar, but the moment quadcopters fly, wind
sets hover power, control effort and trajectory error — and therefore mission
duration and the steadiness of any antenna an agent is pointing. Declaring
`none` in the lab is a statement, not an omission: the comparison scene
outdoors will declare something real.

The Setup tab's scene tree now shows all four, with undeclared ones greyed
`(not declared)`, so a scene author can see the full vocabulary.

## What should join the background (queued, each with its reason)

- **Terrain / occlusion masks** — the Map-builder / topography-import stubs.
  Terrain is the strongest propagation term outdoors (knife-edge losses dwarf
  the exponent) and drives both lidar returns and GNSS sky view.
- **Per-band conditions** — the background is currently one number per
  quantity at one implied band (2.4 GHz). A PACE fallback ladder (primary /
  alternate / contingency / emergency network) is only measurable if the
  scene can make 900 MHz behave differently from 5.8 GHz.
- **Atmospheric attenuation** — considered and deliberately EXCLUDED for now:
  below ~10 GHz over lab-to-field ranges it is noise on the noise. It earns a
  place only when the extreme-bandwidth (mmWave+) regimes from the jamming
  survey in the project library are modelled; the placeholder would otherwise
  invite fake precision.
- **Ambient EM transients** (motors, welders, lightning) — candidate for the
  spectrum block as stochastic events rather than a level; useful later for
  testing detector false-positive rates (a jamming detector that alarms on a
  lift motor is a finding).

## What must NOT creep in here

Jammers, spoofers, and anything with intent. The moment a condition has an
author it is an agent on a network (red side) or an injected attack, with a
position, a power budget, and detectability — the Contested tab's job. A
"jamming level" slider on the scene would erase exactly the distinction this
framework exists to measure.
