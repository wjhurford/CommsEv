# Experiments — design for review (not built yet)

**Status: design for review. Nothing in this document is built.** It records a
proposal of 1 September 2026 for a **Sandbox / Experiment** switch in the Setup
tab, together with one significant amendment. Sandbox is the current mode:
terminals available, interactive exploration. Experiment removes the terminals,
runs a scripted sequence, accumulates results onto preset graphs and advances
automatically to the next run.

## The amendment: watching must not be the only way to obtain results

The proposal is right in spirit, and "no terminals in experiment mode" is
correct because operator interference destroys reproducibility. If every run is
watched in real time, however, a sweep of 5 powers x 3 architectures x 5 seeds
= 75 runs x ~60 s = **75 minutes of observation**. That is not a results
pipeline.

The design therefore specifies **one experiment definition, two ways to run
it.**

- **Headless batch (the results path).** No rendering and no sleeping between
  ticks; the run proceeds as fast as the CPU allows. 75 runs complete in
  seconds. Output is a CSV and the plots. This path generates the results
  chapter.
- **Watch mode (the confidence and demonstration path).** As proposed: real
  time, no terminals, automatic advance from run to run, curves accumulating
  live on the preset graphs. This path is for checking that the sweep does what
  is intended, and for demonstration.

Same experiment file, same metrics, same plots. Only the clock differs.

## An experiment is a file (`kind: experiment`)

Same discipline as scene/fleet/mission: declarative, diffable, re-runnable,
citable. Sketch:

```yaml
spec_version: 0.1
name: architecture_under_jamming
kind: experiment
description: >
  Does command structure survive jamming? Identical scene, fleet, mission and
  jammer; only the authority model changes.

scene: lab_box
fleet: 3_roboracer
red:   red_jammer
mission: test

sweep:                       # every combination is one CONDITION
  authority:        [centralized, decentralized, hierarchical]
  jammer_power_dbm: [0, 5, 10, 15, 20]
seeds: [1, 2, 3, 4, 5]       # each condition repeated per seed

sequence:                    # scripted; no operator, no terminals
  - {at: 0,  do: "SETMISSION test"}
  - {at: 1,  do: "blue launch"}
  - {at: 10, do: "red launch"}
  - {at: 40, do: "red halt"}
  - {at: 50, do: "end"}

metrics:                     # computed per frame, aggregated per run
  - commanded_fraction       # agents whose decider is reachable
  - links_up_fraction
  - distance_covered_m       # did the fleet actually do its job
  - position_error_mean_m    # belief vs truth
plots:
  - {x: jammer_power_dbm, y: commanded_fraction, series: authority}
```

## Metrics — the component that turns watching into results

Per frame, aggregated per run (mean, final, min, time-to-first-event):

- **commanded_fraction** — fraction of vehicles whose decider is reachable.
  This is the niche metric: it measures command-structure survival, which
  nothing else measures.
- **time_to_command_loss** — seconds from jammer on to the first agent losing
  its decider. A responsiveness figure.
- **links_up_fraction / mean packet-delivery ratio (PDR)** — the communications
  layer, for context.
- **distance_covered_m / mission_progress** — whether the fleet continued its
  task.
- **position_error_mean_m / final** — the GNSS-denial cost, in metres.
- **held_fraction** — the proportion of the fleet frozen on its failsafe.

## Six experiments runnable on the current maps, with no new modelling

All six require only the harness; the model already supports them.

1. **Jammer power sweep** (the baseline). `lab_box`, `3_roboracer`, mission
   `test`; power 0->20 dBm. Plot commanded_fraction against power. This gives
   the degradation curve and is the sanity check for everything else.
2. **Architecture comparison** (the niche experiment). Fixed jamming;
   `centralized` vs `decentralized` vs `hierarchical`. Tests whether
   distributed command genuinely retains more of the fleet. Expected: the
   centralised fleet collapses at the hub link; the decentralised fleet is
   barely affected.
3. **Jammer position sweep.** Move the jammer from arena edge to centre. Tests
   the observation that a star dies at its longest link first: the agent lost
   should track jammer proximity x link length.
4. **Routing vs authority** (star vs mesh at the same authority). The claim the
   repository has always made — a mesh survives the hub dying while centralised
   decision-making does not — becomes measurable.
5. **Sensor fit under GNSS denial.** `open_field`; `3_roboracer_no_lidar` vs
   `3_roboracer`, GNSS-band jammer. Plot final position_error. Tests whether
   sensors buy resilience (and, per the fix, that lidar does not help in an
   open field).
6. **Leader-loss doctrine.** `hierarchical`, fallback vs strand, under
   identical jamming. Directly tests the doctrine that was made declarable.

Experiments (2) and (1) pay back immediately.

## Seeds

The simulation contains randomness: the drift direction random-walks, and
sensor noise. A **seed** fixes that random sequence, so the same seed
reproduces a run exactly. Two uses:

- **Reproducibility** — a reported result can be re-run and checked.
- **Separating signal from noise** — run the same condition under several
  seeds. If mesh beats star under all five seeds, that is a result. If it wins
  under one seed and loses under another, that is noise, and reporting the
  single favourable run would misrepresent the outcome. Report mean and spread
  across seeds, never one run.

## Open question

Should an experiment be able to script the red side as well (a power ramp, a
jammer that moves)? The cost is small — the sequence already issues commands —
and it is the cheapest possible "intelligent adversary" before real reactive
jamming is built.

---

# Round 2 — decisions and findings (1 September 2026)

**Decision of 1 September 2026:** experiments 1-4 are in scope. Experiments may
script the red side; when they do, this must be evident in the run name and the
results file (the condition columns carry it, and the run id encodes it).
Claims stay comparative.

## The out-and-back design, assessed

Proposed on 1 September 2026: a 100 x 100 m grid; agents drive out to a
waypoint and back to the start, recording error at both points; GCS centre-left
(x=0), jammer centre-right (x=100).

**Strengths.** Returning to the start gives an unambiguous ground truth: the
starting position is known exactly, so the return error needs no
interpretation. Two measurements (at the far waypoint, and at home) show drift
accumulating with distance travelled. A 100 m grid also makes the RF numbers
meaningful, unlike the 8 m lab where everything is trivially in range.

**One refinement — run along the GCS->jammer axis, not across it.** If the
agents travel in +y at fixed x, their distance to a jammer at (100, 50) barely
changes (107 m -> 95 m -> 107 m): a weak gradient. Instead, start them near the
GCS (x ~ 5), drive to the far side (x ~ 95) and back, in lanes at
y = 40/50/60. Over one leg:

- distance to the GCS sweeps 5 m -> 95 m (the command link stretches), and
- distance to the jammer sweeps 90 m -> 5 m (the jamming closes in),

and both recover on the way home. One run gives a full gradient with a
symmetric control built in.

## Two findings that shape the design (measured from the model)

**1. The clean link holds across the whole grid.** Open field, 20 dBm, n=2.2:
PDR is 100% at 5 m and still 100% at 150 m. There is therefore no range
confound: any communications degradation observed is the jamming. This supports
the design.

**2. GNSS denial has no gradient at this scale, and that is a real effect.**
GNSS signals arrive at about -128 dBm, so even a 0 dBm jammer exceeds the
denial threshold out to kilometres. On a 100 m grid the entire field is
GNSS-denied the moment the jammer is armed, at any power.

This is not a modelling error; it is the actual asymmetry between the two
jammings, and it should be stated as a finding:

- **Comms jamming is local.** It is a signal-versus-jammer contest, so it
  degrades with geometry, and distance-to-jammer is the interesting variable.
- **GNSS jamming is theatre-wide** (corrected in Round 3 below: the range is
  horizon-limited, not unbounded). The victim signal is so weak that denial
  covers everything nearby regardless of power. Distance-to-jammer is not the
  interesting variable; **distance travelled while denied** is, because that is
  what sets the drift.

The out-and-back is therefore the right GNSS experiment — it measures drift
against distance travelled (200 m round trip ~ 8 m of error at 4%) — while the
x-axis traverse is the right comms experiment. Both are obtained in one run.

**Caveat:** the modelled GNSS denial radius (km) assumes free-space line of
sight with no horizon or terrain masking, so it is optimistic. Real GNSS jamming
is horizon-limited. Terrain masking is a queued refinement.

## Outlier drill-down (proposed 1 September 2026; inexpensive to implement)

Every run is fully determined by (experiment, condition, seed), so a point on a
results graph carries everything needed to reproduce that exact run. Selecting
a point offers "replay this run" (re-runs it deterministically in watch mode)
or "open in sandbox" (loads that condition's scene/fleet/mission and jammer
settings with terminals unlocked, for interactive exploration). Frames need not
be stored for all 75 runs; the recipe is stored and the run re-executed on
demand. Optionally a run can be flagged to keep its frames for instant
scrubbing.

## The graph catalogue

**Within one run (time series):**

- commanded_fraction vs time, with the jamming window shaded (the signature
  plot)
- links up / degraded / down, stacked, vs time
- position error (belief vs truth) per agent vs time
- experienced noise floor per agent vs time
- speed / distance covered vs time (failsafe holds appear as flat sections)

**Across runs (the results plots):**

- commanded_fraction vs jammer power, one series per architecture (the
  principal plot)
- time_to_command_loss vs jammer power
- final position error vs distance travelled, series per sensor fit
- commanded_fraction vs jammer position/distance
- mission outcome (waypoint error, return error) vs power, series per doctrine
- seed spread on any of the above (box or scatter, never a single run)
- 2-D heatmap: power x jammer-distance, colour = commanded_fraction

**Per-agent:** which agent lost command first (bar chart); shows the star dying
at its longest link.

---

# Round 3 — architecture everywhere, and formation as a variable

## Correction: GNSS jamming is not "theatre-wide"; it is horizon-limited

The Round 2 finding over-claimed. GNSS jamming has a finite range like any
other emission; it is simply much larger than the comms jamming range for the
same power, because the victim signal is ~80-90 dB weaker (GNSS arrives at
about -128 dBm; a comms link at -40 to -60 dBm). The earlier figures (146 km at
30 dBm) assumed free-space line of sight with no earth in the way, which is
wrong.

**Now modelled:** `radio_horizon_m(h1, h2) = 4120 * (sqrt(h1) + sqrt(h2))`,
the standard 4/3-earth radio horizon. Nothing is received past it, whatever the
power. Result:

| case | GNSS denial radius |
|---|---|
| ground jammer -> ground receiver (2 m antennas) | capped at **11.7 km** at any power |
| ground jammer -> drone at 100 m | **47 km** |
| ground jammer -> receiver at 500 m | **98 km** |

This is why GNSS jamming affects aviation over far wider areas than ground
users, and why the Baltic trial's ">3 km" for a ship-borne jammer is
comfortably inside the ground-to-ground horizon. The point that stands: on a
100 m test grid the whole field is denied uniformly — not because the range is
infinite, but because 100 m is small next to an 11 km radius.

## Architecture must be a sweep dimension in every experiment

A point raised in review, and accepted. Command structure is the niche, so
architecture should not be one experiment among four; it should be an axis on
all of them. Every experiment file gets `authority: [centralized,
decentralized, hierarchical]` in its sweep unless there is a reason not to.
This includes the out-and-back: the interesting question is not only how much
drift accumulates, but whether a distributed fleet holds together while a
centralised one is decapitated, over the same ground.

## Formation as a controllable variable — strong literature support

The question raised in review was whether holding a shape could improve
communications. It can, and it is a published anti-jamming technique:

- *A Critical Analysis of Spoofing and Jamming Approaches* [29] describes a
  **wireless relay network that mitigates jamming by optimising the flight
  path of a UAV relay** — "utilizing the mobility of the UAV to maximize the
  signal-to-interference-plus-noise ratio at the receiver". Formation geometry
  as an anti-jam measure is exactly this idea.
- *Drones* 2025 (9, 401) [9]: "optimizing the placement of relay UAVs to
  maximize end-to-end packet delivery ratio in multi-hop networks... dynamic
  relay positioning"; relays "significantly enhance drone communication...
  during beyond-line-of-sight operations".
- Chen et al. 2018 (channel-modelling survey): single-hop vs multi-hop UAV
  networks in "mesh or star topologies controlled by the ground station", and
  notes that **antenna orientation** measurably changes received power — a
  second geometric lever the model does not yet include.

Why it works in the present model: SINR falls with distance, so **splitting one
long link into two short hops is a large SINR gain**. A relay placed between
the GCS and a distant team converts a dying link into two healthy ones.
Formations worth testing:

- **cluster** — short strong internal links, poor reach, all agents inside the
  jammer's core together.
- **line / chain (relay)** — extends reach by multi-hop; each hop short and
  strong; fragile to losing a middle agent.
- **spread / wedge** — not all agents in the jammer's core at once; trades link
  strength for survivability.
- **adaptive relay** — an agent repositions to maximise the weakest link's
  SINR (the published technique).

**Proposed experiment 6: formation vs jamming.** Same fleet, same jammer, same
mission; sweep formation x architecture. Metrics: commanded_fraction and the
worst link's SINR. Hypothesis from the literature: a relay chain keeps command
at jamming levels where a cluster loses it.

## Correspondence of the architectures to the literature

- **Centralised vs decentralised** is textbook multi-agent control and the
  implementation matches the standard description: one coordinator decides
  (single point of failure; optimal while the link holds) versus every agent
  deciding for itself (loses information, never authority). Chen 2018's star
  vs mesh topologies map onto the routing layer.
- **Hierarchical** is a reasonable middle ground and is common in the
  SAGIN/swarm literature, but the specific fallback semantics — a squad whose
  leader drops reports up to the coordinator, or is stranded — are a design
  choice of this project. That choice is explicit and declarable
  (`leader_loss`), which is the appropriate way to handle it.
- **What should not be claimed:** that the three-way taxonomy is validated
  against a specific published model. It is a defensible engineering
  abstraction.
- **What is arguably a contribution:** separating decision authority from
  network routing. Most treatments conflate them. A mesh can carry
  centralised decision-making, and then the mesh survives a hub loss while the
  decision-making does not. That gap is measurable here and, as far as the
  library goes, is not something the surveyed work isolates.
