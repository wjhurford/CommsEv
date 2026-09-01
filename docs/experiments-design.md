# Experiments — design for review (not built yet)

Will's idea (1 Sep): a **Sandbox / Experiment** switch in Setup. Sandbox is what
we have — terminals, poke at it, explore. Experiment removes the terminals, runs
a scripted sequence, and accumulates results onto preset graphs, auto-advancing
to the next run. This doc is that idea, with my one significant amendment.

## The amendment: watching must not be the only way to get results

The design is right in spirit — and "no terminals in experiment mode" is exactly
right, because operator interference destroys reproducibility. But if runs are
watched in real time, a sweep of 5 powers x 3 architectures x 5 seeds = 75 runs
x ~60 s = **75 minutes of staring**. That is not a results pipeline.

So: **one experiment definition, two ways to run it.**

- **Headless batch (the results path).** No rendering, no sleeping between
  ticks - runs as fast as the CPU allows. 75 runs in seconds. Produces a CSV
  and the plots. This is what generates the results chapter.
- **Watch mode (the confidence + demo path).** Exactly Will's description:
  real time, no terminals, auto-advance run to run, curves accumulating live on
  the preset graphs. This is for sanity-checking that the sweep does what you
  think, and for showing someone.

Same experiment file, same metrics, same plots. Only the clock differs.

## An experiment is a FILE (kind: experiment)

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

## Metrics — the missing piece that turns watching into results

Per frame, aggregated per run (mean, final, min, time-to-first-event):

- **commanded_fraction** — fraction of vehicles whose decider is reachable.
  *The niche metric.* This is command-structure survival, and nothing else
  measures it.
- **time_to_command_loss** — seconds from jammer on to the first agent losing
  its decider. A responsiveness number.
- **links_up_fraction / mean PDR** — the comms layer, for context.
- **distance_covered_m / mission_progress** — did the fleet keep doing its job?
- **position_error_mean_m / final** — the GNSS-denial cost, in metres.
- **held_fraction** — how much of the fleet froze on its failsafe.

## Six experiments runnable on TODAY'S maps, with no new modelling

All of these need only the harness - the model already supports them.

1. **Jammer power sweep** (the baseline). lab_box, 3_roboracer, mission test;
   power 0->20 dBm. Plot commanded_fraction vs power. Gives the degradation
   curve, and it is the sanity check for everything else.
2. **Architecture comparison** (THE niche experiment). Fixed jamming;
   centralized vs decentralized vs hierarchical. Does distributed command
   genuinely retain more of the fleet? Expected: centralized collapses at the
   hub link, decentralized barely notices.
3. **Jammer position sweep.** Move the jammer from arena edge to centre. Tests
   the "a star dies at its longest link first" observation - which agent is
   lost should track jammer proximity x link length.
4. **Routing vs authority** (star vs mesh at the same authority). The claim the
   repo has always made - a mesh survives the hub dying while centralized
   decision-making does not. Now measurable.
5. **Sensor fit under GNSS denial.** open_field; 3_roboracer_no_lidar vs
   3_roboracer, GNSS-band jammer. Plot final position_error. Tests whether
   sensors buy resilience (and, per the fix, that lidar does NOT help in an
   open field).
6. **Leader-loss doctrine.** hierarchical, fallback vs strand, under identical
   jamming. Directly tests the doctrine we made declarable.

(2) and (1) are the ones that pay back immediately.

## Seeds

The sim contains randomness - the drift direction random-walks, sensor noise.
A **seed** fixes that random sequence, so the same seed reproduces a run
exactly. Two uses:

- **Reproducibility** - a reported result can be re-run and checked.
- **Telling signal from noise** - run the SAME condition under several seeds.
  If mesh beats star under all five seeds, it is a result. If it wins under one
  and loses under another, it is noise, and reporting the single good run would
  be dishonest. Report mean and spread across seeds, never one run.

## Open question for Will

Should an experiment be able to script the RED side too (a power ramp, a jammer
that moves)? It costs little - the sequence already issues commands - and it is
the cheapest possible "intelligent adversary" before real reactive jamming.

---

# Round 2 — decisions and findings (Will, 1 Sep)

**Agreed:** experiments 1-4 are in. Experiments MAY script the red side, and
when they do it must be obvious in the run name and the results file (the
condition columns carry it, and the run id encodes it). Claims stay comparative.

## Will's out-and-back design, assessed

Proposed: 100 x 100 m grid, agents out to a waypoint and back to start,
recording error at BOTH; GCS centre-left (x=0), jammer centre-right (x=100).

**What is excellent about it:** returning to the start gives an unambiguous
ground truth - you know exactly where they began, so the return error needs no
interpretation. Two measurements (at the far waypoint, and home) show drift
accumulating with distance travelled. And 100 m makes the RF numbers mean
something, unlike the 8 m lab where everything is trivially in range.

**One refinement - run along the GCS->jammer axis, not across it.** If the
agents travel in +y at fixed x, their distance to a jammer at (100, 50) barely
changes (107 m -> 95 m -> 107 m): a weak gradient. Start them near the GCS
(x ~ 5) and drive to the far side (x ~ 95) and back, in lanes at y = 40/50/60.
Then over one leg:
  - distance to the GCS sweeps 5 m -> 95 m (the command link stretches), and
  - distance to the jammer sweeps 90 m -> 5 m (the jamming closes in),
and both recover on the way home. One run, a full gradient, and a symmetric
control built in.

## Two findings that shape it (measured from the model)

**1. The clean link holds across the whole grid.** Open field, 20 dBm, n=2.2:
PDR is 100% at 5 m and still 100% at 150 m. So there is no range confound -
any comms degradation you see IS the jamming. Good news for the design.

**2. GNSS denial has no gradient at this scale - and that is REAL.** GNSS
signals arrive at about -128 dBm, so even a 0 dBm jammer exceeds the denial
threshold out to kilometres. On a 100 m grid the ENTIRE field is GNSS-denied
the moment the jammer is armed, at any power.

This is not a modelling error, it is the actual asymmetry between the two
jammings, and it should be stated as a finding:

- **Comms jamming is LOCAL** - it is a signal-versus-jammer contest, so it
  degrades with geometry, and distance-to-jammer is the interesting variable.
- **GNSS jamming is THEATRE-WIDE** - the victim signal is so weak that denial
  covers everything nearby regardless of power. Distance-to-jammer is NOT the
  interesting variable; **distance travelled while denied** is, because that is
  what sets the drift.

So the out-and-back is exactly the right GNSS experiment - it measures drift
against distance travelled (200 m round trip ~ 8 m of error at 4%) - while the
x-axis traverse is the right COMMS experiment. Both in one run.

**Honest caveat:** our GNSS denial radius (km) assumes free-space line of
sight with no horizon or terrain masking, so it is optimistic. Real GNSS
jamming is horizon-limited. Terrain masking is a queued refinement.

## Outlier drill-down (Will's idea - yes, and it is cheap)

Every run is fully determined by (experiment, condition, seed). So a point on
a results graph carries everything needed to reproduce that exact run. Click a
point -> "replay this run" (re-runs it deterministically in watch mode) or
"open in sandbox" (loads that condition's scene/fleet/mission and jammer
settings, terminals unlocked, so you can poke at it). No need to store frames
for all 75 runs; store the recipe, re-run on demand. Optionally flag a run to
keep its frames for instant scrubbing.

## The graph catalogue

**Within one run (time series):**
- commanded_fraction vs time, with the jamming window shaded  <- the signature
- links up / degraded / down, stacked, vs time
- position error (belief vs truth) per agent vs time
- experienced noise floor per agent vs time
- speed / distance covered vs time (shows failsafe holds as flat sections)

**Across runs (the results plots):**
- commanded_fraction vs jammer power, one series per architecture  <- THE plot
- time_to_command_loss vs jammer power
- final position error vs distance travelled, series per sensor fit
- commanded_fraction vs jammer position/distance
- mission outcome (waypoint error, return error) vs power, series per doctrine
- seed spread on any of the above (box or scatter, never a single run)
- 2-D heatmap: power x jammer-distance, colour = commanded_fraction

**Per-agent:** which agent lost command first (bar) - shows the star dying at
its longest link.

---

# Round 3 — architecture everywhere, and formation as a variable

## Correction: GNSS jamming is NOT "theatre-wide", it is horizon-limited

I over-claimed. GNSS jamming has a range like anything else - it is simply
much LARGER than comms jamming range for the same power, because the victim
signal is ~80-90 dB weaker (GNSS arrives at about -128 dBm; a comms link at
-40 to -60 dBm). The earlier figures (146 km at 30 dBm) were free-space
line-of-sight with no earth in the way, which is wrong.

**Now modelled:** `radio_horizon_m(h1, h2) = 4120 * (sqrt(h1) + sqrt(h2))`,
the standard 4/3-earth radio horizon. Nothing is received past it, whatever
the power. Result:

| case | GNSS denial radius |
|---|---|
| ground jammer -> ground receiver (2 m antennas) | capped at **11.7 km** at any power |
| ground jammer -> drone at 100 m | **47 km** |
| ground jammer -> receiver at 500 m | **98 km** |

That is why GNSS jamming affects aviation over far wider areas than ground
users, and why the Baltic trial's ">3 km" for a ship-borne jammer is
comfortably inside the ground-to-ground horizon. The point that stands: on a
100 m test grid the whole field is denied uniformly - not because the range is
infinite, but because 100 m is tiny next to an 11 km radius.

## Architecture must be a sweep dimension in EVERY experiment

Will's point, and it is right. Command structure is the niche, so architecture
should not be one experiment among four - it should be an axis on all of them.
Every experiment file gets `authority: [centralized, decentralized,
hierarchical]` in its sweep unless there is a reason not to. The out-and-back
included: the interesting question is not just "how much drift" but "does a
distributed fleet hold together while a centralized one is decapitated, over
the same ground".

## Formation as a controllable variable - strong literature support

Will asked whether holding a shape could improve communications. It can, and
it is a published anti-jamming technique:

- *A Critical Analysis of Spoofing and Jamming Approaches* [29] describes a
  **wireless relay network that mitigates jamming by optimising the FLIGHT
  PATH of a UAV relay** - "utilizing the mobility of the UAV to maximize the
  signal-to-interference-plus-noise ratio at the receiver". Formation geometry
  as an anti-jam measure, exactly the idea.
- *Drones* 2025 (9, 401) [9]: "optimizing the placement of relay UAVs to
  maximize end-to-end packet delivery ratio in multi-hop networks... dynamic
  relay positioning"; relays "significantly enhance drone communication...
  during beyond-line-of-sight operations".
- Chen et al. 2018 (channel-modelling survey): single-hop vs multi-hop UAV
  networks in "mesh or star topologies controlled by the ground station", and
  notes **antenna orientation** measurably changes received power - a second
  geometric lever we do not model yet.

Why it works in our model already: SINR falls with distance, so **splitting
one long link into two short hops is a large SINR gain**. A relay placed
between the GCS and a distant team converts a dying link into two healthy
ones. Formations worth testing:

- **cluster** - short strong internal links, poor reach, all agents inside the
  jammer's core together.
- **line / chain (relay)** - extends reach by multi-hop; each hop short and
  strong; fragile to losing a middle agent.
- **spread / wedge** - not all agents in the jammer's core at once; trades
  link strength for survivability.
- **adaptive relay** - an agent repositions to maximise the weakest link's
  SINR (the published technique).

**Proposed experiment 6: formation vs jamming.** Same fleet, same jammer,
same mission; sweep formation x architecture. Metric: commanded_fraction and
the worst link's SINR. Hypothesis from the literature: a relay chain keeps
command at jamming levels where a cluster loses it.

## Do our architectures match the literature? Honest answer

- **Centralized vs decentralized** is textbook multi-agent control and our
  implementation matches the standard description: one coordinator decides
  (single point of failure; optimal while the link holds) versus every agent
  decides for itself (loses information, never authority). Chen 2018's star vs
  mesh topologies map onto our routing layer.
- **Hierarchical** is a reasonable middle ground and is common in the
  SAGIN/swarm literature, but the specific fallback semantics - a squad whose
  leader drops reports up to the coordinator, or is stranded - are OUR design
  choice. We made that choice explicit and declarable (`leader_loss`), which
  is the honest way to handle it.
- **What we should not claim:** that our three-way taxonomy is validated
  against a specific published model. It is a defensible engineering
  abstraction.
- **What is arguably a contribution:** separating DECISION AUTHORITY from
  NETWORK TOPOLOGY. Most treatments conflate them. A mesh can carry
  centralized decision-making, and then the mesh survives a hub loss while the
  decision-making does not. That gap is measurable here and is not, as far as
  the library goes, something the surveyed work isolates.
