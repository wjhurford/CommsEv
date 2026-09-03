# DEADBAND — handover brief

*Paste this into a new chat to discuss thesis framing, career positioning, or commercial angles. It is a summary, not the working context.*

---

## What it is

**DEADBAND** is a simulation framework for multi-agent robotic systems (ground rovers now, drones later) operating under degraded and contested radio conditions. Built at Loughborough (AACME / LUCAS Lab) as a research assistant project.

It models a causal chain that most tools break in half:

> **RF physics → link state → command authority → agent behaviour → position knowledge → mission outcome**

Robotics simulators (Gazebo, Isaac) move robots and assume communications are free. Network simulators (ns-3, OMNeT++) model the radio properly and don't move robots. DEADBAND sits in the gap: a jammed link becomes a lost command becomes a vehicle that stops — emergently, not scripted.

## The central claim

**Command authority and network routing are independent axes, and both matter.**

- **Authority** — *who decides*: centralized / decentralized / hierarchical
- **Routing** — *how packets travel*: star / mesh / tiered

A mesh network can carry centralized decision-making; the mesh then survives a hub loss while the decision-making does not. Measured result so far: under an identical jammer, a centralized fleet keeps **97%** of its vehicles commanded on mesh routing and **1%** on tiered. Same authority, same jammer — the difference is entirely how packets travel.

Related finding: declaring a *hierarchy* over *star* routing silently degenerates into centralized command, because a star carries no peer links, so every squad member reaches its leader via the coordinator it was supposed to be less dependent on.

## Architecture

Three layers, deliberately separated: **scene** (the world) + **fleet** (the agents) → composed by a **mission** (the command). A fourth layer, **experiment**, declares what varies across runs.

A hard line runs through the whole design: **hardware is a file, everything else is a decision.** `agents/` holds things you could buy — dimensions, sensors, radios. Authority, routing, doctrine, spawn positions and objectives are chosen in the Console at run time and recorded with the results. Nothing is hand-edited.

Every quantity carries `{value, unit, source}`. Unsourced ones are counted and flagged rather than invented — currently ~35 of 57 sourced, with the gaps listed explicitly.

## Doctrine, sourced

Link-loss behaviour is a declared, per-agent doctrine rather than a hardcoded rule:

- **hold** — freeze until the link returns (what most autopilot failsafes do)
- **intent** — keep executing the assigned objective from the picture already held

`intent` is NATO **mission command**, cited to AJP-3 Edition D: *"centralized planning that includes provision of clear guidance and intent combined with decentralized execution based on mission-type orders"* (3.8), and *"subordinates should decide and act within the scope of the commander's intent"* (3.11).

It is not a free win. An intent vehicle steers from its own drifted belief and from position reports that have gone stale, so it keeps working *and* keeps getting more wrong. Which doctrine wins depends on which spectrum is attacked.

## The intended headline result

A **dimensionless, scale-free** measure of how much anti-jam margin a topology buys.

In a corridor, a star holds one long link to the ground station; a mesh hops along a chain of short ones. Path loss goes as `10·n·log₁₀(d)`, so the predicted gain is:

> **G = 10 · n · log₁₀(N)** — N = number of hops

The corridor length isn't in it. Neither is transmit power or frequency. For a three-hop chain at n = 2.2: **≈ 10.5 dB**.

The comparison that makes it land: published work measures **FHSS at about 10 dB** of anti-jam gain. So the claim becomes —

> *Choosing mesh routing over star buys roughly the same anti-jam margin as frequency hopping, with no extra hardware, no synchronisation and no spectrum.*

That puts a **topology** decision on the same measuring stick as a **physical-layer** technique.

## State

- ~183 tests passing; PySide6 Console with live map, network view, spectrum tab, blue/red/white cell terminals
- Headless parallel sweep: 450 runs of 60 simulated seconds in 65 seconds on two cores
- Deterministic — every run is a pure function of (config, seed), so any result replays exactly
- Two significant self-caught bugs: missions were reading ground truth instead of the agents' own beliefs; the routing axis was being silently ignored by the authority calculation

## Known weaknesses (stated, not hidden)

- Contention counts agents rather than traffic, so extra mesh links are currently free — the predicted gain is an **upper bound** until fixed
- Jammer geometry is not yet swept, so topology findings are conditional on one emitter placement
- Mission outcome is not yet a first-class metric
- Decentralized agents don't actually negotiate, so that configuration is flattered
- Hierarchy has no built-in redundancy, which AJP-3 §3.29 says real C2 requires

## Context for the conversation

Will: BEng Aeronautical (Loughborough, first-class), starting an **MSc in Aerial Robotics at Bristol** in 2026/27, UK Security Clearance, targeting a 2027 start in UK defence R&D / counter-UAS / autonomous systems. In contact with Anduril UK about a master's thesis collaboration with their software team, plus a graduate route.

DEADBAND is the strongest technical asset in that story — but the open question is whether its value lies in **the tool**, **the result**, or **the method**, and how to position it accordingly.
