# CommsEv (Communications Evaluator) — orientation for a coding session

Read this first. It is the shortest complete picture of what this is, what the
rules are, and where things live.

## What it is

A **command-and-control resilience testbed** for robot/drone fleets under
jamming. It carries a radio effect all the way through to *who can still be
commanded*, *what a cut-off vehicle does*, and *whether it still knows where it
is*. It is **not** an RF fidelity simulator — see "The scope line" below.

## The scope line (protects every claim)

Supports **comparative** claims: architecture A vs B, sensor fit A vs B,
doctrine A vs B, under identical conditions. Does **not** support absolute
performance figures. Never quote a specific dB or PDR as measured truth.

## The three layers (docs/vocabulary.md)

    scene  +  fleet   ──composed by──►  mission
    world     agents                    the command

- `scenes/` — world only: arena, radio background, named points. `kind: scene`
- `fleets/` — agents, sensors, radios, networks, authority/routing. `kind: fleet`
- `missions/` — objectives only, NO scene and NO fleet. `kind: mission`
- `default_run.yaml` — headless default (lab scene + lab fleet, untasked)

A run is composed in the Console's Setup tab, or by a file naming
`scene:` + `fleet:` (or `fleets: [blue, red]`).

## Non-negotiable rules

1. **Provenance.** Every physical quantity is `{value, unit, source}`. An empty
   source is allowed but counted and flagged. **Never invent a number that
   looks researched.** Free parameters must be declared as free (SOURCES.md).
2. **Missions never import `rclpy`.** Algorithms take plain numbers and return
   plain numbers; the ROS node is a wrapper. This is the sim-to-real argument.
3. **`target(agent, world)` is frozen.** Add new senses to `world`, never to
   the signature.
4. **Authority ≠ topology.** `authority` (who decides) and `routing` (how
   packets travel) are independent fields. Keeping them separate is the
   project's main contribution.
5. **Missions are blue-only.** Red is jamming, driven by launch/halt/JAM.
6. **Comparative claims only.** See the scope line.

## Where things are

| Path | What |
| --- | --- |
| `tools/stub_telemetry.py` | the whole model: RF, jamming, authority, drift, dynamics |
| `console/app.py` | the GUI (PySide6) |
| `commsev/spec.py` | schema loading + validation + provenance report |
| `tools/sweep.py` | headless experiment harness; `tools/plot_results.py` plots it |
| `ros2/src/commsev_ros/` | the same model as ROS 2 nodes + Console bridge (optional) |
| `tests/test_all.py` | 561 checks, `python3 tests/test_all.py`, no pytest |
| `docs/` | see docs/README.md; vocabulary.md and ROADMAP.md first |
| `SOURCES.md` | every constant and its status; `COMMANDS.md` the terminal grammar |

Naming: the project is **CommsEv** in prose, `commsev` in code (package,
CLI `python3 -m commsev`, ROS package `commsev_ros`, env var `COMMSEV_ROOT`).
It was called Deadband until September 2026; do not reintroduce that name.

## The command grammar (terminal / retask spool)

    SETMISSION <name>        apply missions/<name>.yaml, gated by authority
    <scope> launch | halt    arm/disarm a network or agent
    REOBJECTIVE <agent> ...  retask one agent live
    JAM <id> power|band <v>  tune a running jammer

Console cells are scoped: **blue** commands blue, **red** commands red
(JAM only), **white** is the umpire and does anything.

## Before you change the model

- Run `python3 tests/test_all.py` first and last. 561 should pass, 0 fail.
- If you add a constant, add it to `SOURCES.md` with its status.
- If you change behaviour, add a test that would have caught the old bug.
- Known open issues are in `docs/REVIEW-2026-09-01.md` — read the top three
  before touching the model.
