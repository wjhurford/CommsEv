# CommsEv — Communications Evaluator

[![tests](https://github.com/wjhurford/CommsEv/actions/workflows/tests.yml/badge.svg)](https://github.com/wjhurford/CommsEv/actions/workflows/tests.yml)
[![docker](https://github.com/wjhurford/CommsEv/actions/workflows/docker.yml/badge.svg)](https://github.com/wjhurford/CommsEv/pkgs/container/commsev)
[![licence](https://img.shields.io/badge/licence-Apache--2.0-blue.svg)](LICENSE)

A command-and-control resilience testbed for robot and drone fleets under
jamming and GNSS denial. One causal chain, kept whole:

> **RF physics → link state → command authority → agent behaviour →
> position knowledge → mission outcome**

Robotics simulators move robots and assume communications are free; network
simulators model the radio and do not move robots. CommsEv does both, so a
change in the spectrum reaches the mission score without anyone hand-wiring it.

![A jammer comes up on the command band; links drop, cars are held with no commander, the Network tab shows authority shrinking](docs/gifs/03-jamming.gif)

Built at Loughborough University (LUCAS Lab) by William Hurford, 2026,
supervised by Dr Chengyuan Liu. Formerly *Deadband*. Research prototype:
the model, the Console and the 561-test suite work; the API is not frozen.

## Run it

**In a browser, no Python** — needs [Docker Desktop](https://www.docker.com/products/docker-desktop/):

```bash
docker run --rm -p 5800:5800 ghcr.io/wjhurford/commsev     # prebuilt image
docker compose up        # or build from source; Windows: "Open in browser (Docker).bat"
```

then open <http://localhost:5800> and press F11. Building from source takes
about two minutes and runs the test suite as its last step.

**Natively** — Python 3.10+:

```bash
pip install -r requirements.txt
python3 console/app.py   # Windows: "Setup (run once).bat", then console\CommsEv Console.bat
```

**First run:** Setup tab → scene `maze` → blue fleet `8_roboracer_no_lidar` →
red fleet `custom_red` → add a few points → **Play**. Blue terminal:
`SETMISSION advance to P1 P2 P3`, `blue launch`. Red terminal:
`JAM jam1 band 2400 power 30`, `red launch`. Watch the links degrade. See `docs/TUTORIAL.md` for the full walk-through and
`COMMANDS.md` for the terminal grammar.

**Headless:**

```bash
python3 tests/test_all.py                          # 561 passed, 0 failed
python3 -m commsev validate default_run.yaml       # schema + provenance report
python3 tools/sweep.py experiments/penetration.yaml   # sweep → runs/sweep_*/results.csv
python3 tools/plot_results.py runs/sweep_*/results.csv
```

## Tour

Every GIF below is the real Console driven headless by `tools/make_gifs.py`
— regenerate them with one command after any change.

| | |
| --- | --- |
| ![compose a run](docs/gifs/01-setup.gif) **Compose.** Scene, blue fleet, red fleet, points, Play. Nothing else unlocks until the run exists. | ![command it](docs/gifs/02-command.gif) **Command.** `SETMISSION advance to P1 P2 P3`, `blue launch`. Typing a red order into the blue cell is refused with the reason. |
| ![jamming](docs/gifs/03-jamming.gif) **Jam.** `JAM jam1 band 2400 power 30`, `red launch`. Links degrade in every same-band receiver; agents whose decider cannot reach them are held. | ![drift](docs/gifs/04-drift.gif) **Deny GNSS.** `JAM jam1 band 1575.42`. IMU-only cars dead-reckon; belief and truth part company, and a reported arrival that is not true fails the mission. |
| ![wavefront](docs/gifs/05-wavefront.gif) **See whose signal wins.** The wavefront overlay says whose emission dominates where; the spectrum section shows how much. | ![results](docs/gifs/06-results.gif) **Sweep.** `authority × routing × jammer power` from one file; every numeric output plottable; double-click a row to replay that exact cell. |

## What it is for

Comparative claims under identical conditions — architecture A vs B, sensor
fit A vs B, doctrine A vs B. Never absolute figures: no dB or packet-delivery
ratio from CommsEv is a measurement. `SOURCES.md` says which constants are
grounded in literature and which are declared free.

The contribution is the **separation of authority from routing**. `authority`
(centralized / decentralized / hierarchical) is who *decides*; `routing`
(star / mesh / tiered) is how a message *travels*. Both consume one command
tree and either can be swept against the other. Headline so far: under one
jammer, mesh routing kept ~97% of a fleet commanded where tiered kept ~1%.

## Layout

```
  scene  +  fleet   ──composed by──►  mission
  world     agents                    the command
```

| Path | What |
| --- | --- |
| `scenes/` `fleets/` `missions/` `agents/` `experiments/` | the layer files (`kind:` says which) — `docs/vocabulary.md` |
| `tools/stub_telemetry.py` | the whole model: RF, jamming, links, authority, drift, dynamics, missions |
| `console/app.py` | the Console (PySide6) |
| `commsev/spec.py` | schema, validation, provenance report |
| `tools/sweep.py` `plot_results.py` `netcheck.py` | headless experiments and diagnostics |
| `ros2/src/commsev_ros/` | the same model as ROS 2 nodes + Console bridge (optional) |
| `tests/test_all.py` | 561 checks, no pytest |
| `docs/` | design and grounding — start at `docs/README.md` |
| `SOURCES.md` `COMMANDS.md` `CLAUDE.md` | constants ledger · terminal grammar · rules for AI coding sessions |

Not in the repo: `attic/` (gitignored) holds the reference-library PDFs,
supervisor handover notes and working images.

## The rules

Provenance on every physical quantity (`{value, unit, source}`; empty is
allowed, invented is not). Missions never import `rclpy`. `target(agent,
world)` is frozen. Authority ≠ routing. Missions are blue-only. Comparative
claims only. `CONTRIBUTING.md` has the how.

## Licence and citation

Apache-2.0 — `LICENSE`, `NOTICE`. Copyright 2026 William Hurford and
Loughborough University. Cite with `CITATION.cff` (GitHub's "Cite this
repository" button).
