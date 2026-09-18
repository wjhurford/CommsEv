# CommsEv — Communications Evaluator

A command-and-control resilience testbed for robot and drone fleets under
jamming and GNSS denial. It carries a radio effect all the way through to
*who can still be commanded*, *what a cut-off vehicle does*, and *whether it
still knows where it is*.

> **RF physics → link state → command authority → agent behaviour →
> position knowledge → mission outcome**

Most tools break that chain in half. Robotics simulators (Gazebo, Isaac) move
robots and assume communications are free. Network simulators (ns-3, OMNeT++)
model the radio properly and do not move robots. CommsEv keeps the chain
whole so that a change in the spectrum reaches the mission score without
anybody hand-wiring it.

Built at Loughborough University (LUCAS Lab, AACME) by William Hurford during a
2026 Talent Match internship supervised by Dr Chengyuan Liu. Formerly called
*Deadband*; renamed September 2026.

**Status: research prototype under active development.** The model, the
Console and the test suite work; the API is not frozen.

---

## What it is for

CommsEv supports **comparative** claims under identical conditions:
architecture A vs B, sensor fit A vs B, doctrine A vs B. It does **not**
support absolute performance figures — no specific dB or packet-delivery ratio
from it should be quoted as measured truth. See `SOURCES.md` for what is
grounded in literature and what is a declared free parameter.

The contribution the project defends is the **separation of authority from
routing**. `authority` (centralized / decentralized / hierarchical) is who
*decides*. `routing` (star / mesh / tiered) is how a message *travels*. They
are usually conflated; here they are independent axes that both consume one
command tree, and either can be swept against the other. Headline result so
far: mesh routing sustained ~97% fleet coordination under a jammer that
reduced tiered routing to ~1%, identical scene and fleet.

## Quick start

Python 3.10 or newer. Nothing else is needed for the model and the tests.

```bash
git clone https://github.com/wjhurford/commsev.git
cd commsev
pip install -r requirements.txt          # PySide6, PyYAML, ruamel.yaml
python3 tests/test_all.py                # expect: 557 passed, 0 failed
python3 -m commsev validate default_run.yaml
```

Launch the Console:

| Platform | How |
| --- | --- |
| Windows | double-click `Setup (run once).bat`, then `console\CommsEv Console.bat` |
| Linux / macOS | `python3 console/app.py` |
| Any, in a browser | `docker compose up` (Windows: double-click `Open in browser (Docker).bat`), then <http://localhost:5800> — no Python on the host (see below) |

Then, in the Console: **Setup** tab → scene `maze` → blue fleet
`8_roboracer_no_lidar` → red fleet `custom_red` (one jammer) → add a few
points → **Play**. In the blue terminal type `SETMISSION advance` then
`blue launch`; in the red terminal type `red launch` and watch the links
degrade. The `fleets/` dropdown lists only fleets somebody built and saved in
the Console; the fixed fleets the tests use (`3_roboracer`, `red_jammer`, …)
live in `tests/fixtures/fleets/` and can be copied into `fleets/` if you want
them. `docs/HANDOVER-DEMO.md` walks the full ten-minute demo with what you
should see at each step, and `COMMANDS.md` is the complete terminal grammar.

### In a browser, with Docker

```bash
docker compose up          # first time builds the image (~5 min); after that, seconds
```

Open <http://localhost:5800>. That is the same Console, running inside the
container and drawn in your browser tab; `./runs`, `./fleets` and
`./formations` are mounted from the host so nothing you save is lost when
the container stops. It looks soft until the browser shows it at 1:1 — set
`DISPLAY_WIDTH` / `DISPLAY_HEIGHT` in `docker-compose.yml` to your monitor's
size and press F11. Port 5900 is plain VNC if you prefer a VNC client. The
build runs the full test suite as its last step, so an image that builds is
an image that works. The image is Ubuntu 22.04 — ROS 2 Humble's platform —
so the ROS bridge can join it later without changing the base.

> New in September 2026 and not yet built in CI. If `docker compose up`
> fails for you, please open an issue with the last twenty lines of output —
> a missing shared library in the `apt-get` line is the likely cause.

Headless, no GUI:

```bash
python3 tools/stub_telemetry.py                 # run default_run.yaml, stream JSON frames
python3 tools/netcheck.py                       # authority + measured topology tables
python3 tools/sweep.py experiments/penetration.yaml   # parameter sweep → runs/sweep_*/results.csv
python3 tools/plot_results.py runs/sweep_*/results.csv
```

## The three layers

```
  scene  +  fleet   ──composed by──►  mission
  world     agents                    the command
```

| Folder | Holds | `kind:` |
| --- | --- | --- |
| `scenes/` | the world only: arena, walls, radio background, GNSS | `scene` |
| `fleets/` | agents, sensors, radios, networks, authority and routing | `fleet` |
| `missions/` | objectives only — never a scene, never a fleet | `mission` |
| `agents/` | hardware you compose fleets from (a thing you could buy) | `agent` |
| `experiments/` | sweep definitions: which axes, which outputs | `experiment` |

A run is a scene plus a fleet (or two: blue and red), composed in the
Console's Setup tab or named in a run file such as `default_run.yaml`. A
mission is then *issued* to the running fleet from the terminal, gated by
command authority — an agent whose decider cannot reach it is not retasked.
`docs/vocabulary.md` is the authoritative definition of each layer.

## Repository layout

| Path | What |
| --- | --- |
| `tools/stub_telemetry.py` | the whole model: RF propagation, jamming, link state, authority, GNSS drift, vehicle dynamics, missions |
| `console/app.py` | the Console (PySide6 GUI): setup, map, terminals, network view, experiments |
| `Dockerfile`, `docker-compose.yml` | the Console in a browser tab, no host Python |
| `commsev/spec.py` | schema loading, validation and the provenance report |
| `tools/sweep.py`, `tools/plot_results.py`, `tools/netcheck.py` | headless experiment harness and diagnostics |
| `ros2/src/commsev_ros/` | the same model as ROS 2 nodes plus a bridge to the Console (optional; Windows users run it in WSL) |
| `tests/test_all.py` | 557 checks, no pytest dependency |
| `docs/` | design notes, research grounding, handover — start with `docs/README.md` |
| `SOURCES.md` | where every physical constant comes from, and which ones are free parameters |
| `COMMANDS.md` | the terminal command reference |
| `CLAUDE.md` | orientation for AI coding assistants — the rules below, in the form a coding session reads first |

## The rules

These are what keep the results honest. They are enforced by tests where a
test can enforce them, and by review where it cannot.

1. **Provenance.** Every physical quantity is `{value, unit, source}`. An
   empty source is allowed but counted and flagged. Never invent a number that
   looks researched.
2. **Missions never import `rclpy`.** Algorithms take plain numbers and return
   plain numbers; the ROS node is a wrapper. This is the sim-to-real argument.
3. **`target(agent, world)` is frozen.** New senses go into `world`, never
   into the signature.
4. **Authority ≠ topology.** `authority` and `routing` are independent fields.
5. **Missions are blue-only.** Red is jamming and interception, driven by
   `launch` / `halt` / `JAM`.
6. **Comparative claims only.**

## Contributing

See `CONTRIBUTING.md`. In one line: run the tests before and after, add a
constant to `SOURCES.md` when you add one to the code, and add a test that
would have caught the bug you just fixed.

## Licence

Apache-2.0 — see `LICENSE` and `NOTICE`. Copyright 2026 William Hurford and
Loughborough University. ArduPilot (GPL-3.0), where used, runs as a separate
process and is never linked or vendored.

## Citing

If CommsEv contributes to published work, please cite the repository:

```
Hurford, W. (2026). CommsEv: a command-and-control resilience testbed for
multi-agent fleets under jamming and GNSS denial. Loughborough University.
https://github.com/wjhurford/commsev
```
