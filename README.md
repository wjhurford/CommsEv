# Deadband

A scenario-driven simulation framework for multi-agent robotic systems — ground
rovers and quadcopters — operating under degraded and contested radio conditions.

Built at Loughborough University (AACME / LUCAS Lab) as a research platform for
work on resilient and secure networked control.

**Status: early development.** Nothing here is finished.

## The idea

One YAML file describes a whole experiment — the arena, the agents in it, their
dimensions, sensors and radios, the networks they belong to, the radio
environment, and any attacks. A launcher turns that file into a running
simulation. Everything else is machinery for that.

Two ends of a range, one grammar:

- **Lab-faithful** — a bounded arena matching a real room, clean spectrum,
  external ground truth. Check a controller before you fly it.
- **Contested field** — open terrain, a noise floor raised by emitters nobody
  controls, GNSS you cannot rely on.

## The provenance rule

Every physical parameter carries its source:

```yaml
range_max: {value: 10.0, unit: m, source: "Hokuyo UST-10LX datasheet rev 2015"}
```

Sources are never auto-filled and never invented. Unsourced parameters are
counted and reported, not hidden. See `SOURCES.md`.

## Quick start

Nothing but Python is needed yet.

```bash
pip install pyyaml
python3 -m deadband validate default_run.yaml
python3 tools/stub_telemetry.py
```

`stub_telemetry.py` publishes correctly-shaped fake telemetry so the front end
can be developed with no simulator installed.

## Layout

| Path | What it is |
| --- | --- |
| `scenes/` | Worlds: arena, radio background, named points. |
| `fleets/` | Agents and their wiring: bodies, sensors, radios, networks. |
| `missions/` | Commands: a scene + a fleet + tasking. The user-facing surface. |
| `deadband/` | The framework package. |
| `tools/` | Standalone utilities with no framework dependency. |
| `console/` | The GUI. Not started. |
| `SOURCES.md` | Where every number comes from. |

## Licence

Apache-2.0. ArduPilot (GPL-3.0) is run as a separate process and is never
linked or vendored.
