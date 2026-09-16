# CommsEv Console

The application. One file, `app.py`, PySide6.

## Run it

| Platform | How |
| --- | --- |
| Windows | `Setup (run once).bat` at the repo root installs the dependencies into your **Windows** Python; then double-click **CommsEv Console.bat** here. `Debug Console.bat` does the same but keeps the terminal open so a crash is readable. |
| Linux / macOS | `pip install -r requirements.txt` at the repo root, then `python3 console/app.py`. |

A crash is also written to `console/last_error.log` and shown in a dialog.

## What it is

The Console is a client over three things: the layer files (`scenes/`,
`fleets/`, `missions/`), the simulation model in `tools/stub_telemetry.py`,
which it runs as a background process and animates, and — optionally — the ROS 2
bridge in `ros2/`, which on Windows it starts inside WSL.

The left-hand sidebar, in the order a run happens:

- **Setup** — choose a scene, a blue fleet (with a spawn dialog for position,
  formation and spacing), a red fleet, add named points. Every other tab is
  greyed out until the run exists.
- **System** — what each side is: per-agent sensors, radios, and an
  algorithms slot.
- **Mission** — what the fleet is doing: objectives, live retasking.
- **Network** — command authority and measured topology, live: who decides for
  whom, declared routing against measured shape, active against spare links.
- **Comms / Contested** — every emitter on both sides, every link and its
  state; the scene's radio baseline, jammers, and the noise floor each
  receiver actually experiences.
- **Results** — run a sweep from `experiments/*.yaml` in the background, plot
  any numeric output, click a results row to replay that configuration.

The map (TOP / FRONT / ISO views; scroll to zoom, drag to pan) fills the
centre. Below it sit the **Terminals** — three cells, **white / blue / red**;
a cell may only act on its own side and a refusal is printed with its reason
(`COMMANDS.md` is the grammar) — and the **Output** pane with the log and the
publications table.

Unsourced parameters are shown in amber with a running count in the status
bar. Saving a file goes through `ruamel.yaml` so that a one-field edit is a
one-line diff.

## Editing it

`app.py` is large (about 11,000 lines) and deliberately flat. The model is
**not** in here: anything about radio, jamming, authority, drift or motion
lives in `tools/stub_telemetry.py`, and the Console only draws what that
process streams. If a number on screen looks wrong, look there first.

The Console tests live alongside everything else in `tests/test_all.py` and
run headless (`QT_QPA_PLATFORM=offscreen` is set for you).
