# Tutorial — from a fresh clone to a first experiment

Approximately thirty minutes. Every step uses files shipped in the repository
and fleets that appear in the Console's dropdowns. `COMMANDS.md` has every terminal
command; `vocabulary.md` defines scene, fleet and mission.

## 1. Start it

Docker: `docker compose up` (Windows: `windows\Open in browser (Docker).bat`), then
<http://localhost:5800>, F11. Native: `pip install -r requirements.txt`,
`python3 console/app.py`. Both routes present the same window: sidebar on the
left, map in the centre, terminals and log below.

## 2. Compose a run

![compose](gifs/01-setup.gif)

Only the **Setup** tab is enabled; the others are enabled once a run exists.

1. **Scene** → `maze`. A 20 × 20 m room with eight interior walls of mixed
   material. Walls block radio, light and vehicles.
2. **Blue fleet** → `8_roboracer_no_lidar`. A spawn dialog asks where the
   fleet starts, in what shape and at what spacing. Accept the defaults and
   press **Spawn here**.
3. **Red fleet** → `custom_red`. One jammer, `jam1`, spawned the same way.
4. **Add point…** three times and drag the markers into a rough triangle
   away from the fleet. They are named P1, P2, P3 in order.
5. Press **▶**.

The run now exists: Setup is locked, the other tabs are enabled, and nothing
moves until a mission is issued and armed.

> A fleet specifies *who*; a scene specifies *where*; a mission specifies
> *what*. The `fleets/` dropdown lists only fleets that were built and saved
> in the Console (**+ Build custom fleet…**). The fixed fleets used by the
> test suite live in `tests/fixtures/fleets/` and do not appear in the
> dropdown.

## 3. Command it

![command](gifs/02-command.gif)

In the **blue** terminal:

```
SETMISSION advance to P1 P2 P3
blue launch
```

`SETMISSION` applies `missions/advance.yaml` — visit each named point once,
in order — to every agent the coordinator can reach; an agent it cannot reach
is skipped and reported. `launch` arms them. On the
map, the ground station `gcs` is the hub and the vehicles move off in
formation. (The mission and its goals may also be selected in Setup's
**Mission** section before Play, with **Issue mission**; arming is always
done at the terminal.)

Enter `red launch` in the **blue** terminal: it is refused, with the reason
stated. A cell may act only on its own side, which is what keeps an exercise
valid. Enter it in the **red** terminal instead, followed by:

```
JAM jam1 band 2400 power 30
red launch
```

![jamming](gifs/03-jamming.gif)

Open the **Network** tab. Links degrade or drop as the jammer's power lands
in every same-band receiver's SINR; agents whose decider can no longer reach
them stop accepting orders. The **Contested** tab shows the noise floor each
receiver experiences. `red halt` disarms the jammer.

## 4. See what an agent believes

![drift](gifs/04-drift.gif)

Open **System** and select a vehicle: it carries an IMU and no other sensor.
Select the scene `open_field` (a new run), spawn the same fleet, add points,
press Play, then
`SETMISSION advance to P1 P2 P3`, `blue launch`. In the red terminal:

```
JAM jam1 band 1575.42 power 30
red launch
```

That is GPS L1. Jamming 2400 MHz attacks *authority*; jamming 1575.42 MHz
attacks *position knowledge*. Each vehicle now dead-reckons on its IMU. The map
draws both the true position and the believed position; the separation grows
with time. A vehicle that reports an arrival it did not make has failed the
mission, and the log records it. This is the belief-versus-truth model
described in `gnss-drift-model.md`.

## 5. Run an experiment

![results](gifs/06-results.gif)

**Results** tab → **Run an experiment…** → `penetration`. This sweeps
`experiments/penetration.yaml`: a fleet advancing 190 m down
`corridor_200m` under a jammer, across authority × routing × jammer power.
The sweep runs in the background and rows appear as cells complete. Clicking
a row replays that cell on the map. Any numeric output can be plotted against
any axis.

The same sweep can be run headless, which is the appropriate route for
results intended for publication:

```bash
python3 tools/sweep.py experiments/penetration.yaml
python3 tools/plot_results.py runs/sweep_penetration_*/results.csv
```

Each sweep directory contains the experiment file that was run and a
`provenance.json`, so that the run can be reproduced exactly.

## 6. Extending the testbed

**A new experiment.** Copy `experiments/penetration.yaml`, change `name`,
pick `scene`, `blue_fleet`, `red_fleet`, `mission`, and edit `axes:` — any
of `authority`, `routing`, `jam_rel_db`, `spacing`, `formation`. Add
`seeds: [1, 2, 3]` as soon as any stochastic element (GNSS jamming, lidar) is
present. The new experiment appears in the Results dropdown.

**A new scene.** Copy `scenes/open_field.yaml`. Every physical value is
`{value, unit, source}`; leave `source` empty where no source exists — the
provenance report counts empty sources, which is the intended behaviour. Validate with
`python3 -m commsev validate scenes/yours.yaml`.

**A new agent type.** Copy `agents/roboracer.yaml`. An agent file describes
hardware — dimensions, performance, sensors, radios. It never contains a
network, a pose or an objective; those are decisions made in Setup. A file
placed in `agents/` is offered in the fleet builder.

**A new mission.** Copy `missions/advance.yaml`. Objectives only, no scene,
no fleet. A behaviour script (`missions/*.py`) takes plain numbers and
returns plain numbers and never imports `rclpy`.

**A new constant in the model.** Add it to `tools/stub_telemetry.py` as
`{value, unit, source}` and add a row to `SOURCES.md`. Then run
`python3 tests/test_all.py` and add a test that pins the new behaviour.
`CONTRIBUTING.md` sets out the remaining conventions.

## 7. Scope of valid claims

CommsEv does not produce absolute figures. A PDR of 0.62 or a link margin of
7 dB from this model is not a measurement; only the *difference* between two
runs under identical conditions is a result. `jamming-model-justification.md`
sets out the reasoning, and `SOURCES.md` identifies which constants are free
parameters. `REVIEW-2026-09-01.md` lists the known weaknesses; its first three
items should be read before building on the model.
