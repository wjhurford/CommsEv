# Tutorial — from clone to your own experiment

Thirty minutes. Everything here uses files that ship in the repo and fleets
that appear in the Console's dropdowns. `COMMANDS.md` has every terminal
command; `vocabulary.md` defines scene, fleet and mission.

## 1. Start it

Docker: `docker compose up` (Windows: `Open in browser (Docker).bat`), then
<http://localhost:5800>, F11. Native: `pip install -r requirements.txt`,
`python3 console/app.py`. Either way you get the same window: sidebar on the
left, map in the centre, three terminals and the log below.

## 2. Compose a run

![compose](gifs/01-setup.gif)

Only the **Setup** tab is live; the rest unlock when a run exists.

1. **Scene** → `maze`. A 20 × 20 m room with eight interior walls of mixed
   material. Walls block radio, light and vehicles.
2. **Blue fleet** → `8_roboracer_no_lidar`. A spawn dialog asks where the
   fleet starts, in what shape and at what spacing. Accept the defaults and
   press **Spawn here**.
3. **Red fleet** → `custom_red`. One jammer, `jam1`, spawned the same way.
4. **Add point…** three times and drag the markers into a rough triangle
   away from the fleet. They are named P1, P2, P3 in order.
5. Press **▶**.

The run exists. Setup locks, the other tabs light up, nothing moves yet.

> A fleet says *who*; a scene says *where*; a mission says *what*. The
> `fleets/` dropdown lists only fleets somebody built and saved in the
> Console (**+ Build custom fleet…**). The fixed fleets the tests use live in
> `tests/fixtures/fleets/` and never appear in the dropdown.

## 3. Command it

![command](gifs/02-command.gif)

In the **blue** terminal:

```
SETMISSION advance to P1 P2 P3
blue launch
```

`SETMISSION` applies `missions/advance.yaml` — visit each named point once,
in order — to every agent the coordinator can reach; an agent it cannot reach
is skipped and reported. `launch` arms them. Watch the map: the ground
station `gcs` is the hub; the cars move off in formation. (You can also pick
the mission and its goals in Setup's **Mission** section before Play and
press **Issue mission**; arming is always done at the terminal.)

Now try typing `red launch` into the **blue** terminal. It is refused with a
reason. A cell may only act on its own side; that is what keeps an exercise
honest. Type it into the **red** terminal instead, then:

```
JAM jam1 band 2400 power 30
red launch
```

![jamming](gifs/03-jamming.gif)

Open the **Network** tab. Links degrade or drop as the jammer's power lands
in every same-band receiver's SINR; agents whose decider can no longer reach
them stop taking orders. The **Contested** tab shows the noise floor each
receiver actually experiences. `red halt` switches the jammer off again.

## 4. See what an agent believes

![drift](gifs/04-drift.gif)

Open **System**, pick a car, note it carries an IMU and nothing else. Switch
the scene to `open_field` (a new run), spawn the same fleet, add points, play,
`SETMISSION advance to P1 P2 P3`, `blue launch`. In the red terminal:

```
JAM jam1 band 1575.42 power 30
red launch
```

That is GPS L1. Jamming 2400 MHz attacks *authority*; jamming 1575.42 MHz
attacks *position knowledge*. Each car now dead-reckons on its IMU. The map draws
where the car *is* and where it *thinks* it is; the gap grows with time. A
car that reports an arrival it did not make has failed the mission, and the
log says so. This is the belief-vs-truth model — `gnss-drift-model.md`.

## 5. Run an experiment

![results](gifs/06-results.gif)

**Results** tab → **Run an experiment…** → `penetration`. This sweeps
`experiments/penetration.yaml`: a fleet advancing 190 m down
`corridor_200m` under a jammer, across authority × routing × jammer power.
It runs in the background; rows appear as cells finish. Click a row to
replay that cell on the map. Any numeric output can be plotted against any
axis.

The same sweep headless, which is how results in a paper should be made:

```bash
python3 tools/sweep.py experiments/penetration.yaml
python3 tools/plot_results.py runs/sweep_penetration_*/results.csv
```

Each sweep folder carries the experiment file it ran and a `provenance.json`
so the run can be reproduced exactly.

## 6. Make it yours

**A new experiment.** Copy `experiments/penetration.yaml`, change `name`,
pick `scene`, `blue_fleet`, `red_fleet`, `mission`, and edit `axes:` — any
of `authority`, `routing`, `jam_rel_db`, `spacing`, `formation`. Add
`seeds: [1, 2, 3]` the moment anything stochastic (GNSS jamming, lidar) is in
play. It appears in the Results dropdown.

**A new scene.** Copy `scenes/open_field.yaml`. Every physical value is
`{value, unit, source}`; leave `source` empty if you do not have one — the
provenance report counts it, and that is the honest state. Validate with
`python3 -m commsev validate scenes/yours.yaml`.

**A new agent type.** Copy `agents/roboracer.yaml`. An agent file is
hardware you could buy — dimensions, performance, sensors, radios. It never
contains a network, a pose or an objective; those are decisions made in
Setup. Drop the file in `agents/` and it is offered in the fleet builder.

**A new mission.** Copy `missions/advance.yaml`. Objectives only, no scene,
no fleet. A behaviour script (`missions/*.py`) takes plain numbers and
returns plain numbers and never imports `rclpy`.

**A new constant in the model.** Add it to `tools/stub_telemetry.py` as
`{value, unit, source}` and add a row to `SOURCES.md`. Then run
`python3 tests/test_all.py` and add the test that would have caught the bug
you are about to fix. `CONTRIBUTING.md` has the rest.

## 7. What CommsEv will not tell you

Absolute numbers. A PDR of 0.62 or a link margin of 7 dB from this model is
not a measurement of anything; only the *difference* between two runs under
identical conditions is a result. `jamming-model-justification.md` argues
why, and `SOURCES.md` shows which constants are free parameters.
`REVIEW-2026-09-01.md` lists what is known to be weak — read its top three
items before you build on the model.
