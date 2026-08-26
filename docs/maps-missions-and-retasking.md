# Maps, missions, and retasking

This is the split that stops a run being welded to one arena. Read it once and
the three folders — `maps/`, `missions/`, `scenarios/` — stop being confusing.

---

## The idea in one line

A **map** is *where*. A **mission** is *what*. You combine them at run time, so
one mission runs on any map, and one map hosts any mission.

Before this split, a single file in `scenarios/` held both, welded together:
"car3 follows the right wall **of this 8×8 box**". The behaviour and the place
could not be separated, so a mission could not be reused and a map could not be
retasked. Now they are two files.

---

## A map — the world, with no tasking in it

`maps/lab_box.yaml`. It carries the arena, the radios, the agents' **bodies**
(dimensions, sensors, spawn pose, network) and **points of interest** — named
landmarks. It says nothing about what any agent is trying to do.

```yaml
kind: map
arena: { ... }
agents:
  - id: car1
    pose: {x: -3.0, y: 2.0, ...}   # where it SPAWNS
    dimensions: { ... }
    sensors: [ ... ]
    # no mission: — a map does not task
points:
  A: {x: -3.0, y:  2.0}
  B: {x:  3.0, y:  2.0}
  HOME: {x: 0.0, y: -3.4}
```

**Points are the bridge.** A mission says "shuttle between A and B"; the map
says where A and B are. Drop the same mission on a map whose A and B are
somewhere else, and it runs there with no edit. Name them in UPPER CASE so they
read as landmarks. `maps/big_hall.yaml` is a second, larger map with its own
A–F, kept precisely so you can watch a mission move when the points move.

---

## A mission — tasking, with no world in it

`missions/three_lane_shuttle.yaml`. It names a map and gives each agent an
**objective**.

```yaml
kind: mission
map: lab_box                       # -> maps/lab_box.yaml
objectives:
  car1: {do: shuttle, between: [A, B]}
  car2: {do: shuttle, between: [C, D]}
  car3: {do: shuttle, between: [E, F]}
```

An **objective is a verb, not a destination.** `shuttle`, `pursuit`, `patrol`,
`orbit`, `static`, and `script` (a Python file). `shuttle between A and B` is
portable; `shuttle from (-3,2) to (3,2)` is welded — prefer the point names.

The objectives block is the readable form. Under the hood each entry becomes the
agent's `mission:` block, so everything already written still works and you can
still put a full `mission:` block on an agent inline if you want to.

### Two Python missions (scripts) are objectives too

`do: script, file: missions/wall_follow.py` is an objective like any other. The
`.py` missions (`my_first.py`, `wall_follow.py`, `return_on_link_loss.py`) are
the *open-ended* objectives — a named behaviour is a shortcut, a script is the
general case.

---

## Backward compatibility — the `scenarios/` folder still works

Every file in `scenarios/` is a mission with its map inline: arena, agents and
objectives all in one file, no `map:` key. Those still load and run exactly as
before. Nothing forces you to split a file; the split is available when reuse is
worth it. `three_car_fleet.yaml` stays the self-contained benchmark.

---

## Retasking — change an objective mid-run

The point of calling these *missions* rather than *configs*: an objective is not
fixed for the whole run. You can retask an agent while it is driving.

Start a run with a retask channel:

```bash
python3 tools/stub_telemetry.py \
    --scenario missions/three_lane_shuttle.yaml \
    --retask runs/retask
```

Then, from any other terminal, drop a command in:

```bash
echo 'car3: pursue car1' > runs/retask/queue
```

The next frame, car3 abandons its lane and starts chasing car1. No file edited,
no restart. The command grammar is the words you would say:

```
car3: pursue car1
car3: shuttle between E F
car3: wall_follow right
car3: orbit 2.0
car3: stop
car3: script missions/return_on_link_loss.py
```

A command fires once (reading the queue consumes it). An unparseable line is
ignored with a note on stderr, never a crash — a fat-fingered retask must not
take down a running mission.

**Why a file, not a socket.** The same channel then works from a terminal, from
the Console, and later from a ROS 2 service, with no protocol to agree on. When
the Console grows a "retask" box it writes the same line to the same queue.

---

## Where this is going

The mission dropdown (still on the backlog) writes an objective into the mission
file the clickable way. Live retasking is the same operation aimed at a *running*
sim instead of a file. The portable-goal idea — "get the robots from A to B,
wherever A and B are on this map" — is already here for `shuttle`; extending it
to a general `go_to POINT` objective is a small addition, deferred until it is
needed.
