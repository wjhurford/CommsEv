> **SUPERSEDED (1 Sep 2026).** Written against the old map/mission vocabulary and the retired `scenarios/` files.
> The live model is **scene + fleet, composed by a mission** - see `docs/vocabulary.md`.
> Kept for provenance; do not follow the file names or commands here.

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

## Assign, inspect, launch — the gate before anything moves

Opening a file and pressing Play never starts anything moving, on ANY file —
a bare scene, a split mission, or a legacy self-contained `scenarios/` file.
The sequence is always the same five steps:

1. **Open** a scene (or mission, or scenario). Agents spawn at their pose and
   sit still — a scene carries no tasking at all.
2. **Press Play.** The sim is now running — links are live, agents report
   telemetry — but every agent stays idle until it is both ASSIGNED an
   objective and ARMED. Nothing assigned yet is not an error; it idles
   forever, on purpose.
3. **Assign** an objective — by opening a file that already has one baked in,
   by `REOBJECTIVE` (one agent), `REMISSION` (a system order, decomposed —
   see below), or `LOADMISSION` (a mission file's own objectives, verbatim).
   Assigning NEVER arms anything.
4. **Inspect.** The Mission tab shows each agent's objective tagged
   `[assigned, not launched]`. Confirm it is what you meant before anything
   drives — this is the whole reason the gate exists: the tool measures
   deviation from a *commanded* position, so knowing exactly when a command
   was accepted, separately from when it started acting on it, is the point.
5. **Launch.** Type `<network> launch` (e.g. `blue launch`) or `<agent>
   launch` in the Terminal tab. Only now does the objective take effect.

`<network> halt` / `<agent> halt` un-arms without touching the objective —
the agent freezes wherever it currently is and resumes the same objective on
the next launch.

**Retasking an already-armed agent never needs a halt first.** REOBJECTIVE,
REMISSION and LOADMISSION all take effect immediately on an armed agent,
exactly as they always have — the gate above is only ever about the FIRST
time an objective takes effect on a given agent, never about every change
after that. Requiring a halt before every retask would break live retasking
under a live attack, which is the whole point of calling these missions
rather than configs.

---

## Retasking — change an objective mid-run

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

If car3 is armed, the next frame it abandons its lane and starts chasing
car1. No file edited, no restart. The command grammar is the words you would
say:

```
car3: pursue car1
car3: shuttle between E F
car3: shuttle (-3,3,0) (3,3,0)
car3: shuttle A (3,2,0)          # a named point and a literal, mixed
car3: wall_follow right
car3: orbit 2.0
car3: stop
car3: script missions/return_on_link_loss.py
```

**Waypoints are absolute coordinates, always** — `(x,y)` or `(x,y,z)`,
parentheses required, `z` defaulting to `0`, spaces inside the parentheses
tolerated (`(-3, 3, 0)` parses fine). A bare word is a point name, resolved
against the current map. Real metres everywhere a position is expected is
what makes deviation under jamming or spoofing measurable against a real
commanded position, rather than against a fraction of the arena.

A named point that does not exist on the current map, or a literal that
resolves outside the arena, is **rejected, not adjusted** — the agent keeps
doing whatever it was doing before the bad command, and the rejection is
reported both on stderr and in the Console's Mission tree, not silently
swallowed. This used to fail silently: an unresolvable point fell back to
the arena half-width, so a retasked car would quietly drive a
different-sized lane than the one it was told — looking like it worked while
being subtly wrong. Not any more.

A command fires once (reading the queue consumes it). An unparseable line is
ignored with a note on stderr, never a crash — a fat-fingered retask must not
take down a running mission.

**Why a file, not a socket.** The same channel then works from a terminal, from
the Console, and later from a ROS 2 service, with no protocol to agree on. The
Console's Terminal tab writes the same line to the same queue.

---

## REMISSION — a system order, decomposed by authority

`REOBJECTIVE` tasks one agent. `REMISSION` tasks a **network**: you give the
system a mission, and the system works out each agent's own objective from
it — the layer that makes the existing `authority` model (centralized /
decentralized / hierarchical) something you exercise in normal operation,
not just during a link-loss test.

```
REMISSION blue shuttle (-3,0,0) (3,0,0)
```

means "blue, patrol this line" — not "give every agent this exact line".
Ground stations never receive an objective from a REMISSION; they are not
vehicles. For a `shuttle` order, every eligible vehicle gets its **own**
parallel lane, offset along the line's perpendicular, so three cars given
one line come out shuttling three distinct lanes, not stacked on top of each
other. Other verbs broadcast the same objective to every eligible agent
verbatim, for now.

Who is eligible — and who decomposes for whom — depends on the network's
`authority`:

| authority | who decomposes | an unreachable agent/leader |
|---|---|---|
| `centralized` | the coordinator, once, for every member | that agent alone is skipped |
| `decentralized` | every agent, for itself | nothing is ever skipped |
| `hierarchical` | each squad leader, for its own squad | the **whole squad** is skipped |

A skip is reported by name (`alpha SKIPPED - leader car1 unreachable`) and
leaves the skipped agent(s) doing whatever they were doing before — never a
silent partial success. Under `hierarchical` this is a deliberate choice:
the coordinator does not reach around a dead squad leader to task the
members directly, because that would make hierarchical behave exactly like
centralized at the one moment you're trying to measure the difference
between them. Reissue the REMISSION once the leader is reachable again to
deliver it then — it is a one-time check at the moment of issue, not a
queued delivery.

A `shuttle` REMISSION that would put **any** agent's lane outside the arena
is rejected **wholesale** — nothing is retasked, not even the agents whose
lane would have fit. Clamping the spacing or shifting the line to make it
fit was considered and rejected: either would mean an agent's real commanded
position silently differs from what was typed, which is exactly what this
framework exists to prevent elsewhere.

---

## LOADMISSION — a mission file's own objectives, verbatim

Different job from REMISSION: `LOADMISSION <path>` distributes a mission
file's already-written per-agent objectives onto the currently running agent
set, exactly as written — no decomposition, no authority gating. Use it to
swap in a genuinely different, hand-authored mission mid-run:

```
LOADMISSION missions/three_lane_shuttle.yaml
```

Each agent the file names gets that file's exact objective for it; an agent
the file names that isn't in the running scene is warned about, not silently
dropped. `armed` is untouched either way, same as every other assignment —
see "Assign, inspect, launch" above.

---

## Where this is going

The mission dropdown (still on the backlog) writes an objective into the mission
file the clickable way. Live retasking is the same operation aimed at a *running*
sim instead of a file. The portable-goal idea — "get the robots from A to B,
wherever A and B are on this map" — is already here for `shuttle`; extending it
to a general `go_to POINT` objective is a small addition, deferred until it is
needed.
