# CommsEv — Console command reference

Every command accepted by a Console terminal, and which cell may issue it.

The Console has three terminals, and they are not cosmetic. **A cell may only
act on its own side.** Typing a red command into the blue cell is refused with a
reason printed, and the command is never sent. This is what keeps the exercise
valid: blue cannot switch the jammer off, and red cannot retask blue's fleet.

| Cell | May act on | Cannot |
|---|---|---|
| **White** | everything | — |
| **Blue** | blue agents and networks; missions | anything red, including `JAM` |
| **Red** | red agents and networks; jammers | anything blue, including `SETMISSION` |

White is the exercise controller. A single operator can use White and ignore
the split; Blue and Red are for exercises where the refusals should catch
actions the experiment does not allow.

---

## The order of a run

Nothing below works until the run exists. The sequence is always:

```
Setup tab      scene -> blue fleet (spawn dialog) -> red fleet (spawn dialog)
Play           the world starts; Setup locks
BLUE cell      SETMISSION test            give the fleet its objectives
BLUE cell      blue launch                arm them - they start moving
RED cell       JAM jam1 power 25          set the emission
RED cell       red launch                 arm the jammer - it starts transmitting
...
BLUE cell      blue halt
RED cell       red halt
Results tab    Export CSV
```

`SETMISSION` comes **after** Play and **before** `blue launch`. That is
deliberate: a mission is an order issued to a live fleet, and issuing it is
gated by command authority, so an agent whose commander cannot reach it is
**skipped and reported** rather than silently retasked. Setting the mission
before the world exists would skip that gate entirely, and the gate is one of
the things worth measuring.

---

## The map is an editor, before the run starts

Everything below can also be done by hand, on the Setup tab, before Play is
pressed. Nothing here needs a coordinate typed into a file.

| Do this | And this happens |
|---|---|
| Drag a vehicle | It moves. The status bar prints its position and the gap to its nearest neighbour |
| Sweep a box over several | They become one selection |
| Drag any selected one | **All of them move together**, by the same vector — the formation translates instead of coming apart |
| Shift while sweeping | Named points join the selection too |
| Drag a named point | The objective itself moves. `FAR` is wherever it was placed |
| **Add point…** | Invent a destination this scene never had, then drag it |
| Middle-drag | Pans the view (the left button is the placement tool while placing) |

A drag is **clamped to the arena**, using the same bounds check that validates
an objective — so the map cannot express a setup the model would then refuse.

Points are excluded from a box selection unless Shift is held, and that is a
safety rule rather than a preference: penetration is measured to the goal, so a
box swept round a fleet that quietly dragged `FAR` along with it would move the
ruler and the thing being measured together, and the numbers would still look
reasonable afterwards.

A **named formation survives a translation and nothing else.** Move every
mobile vehicle on a side together and the shape is untouched, so the experiment
tab keeps calling it a wedge. Move some of them and it is not a wedge any more,
and the label reverts to `(as spawned)`.

### Spawn point

Choosing a fleet asks one coordinate: **where this side sets up**. The ground
station goes there and the vehicles form up ahead of it, one spacing clear of
the bench, in the chosen formation. Move the spawn point and the whole side
moves with it, shape intact.

### Objectives, from the Setup tab

In **Sandbox** mode the tab carries the mission, the goal, and a list of
recommended objectives with what each one is *for*. Arguments are pre-filled
from the loaded scene's own points, so a suggestion always validates.
**Assign to selection** sends the objective to whatever is selected on the map
— sweep a box round two cars and only those two are retasked — down the same
channel a typed order uses, gated by command authority in the same way.

---

## Arming — `LAUNCH` / `HALT`

**Cell:** blue for blue, red for red. White for either.

```
blue launch                 arm every agent on the blue network
blue halt                   un-arm - freezes each agent at its current pose
red launch                  arm the red side: jammers begin transmitting
red halt                    silence them
car2 launch                 arm one named agent
car2 halt                   un-arm one named agent
```

`launch`/`halt` are also accepted uppercase. An agent that is not armed is
idle regardless of what objective it holds — assignment and activation are
separate steps on purpose, so a fleet can be tasked, its intended behaviour
inspected, and only then set going.

---

## Missions — `SETMISSION`

**Cell: BLUE only.** Missions are a blue-side action; the red cell is refused.

```
SETMISSION <name>                          any file in missions/<name>.yaml
SETMISSION <name> to <P1>                  one goal
SETMISSION <name> to <P1> <P2> <P3>        a route, visited in order
SETMISSION patrol to <P1> <P2> laps 4      and how many times round
```

### A mission says what shape the task is. Everything else is set at the run

There are three, and between them they cover everything:

| Mission | Goals | Laps | What it is |
|---|---|---|---|
| `advance` | as many as are given | 1, fixed | Visit each point once, in order. One goal is the penetration command; three is a route |
| `patrol` | as many as are given | set at the run | The same circuit, repeated. **Two points patrolled is a shuttle** — which is why there is no shuttle mission |
| `forward` | none | — | Drive on until a wall or until command is lost. No arrival, so no score: a probe, not a task |

`advance` fixes one lap deliberately. Offering it a lap count would be offering
to turn it into a patrol under a second name, and a lap count typed at one is
ignored rather than quietly obeyed.

**The objective is always just "go there."** The coordinator holds the route and
hands out one leg at a time; formation is preserved throughout. There is nothing
for an operator to choose between the mission and the vehicle, which is why the
objective palette is gone from Setup. The verbs still exist for retasking **one**
vehicle onto something the fleet is not doing — `car3: pursue car1` — typed at
the terminal, where an exception belongs.

### A mission says how many goals it needs, not where they are

This replaces the old behaviour completely.

`missions/advance.yaml` used to say `to: FAR`. `FAR` existed only in the
corridor, so the one mission the whole experiment programme is built on could
run on exactly one scene — and pointed at any other it did not fail loudly, it
tasked **nobody** and produced a table of vehicles that had not moved. It also
let the author of the *scene* decide the *objective*.

A mission now declares a **count**:

```yaml
plan: {who: all, goals: 2, laps: 4}    # a shuttle needs two ends
```

and the run says where they are — from the Setup tab's goal pickers, which
grow and shrink to match the chosen mission, or from the terminal:

```
SETMISSION advance to P1        corridor
SETMISSION advance to B         open_field — same file, no edit
SETMISSION shuttle to P1 P2     two goals, because shuttle asks for two
```

Giving too few is refused, naming what was needed: a two-goal shuttle handed
one point is not a shorter shuttle, it is a vehicle sitting on a waypoint. A
point the scene does not have is refused, and the refusal lists the ones it
does. `forward` asks for none and is left alone by any goal offered to it.

### Where points come from now

**No scene ships points.** `HOME`, `FAR`, `APEX`, `WINGL`, `WINGR`,
`A`–`F` — every one of them was an objective in disguise.
`APEX`/`WINGL`/`WINGR` were a wedge's formation slots hardcoded into the room
it happened to be standing in, which is exactly what formations-as-functions
removed; `A`–`F` were three lanes across a box, which is a mission somebody
wrote once.

Points are created where they are decided: **Setup → Add point**, typed as
coordinates and then draggable on the map. They are named `P1`, `P2`, … and
they ride with the composed run, so what a result was measured against is
recorded alongside it. This works identically in **sandbox and experiment
mode** — set up `lab_box` with two points in a line and patrol them, or the
corridor with one point at the far end and advance to it, and it is the same
two controls either way.

The one exception is `missions/test.yaml`, the fixed benchmark, which carries
its own **literal coordinates**. It is the control condition everything else is
measured against, so it has to be identical every time and must not depend on
points set up differently in a given session. Every other mission names no
geometry at all — which is what makes one mission file run on any scene.

### The mission is set before Play

Choosing a mission and its goals writes the plan into the composed run, so the
trees, the map and the properties panel show what the fleet has been told
without a simulator having to be running to be told it. **Issue mission** is
the button; **arming stays at the terminal** — `blue launch` — because tasking
a fleet and setting it going are two decisions, and being able to inspect what
it intends to do in between is the reason they were split.

Nothing about the command gate is lost. At t=0 nothing has been jammed yet, so
gating the initial assignment would gate nothing. Every order issued *after* the
run starts still travels the command channel and is still refused when it
cannot get through.

Penetration is measured along the line from where the fleet started to the
**first** goal, so it means the same thing on any scene rather than only on an
east-west corridor.

### Everything placed by hand lands on whole metres

Drags, formations and typed points all snap to the nearest metre. A drag
produces whatever fraction the mouse happened to be on — `2.6371` — and those
numbers propagate into the composed run, the CSV and every figure downstream,
where they read as precision that was never measured. A formation set up by
eye is not accurate to a tenth of a millimetre and should not claim to be.

The model is untouched: `formation_offsets` still returns exact geometry, so a
circle is still a circle. Only the coordinates a human put there are rounded,
at the moment they are written.

Applies that mission file's per-agent objectives to the running fleet and
titles the run with its name, so results come out as
`<scene>_<fleet>_<mission>_<timestamp>`.

**Gated by command authority.** Every agent's decider chain is checked before
it is retasked. An agent its decider cannot currently reach is skipped, and the
skip is printed — a jammed fleet can be observed refusing an order.

---

## Retasking one agent — `REOBJECTIVE`

**Cell:** the side that owns the agent.

```
REOBJECTIVE car3 pursue car1
REOBJECTIVE car3 shuttle between E F
REOBJECTIVE car3 shuttle (-3,3,0) (3,3,0)
REOBJECTIVE car3 wall_follow right
REOBJECTIVE car3 orbit 2.0
REOBJECTIVE car3 stop
REOBJECTIVE car3 script missions/return_on_link_loss.py
```

### The objective verbs

| Verb | Aliases | Arguments | What it does |
|---|---|---|---|
| `shuttle` | — | two points | Drive back and forth between them, forever |
| `pursuit` | `pursue` | an agent id | Follow that agent at a standoff distance |
| `patrol` | — | waypoints | Loop a circuit |
| `orbit` | — | radius (m) | Circle the origin |
| `wall_follow` | `wall` | `left` / `right` | Track a wall (needs a lidar and a wall) |
| `static` | `stop`, `hold` | — | Hold position |
| `script` | — | a `.py` path | Determined by the mission file |

**Points can be named or literal.** `shuttle between E F` uses the points the
*scene* defines, so the same objective runs on any scene that defines E and F.
`shuttle (-3,3,0) (3,3,0)` welds it to these coordinates. Named points are what
make a mission portable; use them unless there is a specific reason not to.

Objectives are validated when they are **assigned** — named points must
resolve, endpoints must be inside the arena. A bad objective is refused with a
reason, not silently turned into something else.

---

## Jamming — `JAM`

**Cell: RED only.** The blue cell is refused — blue cannot switch off the
emitter attacking it.

```
JAM jam1 band 2400 power 25     set the whole emission in one line
JAM jam1 power 25 band 2400     order does not matter
JAM jam1 power 25               one property on its own still works
```

**Set band and power together.** Two separate commands leave a window where
the jammer is on the new band at the *old* power — a third emission, which is
neither the intended emission nor the previous one. One line, one emission.

A malformed line applies **nothing**: a half-applied emission is worse than a
rejected one, because the run carries on using settings nobody chose.

Takes effect **live**, mid-run. Arming and silencing is `red launch` /
`red halt`, not a `JAM` argument — the vehicle is either deployed or not.

### Frequencies worth knowing

| MHz | What it is |
|---|---|
| `2400` | The fleet's command band. Jamming here attacks **authority** |
| `1575.42` | GPS L1. Jamming here attacks **position knowledge** |

These do different things and they are not interchangeable. Comms jamming
strips command, so a `hold` fleet stops — and a stopped vehicle no longer
dead-reckons, so comms jamming *suppresses* the drift GNSS jamming causes.
Sweeping both together makes the two effects inseparable.

### Dual-emitter vehicles — PLANNED, NOT YET BUILT

A `jammer_dual` carries two transmitters, addressed as channels on the vehicle:

```
JAM jam1.a power 25         channel a only
JAM jam1.b band 1575.42     channel b only
JAM jam1.b off              silence one channel, leave the other transmitting
JAM jam1.b on
JAM jam1 power 25           both channels
JAM jam1                    report what every channel is doing
```

`jam1` is the **vehicle**; `jam1.a` and `jam1.b` are the **transmitters bolted
to it**. `red halt` still stops the whole vehicle — `JAM jam1.b off` silences
one radio on a vehicle that is still deployed. That distinction is real, and
it is the only place in the grammar where it appears.

---

## Anything else

A terminal that does not recognise the first word runs the line as a shell
command in WSL, from the repo root, with ROS 2 sourced. So `ls runs/`,
`git status` and `python3 tests/test_all.py` all work from the same box.

---

## Refusals

| Message | Why |
|---|---|
| `[blocked: missions are a BLUE-cell action]` | `SETMISSION` from red or white-as-red |
| `[blocked: JAM is a RED-cell action]` | `JAM` from the blue cell |
| `[blocked: 'car1' is on the blue side; this is the red cell]` | Wrong side for `launch`/`halt`/`REOBJECTIVE` |
| `SETMISSION: <agent> skipped - commander unreachable` | Not a refusal. The order could not get through. This is a **result**, not an error |

The last message is the central measurement of the exercise; record it when it
occurs.

---

## Quick reference

```
                        WHITE   BLUE    RED
blue launch / halt        y       y       -
red launch / halt         y       -       y
<agent> launch / halt     y    own side  own side
SETPLAN <who> <pts> laps  y       y       -
SETMISSION <name> [to P]  y       y       -
REOBJECTIVE <agent> ...   y    own side  own side
JAM <id> band .. power .. y       -       y
LISTEN <MHz> | off        y       -       y
shell commands            y       y       y
```
