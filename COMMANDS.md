# DEADBAND — Console command reference

Every command you can type into a Console terminal, and which cell may type it.

The Console has three terminals, and they are not cosmetic. **A cell may only
act on its own side.** Typing a red command into the blue cell is refused with a
reason printed, and the command is never sent. This is how the exercise stays
honest: blue cannot switch the jammer off, and red cannot retask blue's fleet.

| Cell | May act on | Cannot |
|---|---|---|
| **White** | everything | — |
| **Blue** | blue agents and networks; missions | anything red, including `JAM` |
| **Red** | red agents and networks; jammers | anything blue, including `SETMISSION` |

White is the exercise controller. If you are running alone, use White and
ignore the split; use Blue and Red when you want the refusals to catch you
doing something the experiment shouldn't allow.

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

Everything below can also be done by hand, on the Setup tab, before you press
Play. Nothing here needs a coordinate typed into a file.

| Do this | And this happens |
|---|---|
| Drag a vehicle | It moves. The status bar prints its position and the gap to its nearest neighbour |
| Sweep a box over several | They become one selection |
| Drag any selected one | **All of them move together**, by the same vector — the formation translates instead of coming apart |
| Shift while sweeping | Named points join the selection too |
| Drag a named point | The objective itself moves. `FAR` is where you put it |
| **Add point…** | Invent a destination this scene never had, then drag it |
| Middle-drag | Pans the view (the left button is the placement tool while placing) |

A drag is **clamped to the arena**, using the same bounds check that validates
an objective — so the map cannot express a setup the model would then refuse.

Points are excluded from a box selection unless you hold Shift, and that is a
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
the bench, in whatever formation you picked. Move the spawn point and the whole
side moves with it, shape intact.

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
separate steps on purpose, so you can task a fleet, inspect what it intends to
do, and only then set it going.

---

## Missions — `SETMISSION`

**Cell: BLUE only.** Missions are a blue-side action; the red cell is refused.

```
SETMISSION <name>                  any file in missions/<name>.yaml
SETMISSION <name> to <P1>          one goal
SETMISSION <name> to <P1> <P2>     as many as the mission asks for
```

### A mission says how many goals it needs, not where they are

This is the important one, and it replaces the old behaviour completely.

`missions/advance.yaml` used to say `to: FAR`. `FAR` existed only in the
corridor, so the one mission the whole experiment programme is built on could
run on exactly one scene — and pointed at any other it did not fail loudly, it
tasked **nobody** and produced a table of vehicles that had not moved. Worse,
it let whoever wrote the *scene* decide the *objective*.

A mission now declares a **count**:

```yaml
plan: {who: all, goals: 2, laps: 4}    # a shuttle needs two ends
```

and the run says where they are — from the Setup tab's goal pickers, which
grow and shrink to match the mission you chose, or from the terminal:

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

**Scenes no longer ship objectives.** `corridor_200m` declares no points at
all. `HOME`, `FAR`, `APEX`, `WINGL` and `WINGR` were every one of them an
objective in disguise — `APEX`/`WINGL`/`WINGR` were a wedge's formation slots
hardcoded into the room it happened to be standing in, which is exactly what
formations-as-functions removed.

Points are created where they are decided: **Setup → Add point**, typed as
coordinates and then draggable on the map. They are named `P1`, `P2`, … and
they ride with the composed run, so what a result was measured against is
recorded alongside it.

A scene *may* still declare points — `lab_box` and `open_field` keep `A`–`F`,
because those lanes are the fixed geometry of a benchmark that has to be
identical every time. That is a scene making a claim about itself. A corridor
claiming to know where you want to go is not the same thing.

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
skip is printed — you can watch a jammed fleet refuse an order.

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
| `script` | — | a `.py` path | Your own mission file decides |

**Points can be named or literal.** `shuttle between E F` uses the points the
*scene* defines, so the same objective runs on any scene that defines E and F.
`shuttle (-3,3,0) (3,3,0)` welds it to these coordinates. Named points are what
make a mission portable; use them unless you have a reason not to.

Objectives are validated when they are **assigned** — named points must
resolve, endpoints must be inside the arena. A bad objective is refused with a
reason, not silently turned into something else.

---

## Jamming — `JAM`

**Cell: RED only.** The blue cell is refused — blue does not get to turn off the
thing attacking it.

```
JAM jam1 band 2400 power 25     set the whole emission in one line
JAM jam1 power 25 band 2400     order does not matter
JAM jam1 power 25               one property on its own still works
```

**Set band and power together.** Two separate commands leave a window where
the jammer is on the new band at the *old* power — a third emission, which is
not the one you meant and not the one you had. One line, one emission.

A malformed line applies **nothing**: a half-applied emission is worse than a
rejected one, because the run carries on using settings nobody chose.

Takes effect **live**, mid-run. Arming and silencing is `red launch` /
`red halt`, not a `JAM` argument — the vehicle is deployed or it isn't.

### Frequencies worth knowing

| MHz | What it is |
|---|---|
| `2400` | The fleet's command band. Jamming here attacks **authority** |
| `1575.42` | GPS L1. Jamming here attacks **position knowledge** |

These do different things and they are not interchangeable. Comms jamming
strips command, so a `hold` fleet stops — and a stopped vehicle no longer
dead-reckons, so comms jamming *suppresses* the drift GNSS jamming causes. If
you sweep both together you cannot tell the two effects apart.

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

## Refusals you will meet

| Message | Why |
|---|---|
| `[blocked: missions are a BLUE-cell action]` | `SETMISSION` from red or white-as-red |
| `[blocked: JAM is a RED-cell action]` | `JAM` from the blue cell |
| `[blocked: 'car1' is on the blue side; this is the red cell]` | Wrong side for `launch`/`halt`/`REOBJECTIVE` |
| `SETMISSION: <agent> skipped - commander unreachable` | Not a refusal. The order could not get through. This is a **result**, not an error |

That last one is the point of the whole exercise. Write it down when it happens.

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
