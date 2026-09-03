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
SETMISSION test             load missions/test.yaml and distribute it
SETMISSION <name>           any file in missions/<name>.yaml
```

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
SETMISSION <name>         y       y       -
REOBJECTIVE <agent> ...   y    own side  own side
JAM <id> band .. power .. y       -       y
shell commands            y       y       y
```
