# Writing a mission

A **mission** is one Python function that decides where one agent should be
heading. It is the extension point of this whole framework: everything else —
the arena, the radios, the lidar, the network, the attacks — exists so that a
mission has something real to react to.

This walks through writing one from scratch, then running it three ways.

---

## 1. The contract

One file, one function:

```python
def target(agent, world):
    """Return (x, y) in metres, world frame."""
    return (0.0, 0.0)
```

That is the entire interface. Two rules make it worth defending:

**It returns a point, not a command.** You say where you want to be; the
controller works out the steering, the speed limit still applies, and collision
can stop the agent short. You cannot drive through a wall by returning a target
behind it. This separation is also what stopped two cars phasing through each
other back when missions wrote positions directly.

**It imports nothing from this framework, and never `rclpy`.** A mission takes
plain numbers and returns plain numbers. That is the reason the same file can
run against this simulator today and against a real RoboRacer tomorrow — it
never learns which one it is talking to. If you ever find yourself wanting to
`import rclpy` inside a mission, the thing you want belongs in the node
wrapper instead.

### `agent` — what this agent is

Its own configuration, straight from the scenario file:

| key | what it is |
|---|---|
| `agent["id"]` | `"car3"` |
| `agent["speed"]` | its top speed, m/s |
| `agent["dimensions"]` | `length`, `width`, `height` in metres |
| `agent["start"]` | where it spawned: `x, y, z, yaw` |
| `agent["sensors"]` | list of `{id, type, offset}` |
| `agent["mission"]` | **this block of the scenario file** — every extra key you put there arrives here |

That last row is how a mission takes parameters. Put `standoff: 0.8` under
`mission:` in the YAML and read it as `agent["mission"]["standoff"]`. No
plumbing to write.

### `world` — what this agent knows

```python
world.t                          # seconds since the run started
world.dt                         # seconds per tick
world.arena                      # extent, boundaries, propagation, spectrum

world.pose(id)                   # {x, y, z, yaw, speed} or None
world.distance_to(a, b)          # metres, in the plane
world.bearing_to(a, b)           # radians from a's NOSE to b; 0 is dead ahead

world.scan(id)                   # that agent's last lidar scan, or None
world.nearest_return(id, lo, hi) # (range_m, angle_rad) of the closest return

world.link(a, b)                 # {distance_m, quality, state, latency_ms, pdr}
```

**Why one argument and not four.** This is the interface every mission in the
project is written against. Adding a new sense later — battery, a jammer's
bearing, a neighbour's health — must not change the signature, because changing
it breaks every mission anyone has written. So the signature is frozen and the
world grows instead.

The older four-argument form `target(agent, t, poses, arena)` still runs, with
a deprecation warning on stderr. It cannot see the lidar or the link, which is
most of what makes a mission interesting.

---

## 2. Three things that will bite you

**A ray that returned nothing is `float('inf')`, not `range_max`.** The
UST-10LX reaches 10 m off white card and about 4 m off a dark matt wall; past
that there is *no return at all*. `inf` and `range_max` mean different things —
"I can't see anything there" versus "there is a wall at exactly 10 m" — and a
mission that conflates them will drive confidently into an open doorway. Always
guard with `if r < float("inf")`.

**Angles are in the sensor's frame.** Zero is dead ahead, positive is to the
left (right-handed, z up). The wall on your right is near `-pi/2`. `world.scan()`
gives you `angle_min`, `angle_max` and a `ranges` list; index *i* is at
`angle_min + (angle_max - angle_min) * i / (n - 1)`. `world.nearest_return()`
does that arithmetic for you, and it is worth using, because sign errors in
index arithmetic are the single most common way a mission goes wrong.

**Fallback logic needs hysteresis.** If a mission changes behaviour when link
quality drops below 0.5, an agent sitting at 0.5 will flip every tick. Use two
thresholds — drop out below 0.45, come back above 0.65. Real fallback logic has
this. See `missions/return_on_link_loss.py`.

---

## 3. Write one

Make `missions/my_first.py`. Keep station 2 m to the left of car1:

```python
import math


def target(agent, world):
    lead = world.pose("car1")
    if lead is None:                       # not heard from yet
        return (agent["start"]["x"], agent["start"]["y"])

    offset = float(agent["mission"].get("offset", 2.0))

    # "Left of car1" means left of ITS nose, not left on the map.
    bearing = lead["yaw"] + math.pi / 2
    return (lead["x"] + offset * math.cos(bearing),
            lead["y"] + offset * math.sin(bearing))
```

Point a scenario at it. In `scenarios/three_car_fleet.yaml`, replace car3's
`mission:` block with:

```yaml
    mission:
      type: script
      file: missions/my_first.py
      offset: 2.0
```

Press **Play** in the Console. Every agent spawns and sits still, however its
objective is set - Play only starts the sim, it never starts the moving. Type
`blue launch` in the Terminal tab (or whichever network the fleet is on) to
arm the fleet. Car 3 should then sit off car 1's left flank and stay there
while car 1 shuttles.

This gap between Play and launch is deliberate, not a delay to get past: it
is the point in the run where you can inspect what every agent has actually
been assigned - open the Mission tab and check each objective reads what you
meant - before anything moves. Confirming the *commanded* state before launch
is what makes measuring deviation from it, later, mean something. See
`docs/maps-missions-and-retasking.md` for the full assign → inspect → launch
sequence and the retasking grammar.

### Then make it react to something

The version above is a path follower and could be written against any
simulator. Make it break formation when the network does:

```python
    link = world.link(agent["id"], "gcs")
    if link and link["state"] == "down":
        home = world.pose("gcs")
        return (home["x"], home["y"])       # give up the formation, go home
```

That is a resilient-control experiment. Run it once with `attacks: []` and once
with a jammer against the coordinator, and the difference between the two runs
is a result.

---

## 4. Running it

**From the Console** — set it in the scenario file, press Play, then `blue
launch` (or whichever network) once you've checked the Mission tab shows what
you meant. This is the normal way and it is what the rest of the toolchain
assumes: the run gets recorded to a bag in `runs/`, the timeline scrubs it,
and PlotJuggler opens it.

**Headless, for a sweep** — the simulator core runs on its own and prints one
JSON frame per line. `--retask` opens the same command channel the Console's
Terminal writes to, so a script can launch it the same way:

```bash
python3 tools/stub_telemetry.py --scenario scenarios/three_car_fleet.yaml \
    --retask runs/retask
# from another shell, once you're ready:
echo "LAUNCH blue" > runs/retask/queue
```

**As a real ROS 2 node** — the same file, driving a car over ROS topics:

```bash
ros2 run deadband_ros controller --ros-args \
    -p agent:=car3 \
    -p mission:=missions/my_first.py
```

The node subscribes to `/car*/odom` and `/car3/scan`, builds the identical
`world` object out of what arrived on those topics, calls your `target()`, and
publishes `/car3/drive`. Nothing in your file changes. Point it at a real car's
topics instead and nothing in your file changes then either — **that is the
sim-to-real argument, and it only holds as long as missions stay free of ROS.**

---

## 5. The examples, in the order to read them

| file | what it demonstrates |
|---|---|
| `missions/example_pursuit.py` | the minimum: read another agent's pose, return a point |
| `missions/wall_follow.py` | closing a loop through the **lidar**, including no-return handling and corners |
| `missions/return_on_link_loss.py` | reading **link state** and changing behaviour — the research-shaped one |

---

## 6. Debugging

A mission that raises does not silently become "static" — the exception is
printed with the agent's id, on stderr in headless runs and in the Console's
**Log** panel otherwise. If an agent is not moving and nothing was logged, the
mission is returning its current position, not failing.

`print()` inside a mission works and goes to the same place. It is the fastest
tool here; use it before anything cleverer.

If an agent moves but wrongly, plot it: keep the run's bag, open it in
PlotJuggler, and put `/car3/odom/pose/pose/position/x` next to the value your
mission thought it was aiming at. A trajectory that overshoots in a repeating
pattern is a gain that is too high; one that spirals away from a wall is a sign
error in the correction term.
