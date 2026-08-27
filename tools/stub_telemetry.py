#!/usr/bin/env python3
"""
Deadband — stub telemetry source.

Publishes correctly-shaped telemetry for a scenario, with no ROS, no Gazebo and
no robotics stack: just Python. That means the Console can be built and tested
before the simulator exists, and a future student can work on the front end
without installing anything.

It now READS THE SCENARIO FILE, so what you configure in the Console is what
moves on screen. The motion is simple kinematics and the lidar is a simple
raycast, but both are honest about geometry — nothing here is faked to look
plausible. When the real simulator arrives it emits this same shape and the
Console never notices the difference.

    python3 stub_telemetry.py                          # default scenario
    python3 stub_telemetry.py --scenario path.yaml
    python3 stub_telemetry.py --record 30              # write a sample file
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SCENARIO = REPO_ROOT / "scenarios" / "three_car_fleet.yaml"
RATE_HZ = 10.0


def _num(v, default=0.0):
    """Accept a bare number or a {value, unit, source} quantity."""
    if isinstance(v, dict):
        v = v.get("value")
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def wrap_pi(a):
    """Wrap an angle in radians to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Scenario
# ---------------------------------------------------------------------------

def _load_yaml(path):
    import yaml
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def resolve_mission(path):
    """Read a mission file and merge in its map, if it names one.

    A mission file describes WHAT the agents are doing (their objectives). It
    MAY carry the world inline - arena, agents, radios - exactly as the old
    scenario files did, and then it is self-contained. OR it may say

        map: lab_box

    and the world is read from maps/lab_box.yaml instead, so the same mission
    can be dropped onto any map. The merge rule is simple and one-directional:
    the MAP owns the world (arena, radios, and each agent's body and spawn); the
    MISSION owns the objectives (each agent's `mission:` block) and may add
    points of interest. Where a mission also specifies world keys, the mission
    wins - so a mission can nudge a map without editing it.

    Everything downstream still receives one merged dict shaped exactly like the
    old scenario, so nothing else in the pipeline had to change.
    """
    doc = _load_yaml(path)
    # SCENE is the term: the world a mission is dropped into. `map:` is kept as
    # a silent alias so nothing already written breaks.
    map_ref = doc.get("scene") or doc.get("map")
    if not map_ref:
        return doc                          # self-contained: the old shape

    map_path = Path(map_ref)
    if not map_path.suffix:
        map_path = map_path.with_suffix(".yaml")
    if not map_path.is_absolute():
        # A bare name means scenes/<name>.yaml (maps/ still searched for older
        # files); a path is taken as given.
        if map_path.parent == Path("."):
            cand = REPO_ROOT / "scenes" / map_path
            map_path = cand if cand.exists() else (REPO_ROOT / "maps" / map_path)
        else:
            map_path = REPO_ROOT / map_path
    world_doc = _load_yaml(map_path)

    merged = dict(world_doc)                 # start from the scene's world
    # The mission's own top-level keys win, EXCEPT agents, which are merged
    # per-id so the scene keeps the bodies and the mission supplies the objectives.
    for key, val in doc.items():
        if key in ("map", "scene", "agents"):
            continue
        merged[key] = val

    map_agents = {a.get("id"): a for a in (world_doc.get("agents") or [])}
    mission_agents = {a.get("id"): a for a in (doc.get("agents") or [])}

    # A mission may task agents two ways. The preferred, readable one is a top-
    # level `objectives:` map keyed by agent id, each value {do: <verb>, ...}.
    # That is translated here into the per-agent `mission:` block the rest of
    # the pipeline already understands, so the file speaks in objectives and the
    # machinery underneath is unchanged. `do:` becomes `type:`.
    for aid, obj in (doc.get("objectives") or {}).items():
        if not isinstance(obj, dict):
            continue
        block = {("type" if k == "do" else k): v for k, v in obj.items()}
        block.setdefault("type", "static")
        mission_agents.setdefault(aid, {"id": aid})["mission"] = block

    out_agents = []
    for aid, body in map_agents.items():
        agent = dict(body)
        task = mission_agents.get(aid)
        if task:
            for k, v in task.items():
                if k == "id":
                    continue
                agent[k] = v                 # objective (and any override) wins
        out_agents.append(agent)
    # Agents the mission names that the map does not have are an error worth
    # surfacing loudly rather than silently dropping.
    for aid in mission_agents:
        if aid not in map_agents:
            print(f"mission names agent '{aid}' with no body in the map "
                  f"'{map_ref}' - it will not appear", file=sys.stderr)
    merged["agents"] = out_agents
    return merged


def load_scenario(path):
    doc = resolve_mission(path)

    arena = doc.get("arena") or {}
    extent = arena.get("extent") or {}
    world = {
        "type": arena.get("type", "box"),
        "extent": {k: _num(extent.get(k), 8.0) for k in ("x", "y", "z")},
        "boundaries": dict(arena.get("boundaries") or {}),
        # Sets how far the lidar reaches against a wall. Was being ignored,
        # which is why every beam ran to the geometric wall regardless.
        "surface_reflectivity": arena.get("surface_reflectivity", "high"),
        # Named points of interest the MAP defines, e.g. {A: {x, y}, B: {x, y}}.
        # An objective says "shuttle between A and B"; the map says where A is.
        # This is what lets one mission run on many maps.
        "points": {k: {"x": _num((v or {}).get("x")),
                       "y": _num((v or {}).get("y")),
                       "z": _num((v or {}).get("z"))}
                   for k, v in (doc.get("points") or {}).items()},
        # Networks, carried so command_authority() can read each one's
        # architecture while building a frame.
        "networks": doc.get("networks") or {},
    }

    agents = []
    for a in doc.get("agents") or []:
        dims = a.get("dimensions") or {}
        pose = a.get("pose") or {}
        agents.append({
            "id": a.get("id"),
            "platform": a.get("platform", "unknown"),
            "network": a.get("network", "blue"),
            "colour": a.get("colour") or "#2E6FB0",
            "dimensions": {
                "length": _num(dims.get("length"), 0.4),
                "width": _num(dims.get("width"), 0.4),
                "height": _num(dims.get("height"), 0.2),
            },
            "sensors": [
                {"id": s.get("id"), "type": s.get("type"),
                 "offset": {k: _num((s.get("mount") or {}).get("offset", {}).get(k))
                            for k in ("x", "y", "z")}}
                for s in (a.get("sensors") or [])
            ],
            "start": {"x": _num(pose.get("x")), "y": _num(pose.get("y")),
                      "z": _num(pose.get("z")), "yaw": _num(pose.get("yaw"))},
            "mission": a.get("mission") or {"type": "static"},
            "ghost": bool(a.get("ghost", False)),
            "speed": _num((a.get("performance") or {}).get("max_speed"), 1.5),
        })

    for a in agents:
        # One collision radius per agent, from its own footprint.
        a["radius"] = max(a["dimensions"]["length"], a["dimensions"]["width"]) / 2.0

    # Reachability is EMERGENT, not declared. Every same-network pair is a
    # CANDIDATE; whether it is usable is decided each tick by the RF model, and
    # which candidates are actually used is decided by routing.
    #
    # The old code built links from whether a coordinator existed, so routing
    # was derived from the declaration and could never disagree with it - which
    # made topology unmeasurable. You could declare 'mesh' over a hand-drawn
    # star and the simulator would agree with you. Candidates now; physics and
    # routing decide the rest.
    links = []
    for name in (doc.get("networks") or {}):
        members = [a["id"] for a in agents if a["network"] == name]
        links += [{"a": members[i], "b": members[j], "network": name}
                  for i in range(len(members))
                  for j in range(i + 1, len(members))]

    return world, agents, links


# ---------------------------------------------------------------------------
# Missions
#
# Named behaviours, the same shapes the RoboRacer work uses. Motion is simple
# kinematics — constant speed along a path — not vehicle dynamics. That is the
# right level for this stub: it produces geometry that is true (an agent really
# is where it says it is), without pretending to model a drivetrain.
# ---------------------------------------------------------------------------

_SCRIPTS = {}

# The most recent scan taken by each agent, by agent id.
#
# Module-level because a mission asks for a scan DURING step(), and step() runs
# before this frame's scans are computed - so what a mission gets is the last
# scan taken, one tick old. That is not a compromise, it is what a real
# subscriber gets: you act on the reading you have, not the one that has not
# happened yet. A mission written against this will behave the same on a car.
_LAST_SCANS = {}


class World:
    """Everything a mission is allowed to know, as one argument.

    WHY ONE ARGUMENT AND NOT FOUR. This is the interface every future mission
    in this project is written against. Adding a new sense - link quality,
    battery, a jammer's bearing - must not change the signature, because
    changing it invalidates every mission anyone has written. So the signature
    is frozen at target(agent, world) and the world grows instead.

    Everything here is read-only from a mission's point of view. A mission
    returns a point; it does not move anything itself. Speed limits, collision
    and walls still apply, so you cannot drive through anything by returning a
    target on the far side of it.
    """

    def __init__(self, t, dt, poses, arena, agents, scans=None):
        self.t = t                  # seconds since the run started
        self.dt = dt                # seconds per tick
        self.arena = arena          # extent, boundaries, propagation, spectrum
        self.poses = poses          # {id: {x, y, z, yaw, speed}}
        self.agents = agents        # every agent's config
        self._scans = scans if scans is not None else _LAST_SCANS

    # -- where things are ---------------------------------------------------
    def pose(self, agent_id):
        """One agent's pose, or None if it has never been heard from."""
        return self.poses.get(agent_id)

    def distance_to(self, me, other):
        """Metres between two agents, in the plane."""
        a, b = self.poses.get(me), self.poses.get(other)
        if a is None or b is None:
            return float("inf")
        return math.hypot(b["x"] - a["x"], b["y"] - a["y"])

    def bearing_to(self, me, other):
        """Radians from `me`'s NOSE to `other`. Zero means dead ahead."""
        a, b = self.poses.get(me), self.poses.get(other)
        if a is None or b is None:
            return 0.0
        return wrap_pi(math.atan2(b["y"] - a["y"], b["x"] - a["x"]) - a["yaw"])

    # -- what things can see ------------------------------------------------
    def scan(self, agent_id):
        """That agent's last lidar scan, or None if it has no lidar yet.

        The dict is exactly what goes on the wire and onto the ROS topic:
        angle_min, angle_max, angle_increment, range_min, range_max, ranges.
        A ray that returned nothing is float('inf') - NOT range_max. The two
        mean different things and a mission that treats them alike will drive
        into open doorways.
        """
        return self._scans.get(agent_id)

    def ray(self, agent_id, angle):
        """Range along ONE beam, at the sensor angle nearest to `angle`.

        float('inf') if that beam returned nothing. Two named beams are enough
        to work out a wall's angle as well as its distance, which is what
        stops a wall follower oscillating - see missions/wall_follow.py.
        """
        sc = self.scan(agent_id)
        if not sc:
            return float("inf")
        n = len(sc["ranges"])
        if n == 0:
            return float("inf")
        span = sc["angle_max"] - sc["angle_min"]
        if span <= 0:
            return float("inf")
        i = round((angle - sc["angle_min"]) / span * (n - 1))
        if i < 0 or i > n - 1:
            return float("inf")       # outside the sensor's fan
        r = sc["ranges"][i]
        return float("inf") if r is None else float(r)

    def nearest_return(self, agent_id, lo=None, hi=None):
        """(range_m, angle_rad) of the closest lidar return, optionally only
        within a bearing window. Returns (inf, 0.0) if nothing came back.

        The single most useful thing to ask a lidar, and worth having here so
        that every mission does not reimplement the index arithmetic - which is
        where sign errors live.
        """
        sc = self.scan(agent_id)
        if not sc:
            return (float("inf"), 0.0)
        best, best_a = float("inf"), 0.0
        n = len(sc["ranges"])
        span = sc["angle_max"] - sc["angle_min"]
        for i, r in enumerate(sc["ranges"]):
            if r is None or not (r < float("inf")):
                continue
            a = sc["angle_min"] + span * i / max(1, n - 1)
            if lo is not None and a < lo:
                continue
            if hi is not None and a > hi:
                continue
            if r < best:
                best, best_a = r, a
        return (best, best_a)

    # -- what things can hear ----------------------------------------------
    def link(self, a, b):
        """Link quality between two agents right now.

        {distance_m, quality, state, latency_ms, pdr}, or None if either agent
        is unknown. state is 'up' | 'degraded' | 'down'.

        THIS IS THE POINT OF THE WHOLE FRAMEWORK. A mission that reads this and
        changes behaviour when the link degrades is a resilient-control
        experiment; one that ignores it is just a path follower.
        """
        pa, pb = self.poses.get(a), self.poses.get(b)
        if pa is None or pb is None:
            return None
        return link_state(pa, pb)


def _call_mission(mod, agent, world):
    """Call a mission's target(), accepting either shape.

    target(agent, world)              <- write new missions this way
    target(agent, t, poses, arena)    <- the original four-argument form

    The old form still works so that nothing already written breaks, but it
    cannot see the lidar or the link, which is most of what makes a mission
    interesting. It is deprecated and will be removed once nothing uses it.
    """
    import inspect
    try:
        n = len(inspect.signature(mod.target).parameters)
    except (TypeError, ValueError):
        n = 2
    if n >= 4:
        if not getattr(mod, "_deadband_warned", False):
            mod._deadband_warned = True
            print(f"mission {mod.__name__}: target(agent, t, poses, arena) is "
                  f"deprecated - use target(agent, world); see "
                  f"docs/writing-a-mission.md", file=sys.stderr)
        return mod.target(agent, world.t, world.poses, world.arena)
    return mod.target(agent, world)




def load_mission_script(path):
    """Load a researcher's own mission file.

    The file defines one function:

        def target(agent, world):
            '''Return (x, y) - where this agent should be heading now.'''
            return (0.0, 0.0)

    `agent` is this agent's own configuration; `world` is everything it is
    allowed to know - see the World class above. Speed limits and collision
    still apply, so a script cannot cheat physics.

    The function imports no framework code and no rclpy. That is deliberate and
    it is the whole sim-to-real argument: the same file runs against this
    simulator today and against a real car tomorrow, because nothing in it
    knows which one it is talking to.
    """
    path = str(path)
    if path in _SCRIPTS:
        return _SCRIPTS[path]
    import importlib.util
    full = Path(path)
    if not full.is_absolute():
        full = REPO_ROOT / path
    spec = importlib.util.spec_from_file_location(f"mission_{full.stem}", full)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if not hasattr(mod, "target"):
        raise AttributeError(f"{full} has no target(agent, world)")
    _SCRIPTS[path] = mod
    return mod


def mission_target(agent, t, poses, arena):
    """Where the mission WANTS this agent to be at time t.

    Missions produce a target, not a position. The agent then moves toward that
    target at its own speed limit, and collision can stop it short. Separating
    the two is what fixes the phasing: previously the mission wrote a position
    directly every frame, so anything collision did was overwritten immediately
    and two agents in contact would jitter through each other.
    """
    m = agent["mission"]
    kind = m.get("type", "static")
    start = agent["start"]
    speed = max(agent["speed"], 0.05)
    hx = arena["extent"]["x"] / 2 - 0.6
    hy = arena["extent"]["y"] / 2 - 0.6

    if kind == "script":
        # A researcher's own file decides. Errors are reported, not swallowed:
        # a mission that throws should be obvious, not quietly become "static".
        try:
            mod = load_mission_script(m.get("file", ""))
            world = World(t, 1.0 / RATE_HZ, poses, arena, _ALL_AGENTS or [agent])
            xy = _call_mission(mod, agent, world)
            return (float(xy[0]), float(xy[1]))
        except Exception as exc:
            print(f"mission script failed for {agent['id']}: {exc}", file=sys.stderr)
            return (start["x"], start["y"])

    if kind == "shuttle":
        # An objective can name its endpoints as POINTS the map defines
        # (between: [A, B]) or give raw coordinates (from:/to:). Named points
        # are what make the objective portable: the same "shuttle between A and
        # B" runs on any map that defines A and B. Raw coordinates still work
        # for a one-off welded to this arena.
        pts = arena.get("points") or {}

        def _resolve(spec, fallback):
            if isinstance(spec, str):            # a point name like "A"
                p = pts.get(spec)
                if p is None:
                    print(f"objective for {agent['id']}: no point '{spec}' on "
                          f"this map", file=sys.stderr)
                    return fallback
                return (p["x"], p["y"])
            if isinstance(spec, dict):           # raw {x, y}
                return (_num(spec.get("x")), _num(spec.get("y")))
            return fallback

        between = m.get("between")
        if isinstance(between, (list, tuple)) and len(between) == 2:
            ax, ay = _resolve(between[0], (-hx, start["y"]))
            bx, by = _resolve(between[1], (hx, start["y"]))
        else:
            ax, ay = _resolve(m.get("from"), (-hx, start["y"]))
            bx, by = _resolve(m.get("to"), (hx, start["y"]))
        leg = math.hypot(bx - ax, by - ay) or 1.0
        period = 2.0 * leg / speed
        phase = ((t + _num(m.get("offset"))) % period) / period
        u = phase * 2.0 if phase < 0.5 else (1.0 - phase) * 2.0
        return (ax + (bx - ax) * u, ay + (by - ay) * u)

    if kind == "patrol":
        wps = m.get("waypoints") or [
            {"x": -hx, "y": -hy}, {"x": hx, "y": -hy},
            {"x": hx, "y": hy}, {"x": -hx, "y": hy}]
        pts = [(_num(w.get("x")), _num(w.get("y"))) for w in wps]
        segs = [(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]
        lengths = [math.dist(a, b) for a, b in segs]
        d = (t * speed + _num(m.get("offset"))) % (sum(lengths) or 1.0)
        for (p0, p1), L in zip(segs, lengths):
            if d <= L:
                u = d / (L or 1.0)
                return (p0[0] + (p1[0] - p0[0]) * u, p0[1] + (p1[1] - p0[1]) * u)
            d -= L
        return (start["x"], start["y"])

    if kind == "pursuit":
        tgt = poses.get(m.get("target"))
        if not tgt:
            return (start["x"], start["y"])
        gap = _num(m.get("standoff"), 1.2)
        bearing = tgt["yaw"] + math.pi
        return (tgt["x"] + gap * math.cos(bearing), tgt["y"] + gap * math.sin(bearing))

    if kind == "orbit":
        r = _num(m.get("radius"), 2.0)
        period = max(2.0 * math.pi * r / speed, 1.0)
        ang = 2.0 * math.pi * (t / period) + _num(m.get("phase"))
        return (r * math.cos(ang), r * math.sin(ang))

    return (start["x"], start["y"])


def blocked(agent, nx, ny, poses, agents, arena):
    """Would this agent, at (nx, ny), be inside a wall or another body?

    Returns the blocking reason or None. Agents marked 'ghost' in the scenario
    are furniture: they are drawn and they scatter lidar, but nothing collides
    with them, which is what the ground station wants to be for now.
    """
    r = agent["radius"]
    hx, hy = arena["extent"]["x"] / 2, arena["extent"]["y"] / 2
    b = arena.get("boundaries") or {}
    if b.get("x_min", "solid") == "solid" and nx - r < -hx:
        return "wall x_min"
    if b.get("x_max", "solid") == "solid" and nx + r > hx:
        return "wall x_max"
    if b.get("y_min", "solid") == "solid" and ny - r < -hy:
        return "wall y_min"
    if b.get("y_max", "solid") == "solid" and ny + r > hy:
        return "wall y_max"

    if agent.get("ghost"):
        return None
    az = poses[agent["id"]]["z"]
    atop = az + agent["dimensions"]["height"]
    for other in agents:
        if other["id"] == agent["id"] or other.get("ghost"):
            continue
        op = poses[other["id"]]
        # Different altitudes are not in contact however close the footprints.
        if op["z"] + other["dimensions"]["height"] < az or atop < op["z"]:
            continue
        if math.hypot(nx - op["x"], ny - op["y"]) < r + other["radius"]:
            return f"contact {other['id']}"
    return None


_ALL_AGENTS = []


def step(agents, poses, t, dt, arena):
    """Advance every agent one tick toward its mission target.

    Motion is speed-limited and collision BLOCKS it rather than displacing the
    other body: an agent that cannot move straight tries sliding along each axis
    in turn, and if neither works it stops. Nothing is ever teleported out of an
    overlap, so two agents in contact rest against each other instead of
    jittering. This is a kinematic constraint solver, not a physics engine —
    there is no momentum, restitution or contact force.
    """
    global _ALL_AGENTS
    _ALL_AGENTS = agents            # so a World built inside mission_target
                                    # can answer questions about every agent
    contacts = []
    for a in agents:
        p = poses[a["id"]]
        px0, py0 = p["x"], p["y"]
        if a["mission"].get("type", "static") == "static":
            p["speed"] = 0.0
            continue
        # Look one tick AHEAD. Without this the target advances at exactly the
        # agent's own speed, so the agent keeps catching it exactly and stopping
        # for a frame - which is what put the regular dropouts to zero in the
        # speed plot. A moving target must always be a step away.
        tx, ty = mission_target(a, t + dt, poses, arena)
        dx, dy = tx - p["x"], ty - p["y"]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            p["speed"] = 0.0
            continue
        stepd = min(dist, max(a["speed"], 0.05) * dt)
        ux, uy = dx / dist, dy / dist
        nx, ny = p["x"] + ux * stepd, p["y"] + uy * stepd

        why = blocked(a, nx, ny, poses, agents, arena)
        if why is None:
            p["x"], p["y"] = nx, ny
        else:
            contacts.append((a["id"], why))
            # Try sliding: one axis at a time, so a body against a wall can
            # still travel along it instead of sticking.
            if blocked(a, nx, p["y"], poses, agents, arena) is None:
                p["x"] = nx
            elif blocked(a, p["x"], ny, poses, agents, arena) is None:
                p["y"] = ny
        # Heading follows the direction of travel, not the target bearing.
        moved = math.hypot(p["x"] - px0, p["y"] - py0)
        p["speed"] = round(moved / dt, 4) if dt > 1e-6 else 0.0
        p["yaw"] = round(wrap_pi(math.atan2(uy, ux)), 4)
        p["x"], p["y"] = round(p["x"], 4), round(p["y"], 4)
    return contacts


# ---------------------------------------------------------------------------
# Lidar
#
# A real 2D raycast against the arena walls and the other agents. Two things it
# gets right that a naive version does not:
#
#   1. THE SCAN IS A PLANE. A lidar mounted 0.18 m up sees only what crosses
#      that height. A quadcopter at 2 m and a mast whose base is above the plane
#      are both invisible to it, and an occlusion model that ignores this
#      reports obstacles that are physically not there.
#   2. RANGE IS FINITE. Beyond range_max there is no return at all. That is not
#      the same as "a wall at range_max", so the two are distinguishable on the
#      wire and drawn differently.
#
# pose.z is the BASE of an agent — its ground contact — so an agent occupies
# [z, z + height]. The scan plane must fall inside that band to see it.
# ---------------------------------------------------------------------------

# UST-10LX, from the Hokuyo specification C-42-04077 (2015.03.02), which is in
# the project files. Every number here has that as its source.
#
# IT IS A PLANAR SCANNER, NOT A SPHERICAL ONE. The datasheet is unambiguous:
# "This sensor uses a laser source to scan 270 degree field of view", swept by a
# motor at 2400 rpm through 1081 steps in ONE plane. There is no vertical fan
# and no second axis. A single horizontal slice at the mount height is the
# correct model, and it is why an agent outside that slice is invisible.
LIDAR = {
    "fov_deg": 270.0,            # section 2-2 and section 4, Scan angle
    "steps": 1081,               # section 2-2, Measurement steps
    "angle_increment_deg": 0.25, # section 4, Angular resolution
    "range_min": 0.06,           # section 4, Detection range lower bound
    "range_max": 10.0,           # section 4, 10 m against a white Kent sheet
    "range_max_low_reflect": 4.0,# section 4, 4 m at 10% diffuse reflectance
    "accuracy_m": 0.040,         # section 4, +/-40 mm
    "repeatability_m": 0.030,    # section 4, sigma < 30 mm
    "scan_period_s": 0.025,      # section 4, 25 ms per scan (40 Hz)
    # Rays put on the wire. The sensor takes 1081 samples per scan; sending all
    # of them for every agent at the telemetry rate is a lot of JSON for a
    # picture a few hundred pixels wide, and the raycast cost is linear in this.
    # A display concern, deliberately separate from the sensor's real spec.
    "n_display": 271,
}

NO_RETURN = None           # explicit: the beam went out and nothing came back


def effective_range(reflectivity):
    """How far this lidar actually reaches against a surface of this reflectivity.

    The datasheet gives two points, not a curve: 10 m against a white Kent
    sheet, 4 m at 10% diffuse reflectance. Interpolating between them needs one
    physical assumption - that returned power falls as reflectivity over range
    squared, so usable range goes with the square root of reflectivity:

        range(rho) = 4.0 m * sqrt(rho / 0.10)     capped at the 10 m maximum

    That assumption is ours, not Hokuyo's, and it is the reason this function
    exists rather than the numbers being scattered through the code. It puts the
    white sheet at roughly 90% reflectance, which is about right for Kent paper.

    NEEDS A MEASUREMENT for any real room: point a car at the wall and record
    where the returns stop. That single number replaces the whole guess.
    """
    named = {"low": 0.10, "medium": 0.30, "high": 0.90}
    if isinstance(reflectivity, str):
        rho = named.get(reflectivity, 0.90)
    elif isinstance(reflectivity, (int, float)):
        rho = float(reflectivity)
        if rho > 1.0:
            rho /= 100.0                      # accept 30 as well as 0.30
    else:
        rho = 0.90
    rho = max(0.01, min(1.0, rho))
    return min(LIDAR["range_max"],
               LIDAR["range_max_low_reflect"] * math.sqrt(rho / 0.10))


def _ray_box(ox, oy, dx, dy, hx, hy):
    """Distance to the wall of an origin-centred box, or inf if it misses."""
    best = float("inf")
    for o, d, o2, d2, half, half2 in ((ox, dx, oy, dy, hx, hy),
                                      (oy, dy, ox, dx, hy, hx)):
        if abs(d) < 1e-9:
            continue
        for sign in (-1.0, 1.0):
            t = (sign * half - o) / d
            if t > 1e-6 and abs(o2 + t * d2) <= half2 + 1e-6:
                best = min(best, t)
    return best


def _ray_circle(ox, oy, dx, dy, cx, cy, r):
    ex, ey = ox - cx, oy - cy
    b = 2.0 * (ex * dx + ey * dy)
    c = ex * ex + ey * ey - r * r
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return float("inf")
    root = math.sqrt(disc)
    for t in ((-b - root) / 2.0, (-b + root) / 2.0):
        if t > 1e-6:
            return t
    return float("inf")


def scan_for(agent, sensor, poses, agents, arena, rng):
    """One lidar scan, from the sensor's actual mount height.

    Models what the datasheet specifies and nothing it does not:
      * a single horizontal plane at the mount height
      * returns closer than range_min are errors, not measurements
      * range depends on target reflectivity (10 m white, 4 m at 10% diffuse)
      * measurement noise at the stated accuracy
      * beyond range, an explicit no-return rather than a number
    """
    pose = poses[agent["id"]]
    plane_z = pose["z"] + sensor["offset"]["z"]
    ox, oy, yaw = pose["x"], pose["y"], pose["yaw"]
    hx = arena["extent"]["x"] / 2.0
    hy = arena["extent"]["y"] / 2.0
    walls_in_plane = plane_z <= arena["extent"]["z"]

    # Only obstacles whose vertical extent crosses the scan plane exist to it.
    obstacles = []
    for other in agents:
        if other["id"] == agent["id"]:
            continue
        p = poses[other["id"]]
        base, top = p["z"], p["z"] + other["dimensions"]["height"]
        if not (base - 1e-6 <= plane_z <= top + 1e-6):
            continue           # different plane: genuinely invisible
        radius = max(other["dimensions"]["length"],
                     other["dimensions"]["width"]) / 2.0
        obstacles.append((p["x"], p["y"], radius))

    n = LIDAR["n_display"]
    span = math.radians(LIDAR["fov_deg"])
    a_min = -span / 2.0
    sigma = LIDAR["accuracy_m"] / 2.0    # +/-40 mm read as a ~2-sigma bound
    wall_limit = effective_range(arena.get("surface_reflectivity"))

    ranges = []
    for i in range(n):
        a = yaw + a_min + span * i / (n - 1)
        dx, dy = math.cos(a), math.sin(a)

        # Effective range depends on what the beam hits. The datasheet gives
        # 10 m against a white Kent sheet and 4 m at 10% diffuse reflectance, so
        # the surface matters as much as the sensor. Wall reflectivity comes
        # from the arena and NEEDS A MEASUREMENT for any real room.
        r_wall = _ray_box(ox, oy, dx, dy, hx, hy) if walls_in_plane else float("inf")
        best = r_wall if r_wall <= wall_limit else float("inf")
        # Agents carry higher-reflectivity bodywork: the full range applies.
        for cx, cy, rad in obstacles:
            r = _ray_circle(ox, oy, dx, dy, cx, cy, rad)
            if r <= LIDAR["range_max"]:
                best = min(best, r)

        if best == float("inf") or best > LIDAR["range_max"]:
            ranges.append(NO_RETURN)
        elif best < LIDAR["range_min"]:
            ranges.append(NO_RETURN)      # inside the blind zone: an error code
        else:
            ranges.append(round(max(LIDAR["range_min"], best + rng.gauss(0, sigma)), 3))

    return {
        "frame": f"{agent['id']}/{sensor['id']}",
        "model": "UST-10LX",
        "plane_z": round(plane_z, 3),
        "angle_min": a_min,
        "angle_max": span / 2.0,
        "range_min": LIDAR["range_min"],
        "range_max": LIDAR["range_max"],          # datasheet best case, white target
        "range_effective": round(wall_limit, 2),  # what it actually reaches here
        "surface_reflectivity": arena.get("surface_reflectivity"),
        "steps_true": LIDAR["steps"],
        "ranges": ranges,
    }


def publications_for(agent):
    """What this agent puts on the wire. Namespaced /<id>/* per the design."""
    out = [
        {"topic": f"/{agent['id']}/odom", "type": "nav_msgs/Odometry", "rate_hz": 50.0},
        {"topic": f"/{agent['id']}/state", "type": "deadband/AgentState", "rate_hz": 10.0},
        {"topic": f"/{agent['id']}/speed", "type": "std_msgs/Float32", "rate_hz": 50.0},
    ]
    kinds = {"ust10lx": ("scan", "sensor_msgs/LaserScan", 40.0),
             "generic_imu": ("imu", "sensor_msgs/Imu", 200.0),
             "generic_gnss": ("navsat", "sensor_msgs/NavSatFix", 5.0)}
    for s in agent["sensors"]:
        k = kinds.get(s["type"])
        if k:
            out.append({"topic": f"/{agent['id']}/{k[0]}", "type": k[1], "rate_hz": k[2]})
    if agent["platform"] != "ground_station":
        out.append({"topic": f"/{agent['id']}/cmd",
                    "type": "deadband/AgentCommand", "rate_hz": 20.0})
    return out


def command_authority(agent, arena, links, poses, networks):
    """Who decides for this agent right now, and can it be reached?

    ARCHITECTURE is not a label - it is the answer to "when the link to whoever
    decides for me goes down, what happens?" Three architectures, three answers:

      centralized    One coordinator decides for everyone. If an agent cannot
                     reach the coordinator it has NO authority: it is on its
                     own, and what it does is set by `on_link_loss`. Brittle,
                     but optimal while the link holds - one node sees
                     everything.

      decentralized  Every agent decides for itself. Losing a link costs
                     information, never authority. Robust, but no agent has the
                     whole picture, so decisions are locally good and globally
                     mediocre.

      hierarchical   Agents answer to a squad leader; leaders answer to the
                     coordinator. Losing the top link leaves the squad still
                     commanded by its leader - degraded, not decapitated. The
                     middle ground, and the one worth measuring.

    Returns {"decider": <agent id or None>, "reachable": bool, "tier": str}.

    NOTE what this deliberately separates: DECISION AUTHORITY is not the same
    as network TOPOLOGY. A mesh network can carry centralized decision-making,
    and then the mesh survives a hub loss while the decision-making does not.
    That gap is the thing this framework is unusually able to show, because it
    models the comms as a first-class object rather than assuming them free.
    """
    net_name = agent.get("network")
    net = (networks or {}).get(net_name) or {}
    arch = net.get("architecture") or net.get("topology") or "centralized"
    aid = agent["id"]

    def _reaches(other):
        """Is there a usable link between aid and other, right now?"""
        if other is None or other == aid:
            return True
        for l in links:
            if {l["a"], l["b"]} == {aid, other}:
                return link_state(poses[l["a"]], poses[l["b"]])["state"] != "down"
        return False

    if arch == "decentralized":
        # Each agent is its own authority. Never decapitated.
        return {"decider": aid, "reachable": True, "tier": "self"}

    if arch == "hierarchical":
        leader = (agent.get("reports_to")
                  or net.get("squad_leader")
                  or net.get("coordinator"))
        if _reaches(leader):
            return {"decider": leader, "reachable": True, "tier": "leader"}
        # Leader unreachable: try the top-level coordinator before giving up.
        top = net.get("coordinator")
        if top != leader and _reaches(top):
            return {"decider": top, "reachable": True, "tier": "coordinator"}
        return {"decider": leader, "reachable": False, "tier": "orphaned"}

    # centralized (the default)
    hub = net.get("coordinator")
    return {"decider": hub, "reachable": _reaches(hub), "tier": "coordinator"}
def apply_routing(links, agents, networks, poses):
    """Decide which physically-reachable candidates are actually USED.

    Reachability says who CAN hear whom; routing says who DOES relay for whom.
    Keeping them separate is what makes topology measurable: you can declare
    mesh and then discover the physics only gave you a star's worth of edges.

      star    only hub<->member edges carry traffic. Two hops between any two
              non-hub agents, always via the hub.
      mesh    every reachable pair carries traffic. Multi-hop, routes around
              damage.
      tiered  intra-squad edges, plus leader<->coordinator. Cross-squad traffic
              climbs to a leader rather than going direct.

    Each link gets `active` (is it used by this routing?) and `usable` (is it
    physically up?). A link can be reachable but inactive - that is precisely
    the spare capacity a mesh has and a star does not, and it is what makes
    'route around damage' possible.
    """
    by_net = {}
    for a in agents:
        by_net.setdefault(a["network"], []).append(a["id"])

    squads_of = {}
    for name, net in (networks or {}).items():
        for sq, spec in ((net or {}).get("squads") or {}).items():
            leader = (spec or {}).get("leader")
            for m in list((spec or {}).get("members") or []) + ([leader] if leader else []):
                squads_of[m] = (sq, leader)

    out = []
    for l in links:
        net = (networks or {}).get(l["network"]) or {}
        routing = net.get("routing") or ("star" if net.get("coordinator") else "mesh")
        hub = net.get("coordinator")
        a, b = l["a"], l["b"]

        if routing == "mesh":
            active = True
        elif routing == "star":
            active = hub in (a, b)
        elif routing == "tiered":
            sa, la = squads_of.get(a, (None, None))
            sb, lb = squads_of.get(b, (None, None))
            same_squad = sa is not None and sa == sb
            leader_to_hub = hub in (a, b) and (a in (la, lb) or b in (la, lb))
            active = bool(same_squad or leader_to_hub)
        else:
            active = True

        state = rf_link(poses[a], poses[b])
        out.append({**l, **state,
                    "routing": routing,
                    "active": active,
                    "usable": state["state"] != "down"})
    return out


def observed_topology(links_out):
    """What the topology ACTUALLY is, measured, not what it was declared to be.

    Betweenness-style check: in a star the hub sits on essentially every path
    and scores near 1.0 while everyone else scores 0; in a mesh the load is
    spread. Reporting this next to the declared routing is what lets the tool
    say "you declared mesh, the graph says star" instead of taking the label's
    word for it.
    """
    active = [l for l in links_out if l.get("active") and l.get("usable")]
    nodes = sorted({n for l in active for n in (l["a"], l["b"])})
    if len(nodes) < 3:
        return {"nodes": len(nodes), "edges": len(active), "shape": "trivial",
                "max_betweenness": 0.0, "hub": None}

    adj = {n: set() for n in nodes}
    for l in active:
        adj[l["a"]].add(l["b"])
        adj[l["b"]].add(l["a"])

    # Count, for each node, how many shortest paths between OTHER pairs it lies
    # on. Small graphs, so brute-force BFS is fine and stays readable.
    from collections import deque
    on_path = {n: 0 for n in nodes}
    pairs = 0
    for s in nodes:
        # BFS shortest-path tree from s
        prev, dist = {s: []}, {s: 0}
        q = deque([s])
        while q:
            u = q.popleft()
            for v in adj[u]:
                if v not in dist:
                    dist[v] = dist[u] + 1
                    prev[v] = [u]
                    q.append(v)
                elif dist[v] == dist[u] + 1:
                    prev[v].append(u)
        for t in nodes:
            if t <= s or t not in dist:
                continue
            # walk back collecting intermediates
            seen, stack = set(), [t]
            while stack:
                u = stack.pop()
                for p in prev.get(u, []):
                    if p != s and p not in seen:
                        seen.add(p)
                        stack.append(p)
            if not seen:
                continue        # adjacent pair: no intermediate to credit
            # Only pairs that HAVE an intermediate can discriminate topology.
            # Counting adjacent pairs in the denominator caps a 4-node star at
            # 0.5 and makes it indistinguishable from a partial mesh.
            pairs += 1
            for n in seen:
                on_path[n] += 1

    maxb = max(on_path.values()) / pairs if pairs else 0.0
    hub = max(on_path, key=on_path.get) if pairs else None
    shape = "star" if maxb > 0.75 else ("mesh" if maxb < 0.35 else "mixed")
    return {"nodes": len(nodes), "edges": len(active), "shape": shape,
            "max_betweenness": round(maxb, 3), "hub": hub}


def rf_link(pa, pb, tx_dbm=20.0, freq_mhz=2400.0, plexp=2.8,
            noise_dbm=-95.0, interference_mw=0.0, sensitivity_dbm=-85.0):
    """Signal-to-interference-plus-noise for one pair, and what it implies.

    SINR is the single currency. Distance, walls and jamming all reduce to
    signal-versus-noise, which becomes packet delivery ratio, which becomes link
    state. Nothing gets special-cased: a jammer is just another term in the
    denominator, which is what lets a jammer be an ordinary agent rather than a
    global flag bolted onto the side.

    Log-distance path loss:  PL(d) = PL(d0) + 10 * n * log10(d/d0)
    with n the path loss exponent (2.0 free space, 2.7-3.5 indoor with
    multipath). d0 = 1 m reference, computed from the Friis free-space loss at
    the carrier frequency.

    NOTE ON PROVENANCE: plexp and noise_dbm are the scene's to declare and are
    currently unsourced in every scene - they are on the measurements list.
    Until then these are defensible defaults, NOT measurements, and any result
    that turns on their exact value has to say so.
    """
    d = max(0.1, math.dist((pa["x"], pa["y"], pa["z"]),
                           (pb["x"], pb["y"], pb["z"])))
    # Friis at 1 m: 20log10(f_MHz) + 20log10(d_km) + 32.44, with d = 0.001 km
    pl_d0 = 20.0 * math.log10(freq_mhz) + 20.0 * math.log10(0.001) + 32.44
    path_loss = pl_d0 + 10.0 * plexp * math.log10(d)
    rx_dbm = tx_dbm - path_loss

    # Noise plus any interference, summed in linear power then back to dB.
    noise_mw = 10.0 ** (noise_dbm / 10.0)
    total_mw = noise_mw + max(0.0, interference_mw)
    effective_noise_dbm = 10.0 * math.log10(total_mw)
    sinr_db = rx_dbm - effective_noise_dbm

    # PDR from SINR with a logistic curve: near 0 well below threshold, near 1
    # well above, with a few dB of transition. A stand-in for a modulation
    # curve, honest about being one.
    margin = sinr_db - (sensitivity_dbm - noise_dbm)
    pdr = 1.0 / (1.0 + math.exp(-0.8 * margin))
    state = "up" if pdr > 0.85 else ("degraded" if pdr > 0.25 else "down")
    return {"distance_m": round(d, 3), "rx_dbm": round(rx_dbm, 2),
            "sinr_db": round(sinr_db, 2), "pdr": round(pdr, 3),
            "quality": round(pdr, 3), "state": state,
            "latency_ms": round(8.0 + 40.0 * (1.0 - pdr), 2)}


def link_state(pa, pb):
    """Link quality between two poses, from the SINR model.

    Kept as a thin wrapper because several callers want "just tell me if this
    pair can talk" without assembling radio parameters. Callers that DO have
    the radio and interference picture should call rf_link directly.
    """
    return rf_link(pa, pb)


def _unused_enforce_bounds(agents, poses, arena):
    """Walls and other agents are solid.

    This is a KINEMATIC CONSTRAINT, not a physics engine: a mission path that
    would leave the room or overlap another agent is clipped back to the nearest
    legal position, and no momentum, restitution or contact force is modelled.
    That is the honest level for a stub. What it does guarantee is that no agent
    is ever reported somewhere it could not physically be, which is what makes
    the lidar and the link distances trustworthy.
    """
    hx = arena["extent"]["x"] / 2.0
    hy = arena["extent"]["y"] / 2.0
    b = arena.get("boundaries") or {}
    radii = {a["id"]: max(a["dimensions"]["length"],
                          a["dimensions"]["width"]) / 2.0 for a in agents}
    contacts = []

    # Walls, for boundaries that are actually solid.
    for a in agents:
        p, r = poses[a["id"]], radii[a["id"]]
        if b.get("x_min", "solid") == "solid" and p["x"] < -hx + r:
            p["x"] = -hx + r; contacts.append((a["id"], "wall x_min"))
        if b.get("x_max", "solid") == "solid" and p["x"] > hx - r:
            p["x"] = hx - r; contacts.append((a["id"], "wall x_max"))
        if b.get("y_min", "solid") == "solid" and p["y"] < -hy + r:
            p["y"] = -hy + r; contacts.append((a["id"], "wall y_min"))
        if b.get("y_max", "solid") == "solid" and p["y"] > hy - r:
            p["y"] = hy - r; contacts.append((a["id"], "wall y_max"))
        if p["z"] < 0.0:
            p["z"] = 0.0

    # Agent against agent, only where they share vertical extent. Two agents at
    # different altitudes are not in contact however close their footprints.
    ids = [a["id"] for a in agents]
    heights = {a["id"]: a["dimensions"]["height"] for a in agents}
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            pa, pb = poses[ids[i]], poses[ids[j]]
            if pa["z"] + heights[ids[i]] < pb["z"] or pb["z"] + heights[ids[j]] < pa["z"]:
                continue
            dx, dy = pb["x"] - pa["x"], pb["y"] - pa["y"]
            d = math.hypot(dx, dy)
            need = radii[ids[i]] + radii[ids[j]]
            if d >= need:
                continue
            if d < 1e-6:
                dx, dy, d = 1.0, 0.0, 1.0
            push = (need - d) / 2.0
            ux, uy = dx / d, dy / d
            pa["x"] -= ux * push; pa["y"] -= uy * push
            pb["x"] += ux * push; pb["y"] += uy * push
            contacts.append((ids[i], f"contact {ids[j]}"))
    return contacts


def frame(t, dt, seq, arena, agents, links, poses, rng):
    """One telemetry frame. THIS DICTIONARY IS THE CONTRACT."""
    contacts = step(agents, poses, t, dt, arena)

    agents_out = []
    for a in agents:
        lidars = [s for s in a["sensors"] if s["type"] == "ust10lx"]
        agents_out.append({
            "id": a["id"],
            "platform": a["platform"],
            "network": a["network"],
            "colour": a["colour"],
            "mission": a["mission"].get("type", "static"),
            # The FULL objective, not just its type, so the Console can show
            # "pursue car3" rather than a bare "pursuit" - and so a retask is
            # visible in the tree the moment it takes effect.
            "objective": a["mission"],
            # Who decides for this agent right now, and whether they are
            # reachable. This is what makes 'architecture' a behaviour rather
            # than a label in a file.
            "authority": command_authority(a, arena, links, poses,
                                           arena.get("networks") or {}),
            "dimensions": a["dimensions"],
            "pose": poses[a["id"]],
            "scan": scan_for(a, lidars[0], poses, agents, arena, rng) if lidars else None,
            "publishes": publications_for(a),
            "sensors": [
                {"id": s["id"], "type": s["type"], "offset": s["offset"], "ok": True,
                 "summary": {"rate_hz": 40.0 if s["type"] == "ust10lx" else 200.0}}
                for s in a["sensors"]
            ],
            "health": {"ok": True,
                       "warnings": [w for i, w in contacts if i == a["id"]]},
        })

    # Keep the scans so the next tick's missions can read them. See _LAST_SCANS.
    _LAST_SCANS.clear()
    _LAST_SCANS.update({a["id"]: a["scan"] for a in agents_out if a["scan"]})

    links_out = apply_routing(links, agents, arena.get("networks") or {}, poses)

    return {
        "seq": seq,
        "sim_time_s": round(t, 3),
        "wall_time": time.time(),
        "run_state": "running",
        "arena": arena,
        "agents": agents_out,
        "links": links_out,
        # What the topology MEASURES as, independent of what it was declared
        # to be. The gap between this and the declared routing is the finding.
        "topology": observed_topology(links_out),
        "attacks_active": [],
        "contacts": [{"agent": i, "with": w} for i, w in contacts],
    }


def parse_retask(text, agents_by_id):
    """Turn a line like 'car3: pursue car1' into a new objective dict.

    The grammar is deliberately the same words a person would say out loud:

        car3: pursue car1
        car3: shuttle between E F
        car3: wall_follow right
        car3: stop                 (alias for static - hold position)
        car3: script missions/return_on_link_loss.py

    Returns (agent_id, mission_dict) or None if it does not parse. Kept
    forgiving on purpose: a fat-fingered command should be ignored with a note,
    never crash a running mission.
    """
    if ":" not in text:
        return None
    aid, rest = text.split(":", 1)
    aid, parts = aid.strip(), rest.split()
    if aid not in agents_by_id or not parts:
        return None
    verb, args = parts[0].lower(), parts[1:]

    if verb in ("stop", "static", "hold"):
        return aid, {"type": "static"}
    if verb == "pursuit" or verb == "pursue":
        return aid, {"type": "pursuit", "target": args[0] if args else "car1"}
    if verb == "shuttle":
        # 'shuttle between A B' or 'shuttle A B'
        pts = [p for p in args if p.lower() != "between"]
        if len(pts) >= 2:
            return aid, {"type": "shuttle", "between": [pts[0], pts[1]]}
        return aid, {"type": "shuttle"}
    if verb in ("wall_follow", "wall"):
        return aid, {"type": "script", "file": "missions/wall_follow.py",
                     "side": args[0] if args else "right"}
    if verb == "orbit":
        return aid, {"type": "orbit",
                     "radius": float(args[0]) if args else 2.0}
    if verb == "script":
        return aid, {"type": "script", "file": args[0]} if args else None
    # Bare verb we do not know: report it, change nothing.
    print(f"retask: don't understand '{verb}' for {aid}", file=sys.stderr)
    return None


def drain_retasks(retask_dir, agents_by_id):
    """Apply any pending retask commands and return a list of what changed.

    A command is one line in a file dropped into retask_dir (or appended to
    retask_dir/queue). Reading a file consumes it, so a command fires once.
    File-based rather than a socket because the same channel then works from a
    terminal (`echo 'car3: pursue car1' > retask/queue`), from the Console, and
    later from a ROS service, with no protocol to agree on.
    """
    changed = []
    if not retask_dir.exists():
        return changed
    queue = retask_dir / "queue"
    lines = []
    if queue.exists():
        try:
            lines = queue.read_text(encoding="utf-8").splitlines()
            queue.unlink()                    # consume: each command fires once
        except OSError:
            pass
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        result = parse_retask(line, agents_by_id)
        if result:
            aid, block = result
            agents_by_id[aid]["mission"] = block
            changed.append((aid, block))
    return changed


def stream(arena, agents, links, duration=None, out=sys.stdout, seed=1,
           retask_dir=None):
    import random
    rng = random.Random(seed)          # seeded, so a run is reproducible

    agents_by_id = {a["id"]: a for a in agents}

    # Persistent state. This is what makes collision behave: an agent's pose is
    # carried from tick to tick and only ever changed by a legal move.
    poses = {a["id"]: {"x": a["start"]["x"], "y": a["start"]["y"],
                       "z": a["start"]["z"], "yaw": a["start"]["yaw"],
                       # Speed is REPORTED by the agent, the way a real
                       # odometry topic reports it, rather than being
                       # reconstructed downstream by differencing positions.
                       "speed": 0.0}
             for a in agents}

    seq, t0 = 0, time.time()
    period = 1.0 / RATE_HZ
    last = 0.0
    try:
        while True:
            t = time.time() - t0
            if duration is not None and t > duration:
                return
            # Live retasking: an agent's objective can be replaced mid-run by a
            # command on the channel. Because mission_target reads the agent's
            # mission dict every tick, swapping that dict here is all it takes -
            # the very next frame the agent is doing the new thing.
            if retask_dir is not None:
                for aid, block in drain_retasks(retask_dir, agents_by_id):
                    print(f"retask: {aid} -> {block.get('type')} "
                          f"{block.get('target') or block.get('between') or block.get('file') or ''}",
                          file=sys.stderr)
            dt, last = t - last, t
            out.write(json.dumps(
                frame(t, dt, seq, arena, agents, links, poses, rng)) + "\n")
            out.flush()
            seq += 1
            time.sleep(period)
    except (KeyboardInterrupt, BrokenPipeError):
        pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Deadband stub telemetry source")
    ap.add_argument("--scenario", default=str(DEFAULT_SCENARIO))
    ap.add_argument("--record", type=float, metavar="SECONDS")
    ap.add_argument("--retask", metavar="DIR", default=None,
                    help="watch DIR/queue for live retask commands, e.g. "
                         "echo 'car3: pursue car1' > DIR/queue")
    args = ap.parse_args()

    path = Path(args.scenario)
    if not path.exists():
        sys.exit(f"scenario not found: {path}")
    arena, agents, links = load_scenario(path)

    retask_dir = Path(args.retask) if args.retask else None
    if retask_dir:
        retask_dir.mkdir(parents=True, exist_ok=True)

    if args.record:
        with open("sample_telemetry.jsonl", "w") as fh:
            stream(arena, agents, links, duration=args.record, out=fh)
        print(f"wrote sample_telemetry.jsonl ({args.record}s)")
    else:
        stream(arena, agents, links, retask_dir=retask_dir)
