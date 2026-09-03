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
import os
import re
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SCENARIO = REPO_ROOT / "default_run.yaml"
RATE_HZ = 10.0

# Keeps a shuttle lane off the literal wall. Shared by mission_target()'s
# default endpoints AND validate_objective()'s bounds check, so "in bounds"
# means the same thing everywhere in this file - see docs/PATCH-07-CHECKS.md.
WALL_MARGIN_M = 0.6

# --- GNSS / dead-reckoning drift model. See docs/jamming-effects-research.md
# and docs/gnss-drift-model.md. Every figure here is a declared free parameter
# with a source; none is invented.
# LINK-LOSS DOCTRINE. What a vehicle does when it cannot reach whoever
# commands it. This is a DOCTRINAL choice, not a physical one, so it is a
# declared field on the agent rather than a rule baked into the simulator.
#
#   hold      Freeze until the link returns. The subordinate has no orders it
#             is authorised to act on, so it stops. This is the conservative
#             default and it is what most small-UAS/UGV autopilot failsafes
#             actually do.
#
#   intent    Keep executing the objective already assigned, from the picture
#             the vehicle already has, without further direction. This is
#             MISSION COMMAND, and it is NATO's stated philosophy of command:
#             "centralized planning that includes provision of clear guidance
#             and intent combined with DECENTRALIZED EXECUTION based on
#             mission-type orders and disciplined initiative; describing the
#             'what', without necessarily prescribing the 'how'"
#             (AJP-3 Edition D Version 1, para 3.8). The subordinate's
#             authority to execute was delegated when the objective was
#             issued, so losing the link costs INFORMATION, not AUTHORITY:
#             "subordinates should decide and act within the scope of the
#             commander's intent" (ibid., para 3.11).
#
#             Note what makes this measurable rather than a label: an intent
#             vehicle steers from its OWN belief and from position reports
#             that have gone stale (see update_knowledge). It keeps working,
#             and it keeps getting more wrong. That trade is the finding.
#
#   continue  Accepted as a legacy alias for `intent`. It named the same
#             mechanism before the doctrine had a source; the two are
#             identical for a cyclic objective. They would diverge for a
#             TERMINATING objective, where mission command requires the
#             subordinate to seek new direction once the assigned effect is
#             achieved rather than invent a next task - not yet modelled.
#
# See docs/jamming-model-justification.md and SOURCES.md.
LINK_LOSS_KEEPS_GOING = {"intent", "continue"}


def keeps_going_on_link_loss(agent):
    """Does this agent's doctrine let it act without a reachable commander?"""
    return (agent.get("on_link_loss") or "hold").lower() in LINK_LOSS_KEEPS_GOING


GNSS_BAND_MHZ = 1575.42          # GPS L1 - the band a GNSS jammer occupies
# Jammer power (received, dBm) above which an agent loses its GNSS fix. GNSS
# signals arrive at about -128 dBm, so a jammer only tens of dB stronger denies
# them ("GNSS signals are weak and easily jammed", Critical Analysis of
# Spoofing and Jamming). -120 dBm is a defensible denial threshold, NOT a
# measurement.
GNSS_DENIAL_DBM = -120.0
# Dead-reckoning drift as a FRACTION OF DISTANCE TRAVELLED. 0.04 (4%) is the
# UAV Navigation VECTOR autopilot figure for pure MEMS inertial (~33 m/min);
# with visual aiding it falls to ~0.01 (1%). Source: UAV Navigation, "Dead
# Reckoning Operations".
DRIFT_RATE_UNAIDED = 0.04
DRIFT_RATE_AIDED = 0.01
# A sensor that gives an RF-immune position fix (lidar/vision localisation)
# greatly reduces drift but does NOT remove it: scan-matching and loop-closure
# error accumulate too. UAV Navigation measured ~1% of distance with their
# Visual Navigation System in UNKNOWN terrain, and "no measurable drift" only
# against KNOWN terrain (a surveyed map). 1% is the honest default; a scene
# that declares a surveyed map could justify less. Zero was an over-claim.
DRIFT_RATE_LIDAR = 0.01
# Heading of the accumulating error random-walks; this is its per-tick sigma
# (rad). Free parameter - it sets how the error meanders, not how fast it grows.
DRIFT_TURN_SIGMA = 0.15
# When a fix returns, the estimate is pulled back to truth at this rate (m/s),
# not snapped - matches ArduPilot's EKF offset correction "reducing at 1 m/s".
GNSS_RECOVER_MPS = 1.0
# --- Self-interference: a fleet degrading its OWN comms as it grows.
# The README's "8 or more drones start jamming each other". In a coordinated
# fleet this is NOT an SINR problem - the radios share a medium-access
# protocol, so they take turns rather than shouting over each other. (An
# external jammer is different precisely because it IGNORES the protocol -
# that is what makes it a jammer.) What degrades is AIRTIME.
#
# The model uses no invented penalty coefficient. Two grounded steps:
#   1. AIRTIME SHARE. n contenders sharing one channel each get 1/(n+1) of it,
#      so the delay to get a packet out scales by (n+1). That is arithmetic,
#      not a fitted constant.
#   2. DEADLINE MISSES. A packet is not "lost" by contention, it is DELAYED -
#      it only counts as lost if it misses the deadline the network declares
#      in its own QoS block (`deadline_ms`, already in the schema). Service
#      time is taken as exponential (the standard M/M/1 queueing assumption),
#      so the fraction arriving in time is 1 - exp(-deadline / delay).
# Consequence worth noting: a network that declares a lax deadline tolerates
# a crowded channel; a tight real-time deadline does not. That is a real
# design trade and now an experimental variable.
DEFAULT_DEADLINE_MS = 100.0     # used only if a network declares none

# --- Adjacent Channel Interference (ACI).
# Band separation was a BINARY test: within 0.5 MHz = full interference, else
# none. Real receivers are not that clean - a strong nearby transmitter leaks
# into neighbouring channels through imperfect filters, and a close UAV can
# swamp a distant one on an adjacent channel (the near-far problem).
#
# Zhou, Chen, Hong, Jin & Shi, "Joint Channel Assignment and Power Allocation
# for Multi-UAV Communication" (arXiv:2008.08212, 2020) model this with an
# interference correlation coefficient mu(f1,f2) with exactly three properties:
#   0 <= mu <= 1,  mu is symmetric,
#   mu = 1  when |f1 - f2| = 0        (same channel: full interference)
#   mu -> 0 when |f1 - f2| -> infinity (well separated: none)
# and note mu "is proportional to the intensity of ACI and can be measured in
# practical systems". We adopt that STRUCTURE and parameterise the rolloff by
# adjacent-channel rejection, which is a real receiver spec: 30 dB of rejection
# one channel away is a typical commodity figure. The exact curve is a declared
# free parameter; the shape is the paper's.
CHANNEL_BW_MHZ = 20.0           # nominal channel width
ACR_DB_PER_CHANNEL = 30.0       # adjacent-channel rejection, one channel away


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
# Coordinate grammar for the terminal: (x,y) or (x,y,z), field-grid style.
# ---------------------------------------------------------------------------

_POS_TOKEN_RE = re.compile(r"\([^)]*\)|\S+")


def _tokenize_args(text):
    """Split a retask argument string into tokens, keeping a parenthesized
    coordinate like '(-3, 3, 0)' as ONE token even though it has spaces in
    it. Anything else splits on whitespace exactly like str.split() would.
    """
    return _POS_TOKEN_RE.findall(text)


def _parse_position_token(tok):
    """A bare word is a point name; '(x,y[,z])' is an absolute literal.

    Returns the token unchanged (str) for a point name - resolved later
    against the map's points - or a {"x","y","z"} dict for a literal, with z
    defaulting to 0.0 when only two numbers are given. Raises ValueError on
    a malformed parenthesized token, so the caller rejects the whole command
    rather than silently mis-parsing it.
    """
    if tok.startswith("("):
        inner = tok.strip("()")
        parts = [p.strip() for p in inner.split(",")]
        if len(parts) not in (2, 3):
            raise ValueError(f"'{tok}' needs (x,y) or (x,y,z)")
        try:
            x, y = float(parts[0]), float(parts[1])
            z = float(parts[2]) if len(parts) == 3 else 0.0
        except ValueError:
            raise ValueError(f"'{tok}' is not a valid coordinate")
        return {"x": x, "y": y, "z": z}
    return tok


# ---------------------------------------------------------------------------
# Scenario
# ---------------------------------------------------------------------------

def _load_yaml(path):
    import yaml
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _base_path(kind, ref):
    """Turn a bare base name into a path. A fleet name resolves under fleets/,
    a scene name under scenes/ (maps/ still searched for older files). A path
    with a suffix or directory is taken as given."""
    p = Path(ref)
    if not p.suffix:
        p = p.with_suffix(".yaml")
    if p.is_absolute():
        return p
    if p.parent != Path("."):
        return REPO_ROOT / p
    if kind == "fleet":
        # fleets/ is USER OUTPUT: it holds only fleets somebody built and saved
        # in the Console. The test suite needs fixed inputs, and putting those
        # in fleets/ would make them appear in the operator's dropdown as
        # fleets they did not make - which is exactly the junk-file problem
        # this layout exists to remove. So fixtures live apart and are found
        # here as a fallback.
        for folder in ("fleets", "tests/fixtures/fleets"):
            cand = REPO_ROOT / folder / p
            if cand.exists():
                return cand
        return REPO_ROOT / p
    # kind == "scene"
    cand = REPO_ROOT / "scenes" / p
    return cand if cand.exists() else (REPO_ROOT / "maps" / p)


def _overlay(base, doc):
    """Overlay one layer (`doc`) onto a resolved base dict.

    One merge rule serves every layer of scene < fleet < mission: the upper
    layer's top-level keys win, EXCEPT agents, which are merged per-id so the
    lower layer keeps each agent's body and the upper layer supplies whatever it
    adds (a fleet introduces the agents onto a bare scene; a mission adds
    objectives). A top-level `objectives:` map keyed by agent id is translated
    into each agent's `mission:` block, so a file can speak in objectives while
    the machinery underneath is unchanged (`do:` becomes `type:`).

    Agents the upper layer names that the base does not have are surfaced
    loudly rather than dropped: a fleet defining agents on an agent-less scene
    is the normal case (kept, silent); a mission naming an agent its fleet does
    not have is almost always a typo (kept, warned).
    """
    merged = dict(base)
    for key, val in doc.items():
        if key in ("map", "scene", "fleet", "fleets", "agents"):
            continue
        # Dict-valued sections that COLLECT across layers (a blue fleet and a
        # red fleet each contribute their own networks; neither should erase
        # the other). Union by key, upper layer wins a genuine clash.
        if key in ("networks", "radios", "points") and isinstance(val, dict) \
                and isinstance(merged.get(key), dict):
            # Union by key - AND one level deeper for each entry, so a layer
            # can override ONE property of a network without restating it.
            # This is what makes a Console override layer possible at all:
            # the Setup tab writes {"networks": {"blue": {"routing": "mesh"}}}
            # and blue keeps its coordinator, band and squads. A shallow merge
            # replaced the whole network dict and silently dropped them.
            out = dict(merged[key])
            for k, v in val.items():
                if isinstance(v, dict) and isinstance(out.get(k), dict):
                    out[k] = {**out[k], **v}
                else:
                    out[k] = v
            merged[key] = out
        else:
            merged[key] = val

    base_agents = {a.get("id"): a for a in (base.get("agents") or [])}
    doc_agents = {a.get("id"): a for a in (doc.get("agents") or [])}
    # Agents the doc introduces via its own `agents:` list are legitimate
    # (a fleet defining its side); only a bare `objectives:` entry with no
    # body anywhere is a phantom worth warning about.
    bodied = set(base_agents) | set(doc_agents)

    for aid, obj in (doc.get("objectives") or {}).items():
        if not isinstance(obj, dict):
            continue
        block = {("type" if k == "do" else k): v for k, v in obj.items()}
        block.setdefault("type", "static")
        doc_agents.setdefault(aid, {"id": aid})["mission"] = block

    out_agents = []
    seen = set()
    for aid, body in base_agents.items():
        agent = dict(body)
        task = doc_agents.get(aid)
        if task:
            for k, v in task.items():
                if k == "id":
                    continue
                agent[k] = v
        out_agents.append(agent)
        seen.add(aid)
    for aid, task in doc_agents.items():
        if aid in seen:
            continue
        # A phantom = named only by objectives, with a body nowhere.
        if base_agents and aid not in bodied:
            print(f"mission names agent '{aid}' with no body - ignoring",
                  file=sys.stderr)
        out_agents.append(dict(task, id=aid))
    merged["agents"] = out_agents
    return merged


def resolve_mission(path):
    """Read a file and merge in whatever it sits on, producing one flat dict
    shaped exactly like the old self-contained scenario.

    Three layers (see docs/vocabulary.md), composed by the top one:

        scene    - the world only: arena, radio medium, points. Terminal.
        fleet    - the agents: bodies, sensors, radios, networks, authority.
                   Carries no scene of its own.
        mission  - the command: names `scene:` AND `fleet:`, then issues
                   objectives (or a fleet-wide order at runtime).

    Merge order is scene, then fleet, then the file itself, so a mission can
    nudge either lower layer without editing it. A file that names no base is
    self-contained - the old shape, world+fleet+tasking in one file - and is
    returned unchanged, so every legacy file still loads. `map:` remains a
    silent alias for `scene:`. Everything downstream receives one merged dict,
    so nothing else in the pipeline had to change.
    """
    return resolve_doc(_load_yaml(path))


def resolve_doc(doc):
    """resolve_mission for an in-memory dict - the Console's Setup tab
    composes a run as {scene: ..., fleet: ..., agents: [pose overrides]}
    without a file ever existing."""
    scene_ref = doc.get("scene") or doc.get("map")
    # One `fleet:` or a `fleets:` list (blue + red spawned as separate sides,
    # composed by the Setup tab). Overlaid in listed order onto the scene.
    fleet_refs = doc.get("fleets")
    if fleet_refs is None:
        fleet_refs = [doc["fleet"]] if doc.get("fleet") else []
    if not scene_ref and not fleet_refs:
        return doc                          # self-contained: the old shape
    base = {}
    if scene_ref:
        base = resolve_mission(_base_path("scene", scene_ref))
    for fref in fleet_refs:
        base = _overlay(base, resolve_mission(_base_path("fleet", fref)))
    return _overlay(base, doc)


def load_scenario(path):
    """Accepts a path, or an already-composed dict (the Setup flow)."""
    doc = resolve_doc(dict(path)) if isinstance(path, dict) \
        else resolve_mission(path)

    arena = doc.get("arena") or {}
    extent = arena.get("extent") or {}

    # lat/lon/alt of this scene's local (0,0,0) - a schema seam only, no
    # conversion happens yet. See docs/maps-missions-and-retasking.md.
    origin_doc = arena.get("origin")
    origin = None
    if origin_doc:
        origin = {"lat": _num(origin_doc.get("lat")),
                  "lon": _num(origin_doc.get("lon")),
                  "alt": _num(origin_doc.get("alt")),
                  "heading": _num(origin_doc.get("heading"))}

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
        # The scene's contested-environment BASELINE (propagation, spectrum,
        # gnss, wind), carried whole so the RF model reads the SCENE's
        # numbers, not function defaults. docs/contested-background.md.
        "background": {k: arena[k] for k in
                       ("propagation", "spectrum", "gnss", "wind")
                       if k in arena},
        "origin": origin,
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
            # Assigning an objective never arms it - only LAUNCH/HALT do. See
            # "The state machine" in docs/PATCH-07-CHECKS.md.
            "armed": False,
            "last_rejection": None,
            # The sim time this agent's CURRENT objective became active -
            # reset on every (re)launch and every (re)assignment. shuttle/
            # patrol/orbit phase is measured from t - phase_t0, never from
            # raw absolute t, so an agent armed at t=30s starts its cycle
            # cleanly from that moment instead of jumping to wherever a
            # 30-second-old clock would put it. See "Bug fix: the launch
            # hiccup" in docs/PATCH-07-CHECKS.md.
            "phase_t0": 0.0,
            # POSITION ESTIMATE. `belief` is where the agent THINKS it is;
            # the pose in `poses` is the truth. They agree while the agent has
            # a fix (GNSS or a lidar/vision loop); under GNSS denial the belief
            # dead-reckons and DRIFTS. `drift` is the accumulated error vector
            # (belief - truth), `drift_dir` the heading it is currently
            # accumulating along. See step() and docs/gnss-drift-model.md.
            "belief": {"x": _num(pose.get("x")), "y": _num(pose.get("y"))},
            # WHAT THIS AGENT KNOWS ABOUT THE OTHERS. Not ground truth - each
            # entry is the other agent's OWN BELIEF about itself, as reported
            # over the network, and it only updates while a route exists. Lose
            # the link and the entry goes STALE: you keep acting on where they
            # last said they were. Missions read this, never the true poses.
            "knowledge": {},
            "drift": {"x": 0.0, "y": 0.0},
            "drift_dir": 0.0,
            # What this agent does when it cannot reach its commander: hold
            # (freeze, default) or continue. Read by step(). A real field on
            # the fleet already (lab cars declare on_link_loss: hold).
            "on_link_loss": a.get("on_link_loss", "hold"),
            "ghost": bool(a.get("ghost", False)),
            "speed": _num((a.get("performance") or {}).get("max_speed"), 1.5),
            # VEHICLE DYNAMICS. Until now an agent reached full speed in one
            # tick and its heading TELEPORTED to the direction of travel - so
            # a shuttling car "turned round" instantly and its forward-mounted
            # lidar snapped with it. Both are now rate-limited from the
            # performance block the schema always had:
            #   max_accel        m/s^2 - how fast it can change speed
            #   min_turn_radius  m     - tightest arc; yaw rate = v / R
            # MOTION MODEL by platform: a car is ACKERMANN (it can only drive
            # along its heading, so it must arc round or reverse); a
            # quadcopter is HOLONOMIC (it can translate any direction and its
            # yaw is independent of travel).
            "max_accel": _num((a.get("performance") or {}).get("max_accel"),
                              2.0),
            "turn_radius": _num((a.get("performance") or {})
                                .get("min_turn_radius"), 0.6),
            "motion": (a.get("motion")
                       or ("holonomic"
                           if a.get("platform") in ("quadcopter", "drone",
                                                    "multirotor")
                           else "ackermann")),
            # A car may reverse rather than execute a U-turn. Reversing keeps
            # the heading (and therefore points a forward lidar BACKWARDS -
            # a real sensing consequence, surfaced as `reversing` per frame).
            "can_reverse": bool(a.get("can_reverse", True)),
            # A jammer is an ORDINARY agent that happens to transmit noise:
            # {tx_power, band} quantities. Armed = transmitting - the same
            # LAUNCH/HALT state machine as everything else, so `red launch`
            # is what turns the jamming on.
            "jammer": a.get("jammer"),
        })

    for a in agents:
        # One collision radius per agent, from its own footprint.
        a["radius"] = max(a["dimensions"]["length"], a["dimensions"]["width"]) / 2.0

    # A scene or a legacy self-contained scenario can bake a mission straight
    # onto an agent. Validate it exactly like a live REOBJECTIVE would - an
    # unresolvable or out-of-bounds shuttle at load time gets rejected loudly
    # and the agent holds static, rather than silently doing the wrong thing
    # for the whole run. See "Bug fix" in docs/PATCH-07-CHECKS.md.
    for a in agents:
        ok, err = validate_objective(a["mission"], world["points"], world)
        if not ok:
            print(f"{a['id']}: initial objective rejected - {err} - "
                  f"holding static", file=sys.stderr)
            a["last_rejection"] = err
            a["mission"] = {"type": "static"}

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

# The mission NAME set by the last successful SETMISSION, carried into every
# frame so results and bags can be titled by mission. None until one is set.
CURRENT_MISSION = {"name": None}


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


def resolve_waypoint(spec, points):
    """Turn a waypoint spec into (ok, x, y, z, error). No fallback, ever.

    `spec` is a point name (str, looked up in `points`) or a {x, y[, z]}
    dict - a literal coordinate, from YAML or the retask grammar's
    `(x,y[,z])` parser. This is the fix for the bug where an unresolved
    point silently substituted the arena half-width: the caller decides
    what "no valid waypoint" means, and it is never this function's job to
    make one up.
    """
    if isinstance(spec, str):
        p = (points or {}).get(spec)
        if p is None:
            return False, 0.0, 0.0, 0.0, f"no point '{spec}' on this map"
        return True, _num(p.get("x")), _num(p.get("y")), _num(p.get("z")), None
    if isinstance(spec, dict):
        return True, _num(spec.get("x")), _num(spec.get("y")), _num(spec.get("z")), None
    return False, 0.0, 0.0, 0.0, f"not a point name or coordinate: {spec!r}"


def _in_bounds(x, y, arena):
    """Is (x, y) inside the arena, with the same wall clearance
    mission_target()'s own shuttle defaults use? Returns (ok, hx, hy) so a
    caller can report the usable range, not just pass/fail."""
    hx = arena["extent"]["x"] / 2 - WALL_MARGIN_M
    hy = arena["extent"]["y"] / 2 - WALL_MARGIN_M
    return (-hx <= x <= hx and -hy <= y <= hy), hx, hy


def validate_objective(mission_dict, points, arena):
    """Can this objective actually be ACCEPTED onto an agent right now?

    Checked at the moment an objective is set - REOBJECTIVE, SETMISSION,
    or the initial scene+mission merge - never at tick time.
    Only 'shuttle' has anything to check today: its endpoints must resolve
    (named point exists, or a literal was given) AND land inside the arena.
    Every other objective type is accepted as-is. See "Bug fix" and
    "Out-of-bounds decisions" in docs/PATCH-07-CHECKS.md.
    """
    if not isinstance(mission_dict, dict):
        return False, "not an objective"
    if mission_dict.get("type") == "advance":
        # An advance is rejected at ASSIGNMENT time if its goal does not
        # resolve or lands outside the arena - the same gate shuttle gets, and
        # for the same reason: a bad objective must be refused with a reason,
        # never silently turned into something else mid-run.
        ok, x, y, _z, err = resolve_waypoint(mission_dict.get("to"), points)
        if not ok:
            return False, err
        if not _in_bounds(x, y, arena):
            return False, f"advance goal ({x:.1f}, {y:.1f}) is outside the arena"
        return True, None
    if mission_dict.get("type") != "shuttle":
        return True, None

    between = mission_dict.get("between")
    if isinstance(between, (list, tuple)) and len(between) == 2:
        specs = list(between)
    else:
        specs = [mission_dict.get("from"), mission_dict.get("to")]
        if specs[0] is None and specs[1] is None:
            return True, None      # no endpoints given yet; nothing to check

    resolved = []
    for spec in specs:
        ok, x, y, z, err = resolve_waypoint(spec, points)
        if not ok:
            return False, err
        resolved.append((x, y, z))

    for x, y, _z in resolved:
        ok, hx, hy = _in_bounds(x, y, arena)
        if not ok:
            return False, (f"endpoint ({x:.2f}, {y:.2f}) is outside the "
                           f"arena - usable range is x ±{hx:.2f} m, "
                           f"y ±{hy:.2f} m")
    return True, None


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
    hx = arena["extent"]["x"] / 2 - WALL_MARGIN_M
    hy = arena["extent"]["y"] / 2 - WALL_MARGIN_M
    # Cyclic objectives (shuttle/patrol/orbit) measure their phase from HERE,
    # not from the run's absolute clock - phase_t0 resets to "now" on every
    # (re)launch and every (re)assignment (see drain_retasks). Without this,
    # an agent armed at t=30s computes its phase as if it had been shuttling
    # since t=0, so its target snaps to wherever a 30-second-old cycle would
    # be - often nowhere near where the agent actually is - and it lurches
    # off to catch up before settling into the real oscillation.
    t_eff = t - _num(agent.get("phase_t0"))

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
        #
        # Objectives are VALIDATED (named points resolve, endpoints are in
        # bounds) at the moment they are ASSIGNED - see validate_objective().
        # What follows is a defence-in-depth backstop only, for a path that
        # somehow skipped that gate: on failure it holds the agent at its
        # CURRENT pose, never a fabricated point (the arena half-width used
        # to sneak in here silently - that was the bug), and warns once per
        # agent rather than every tick.
        pts = arena.get("points") or {}
        between = m.get("between")
        specs = (list(between) if isinstance(between, (list, tuple))
                 and len(between) == 2 else [m.get("from"), m.get("to")])

        ok_a, ax, ay, _az, err_a = resolve_waypoint(specs[0], pts)
        ok_b, bx, by, _bz, err_b = resolve_waypoint(specs[1], pts)
        if not (ok_a and ok_b):
            if not agent.get("_warned_bad_shuttle"):
                agent["_warned_bad_shuttle"] = True
                print(f"objective for {agent['id']}: {err_a or err_b} - "
                      f"holding position, not the arena edge", file=sys.stderr)
            here = poses.get(agent["id"]) or start
            return (here["x"], here["y"])
        agent["_warned_bad_shuttle"] = False

        leg = math.hypot(bx - ax, by - ay) or 1.0
        period = 2.0 * leg / speed
        phase = ((t_eff + _num(m.get("offset"))) % period) / period
        u = phase * 2.0 if phase < 0.5 else (1.0 - phase) * 2.0
        return (ax + (bx - ax) * u, ay + (by - ay) * u)

    if kind == "patrol":
        wps = m.get("waypoints") or [
            {"x": -hx, "y": -hy}, {"x": hx, "y": -hy},
            {"x": hx, "y": hy}, {"x": -hx, "y": hy}]
        pts = [(_num(w.get("x")), _num(w.get("y"))) for w in wps]
        segs = [(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]
        lengths = [math.dist(a, b) for a, b in segs]
        d = (t_eff * speed + _num(m.get("offset"))) % (sum(lengths) or 1.0)
        for (p0, p1), L in zip(segs, lengths):
            if d <= L:
                u = d / (L or 1.0)
                return (p0[0] + (p1[0] - p0[0]) * u, p0[1] + (p1[1] - p0[1]) * u)
            d -= L
        return (start["x"], start["y"])

    if kind == "advance":
        # ADVANCE TO A POINT AND STOP. Every other objective is cyclic or
        # reactive - shuttle turns around, patrol loops, orbit circles - so
        # none of them can express "get as far as you can and hold there",
        # which is the only shape that has a PENETRATION DEPTH to measure.
        #
        # It is also the framework's first TERMINATING objective, which is
        # what makes `intent` and `continue` finally distinguishable: mission
        # command says a subordinate acts within the commander's intent, and
        # once the assigned effect is achieved it seeks new direction rather
        # than inventing a next task. An advance that has arrived is done.
        pts = arena.get("points") or {}
        ok, gx, gy, _gz, err = resolve_waypoint(m.get("to"), pts)
        if not ok:
            if not agent.get("_warned_bad_advance"):
                agent["_warned_bad_advance"] = True
                print(f"objective for {agent['id']}: {err} - holding position",
                      file=sys.stderr)
            here = poses.get(agent["id"]) or start
            return (here["x"], here["y"])
        # The agent keeps its own lane: it advances along x to the goal's x,
        # holding the y it started on, so a formation ADVANCES rather than
        # collapsing onto one point and colliding. The wedge keeps its shape
        # until the doctrine breaks it, which is the effect under test.
        return (gx, start["y"] if m.get("keep_lane", True) else gy)

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
        ang = 2.0 * math.pi * (t_eff / period) + _num(m.get("phase"))
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


def step(agents, poses, t, dt, arena, unreachable=None,
         position_lost=None, drift_rates=None, rng=None):
    """Advance every agent one tick toward its mission target.

    Motion is speed-limited and collision BLOCKS it rather than displacing the
    other body: an agent that cannot move straight tries sliding along each axis
    in turn, and if neither works it stops. Nothing is ever teleported out of an
    overlap, so two agents in contact rest against each other instead of
    jittering. This is a kinematic constraint solver, not a physics engine —
    there is no momentum, restitution or contact force.

    LINK LOSS. `unreachable` is the set of agent ids that, this tick, cannot
    reach whoever commands them (from command_authority on the pre-step link
    state). What such an agent DOES is its `on_link_loss` doctrine:
      hold (default) - freeze in place until the link returns. A centralized
                       agent that loses its coordinator has no orders, so it
                       stops. THIS is what makes jamming change behaviour, not
                       just a readout.
      intent         - keep executing the assigned objective from the picture
                       already held, without further direction. NATO mission
                       command: centralized intent, decentralized execution
                       (AJP-3 3.8, 3.11). `continue` is a legacy alias.
    A decentralized agent is never in `unreachable` (it decides for itself),
    so it keeps moving under jamming - the robustness story, made physical.
    """
    global _ALL_AGENTS
    _ALL_AGENTS = agents            # so a World built inside mission_target
                                    # can answer questions about every agent
    unreachable = unreachable or set()
    position_lost = position_lost or set()
    drift_rates = drift_rates or {}
    if rng is None:
        import random as _r
        rng = _r.Random()
    contacts = []
    for a in agents:
        p = poses[a["id"]]
        px0, py0 = p["x"], p["y"]
        # Unarmed means idle regardless of what the objective is - this is
        # the assign/inspect/launch gate. An armed agent with a static
        # objective is already covered by the same check.
        if (a["mission"].get("type", "static") == "static"
                or not a.get("armed", False)):
            p["speed"] = 0.0
            p["v"] = 0.0
            _update_belief(a, poses, 0.0, 0.0, dt, position_lost, drift_rates,
                           rng)
            continue
        # Command lost: apply the agent's link-loss doctrine. `hold` freezes;
        # `intent` carries on executing the objective it already holds, on the
        # picture it already has (AJP-3 3.8/3.11 - see LINK_LOSS_KEEPS_GOING).
        # This is the causal step that was missing: jamming that strips
        # authority now stops a `hold` vehicle, and only degrades an `intent`
        # one.
        if a["id"] in unreachable and not keeps_going_on_link_loss(a):
            p["speed"] = 0.0
            _update_belief(a, poses, 0.0, 0.0, dt, position_lost, drift_rates,
                           rng)
            continue
        # Look one tick AHEAD. Without this the target advances at exactly the
        # agent's own speed, so the agent keeps catching it exactly and stopping
        # for a frame - which is what put the regular dropouts to zero in the
        # speed plot. A moving target must always be a step away.
        # THE KEY LINE: a mission is handed what this agent KNOWS - its own
        # drifted belief and the reports it has received - never ground truth.
        tx, ty = mission_target(a, t + dt, a.get("knowledge") or poses, arena)
        # A vehicle steers toward its target from where it BELIEVES it is - it
        # has no other position to use. While it has a fix, belief == truth
        # and this is the old behaviour. Under GNSS denial, belief has drifted,
        # so the heading is computed from a wrong origin and the true path
        # bends off-target: the fault becomes visible, and emergent.
        b = a.get("belief") or {"x": p["x"], "y": p["y"]}
        dx, dy = tx - b["x"], ty - b["y"]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            p["speed"] = 0.0
            _update_belief(a, poses, 0.0, 0.0, dt, position_lost, drift_rates,
                           rng)
            continue
        # --- speed: accelerate toward the cap, decelerate to stop on target.
        vmax = max(a["speed"], 0.05)
        accel = max(_num(a.get("max_accel"), 2.0), 0.05)
        # Slow down in time to stop at the target: v = sqrt(2*a*d).
        v_want = min(vmax, math.sqrt(max(2.0 * accel * dist, 0.0)))
        v = _num(p.get("v"), 0.0)
        v += max(-accel * dt, min(accel * dt, v_want - v))
        v = max(0.0, min(v, vmax))

        desired = math.atan2(dy, dx)
        reversing = False
        if a.get("motion") == "holonomic":
            # A quadcopter translates in any direction; yaw is free, so point
            # it along travel for display (a camera could point elsewhere).
            hx_, hy_ = math.cos(desired), math.sin(desired)
            p["yaw"] = round(wrap_pi(desired), 4)
        else:
            # ACKERMANN: it can only move along its own heading, and the
            # heading changes no faster than v / turn_radius.
            yaw = _num(p.get("yaw"))
            err = wrap_pi(desired - yaw)
            if abs(err) > math.radians(120.0) and a.get("can_reverse", True):
                # Too sharp to turn into - back up along the current heading
                # instead. The lidar now faces AWAY from travel.
                reversing = True
                err = wrap_pi(err - math.pi)
            radius = max(_num(a.get("turn_radius"), 0.6), 0.05)
            max_rate = max(v, 0.05) / radius        # rad/s
            yaw = wrap_pi(yaw + max(-max_rate * dt,
                                    min(max_rate * dt, err)))
            p["yaw"] = round(yaw, 4)
            hx_, hy_ = math.cos(yaw), math.sin(yaw)
            if reversing:
                hx_, hy_ = -hx_, -hy_
        p["v"] = round(v, 4)
        p["reversing"] = reversing
        stepd = min(dist, v * dt)
        ux, uy = hx_, hy_
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
        # Speed is what the body ACTUALLY achieved (collision may have cut it
        # short); heading was integrated above, never snapped to the bearing.
        moved = math.hypot(p["x"] - px0, p["y"] - py0)
        p["speed"] = round(moved / dt, 4) if dt > 1e-6 else 0.0
        p["x"], p["y"] = round(p["x"], 4), round(p["y"], 4)
        _update_belief(a, poses, p["x"] - px0, p["y"] - py0, dt,
                       position_lost, drift_rates, rng)
    return contacts


def _update_belief(a, poses, adx, ady, dt, position_lost, drift_rates, rng):
    """Advance an agent's position estimate one tick.

    With a fix: pull the belief back toward truth at GNSS_RECOVER_MPS (a
    regained fix corrects gradually, not with a jump). Without a fix (GNSS
    denied and no lidar/vision aiding): dead-reckon. The estimate follows the
    real motion, but an error accumulates at drift_rate * distance travelled in
    a slowly random-walking direction - so the drift GROWS with distance,
    exactly the % -of-distance behaviour the literature reports, and it is the
    integral of a per-tick error, never a hardcoded wander.
    """
    p = poses[a["id"]]
    b = a.setdefault("belief", {"x": p["x"], "y": p["y"]})
    drift = a.setdefault("drift", {"x": 0.0, "y": 0.0})
    if a["id"] in position_lost:
        rate = drift_rates.get(a["id"], DRIFT_RATE_UNAIDED)
        step_d = math.hypot(adx, ady)
        a["drift_dir"] = wrap_pi(a.get("drift_dir", 0.0)
                                 + rng.gauss(0.0, DRIFT_TURN_SIGMA))
        emag = rate * step_d
        drift["x"] += emag * math.cos(a["drift_dir"])
        drift["y"] += emag * math.sin(a["drift_dir"])
        # belief = truth + accumulated error (the estimate the vehicle holds)
        b["x"] = p["x"] + drift["x"]
        b["y"] = p["y"] + drift["y"]
    else:
        # A fix (GNSS or a lidar/vision loop) gives an ABSOLUTE position every
        # tick, so the belief tracks truth: no lag, no residual. The estimate
        # is truth plus small fix noise (negligible here). A regained fix
        # therefore corrects the accumulated drift immediately - which is what
        # makes "jam -> drift, unjam -> recover" clean to watch.
        b["x"], b["y"] = p["x"], p["y"]
        drift["x"], drift["y"] = 0.0, 0.0


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


def command_authority(agent, arena, links, poses, networks,
                      link_states=None):
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
    # AUTHORITY is the current key; architecture/topology are legacy aliases
    # kept so scenes written before the split still load.
    arch = (net.get("authority") or net.get("architecture")
            or net.get("topology") or "centralized")
    aid = agent["id"]

    def _reaches(other):
        """Can a command from `other` reach this agent, right now?

        NOT "is there a direct link". Two corrections that the routing axis
        is meaningless without:

          ACTIVE ONLY. A link the routing does not carry cannot deliver an
          order, however good its SINR. A star does not carry peer links; a
          tiered network does not carry member-to-coordinator links. Scoring
          those anyway is what made star, mesh and tiered produce IDENTICAL
          command availability in the first sweep - the axis was declared,
          measured, and silently ignored.

          MULTI-HOP. Authority follows a PATH, not one hop. In a tiered
          network car2 has no direct link to the coordinator and is still
          commanded, via its leader. Asking only about the direct link
          would stand the whole squad down the moment it was tiered.

        When the caller has already scored the links (frame() computes them
        WITH jamming and the scene baseline), that verdict is used, so
        authority genuinely degrades when the spectrum does. The fallback
        re-derives from clean-spectrum geometry, for callers with no frame
        in hand - single-hop only, and honest about it.
        """
        if other is None or other == aid:
            return True
        if link_states is not None:
            adj = {}
            for pair, meta in link_states.items():
                # Accept both the scored-dict form {pair: {...}} and the older
                # {pair: state_string} form, so nothing that calls this breaks.
                if isinstance(meta, dict):
                    if not meta.get("active") or meta.get("state") == "down":
                        continue
                elif meta == "down":
                    continue
                x, y = tuple(pair)
                adj.setdefault(x, set()).add(y)
                adj.setdefault(y, set()).add(x)
            seen, stack = set(), [aid]
            while stack:
                n = stack.pop()
                if n in seen:
                    continue
                seen.add(n)
                if n == other:
                    return True
                stack += [m for m in adj.get(n, ()) if m not in seen]
            return False
        for l in links:
            if {l["a"], l["b"]} == {aid, other}:
                return link_state(poses[l["a"]], poses[l["b"]])["state"] != "down"
        return False

    if arch == "decentralized":
        # Each agent is its own authority. Never decapitated.
        return {"decider": aid, "reachable": True, "tier": "self"}

    if arch == "hierarchical":
        # Find this agent's squad leader from the squads block. An explicit
        # reports_to on the agent wins; otherwise look it up by membership.
        # Without this the hierarchy exists in the routing but not in the
        # authority, and every agent reports straight to the coordinator - a
        # hierarchy in name only.
        leader = agent.get("reports_to")
        if not leader:
            for _sq, spec in (net.get("squads") or {}).items():
                spec = spec or {}
                if aid in (spec.get("members") or []):
                    leader = spec.get("leader")
                    break
                if aid == spec.get("leader"):
                    # A leader answers upward, not to itself.
                    leader = net.get("coordinator")
                    break
        leader = leader or net.get("squad_leader") or net.get("coordinator")
        if _reaches(leader):
            tier = "coordinator" if leader == net.get("coordinator") else "leader"
            return {"decider": leader, "reachable": True, "tier": tier}
        # Leader unreachable. DOCTRINE, declarable per network as `leader_loss`:
        #   fallback (default) - the squad reports up to the coordinator;
        #                        degraded, not decapitated.
        #   strand             - the squad is on its own the moment its leader
        #                        drops; no automatic reach-up.
        # This is exactly the kind of command-resilience choice the framework
        # exists to let you compare - so it is a field, not a hard-coded rule.
        doctrine = (net.get("leader_loss") or "fallback").lower()
        top = net.get("coordinator")
        if doctrine == "fallback" and top != leader and _reaches(top):
            return {"decider": top, "reachable": True, "tier": "coordinator"}
        return {"decider": leader, "reachable": False, "tier": "orphaned"}

    # centralized (the default)
    hub = net.get("coordinator")
    return {"decider": hub, "reachable": _reaches(hub), "tier": "coordinator"}
def _qty(node, default=None):
    """Unwrap a {value, unit, source} quantity - or a plain number - to float."""
    if isinstance(node, dict):
        node = node.get("value")
    try:
        return float(node)
    except (TypeError, ValueError):
        return default


def scene_rf(world):
    """The scene's declared RF baseline, falling back to rf_link()'s own
    defaults where the scene declares nothing. The point: the BASELINE is the
    scene's to own (docs/contested-background.md); the defaults are only a
    stand-in for scenes written before the background existed."""
    bg = (world or {}).get("background") or {}
    return {
        "plexp": _qty((bg.get("propagation") or {})
                      .get("path_loss_exponent"), 2.8),
        "noise_dbm": _qty((bg.get("spectrum") or {})
                          .get("noise_floor"), -95.0),
    }


def jammer_rx_mw(pos, jammers, poses, plexp, band_mhz, exclude=()):
    """Total jamming power (mW) arriving at `pos` on `band_mhz`.

    Each armed jammer's transmit power travels the SAME log-distance path
    loss as a legitimate signal - a jammer is not special physics, just an
    unwanted transmitter - and lands in the receiver's noise denominator
    (rf_link's interference_mw). Off-band jammers contribute nothing: band
    separation is a real (first-order) defence, and later frequency-hopping
    work depends on the model honouring it.
    """
    total = 0.0
    for j in jammers:
        if j["id"] in exclude:
            continue
        jcfg = j.get("jammer") or {}
        jband = _qty(jcfg.get("band"), 2400.0)
        # ACI: a jammer off the victim's channel still leaks in, by mu.
        mu = aci_mu(jband - band_mhz, jcfg.get("bandwidth")) if band_mhz else 1.0
        if mu < 1e-9:
            continue                     # far enough away to be irrelevant
        tx = _qty(jcfg.get("tx_power"), 20.0)
        jp = poses.get(j["id"])
        if not jp:
            continue
        d = max(0.1, math.dist((pos["x"], pos["y"], pos["z"]),
                               (jp["x"], jp["y"], jp["z"])))
        # Past the radio horizon the earth blocks it, whatever the power.
        if d > radio_horizon_m(pos.get("z"), jp.get("z")):
            continue
        pl_d0 = 20.0 * math.log10(jband) + 20.0 * math.log10(0.001) + 32.44
        rx_dbm = tx - (pl_d0 + 10.0 * plexp * math.log10(d))
        total += mu * 10.0 ** (rx_dbm / 10.0)
    return total


def jammer_range_m(tx_dbm, band_mhz, noise_dbm, plexp, jn_db=0.0,
                   tx_h_m=2.0, rx_h_m=2.0):
    """Nominal influence radius of an omnidirectional jammer: the distance at
    which its received power falls to `jn_db` above the ambient noise floor
    (J/N = jn_db). At jn_db = 0 this is the classic jammed-area boundary
    (Tedeschi & Di Pietro, SpaCCS 2021: the RSS "at the boundary of the jammed
    area"). It is a NOMINAL contour for a point omni source - real jammed
    areas are ragged ("effect is not uniform", Baltic Sea trial, sensors
    2024), and a directional jammer is not a circle at all.

        rx(d) = tx - (PL(d0) + 10*n*log10(d))   set equal to noise + jn_db
        => d = 10 ** ((tx - (noise+jn_db) - PL(d0)) / (10*n))
    """
    pl_d0 = 20.0 * math.log10(band_mhz) + 20.0 * math.log10(0.001) + 32.44
    exponent = (tx_dbm - (noise_dbm + jn_db) - pl_d0) / (10.0 * max(plexp, 0.1))
    free_space = 10.0 ** exponent
    # A jammer cannot reach past the horizon however much power it has. Both
    # ends assumed near the ground unless told otherwise; pass heights for an
    # airborne case. See radio_horizon_m.
    return min(free_space, radio_horizon_m(tx_h_m, rx_h_m))


def _scene_has_features(arena):
    """Does this world give a lidar/camera something to localise AGAINST?

    A lidar knows where it is by matching what it scans to a known shape - it
    needs walls, buildings, terrain. In an OPEN FIELD with nothing around, a
    lidar sees no returns and cannot localise at all (Will's point, and it is
    correct). Proxy: the arena has reference features if any boundary is solid
    (walls) or there is a static obstacle. Open boundaries + no obstacles =
    featureless = no lidar aiding. See docs/gnss-drift-model.md."""
    arena = arena or {}
    for face, kind in (arena.get("boundaries") or {}).items():
        if kind == "solid":
            return True
    return False


def aci_mu(df_mhz, bw_mhz=None, acr_db=None):
    """Interference correlation coefficient between two channels separated by
    `df_mhz` - the fraction of a transmitter's power that lands in the victim's
    channel. 1.0 on the same frequency, falling by `acr_db` per channel width.
    See Zhou et al. 2020 for the model's properties."""
    bw = max(_num(bw_mhz, CHANNEL_BW_MHZ), 1e-6)
    acr = _num(acr_db, ACR_DB_PER_CHANNEL)
    channels_away = abs(_num(df_mhz, 0.0)) / bw
    return min(1.0, 10.0 ** (-acr * channels_away / 10.0))


def radio_horizon_m(h1_m, h2_m):
    """Line-of-sight range limit between two antennas, metres.

    d_km ~ 4.12 * (sqrt(h1) + sqrt(h2)) with heights in metres - the standard
    radio-horizon approximation (4/3-earth refraction). This is what stops a
    jammer, or any transmitter, reaching arbitrarily far just by adding power:
    past the horizon the earth is in the way.

    It matters most for GNSS jamming. GNSS signals arrive at about -128 dBm, so
    a jammer needs very little power to out-shout them, and a naive free-space
    model gives denial radii of tens or hundreds of kilometres. Real GNSS
    jamming is HORIZON-limited: two ~2 m antennas see about 11.6 km of each
    other; a receiver at 100 m altitude sees a ground jammer from ~47 km, which
    is why GNSS jamming affects aviation over far wider areas than ground
    users. The Baltic Sea trial's ">3 km area of influence" for a ship-borne
    jammer sits comfortably inside the ground-to-ground horizon.
    """
    h1 = max(_num(h1_m, 0.0), 1.0)      # assume a 1 m antenna at minimum
    h2 = max(_num(h2_m, 0.0), 1.0)
    return 4120.0 * (math.sqrt(h1) + math.sqrt(h2))


def _position_aiding(agent, arena):
    """An RF-immune source of a position fix this agent carries. A lidar or
    camera can localise WITHOUT GNSS or the radio - but ONLY where there is
    something to localise against (see _scene_has_features). An IMU is NOT
    aiding: it is the dead-reckoning source that DRIFTS. Returns the drift
    rate under GNSS denial, or None if this agent has no usable fix source."""
    if not _scene_has_features(arena):
        return None                      # featureless field: lidar cannot help
    for sen in agent.get("sensors") or []:
        t = (sen.get("type") or "").lower()
        if "lidar" in t or "ust10" in t or "camera" in t or "vision" in t:
            return DRIFT_RATE_LIDAR
    return None


def gnss_denied(agent, poses, jammers, plexp):
    """Is this agent's GNSS fix denied right now by a GNSS-band jammer?

    Only a jammer ON the GNSS band counts (a comms-band jammer denies the
    radio link, not the position fix - two different jammings, the whole
    point). Denied when the received jamming power on L1 exceeds the denial
    threshold. See docs/jamming-effects-research.md."""
    gnss_jams = [j for j in jammers
                 if abs(_qty((j.get("jammer") or {}).get("band"), 2400.0)
                        - GNSS_BAND_MHZ) < 5.0]
    if not gnss_jams:
        return False
    mw = jammer_rx_mw(poses[agent["id"]], gnss_jams, poses, plexp,
                      GNSS_BAND_MHZ, exclude=(agent["id"],))
    if mw <= 0:
        return False
    return 10.0 * math.log10(mw) > GNSS_DENIAL_DBM


def active_jammers(agents):
    """The jammers currently transmitting: a jammer block AND armed."""
    return [a for a in agents if a.get("jammer") and a.get("armed")]


def _co_channel_contenders(a, b, agents, networks, poses, band, rf,
                           routing="mesh", squads_of=None):
    """How many OTHER agents actually compete for this receiver's channel.

    Two filters, and the second is where ROUTING matters - how the fleet is
    ORGANISED changes who shares a channel with whom:

      star    every member talks to the hub, so every member contends with
              every other. Contention concentrates at one point and scales
              with the whole fleet.
      mesh    every in-range peer relays, so everyone in earshot contends -
              the same crowd as a star, plus relayed traffic.
      tiered  intra-squad traffic stays inside the squad, so a member contends
              only with its own squad and its leader. Contention is PARTITIONED
              by the hierarchy - which is one of the real reasons militaries
              organise this way.

    "In earshot" = the other agent's signal arrives above receiver sensitivity,
    i.e. loud enough to collide. Jammers are excluded: they are already counted
    as interference power, and they do not obey the protocol.
    """
    squads_of = squads_of or {}
    my_squad = (squads_of.get(b) or (None, None))[0]
    n = 0
    for other in agents:
        oid = other["id"]
        if oid in (a, b) or other.get("jammer"):
            continue
        onet = (networks or {}).get(other.get("network")) or {}
        oband = _qty(onet.get("band"), 2400.0)
        # Only agents whose energy substantially lands in this channel contend
        # for it; ACI decides "substantially" rather than an exact match.
        if aci_mu(oband - band) < 0.5:
            continue                     # separated enough: no contention
        if routing == "tiered" and my_squad is not None:
            osq, oleader = squads_of.get(oid, (None, None))
            if osq != my_squad and oid != oleader:
                continue                 # a different squad's traffic is not
                                         # on this receiver's local channel
        try:
            r = rf_link(poses[oid], poses[b], plexp=rf["plexp"],
                        noise_dbm=rf["noise_dbm"])
        except KeyError:
            continue
        if r["rx_dbm"] > -85.0:          # audible => can collide
            n += 1
    return n


def apply_routing(links, agents, networks, poses, world=None):
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

    _rf = scene_rf(world)
    _jam = active_jammers(agents)
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

        # Score the link against the SCENE's baseline (path loss exponent,
        # noise floor), with every armed same-band jammer's power summed
        # into the receiver's denominator. The worse endpoint governs: a
        # link is only as good as its more-jammed end. Self-jamming (the
        # fleet's OWN transmitters raising each other's floor) remains
        # unmodelled - it would sum here identically when built.
        band = _qty(net.get("band"), 2400.0)
        interf = 0.0
        if _jam:
            interf = max(
                jammer_rx_mw(poses[a], _jam, poses, _rf["plexp"], band,
                             exclude=(a, b)),
                jammer_rx_mw(poses[b], _jam, poses, _rf["plexp"], band,
                             exclude=(a, b)))
        state = rf_link(poses[a], poses[b], plexp=_rf["plexp"],
                        noise_dbm=_rf["noise_dbm"], interference_mw=interf)

        # CONTENTION: every other agent sharing this band and within earshot of
        # the receiver competes for airtime. Each costs a little delivered
        # throughput and adds collision risk; latency rises with the share.
        # This is the fleet jamming ITSELF - a crowding effect, distinct from
        # the external jammer already in the SINR above.
        contenders = _co_channel_contenders(a, b, agents, networks, poses,
                                            band, _rf, routing, squads_of)
        if contenders > 0:
            # 1. Airtime share: n contenders -> delay scales by (n + 1).
            delay = state["latency_ms"] * (1.0 + contenders)
            # 2. A packet is lost only if it misses the declared deadline.
            deadline = _qty((net.get("qos") or {}).get("deadline_ms"),
                            DEFAULT_DEADLINE_MS)
            on_time = 1.0 - math.exp(-max(deadline, 1e-6) / max(delay, 1e-6))
            state["latency_ms"] = round(delay, 2)
            state["pdr"] = round(state["pdr"] * on_time, 3)
            state["quality"] = state["pdr"]
            state["state"] = ("up" if state["pdr"] > 0.85
                              else ("degraded" if state["pdr"] > 0.25
                                    else "down"))
        out.append({**l, **state,
                    "routing": routing,
                    "band_mhz": band,          # so the UI can filter by band
                    "contenders": contenders,
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
    """Link quality between two poses on a CLEAN SPECTRUM.

    A thin wrapper for callers that want "can this pair talk" without
    assembling radio parameters. LIMITATION, stated because it matters: this
    ignores jamming, the scene's declared noise floor and path-loss exponent,
    and contention - it is clean-spectrum geometry only. Anything that has the
    real picture must call rf_link() directly and pass it. command_authority()
    only falls back to this when no scored link states are supplied; every
    call inside frame() supplies them.
    """
    return rf_link(pa, pb)


def update_knowledge(agents, poses, links_out, t):
    """Share position reports along the routes that actually carry traffic.

    Each agent knows its OWN belief exactly (it is its own estimate) and knows
    other agents only as the belief THEY reported. A report reaches an agent if
    a path of active, non-down links connects them - so this is multi-hop and
    routing-dependent: in a star, two members exchange positions via the hub;
    in a mesh, directly; in a tiered network, within the squad. Agents in
    different connected components keep whatever they last heard, timestamped,
    and act on stale information - which is what actually happens.

    This replaces missions reading ground truth. Fixes the hole where a pursuit
    under total GNSS denial still knew exactly where its target really was.
    """
    adj = {}
    for l in links_out:
        if l.get("active") and l.get("state") != "down":
            adj.setdefault(l["a"], set()).add(l["b"])
            adj.setdefault(l["b"], set()).add(l["a"])

    by_id = {a["id"]: a for a in agents}

    def reported(aid):
        """What agent `aid` would transmit about itself: its own belief for
        position (which drifts), plus heading/height, which a compass, IMU and
        barometer give it without GNSS."""
        ag = by_id.get(aid) or {}
        b = ag.get("belief") or {}
        p = poses.get(aid) or {}
        return {"x": _num(b.get("x"), _num(p.get("x"))),
                "y": _num(b.get("y"), _num(p.get("y"))),
                "z": _num(p.get("z")), "yaw": _num(p.get("yaw")),
                "speed": _num(p.get("speed")), "t": t}

    seen, comps = set(), []
    for a in agents:
        aid = a["id"]
        if aid in seen:
            continue
        comp, stack = set(), [aid]
        while stack:
            n = stack.pop()
            if n in comp:
                continue
            comp.add(n)
            seen.add(n)
            stack += [m for m in adj.get(n, ()) if m not in comp]
        comps.append(comp)

    for comp in comps:
        shared = {aid: reported(aid) for aid in comp if aid in by_id}
        for aid in comp:
            ag = by_id.get(aid)
            if ag is not None:
                ag.setdefault("knowledge", {}).update(shared)
    # An agent always knows its own current belief, connected or not.
    for a in agents:
        a.setdefault("knowledge", {})[a["id"]] = reported(a["id"])


def frame(t, dt, seq, arena, agents, links, poses, rng):
    """One telemetry frame. THIS DICTIONARY IS THE CONTRACT."""
    _nets0 = arena.get("networks") or {}
    # Reachability BEFORE moving: score the links on the poses as they enter
    # this tick, work out who cannot reach their commander, and let that gate
    # movement. Using the pre-step state is the honest one-tick lag - an agent
    # acts on the link it last had, not one it cannot yet know. Without this,
    # a jammed centralized car kept driving as if still commanded.
    _pre = apply_routing(links, agents, _nets0, poses, world=arena)
    # The full verdict per pair - state AND whether the routing carries it.
    _pre_states = {frozenset((l["a"], l["b"])):
                   {"state": l["state"], "active": l.get("active", True)}
                   for l in _pre}
    _unreachable = {a["id"] for a in agents
                    if not command_authority(a, arena, links, poses, _nets0,
                                             link_states=_pre_states
                                             ).get("reachable", True)}
    # GNSS: is each agent's position fix available? Denied by a GNSS-band
    # jammer, OR simply absent if the scene provides no GNSS (indoors). An
    # agent with a lidar/vision aiding sensor keeps a fix regardless (RF-
    # immune). Those with no fix this tick DRIFT; drift_rates says how fast.
    _rf0 = scene_rf(arena)
    _jam0 = active_jammers(agents)
    _bg = (arena or {}).get("background") or {}
    _gnss_present = ((_bg.get("gnss") or {}).get("availability", "available")
                     != "denied")
    _position_lost, _drift_rates = set(), {}
    for a in agents:
        aid = a["id"]
        aided = _position_aiding(a, arena)   # None, or a (low) drift rate
        gnss_ok = (_gnss_present
                   and not gnss_denied(a, poses, _jam0, _rf0["plexp"]))
        if gnss_ok:
            # An absolute fix: the estimate tracks truth exactly.
            _drift_rates[aid] = 0.0
        elif aided is not None:
            # No GNSS, but a lidar/vision loop: localises, yet still drifts
            # slowly (scan-matching error accumulates). Much better than
            # inertial-only, not perfect.
            _position_lost.add(aid)
            _drift_rates[aid] = aided
        else:
            _position_lost.add(aid)
            _drift_rates[aid] = DRIFT_RATE_UNAIDED
    # Distribute position reports over the routes as they stand entering this
    # tick, THEN act. An agent acts on what it has been told, not on truth.
    update_knowledge(agents, poses, _pre, t)
    contacts = step(agents, poses, t, dt, arena, unreachable=_unreachable,
                    position_lost=_position_lost, drift_rates=_drift_rates,
                    rng=rng)

    # Re-score after movement for the DISPLAY (positions changed).
    links_out = apply_routing(links, agents, arena.get("networks") or {},
                              poses, world=arena)
    _states = {frozenset((l["a"], l["b"])):
               {"state": l["state"], "active": l.get("active", True)}
               for l in links_out}
    _rf = scene_rf(arena)
    _jam = active_jammers(agents)
    _noise_mw = 10.0 ** (_rf["noise_dbm"] / 10.0)
    _nets = arena.get("networks") or {}

    agents_out = []
    for a in agents:
        lidars = [s for s in a["sensors"] if s["type"] == "ust10lx"]
        # The noise floor THIS agent actually experiences, on its own
        # network's band, jammers included. The gap between this and the
        # scene's baseline is jamming, as a number, per agent.
        band = _qty((_nets.get(a["network"]) or {}).get("band"), 2400.0)
        interf = jammer_rx_mw(poses[a["id"]], _jam, poses, _rf["plexp"],
                              band, exclude=(a["id"],)) if _jam else 0.0
        eff_dbm = 10.0 * math.log10(_noise_mw + interf)
        rf_out = {"noise_floor_dbm": round(eff_dbm, 1),
                  "baseline_dbm": round(_rf["noise_dbm"], 1),
                  "jammed": (eff_dbm - _rf["noise_dbm"]) > 3.0}
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
            # Assigned vs active - the whole point of this patch. An agent
            # can hold a fully-formed objective and still not be armed.
            "armed": bool(a.get("armed", False)),
            # Driving BACKWARDS (too sharp to turn into), so a forward-mounted
            # lidar is pointing away from the direction of travel - a real
            # sensing gap, not a display quirk.
            "reversing": bool(poses[a["id"]].get("reversing")),
            "motion": a.get("motion"),
            # Is this agent holding because it lost its commander this tick?
            "link_loss_hold": (a["id"] in _unreachable
                               and not keeps_going_on_link_loss(a)
                               and a["mission"].get("type") != "static"),
            # The doctrine itself, so a run's CSV records WHY a vehicle behaved
            # as it did rather than leaving it to be inferred from the fleet.
            "on_link_loss": (a.get("on_link_loss") or "hold"),
            # Acting on delegated intent with no reachable commander - the
            # mission-command condition. Distinct from link_loss_hold: this
            # vehicle is still working, on information that is going stale.
            "on_intent": (a["id"] in _unreachable
                          and keeps_going_on_link_loss(a)
                          and a["mission"].get("type") != "static"),
            # POSITION ESTIMATE vs truth. `believed` is where the agent thinks
            # it is; `position_error_m` is how wrong that is (0 with a fix,
            # growing under GNSS denial). `gnss_denied` flags the cause.
            "believed": dict(a.get("belief") or {}),
            "position_error_m": round(
                math.hypot(_num((a.get("drift") or {}).get("x")),
                           _num((a.get("drift") or {}).get("y"))), 3),
            "gnss_denied": a["id"] in _position_lost,
            "rf": rf_out,
            # A jammer's own emitter, for the Contested tab.
            "jammer": ({"tx_dbm": _qty((a.get("jammer") or {})
                                       .get("tx_power"), 20.0),
                        "band_mhz": _qty((a.get("jammer") or {})
                                         .get("band"), 2400.0),
                        "on": bool(a.get("armed", False))}
                       if a.get("jammer") else None),
            # Who decides for this agent right now, and whether they are
            # reachable - judged from the SCORED links, so jamming that
            # kills a link kills the authority that flowed over it.
            "authority": command_authority(a, arena, links, poses,
                                           arena.get("networks") or {},
                                           link_states=_states),
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
                       "warnings": [w for i, w in contacts if i == a["id"]] +
                                   ([a["last_rejection"]] if a.get("last_rejection")
                                    else [])},
        })

    # Keep the scans so the next tick's missions can read them. See _LAST_SCANS.
    _LAST_SCANS.clear()
    _LAST_SCANS.update({a["id"]: a["scan"] for a in agents_out if a["scan"]})

    return {
        "seq": seq,
        "sim_time_s": round(t, 3),
        "wall_time": time.time(),
        "run_state": "running",
        "mission": CURRENT_MISSION["name"],
        "arena": arena,
        "agents": agents_out,
        "links": links_out,
        # What the topology MEASURES as, independent of what it was declared
        # to be. The gap between this and the declared routing is the finding.
        "topology": observed_topology(links_out),
        # MAX-MIN FAIRNESS: the fleet's capability is governed by its WORST
        # link, not its average - which is the objective Zhou et al. optimise
        # ("maximize the minimum SINR among all the UAVs"). Reported so an
        # experiment can score on the worst case rather than hide it in a mean.
        "worst_link": (min(
            ({"pair": f"{l['a']}-{l['b']}", "sinr_db": l["sinr_db"],
              "pdr": l["pdr"]}
             for l in links_out if l.get("active")),
            key=lambda x: x["sinr_db"], default=None)),
        # Jammers currently transmitting - the Contested tab's "what is
        # degrading the spectrum right now" list.
        "attacks_active": [
            {"id": a["id"], "network": a["network"],
             "tx_dbm": a["jammer"]["tx_dbm"], "band_mhz": a["jammer"]["band_mhz"]}
            for a in agents_out if a.get("jammer") and a["jammer"]["on"]],
        "contacts": [{"agent": i, "with": w} for i, w in contacts],
    }


def _parse_objective_verb(verb, args):
    """The verb+args grammar behind an agent-scoped REOBJECTIVE (via
    parse_retask, below): turns 'shuttle A B', 'pursue car3', etc. into a
    mission dict.

    Returns the mission dict, or None for a verb this grammar does not know
    (the caller reports that). Raises ValueError on a malformed coordinate
    literal, so the caller can reject the whole command explicitly rather
    than silently mis-parsing it.
    """
    verb = verb.lower()
    if verb in ("stop", "static", "hold"):
        return {"type": "static"}
    if verb in ("advance", "goto", "push"):
        # 'advance FAR' - drive to that named point and stop there.
        return {"type": "advance", "to": (_parse_position_token(args[0])
                                          if args else "FAR")}
    if verb in ("pursuit", "pursue"):
        return {"type": "pursuit", "target": args[0] if args else "car1"}
    if verb == "shuttle":
        # 'shuttle between A B', 'shuttle A B', or literal points:
        # 'shuttle (-3,3,0) (3,3,0)', 'shuttle A (3,2,0)' (named + literal mix)
        toks = [p for p in args if p.lower() != "between"]
        if len(toks) >= 2:
            return {"type": "shuttle",
                    "between": [_parse_position_token(toks[0]),
                                _parse_position_token(toks[1])]}
        return {"type": "shuttle"}
    if verb in ("wall_follow", "wall"):
        return {"type": "script", "file": "missions/wall_follow.py",
                "side": args[0] if args else "right"}
    if verb == "orbit":
        return {"type": "orbit", "radius": float(args[0]) if args else 2.0}
    if verb == "script":
        return {"type": "script", "file": args[0]} if args else None
    return None


def parse_retask(text, agents_by_id):
    """Turn a line like 'car3: pursue car1' into a new objective dict.

    The grammar is deliberately the same words a person would say out loud:

        car3: pursue car1
        car3: advance FAR
        car3: shuttle between E F
        car3: shuttle (-3,3,0) (3,3,0)
        car3: wall_follow right
        car3: stop                 (alias for static - hold position)
        car3: script missions/return_on_link_loss.py

    Returns (agent_id, mission_dict) or None if it does not parse. Kept
    forgiving on purpose: a fat-fingered command should be ignored with a note,
    never crash a running mission. Note this only PARSES the objective - it is
    not validated against the map here; see validate_objective() and
    drain_retasks() below, which is what actually decides whether it takes
    effect.
    """
    if ":" not in text:
        return None
    aid, rest = text.split(":", 1)
    aid, parts = aid.strip(), _tokenize_args(rest)
    if aid not in agents_by_id or not parts:
        return None
    verb, args = parts[0], parts[1:]
    try:
        block = _parse_objective_verb(verb, args)
    except ValueError as exc:
        print(f"retask: {exc}", file=sys.stderr)
        return None
    if block is None:
        print(f"retask: don't understand '{verb}' for {aid}", file=sys.stderr)
        return None
    return aid, block


# ---------------------------------------------------------------------------
# SETMISSION - the run's mission: a file of per-agent objectives applied as
# one command, gated by command authority. See "SETMISSION"
# in docs/PATCH-07-CHECKS.md for the full reasoning; summary here.
# ---------------------------------------------------------------------------

def _is_taskable(agent, networks):
    """Can this agent be given an OBJECTIVE (a mission / REOBJECTIVE)?

    No, if it sits on an adversary network - red is jamming, not manoeuvre,
    and it is driven by LAUNCH/HALT/JAM, never by a mission. Also no if it is
    a jammer by platform, wherever it sits. Everything else (blue fleet) is
    taskable. Keeping this one predicate means the mission path and the live
    REOBJECTIVE path agree. See docs/vocabulary.md.
    """
    if (agent or {}).get("platform") == "jammer" or (agent or {}).get("jammer"):
        return False
    net = (networks or {}).get(agent.get("network")) or {}
    return net.get("system", "friendly") != "adversary"


def apply_mission_file(path, agents_by_id, points, arena,
                       links=None, poses=None):
    """SETMISSION: distribute a mission file's per-agent objectives onto the
    currently running agent set, gated by command authority.

    A mission is a COMMAND, and a command has to reach an agent to task it:
    when `links`/`poses` are given, each agent's decider chain is checked with
    command_authority(), and an agent its decider cannot currently reach is
    SKIPPED (reported, mission unchanged) rather than silently retasked. This
    keeps the property REMISSION existed for - contested comms gate what you
    can command - inside the one remaining order verb. Pass links/poses as
    None to apply verbatim (pre-run, nothing is jammed yet).

    Returns (changed, messages, mission_name).
    """
    try:
        doc = _load_yaml(path)
    except OSError as exc:
        return [], [f"SETMISSION: cannot read {path}: {exc}"], None

    mission_name = doc.get("name") or Path(path).stem
    networks = (arena or {}).get("networks") or {}
    changed, messages = [], []
    for aid, obj in (doc.get("objectives") or {}).items():
        if not isinstance(obj, dict):
            continue
        block = {("type" if k == "do" else k): v for k, v in obj.items()}
        block.setdefault("type", "static")
        if aid not in agents_by_id:
            messages.append(f"SETMISSION: '{aid}' is not in the running "
                            f"scene - it will not appear")
            continue
        if not _is_taskable(agents_by_id[aid], networks):
            messages.append(f"SETMISSION: '{aid}' is on the adversary side "
                            f"- missions are blue only, skipped")
            continue
        if links is not None and poses is not None:
            auth = command_authority(agents_by_id[aid], arena, links, poses,
                                     networks)
            if not auth.get("reachable", True):
                messages.append(
                    f"SETMISSION: {aid} unreachable (decider "
                    f"{auth.get('decider')}) - not retasked")
                continue
        ok, err = validate_objective(block, points, arena)
        if not ok:
            messages.append(f"SETMISSION: {aid} rejected - {err}")
            agents_by_id[aid]["last_rejection"] = err
            continue
        agents_by_id[aid]["mission"] = block
        agents_by_id[aid]["last_rejection"] = None
        changed.append((aid, block))
    return changed, messages, mission_name


def _resolve_scope(token, agents_by_id, networks):
    """A LAUNCH/HALT scope: a network name, or an agent id. (None, None) if
    it's neither."""
    if token in (networks or {}):
        return "network", token
    if token in agents_by_id:
        return "agent", token
    return None, None


def drain_retasks(retask_dir, agents_by_id, arena, links, poses, t=0.0):
    """Apply any pending commands from the retask queue and return what
    changed.

    A command is one file, `cmd_*.txt`, one line, in retask_dir - see
    "Bug fix: the retask race" in docs/PATCH-08-CHECKS.md for why it isn't
    one shared file any more. Claiming a file (by renaming it) consumes it,
    so a command fires once. File-based rather than a socket because the
    same channel then works from a terminal, from the Console, and later
    from a ROS service, with no protocol to agree on. `t` is the current
    sim time - every place an objective is (re)assigned or an agent is
    (re)armed also resets that agent's `phase_t0` to it, so a shuttle/
    patrol/orbit phase is always measured from "since this became active,"
    never from the run's absolute clock - see "Bug fix: the launch hiccup"
    in docs/PATCH-07-CHECKS.md. Recognises, per line:

        <agent>: <verb> <args>      REOBJECTIVE, one agent (parse_retask)
        LAUNCH <network-or-agent>   arm - see "The state machine"
        HALT <network-or-agent>     un-arm, freezes at current pose
        SETMISSION <name-or-path>   set the run's mission: apply the file's
                                    objectives, gated by command authority
                                    (an unreachable agent is not retasked),
                                    and title the run with its name
        JAM <id> [band <MHz>] [power <dBm>]
                                    tune a running jammer's emission live, in
                                    ONE line so band and power never disagree
                                    (on/off is LAUNCH/HALT, e.g. red launch)
    """
    changed = []
    if not retask_dir.exists():
        return changed

    # ONE FILE PER COMMAND, not one shared file. A shared file that the
    # Console appends to and this function reads-then-clears is a genuine
    # cross-process race on Windows: Python's open() doesn't request
    # FILE_SHARE_DELETE, so while the Console's handle is open (even
    # briefly) THIS function's attempt to claim the file by renaming it can
    # fail outright - and failing to claim it meant reading nothing that
    # poll, not even commands already sitting there from earlier. That
    # failure used to be swallowed silently (`except OSError: pass`), which
    # is exactly the class of bug this whole patch series exists to
    # remove: it looked like "type it again and it works," but what
    # actually happened was several appends piling up unread until a LATER
    # poll finally got in and processed all of them together - which is
    # also why a mistyped command could ride along with its correction.
    #
    # One file per command removes the contention instead of racing it: the
    # Console never reopens an existing path (each command gets a brand new
    # filename, written to a temp name and atomically renamed into place),
    # so nothing here is ever contending with a writer that still has the
    # file open. A claim failure is now genuinely rare, and NEVER silent -
    # see the print() below - and a file that fails to be claimed is left
    # in place for the next poll rather than lost.
    lines = []
    for path in sorted(retask_dir.glob("cmd_*.txt")):
        pending = retask_dir / f"{path.name}.{os.getpid()}.reading"
        try:
            path.rename(pending)
        except OSError as exc:
            print(f"drain_retasks: could not claim {path.name}: {exc}",
                  file=sys.stderr)
            continue
        try:
            lines += pending.read_text(encoding="utf-8").splitlines()
        except OSError as exc:
            print(f"drain_retasks: could not read {pending.name}: {exc}",
                  file=sys.stderr)
        finally:
            try:
                pending.unlink()
            except OSError as exc:
                print(f"drain_retasks: could not remove {pending.name}: "
                      f"{exc}", file=sys.stderr)

    networks = (arena or {}).get("networks") or {}
    points = (arena or {}).get("points") or {}

    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        head = line.split(None, 1)
        verb0 = head[0].upper() if head else ""

        if verb0 in ("LAUNCH", "HALT") and len(head) == 2:
            scope_kind, scope = _resolve_scope(head[1], agents_by_id, networks)
            if scope_kind is None:
                print(f"{verb0}: '{head[1]}' is not a network or agent id",
                      file=sys.stderr)
                continue
            armed = verb0 == "LAUNCH"
            if scope_kind == "network":
                # Ground stations aren't vehicles, so a network-wide
                # LAUNCH/HALT does too now, for the same reason. Naming one
                # explicitly (`gcs launch`) still works - that's a deliberate
                # per-agent action, not this bulk one.
                targets = [a for a in agents_by_id.values()
                          if a["network"] == scope
                          and a.get("platform") != "ground_station"]
            else:
                targets = [agents_by_id[scope]]
            for a in targets:
                a["armed"] = armed
                if armed:
                    # Start cleanly from now, not from wherever the run's
                    # absolute clock happens to be - see the docstring above.
                    a["phase_t0"] = t
            print(f"{verb0}: {scope} ({len(targets)} agent"
                  f"{'s' if len(targets) != 1 else ''})", file=sys.stderr)
            continue

        if verb0 == "JAM" and len(head) == 2:
            # Live jammer edit: JAM <id> power <dBm> | JAM <id> band <MHz>.
            # On/off is the ordinary LAUNCH/HALT state machine (red launch),
            # so this only tunes an existing jammer's emission. Editing the
            # jammer from the Console, not baked into a file.
            # ONE LINE SETS AN EMISSION. `JAM jam1 band 2400 power 25` is
            # unambiguous; two separate commands left a window where the
            # jammer was tuned to the new band at the OLD power, which is a
            # different emission from either of the ones you meant. Any number
            # of key/value pairs, in any order; a single pair still works.
            parts = head[1].split()
            if len(parts) < 3 or (len(parts) - 1) % 2:
                print("JAM: usage JAM <id> [band <MHz>] [power <dBm>]   "
                      "e.g. JAM jam1 band 2400 power 25", file=sys.stderr)
                continue
            jid, rest = parts[0], parts[1:]
            tgt = agents_by_id.get(jid)
            if not tgt or not tgt.get("jammer"):
                print(f"JAM: '{jid}' is not a jammer", file=sys.stderr)
                continue
            pairs, bad = [], False
            for what, valtxt in zip(rest[0::2], rest[1::2]):
                if what.lower() not in ("power", "band"):
                    print(f"JAM: '{what}' is not power or band",
                          file=sys.stderr)
                    bad = True
                    break
                try:
                    pairs.append((what.lower(), float(valtxt)))
                except ValueError:
                    print(f"JAM: '{valtxt}' is not a number", file=sys.stderr)
                    bad = True
                    break
            if bad:
                # NOTHING is applied on a bad line. A half-applied emission is
                # worse than a rejected one: the run continues on settings
                # nobody chose.
                continue
            for what, v in pairs:
                key = "tx_power" if what == "power" else "band"
                cur = tgt["jammer"].get(key)
                if isinstance(cur, dict):
                    cur["value"] = v
                else:
                    tgt["jammer"][key] = {"value": v}
            print("JAM: " + jid + " " +
                  ", ".join(f"{w} -> {v:g}" for w, v in pairs),
                  file=sys.stderr)
            continue

        if verb0 == "SETMISSION" and len(head) == 2:
            ref = head[1].strip()
            mpath = Path(ref)
            if mpath.parent == Path(".") and not mpath.suffix:
                mpath = REPO_ROOT / "missions" / f"{ref}.yaml"
            elif not mpath.is_absolute():
                mpath = REPO_ROOT / mpath
            file_changed, messages, mission_name = apply_mission_file(
                mpath, agents_by_id, points, arena, links=links, poses=poses)
            for msg in messages:
                print(msg, file=sys.stderr)
            if mission_name and file_changed:
                CURRENT_MISSION["name"] = mission_name
                print(f"SETMISSION: mission '{mission_name}' set "
                      f"({len(file_changed)} agents tasked)", file=sys.stderr)
            for aid, _mission in file_changed:
                agents_by_id[aid]["phase_t0"] = t
            changed += file_changed
            continue

        result = parse_retask(line, agents_by_id)
        if result:
            aid, block = result
            if not _is_taskable(agents_by_id[aid], networks):
                print(f"retask: '{aid}' is on the adversary side - missions "
                      f"are blue only, ignored", file=sys.stderr)
                continue
            ok, err = validate_objective(block, points, arena)
            if not ok:
                print(f"retask: {aid} rejected - {err}", file=sys.stderr)
                agents_by_id[aid]["last_rejection"] = err
                continue
            agents_by_id[aid]["mission"] = block
            agents_by_id[aid]["last_rejection"] = None
            agents_by_id[aid]["phase_t0"] = t
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
                for aid, block in drain_retasks(retask_dir, agents_by_id,
                                                arena, links, poses, t):
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
