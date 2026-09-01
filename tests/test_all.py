#!/usr/bin/env python3
"""Deadband test suite.

    python3 tests/test_all.py

No pytest dependency: this must run anywhere the simulator runs, including a
fresh WSL with nothing installed.

WHY THESE TESTS EXIST. Every test here corresponds to a bug that actually
shipped and was caught by hand, late, or by Will rather than by the person who
wrote it:

  * a str_replace clobbered the `def link_state(...)` line and orphaned its
    body - the file still parsed, and only blew up at runtime
  * the sidebar tab order was reversed because the render order was assumed
    rather than checked
  * command_authority() ignored the squads block entirely, so hierarchical mode
    had a hierarchy in the routing and none in the command structure
  * betweenness normalisation capped a 4-node star at 0.5, making it
    indistinguishable from a partial mesh
  * an unresolvable shuttle waypoint silently fell back to the arena
    half-width instead of failing - a retasked car quietly drove a
    different-sized lane than the one it was told
  * the first REMISSION decomposition indexed each squad's members from
    zero, so two squads of three put three pairs of cars in the same lane
  * a REMISSION shuttle line could compute a lane outside the arena and
    would have sailed through unchecked, pinning a car at the wall

Each of those would have been caught in under a second by a test.
"""

import contextlib
import io
import math
import os
import sys
import tempfile
import time
import traceback
from pathlib import Path
from unittest import mock

import yaml

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO))

import stub_telemetry as st

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"   {detail}" if detail and not cond else ""))


# --------------------------------------------------------------------------
# RF model
# --------------------------------------------------------------------------
def test_rf():
    print("\nRF LINK MODEL")
    a = {"x": 0, "y": 0, "z": 0}
    near = st.rf_link(a, {"x": 1, "y": 0, "z": 0})
    far = st.rf_link(a, {"x": 500, "y": 0, "z": 0})

    check("close link is up", near["state"] == "up", near["state"])
    check("distant link is down", far["state"] == "down", far["state"])
    check("received power falls with distance", near["rx_dbm"] > far["rx_dbm"])
    check("PDR falls with distance", near["pdr"] > far["pdr"])
    check("1 m rx power is physically sane (-30..-10 dBm)",
          -30 < near["rx_dbm"] < -10, f"{near['rx_dbm']} dBm")
    check("PDR stays within 0..1",
          all(0.0 <= st.rf_link(a, {"x": d, "y": 0, "z": 0})["pdr"] <= 1.0
              for d in (0.1, 1, 10, 100, 1000)))
    check("monotonic: power never rises with distance",
          all(st.rf_link(a, {"x": d, "y": 0, "z": 0})["rx_dbm"]
              >= st.rf_link(a, {"x": d * 2, "y": 0, "z": 0})["rx_dbm"]
              for d in (1, 5, 25, 125)))
    # Interference must make things worse, never better - this is the hook the
    # jammer will use, so it has to hold before the jammer exists.
    clean = st.rf_link(a, {"x": 50, "y": 0, "z": 0}, interference_mw=0.0)
    jammed = st.rf_link(a, {"x": 50, "y": 0, "z": 0}, interference_mw=1e-6)
    check("interference lowers SINR", jammed["sinr_db"] < clean["sinr_db"],
          f"{jammed['sinr_db']} vs {clean['sinr_db']}")
    check("link_state still callable (it has been clobbered before)",
          isinstance(st.link_state(a, {"x": 1, "y": 0, "z": 0}), dict))


# --------------------------------------------------------------------------
# Topology measurement
# --------------------------------------------------------------------------
def _ring(ids):
    return {n: {"x": 3 * math.cos(2 * math.pi * i / len(ids)),
                "y": 3 * math.sin(2 * math.pi * i / len(ids)),
                "z": 0, "yaw": 0} for i, n in enumerate(ids)}


def test_topology():
    print("\nROUTING AND MEASURED TOPOLOGY")
    ids = ["gcs", "car1", "car2", "car3"]
    poses = _ring(ids)
    agents = [{"id": n, "network": "blue"} for n in ids]
    links = [{"a": ids[i], "b": ids[j], "network": "blue"}
             for i in range(len(ids)) for j in range(i + 1, len(ids))]

    def shape(routing, squads=None):
        net = {"routing": routing, "coordinator": "gcs"}
        if squads:
            net["squads"] = squads
        out = st.apply_routing(links, agents, {"blue": net}, poses)
        return st.observed_topology(out), out

    star, star_links = shape("star")
    mesh, mesh_links = shape("mesh")

    check("declared star measures as star", star["shape"] == "star", star["shape"])
    check("star hub is the coordinator", star["hub"] == "gcs", str(star["hub"]))
    check("star betweenness is high (was capped at 0.5 by a bug)",
          star["max_betweenness"] > 0.9, str(star["max_betweenness"]))
    check("declared mesh measures as mesh", mesh["shape"] == "mesh", mesh["shape"])
    check("mesh betweenness is low", mesh["max_betweenness"] < 0.35,
          str(mesh["max_betweenness"]))
    check("mesh has more active edges than star",
          sum(1 for l in mesh_links if l["active"])
          > sum(1 for l in star_links if l["active"]))
    check("star leaves spare capacity unused",
          any(not l["active"] for l in star_links))
    check("every link reports both active and usable",
          all("active" in l and "usable" in l for l in star_links))


def test_two_squad_hierarchy():
    print("\nTWO-SQUAD HIERARCHY")
    ids = ["gcs", "car1", "car2", "car3", "car4"]
    poses = _ring(ids)
    agents = [{"id": n, "network": "blue"} for n in ids]
    links = [{"a": ids[i], "b": ids[j], "network": "blue"}
             for i in range(len(ids)) for j in range(i + 1, len(ids))]
    net = {"routing": "tiered", "authority": "hierarchical", "coordinator": "gcs",
           "squads": {"alpha": {"leader": "car1", "members": ["car2"]},
                      "bravo": {"leader": "car3", "members": ["car4"]}}}
    out = st.apply_routing(links, agents, {"blue": net}, poses)
    active = {(l["a"], l["b"]) for l in out if l["active"]}

    check("no cross-squad edge (car2-car4)",
          ("car2", "car4") not in active and ("car4", "car2") not in active)
    check("leader reaches coordinator",
          ("gcs", "car1") in active or ("car1", "gcs") in active)
    check("member reaches its leader",
          ("car1", "car2") in active or ("car2", "car1") in active)

    # Authority must follow the squads block. It silently did not, once.
    def auth(aid):
        agent = {"id": aid, "network": "blue"}
        return st.command_authority(agent, {}, links, poses, {"blue": net})

    check("member reports to its squad leader, not the coordinator",
          auth("car2")["decider"] == "car1", str(auth("car2")))
    check("second squad's member reports to ITS leader",
          auth("car4")["decider"] == "car3", str(auth("car4")))
    check("leader reports upward to the coordinator",
          auth("car1")["decider"] == "gcs", str(auth("car1")))
    check("member tier is 'leader'", auth("car2")["tier"] == "leader")


def test_authority_modes():
    print("\nAUTHORITY UNDER LINK LOSS")
    poses = {"gcs": {"x": 0, "y": 0, "z": 0, "yaw": 0},
             "car1": {"x": 900, "y": 0, "z": 0, "yaw": 0}}   # far: link down
    links = [{"a": "gcs", "b": "car1", "network": "blue"}]
    agent = {"id": "car1", "network": "blue"}

    def auth(mode):
        return st.command_authority(
            agent, {}, links, poses,
            {"blue": {"authority": mode, "coordinator": "gcs"}})

    check("centralized loses authority when the hub is unreachable",
          auth("centralized")["reachable"] is False)
    check("decentralized keeps authority regardless",
          auth("decentralized")["reachable"] is True)
    check("decentralized decides for itself",
          auth("decentralized")["decider"] == "car1")
    check("hierarchical with no reachable decider reports orphaned",
          auth("hierarchical")["tier"] == "orphaned",
          str(auth("hierarchical")))


def test_blast_radius():
    print("\nBLAST RADIUS (the architecture result)")
    ids = ["gcs", "car1", "car2", "car3", "car4"]
    poses = _ring(ids)
    agents = [{"id": n, "network": "blue"} for n in ids]
    links = [{"a": ids[i], "b": ids[j], "network": "blue"}
             for i in range(len(ids)) for j in range(i + 1, len(ids))]
    squads = {"alpha": {"leader": "car1", "members": ["car2"]},
              "bravo": {"leader": "car3", "members": ["car4"]}}

    def worst(mode):
        net = {"authority": mode, "coordinator": "gcs", "squads": squads}
        out = []
        for dead in ids:
            L = [l for l in links if dead not in (l["a"], l["b"])]
            lost = sum(1 for a in agents if a["id"] != dead and not
                       st.command_authority(a, {}, L, poses, {"blue": net})["reachable"])
            out.append(lost)
        return max(out)

    c, h, d = worst("centralized"), worst("hierarchical"), worst("decentralized")
    print(f"     worst case: centralized {c}, hierarchical {h}, decentralized {d}")
    check("decentralized is immune to node loss", d == 0, str(d))
    check("hierarchical bounds the damage below centralized", h < c, f"{h} vs {c}")
    check("centralized loses everyone when the hub dies",
          c == len(ids) - 1, str(c))


# --------------------------------------------------------------------------
# Scenes and missions must all load
# --------------------------------------------------------------------------
def test_files_load():
    print("\nEVERY SCENE AND MISSION LOADS")
    for folder in ("scenes", "fleets", "missions"):
        for f in sorted((REPO / folder).glob("*.yaml")):
            try:
                doc = st.resolve_mission(str(f))
                is_dict = isinstance(doc, dict)
                # A SCENE is the world only and legitimately has no agents; a
                # fleet or mission must resolve to agents. See
                # docs/vocabulary.md.
                if folder == "scenes":
                    ok = is_dict
                    why = "did not resolve to a dict"
                elif folder == "missions":
                    # A mission is the COMMAND only: objectives, and NO
                    # scene/fleet of its own. See docs/vocabulary.md.
                    ok = (is_dict and bool(doc.get("objectives"))
                          and not doc.get("agents")
                          and "scene" not in doc and "fleet" not in doc)
                    why = "a mission must be objectives-only"
                else:
                    ok = is_dict and bool(doc.get("agents"))
                    why = "no agents after merge"
                check(f"{folder}/{f.name}", ok, "" if ok else why)
            except Exception as exc:
                check(f"{folder}/{f.name}", False, repr(exc))


def test_three_layer_chain():
    """scene < fleet < mission composes recursively and each layer is honest
    about its job. See docs/vocabulary.md."""
    print("\nTHREE-LAYER CHAIN (scene < fleet < mission)")

    # A scene is the world only.
    scene = st.resolve_mission(str(REPO / "scenes" / "lab_box.yaml"))
    check("scene has an arena", bool(scene.get("arena")))
    check("scene defines the lane points",
          set("ABCDEF") <= set(scene.get("points") or {}))
    check("scene has NO agents", not scene.get("agents"))
    check("scene has NO networks", not scene.get("networks"))

    # A fleet is the agents and their wiring, no world, no tasking.
    fleet = st.resolve_mission(str(REPO / "fleets" / "3_roboracer.yaml"))
    check("fleet defines the lab agents",
          {a.get("id") for a in fleet.get("agents") or []}
          == {"gcs", "car1", "car2", "car3"})
    check("fleet declares the blue network",
          "blue" in (fleet.get("networks") or {}))
    check("fleet has NO arena of its own", not fleet.get("arena"))
    check("fleet bakes NO objectives",
          not any(a.get("mission") for a in fleet.get("agents") or []))

    # A mission is the COMMAND only - objectives, cut off from scene and
    # fleet entirely. Composition happens in Setup (or default_run.yaml);
    # the mission is applied at the terminal with SETMISSION.
    msn = st.resolve_mission(str(REPO / "missions" / "test.yaml"))
    check("mission carries objectives",
          set(msn.get("objectives") or {}) == {"car1", "car2", "car3"})
    check("mission names NO scene and NO fleet",
          "scene" not in msn and "fleet" not in msn)
    check("mission carries NO agents of its own", not msn.get("agents"))

    # Scene + fleet compose into a runnable world (a dict, no file needed -
    # this is exactly what the Console's Setup tab does)...
    arena, agents, links = st.load_scenario(
        {"scene": "lab_box", "fleet": "3_roboracer"})
    check("scene+fleet compose into the full fleet", len(agents) == 4)
    check("composition carries the blue links",
          any(l["network"] == "blue" for l in links))
    composed = st.resolve_doc({"scene": "lab_box", "fleet": "3_roboracer"})
    car1a = next(a for a in composed["agents"] if a["id"] == "car1")
    check("provenance survives the merge",
          "Measured" in str(car1a["dimensions"]))

    # ...and SETMISSION then applies the command to the running fleet.
    agents_by_id = {a["id"]: a for a in agents}
    poses = {a["id"]: {"x": a["start"]["x"], "y": a["start"]["y"],
                       "z": a["start"]["z"], "yaw": a["start"]["yaw"],
                       "speed": 0.0} for a in agents}
    changed, msgs, name = st.apply_mission_file(
        str(REPO / "missions" / "test.yaml"), agents_by_id,
        arena["points"], arena, links=links, poses=poses)
    check("SETMISSION tasks the three cars", len(changed) == 3, str(msgs))
    check("SETMISSION reports the mission name for the run title",
          name == "test")
    check("car1 got its lane",
          agents_by_id["car1"]["mission"].get("between") == ["A", "B"])


def test_mission_contract():
    print("\nMISSION CONTRACT")
    for f in sorted((REPO / "missions").glob("*.py")):
        try:
            mod = st.load_mission_script(str(f.relative_to(REPO)))
            check(f"{f.name} defines target()", hasattr(mod, "target"))
        except Exception as exc:
            check(f"{f.name} imports", False, repr(exc))


def test_retask_grammar():
    print("\nRETASK GRAMMAR")
    agents = {"car1": {"id": "car1", "mission": {}},
              "car3": {"id": "car3", "mission": {}}}
    cases = [
        ("car1: pursue car3", "pursuit"),
        ("car1: shuttle between A B", "shuttle"),
        ("car1: stop", "static"),
        ("car1: wall_follow right", "script"),
    ]
    for line, expect in cases:
        got = st.parse_retask(line, agents)
        check(f"'{line}' -> {expect}",
              got is not None and got[1].get("type") == expect, str(got))
    check("unknown agent is rejected",
          st.parse_retask("nosuch: pursue car3", agents) is None)
    check("garbage is rejected without raising",
          st.parse_retask("!!!", agents) is None)


# --------------------------------------------------------------------------
# Patch 7: assign/inspect/launch gate, REMISSION decomposition, LOADMISSION
# --------------------------------------------------------------------------
def _poses_for(agents):
    """Fresh poses at each agent's spawn point - the shape stream() builds."""
    return {a["id"]: {"x": a["start"]["x"], "y": a["start"]["y"],
                      "z": a["start"]["z"], "yaw": a["start"]["yaw"],
                      "speed": 0.0}
           for a in agents}


def _queue_with(qdir, line):
    """Write one command exactly the way the Console does now - its own
    brand-new cmd_*.txt file, never an overwrite of a shared path. See
    "Bug fix: the retask race" in docs/PATCH-08-CHECKS.md."""
    qdir.mkdir(parents=True, exist_ok=True)
    (qdir / f"cmd_{time.monotonic_ns():020d}.txt").write_text(line)


def test_armed_gating():
    print("\nARMED GATING (the assign/inspect/launch state machine)")
    arena, agents, links = st.load_scenario(str(REPO / "default_run.yaml"))
    car1 = next(a for a in agents if a["id"] == "car1")
    car1["mission"] = {"type": "shuttle",
                       "between": [{"x": -3.0, "y": 0.0}, {"x": 3.0, "y": 0.0}]}
    poses = _poses_for(agents)
    poses["car1"]["x"] = -3.0
    check("armed defaults to False", car1["armed"] is False)

    st.step(agents, poses, 0.0, 0.1, arena)
    check("an unarmed shuttle-assigned agent does not move",
          poses["car1"]["x"] == -3.0)

    car1["armed"] = True
    st.step(agents, poses, 0.0, 0.1, arena)
    check("the same agent, armed, moves",
          poses["car1"]["x"] != -3.0, str(poses["car1"]["x"]))


def test_launch_halt_scope():
    print("\nLAUNCH / HALT SCOPE RESOLUTION")
    arena, agents, links = st.load_scenario(str(REPO / "default_run.yaml"))
    agents_by_id = {a["id"]: a for a in agents}
    poses = _poses_for(agents)
    qdir = Path(tempfile.mkdtemp())

    _queue_with(qdir, "LAUNCH blue\n")
    st.drain_retasks(qdir, agents_by_id, arena, links, poses)
    check("LAUNCH blue arms every blue-network VEHICLE",
          all(a["armed"] for a in agents
              if a["network"] == "blue" and a["platform"] != "ground_station"))
    check("LAUNCH blue does NOT arm the ground station (matches REMISSION)",
          agents_by_id["gcs"]["armed"] is False)

    _queue_with(qdir, "LAUNCH gcs\n")
    st.drain_retasks(qdir, agents_by_id, arena, links, poses)
    check("naming the ground station explicitly still arms it",
          agents_by_id["gcs"]["armed"] is True)

    before = dict(agents_by_id["car1"]["mission"])
    _queue_with(qdir, "HALT car1\n")
    st.drain_retasks(qdir, agents_by_id, arena, links, poses)
    check("HALT car1 un-arms only car1",
          agents_by_id["car1"]["armed"] is False
          and agents_by_id["car2"]["armed"] is True)
    check("HALT does not touch the mission dict",
          agents_by_id["car1"]["mission"] == before)

    _queue_with(qdir, "LAUNCH nosuchthing\n")
    try:
        st.drain_retasks(qdir, agents_by_id, arena, links, poses)
        raised = False
    except Exception:
        raised = True
    check("an unknown LAUNCH/HALT scope is rejected without raising", not raised)


def test_coordinate_grammar():
    print("\nCOORDINATE GRAMMAR: (x,y[,z])")
    check("'(-3, 3, 0)' tolerates the internal space",
          st._parse_position_token("(-3, 3, 0)") == {"x": -3.0, "y": 3.0, "z": 0.0})
    check("'(3,2)' defaults z to 0.0",
          st._parse_position_token("(3,2)") == {"x": 3.0, "y": 2.0, "z": 0.0})
    check("a bare word stays a point name",
          st._parse_position_token("A") == "A")

    agents = {"car1": {"id": "car1", "mission": {}}}
    mixed = st.parse_retask("car1: shuttle A (3,2,0)", agents)
    check("'shuttle A (3,2,0)' resolves one name, one literal",
          mixed is not None
          and mixed[1]["between"] == ["A", {"x": 3.0, "y": 2.0, "z": 0.0}],
          str(mixed))
    check("a malformed literal is rejected without raising",
          st.parse_retask("car1: shuttle (1,2,3,4) (3,2)", agents) is None)


def test_waypoint_rejection_exact_repro():
    print("\nWAYPOINT REJECTION - THE EXACT REPRO")
    # The original repro lived in three_car_fleet.yaml (now attic/): a file
    # defining no named points at all, car1's own shuttle raw {x,y} literals.
    # REOBJECTIVE car1 shuttle A B used to fall back to the arena half-width
    # on BOTH ends and quietly sweep -4..+4 instead of car1's real -3..+3
    # lane. Reproduced here with an equivalent point-less fixture.
    fix = {"spec_version": 0.1, "name": "no_points_fixture",
           "arena": {"type": "box", "extent": {"x": 8.0, "y": 8.0, "z": 3.0}},
           "agents": [{"id": "car1", "platform": "roboracer",
                       "network": "blue",
                       "pose": {"x": -3.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
                       "mission": {"type": "shuttle",
                                   "between": [{"x": -3.0, "y": 2.0},
                                               {"x": 3.0, "y": 2.0}]}}],
           "networks": {"blue": {"topology": "centralized"}}}
    tmp = Path(tempfile.mkdtemp()) / "no_points.yaml"
    tmp.write_text(yaml.safe_dump(fix))
    arena, agents, links = st.load_scenario(str(tmp))
    agents_by_id = {a["id"]: a for a in agents}
    check("the fixture defines no named points", not arena["points"])
    before = dict(agents_by_id["car1"]["mission"])

    result = st.parse_retask("car1: shuttle A B", agents_by_id)
    ok, err = st.validate_objective(result[1], arena["points"], arena)
    check("'shuttle A B' is rejected - A/B are undefined on this map",
          not ok, str(err))
    check("car1's mission dict is byte-for-byte unchanged",
          agents_by_id["car1"]["mission"] == before)


def _two_squad_path():
    """A 30x20 hall with gcs + six cars in two squads (alpha: car1 leads
    car2/car3; bravo: car4 leads car5/car6) - the shape squad_hall.yaml (now
    attic/) existed for, as a generated fixture so the archive stays optional."""
    def car(cid, x, y):
        return {"id": cid, "platform": "roboracer", "network": "blue",
                "pose": {"x": x, "y": y, "z": 0.0, "yaw": 0.0},
                "dimensions": {"length": 0.55, "width": 0.30, "height": 0.20},
                "performance": {"max_speed": 2.0}}
    doc = {"spec_version": 0.1, "name": "two_squad_fixture",
           "arena": {"type": "box",
                     "extent": {"x": 30.0, "y": 20.0, "z": 5.0}},
           "networks": {"blue": {
               "authority": "hierarchical", "routing": "tiered",
               "topology": "hierarchical", "coordinator": "gcs",
               "squads": {"alpha": {"leader": "car1",
                                    "members": ["car2", "car3"]},
                          "bravo": {"leader": "car4",
                                    "members": ["car5", "car6"]}}}},
           "agents": [{"id": "gcs", "platform": "ground_station",
                       "network": "blue", "ghost": True,
                       "pose": {"x": 0.0, "y": -8.0, "z": 0.0, "yaw": 0.0}},
                      car("car1", -10.0, 5.0), car("car2", -10.0, 2.0),
                      car("car3", -10.0, -1.0), car("car4", 10.0, 5.0),
                      car("car5", 10.0, 2.0), car("car6", 10.0, -1.0)]}
    tmp = Path(tempfile.mkdtemp()) / "two_squads.yaml"
    tmp.write_text(yaml.safe_dump(doc))
    return str(tmp)


def _mission_file(objectives, name="tmp_mission"):
    """Write a pure-command mission file (objectives only) and return its
    path - the shape SETMISSION consumes."""
    doc = {"spec_version": 0.1, "name": name, "kind": "mission",
           "objectives": objectives}
    tmp = Path(tempfile.mkdtemp()) / f"{name}.yaml"
    tmp.write_text(yaml.safe_dump(doc))
    return str(tmp)


def test_setmission_applies_and_names():
    print("\nSETMISSION - APPLIES THE COMMAND, NAMES THE RUN")
    arena, agents, links = st.load_scenario(str(REPO / "default_run.yaml"))
    agents_by_id = {a["id"]: a for a in agents}
    poses = _poses_for(agents)
    qdir = Path(tempfile.mkdtemp())
    mpath = _mission_file({"car1": {"do": "shuttle", "between": ["A", "B"]},
                           "car2": {"do": "shuttle", "between": ["C", "D"]}},
                          name="named_run")
    _queue_with(qdir, f"SETMISSION {mpath}\n")
    changed = st.drain_retasks(qdir, agents_by_id, arena, links, poses)
    check("both named agents are tasked",
          {aid for aid, _ in changed} == {"car1", "car2"})
    check("the run is titled by the mission's name",
          st.CURRENT_MISSION["name"] == "named_run")
    check("car3 (not in the mission) is untouched",
          agents_by_id["car3"]["mission"] == {"type": "static"}
          or "between" not in agents_by_id["car3"]["mission"]
          or agents_by_id["car3"]["mission"].get("between") != ["A", "B"])


def test_setmission_by_name():
    print("\nSETMISSION - BARE NAME RESOLVES UNDER missions/")
    arena, agents, links = st.load_scenario(str(REPO / "default_run.yaml"))
    agents_by_id = {a["id"]: a for a in agents}
    poses = _poses_for(agents)
    qdir = Path(tempfile.mkdtemp())
    _queue_with(qdir, "SETMISSION test\n")
    changed = st.drain_retasks(qdir, agents_by_id, arena, links, poses)
    got = {aid: m.get("between") for aid, m in changed}
    check("SETMISSION test applies missions/test.yaml's three lanes",
          got == {"car1": ["A", "B"], "car2": ["C", "D"],
                  "car3": ["E", "F"]}, str(got))
    check("run titled 'test'", st.CURRENT_MISSION["name"] == "test")


def test_setmission_gated_by_reachability():
    print("\nSETMISSION - A COMMAND MUST REACH AN AGENT TO TASK IT")
    arena, agents, links = st.load_scenario(_two_squad_path())
    agents_by_id = {a["id"]: a for a in agents}
    poses = _poses_for(agents)
    # car2 wanders far out of range of EVERYONE - its leader car1, and the
    # coordinator the hierarchy would fall back to. Orphaned: no decider can
    # reach it, so no command can. (A leader merely losing its uplink is NOT
    # enough to strand its squad - command_authority falls back to the
    # coordinator by design: degraded, not decapitated.)
    poses["car2"]["x"] = 9000.0
    before = {aid: dict(a["mission"]) for aid, a in agents_by_id.items()}
    auth = st.command_authority(agents_by_id["car2"], arena, links, poses,
                                arena["networks"])
    check("car2 is genuinely orphaned first", auth["reachable"] is False,
          str(auth))

    qdir = Path(tempfile.mkdtemp())
    mpath = _mission_file({c: {"do": "shuttle",
                               "between": [{"x": -10.0, "y": float(i)},
                                           {"x": 10.0, "y": float(i)}]}
                           for i, c in enumerate(
                               ["car1", "car2", "car3", "car4", "car5",
                                "car6"], start=-3)}, name="squad_order")
    _queue_with(qdir, f"SETMISSION {mpath}\n")
    changed = st.drain_retasks(qdir, agents_by_id, arena, links, poses)
    tasked = {aid for aid, _ in changed}

    check("the five reachable agents are freshly tasked",
          {"car1", "car3", "car4", "car5", "car6"} <= tasked)
    check("the orphaned agent is skipped - no decider can reach it",
          "car2" not in tasked)
    check("the orphaned agent's mission is unchanged from before the order",
          agents_by_id["car2"]["mission"] == before["car2"])


def test_setmission_out_of_bounds():
    print("\nSETMISSION - OUT OF BOUNDS IS REJECTED PER AGENT, NOT CLAMPED")
    arena, agents, links = st.load_scenario(str(REPO / "default_run.yaml"))
    agents_by_id = {a["id"]: a for a in agents}
    poses = _poses_for(agents)
    before = {aid: dict(a["mission"]) for aid, a in agents_by_id.items()}
    qdir = Path(tempfile.mkdtemp())
    # lab_box is 8x8; car1's lane is a metre outside the room, car2's fits.
    mpath = _mission_file({"car1": {"do": "shuttle",
                                    "between": [{"x": -3.0, "y": 5.0},
                                                {"x": 3.0, "y": 5.0}]},
                           "car2": {"do": "shuttle", "between": ["C", "D"]}})
    _queue_with(qdir, f"SETMISSION {mpath}\n")
    changed = st.drain_retasks(qdir, agents_by_id, arena, links, poses)
    tasked = {aid for aid, _ in changed}
    check("the in-bounds agent is tasked", tasked == {"car2"})
    check("the out-of-bounds agent is rejected, mission unchanged",
          agents_by_id["car1"]["mission"] == before["car1"])
    check("the rejection is recorded on the agent",
          bool(agents_by_id["car1"]["last_rejection"]))


def test_phase_t0_clean_launch():
    print("\nBUG FIX: LAUNCH STARTS THE PHASE CLEANLY, NOT FROM ABSOLUTE TIME")
    arena, agents, links = st.load_scenario(str(REPO / "default_run.yaml"))
    agents_by_id = {a["id"]: a for a in agents}
    agents_by_id["car1"]["mission"] = {
        "type": "shuttle",
        "between": [{"x": -3.0, "y": 0.0}, {"x": 3.0, "y": 0.0}]}
    poses = _poses_for(agents)
    poses["car1"]["x"], poses["car1"]["y"] = -3.0, 0.0

    qdir = Path(tempfile.mkdtemp())
    _queue_with(qdir, "LAUNCH car1\n")
    # Arm at t=30s, as if the agent had sat idle, unarmed, that whole time -
    # exactly the repro that caught this bug.
    st.drain_retasks(qdir, agents_by_id, arena, links, poses, t=30.0)
    check("phase_t0 is set to the arm time, not left at the default 0.0",
          agents_by_id["car1"]["phase_t0"] == 30.0)

    tx, ty = st.mission_target(agents_by_id["car1"], 30.0, poses, arena)
    dist = math.hypot(tx - (-3.0), ty - 0.0)
    check("the target at the instant of launch is at the agent's own "
          "position, not wherever a stale 30s-old absolute clock would put "
          "it (that used to be 3 m away, a visible jump)",
          dist < 1e-6, f"target=({tx:.2f},{ty:.2f}) dist={dist:.2f}")


def test_retask_spool_is_one_file_per_command():
    print("\nBUG FIX: RETASK QUEUE IS ONE FILE PER COMMAND, NOT ONE SHARED FILE")
    arena, agents, links = st.load_scenario(str(REPO / "default_run.yaml"))
    agents_by_id = {a["id"]: a for a in agents}
    poses = _poses_for(agents)
    qdir = Path(tempfile.mkdtemp())

    # Order is preserved (monotonic filenames), and a stale ".reading" file
    # left behind by a crashed claim doesn't block anything real - it
    # simply doesn't match the cmd_*.txt glob, so it's inert.
    stale = qdir / f"cmd_000000000000000.txt.{os.getpid()}.reading"
    stale.write_text("car1: stop\n")
    _queue_with(qdir, "car1: pursue car3\n")
    _queue_with(qdir, "car2: stop\n")
    changed = st.drain_retasks(qdir, agents_by_id, arena, links, poses)
    check("both commands are processed, in the order they were sent",
          [aid for aid, _ in changed] == ["car1", "car2"], str(changed))
    check("a non-matching stale .reading file is left alone, not read",
          agents_by_id["car1"]["mission"].get("type") == "pursuit")
    check("every cmd_*.txt file is gone after being processed",
          not list(qdir.glob("cmd_*.txt")))


def test_retask_claim_failure_is_never_silent_and_not_lost():
    print("\nBUG FIX: A FAILED CLAIM IS LOGGED, NEVER SILENT, AND NEVER LOST")
    # This is the actual live bug: on Windows, claiming a command file (by
    # renaming it) can fail while a writer's handle is still open. The old
    # code swallowed that with `except OSError: pass` and skipped reading
    # ANYTHING that poll - not just the contended file - which is why a
    # command sometimes needed to be typed twice. Simulate exactly that:
    # the claim (Path.rename) fails once, as if Windows had refused it.
    arena, agents, links = st.load_scenario(str(REPO / "default_run.yaml"))
    agents_by_id = {a["id"]: a for a in agents}
    poses = _poses_for(agents)
    qdir = Path(tempfile.mkdtemp())
    _queue_with(qdir, "car1: stop\n")
    cmd_path = next(qdir.glob("cmd_*.txt"))

    real_rename = Path.rename
    calls = {"n": 0}

    def flaky_rename(self, target):
        if self.name == cmd_path.name and calls["n"] == 0:
            calls["n"] += 1
            raise OSError("simulated Windows sharing violation")
        return real_rename(self, target)

    stderr = io.StringIO()
    with mock.patch.object(Path, "rename", flaky_rename):
        with contextlib.redirect_stderr(stderr):
            changed = st.drain_retasks(qdir, agents_by_id, arena, links, poses)
    check("a failed claim is printed, not silently swallowed",
          "could not claim" in stderr.getvalue() and cmd_path.name in stderr.getvalue(),
          stderr.getvalue())
    check("nothing was processed on the failed attempt", changed == [])
    check("the command file is left in place, not lost", cmd_path.exists())

    # The next poll - the "lock" is gone now - picks it up successfully.
    changed2 = st.drain_retasks(qdir, agents_by_id, arena, links, poses)
    check("a later poll successfully processes the same command",
          [aid for aid, _ in changed2] == ["car1"], str(changed2))
    check("the command file is gone once actually processed",
          not cmd_path.exists())


def test_origin_passthrough():
    print("\nGEODETIC ORIGIN - SCHEMA SEAM, NO CONVERSION")
    arena, _, _ = st.load_scenario(str(REPO / "scenes" / "lab_box.yaml"))
    check("a scene with no arena.origin loads with origin None",
          arena["origin"] is None)

    doc = yaml.safe_load((REPO / "scenes" / "lab_box.yaml").read_text())
    doc["arena"]["origin"] = {
        "lat": {"value": 52.5, "unit": "deg", "source": "test"},
        "lon": {"value": -1.2, "unit": "deg", "source": "test"},
        "alt": {"value": 78.0, "unit": "m", "source": "test"},
        "heading": {"value": 0.0, "unit": "deg", "source": "test"},
    }
    tmp = Path(tempfile.mkdtemp()) / "with_origin.yaml"
    tmp.write_text(yaml.safe_dump(doc))
    arena2, _, _ = st.load_scenario(str(tmp))
    check("a scene with arena.origin carries lat/lon/alt/heading through",
          arena2["origin"] == {"lat": 52.5, "lon": -1.2, "alt": 78.0, "heading": 0.0},
          str(arena2["origin"]))


if __name__ == "__main__":
    for fn in (test_rf, test_topology, test_two_squad_hierarchy,
               test_authority_modes, test_blast_radius, test_files_load, test_three_layer_chain,
               test_mission_contract, test_retask_grammar,
               test_armed_gating, test_launch_halt_scope,
               test_coordinate_grammar, test_waypoint_rejection_exact_repro,
               test_setmission_applies_and_names, test_setmission_by_name,
               test_setmission_gated_by_reachability,
               test_setmission_out_of_bounds,
               test_phase_t0_clean_launch,
               test_retask_spool_is_one_file_per_command,
               test_retask_claim_failure_is_never_silent_and_not_lost,
               test_origin_passthrough):
        try:
            fn()
        except Exception:
            FAIL.append(fn.__name__)
            print(f"  FAIL  {fn.__name__} raised:")
            traceback.print_exc()

    print(f"\n{'=' * 60}")
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        for n in FAIL:
            print(f"  failed: {n}")
    sys.exit(1 if FAIL else 0)
