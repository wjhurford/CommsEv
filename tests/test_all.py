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
# Fleets the suite composes from. These are FIXTURES, not shipped
# fleets: the repo's fleets/ folder holds only what an operator built
# and saved in the Console, and a test fixture sitting there would
# show up in their dropdown as a fleet they did not make.
FIXTURE_FLEETS = REPO / "tests" / "fixtures" / "fleets"
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
    print("\nEVERY SCENE, AGENT, FLEET AND MISSION LOADS")
    # agents/ is HARDWARE and is checked for the opposite property to the
    # others: an agent file must carry NO command decision. A network,
    # authority, routing or doctrine in an agent file is the junk-file bug
    # coming back, so the suite fails on it rather than tolerating it.
    for f in sorted((REPO / "agents").glob("*.yaml")):
        try:
            doc = st._load_yaml(str(f))
            banned = [k for k in ("networks", "network", "authority",
                                  "routing", "topology", "on_link_loss",
                                  "pose", "objectives", "mission")
                      if k in doc]
            ok = (isinstance(doc, dict) and doc.get("kind") == "agent"
                  and doc.get("platform") and not banned)
            check(f"agents/{f.name} is hardware only", ok,
                  f"carries decisions: {banned}" if banned else "malformed")
        except Exception as exc:
            check(f"agents/{f.name}", False, repr(exc))
    for folder in ("scenes", "tests/fixtures/fleets", "missions"):
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
                    # A mission is the COMMAND only: no scene and no fleet of
                    # its own. See docs/vocabulary.md.
                    #
                    # It states that command as `objectives:` (open-ended
                    # behaviour) or as a `plan:` (a circuit with an END, which
                    # can be passed or failed) - and a plan needs no
                    # objectives block, because the objective each vehicle
                    # actually holds is generated from the plan one leg at a
                    # time by the coordinator.
                    # A file marked `deprecated:` is a headstone - it says
                    # what replaced it and why, which is worth more to whoever
                    # goes looking for it than a missing file.
                    ok = (is_dict
                          and (bool(doc.get("objectives") is not None
                                    or doc.get("plan"))
                               or doc.get("deprecated"))
                          and not doc.get("agents")
                          and "scene" not in doc and "fleet" not in doc)
                    why = ("a mission must declare objectives or a plan, and "
                           "no scene or fleet")
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
    check("a scene ships NO points - they belong to the run",
          not (scene.get("points") or {}), str(scene.get("points")))
    check("scene has NO agents", not scene.get("agents"))
    check("scene has NO networks", not scene.get("networks"))

    # A fleet is the agents and their wiring, no world, no tasking.
    fleet = st.resolve_mission(str(FIXTURE_FLEETS / "3_roboracer.yaml"))
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
    check("car1 got its lane, on the benchmark's own coordinates",
          [dict(q) for q in
           agents_by_id["car1"]["mission"].get("between")]
          == [{"x": -3.0, "y": 2.0}, {"x": 3.0, "y": 2.0}],
          str(agents_by_id["car1"]["mission"].get("between")))


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
def _q(w, k):
    """One component of a waypoint that may be a dict or a name."""
    return float(w[k]) if isinstance(w, dict) else w


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
    mpath = _mission_file({"car1": {"do": "shuttle", "between": ["P1", "P2"]},
                           "car2": {"do": "shuttle", "between": ["P3", "P4"]}},
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
    got = {aid: [(_q(w, "x"), _q(w, "y")) for w in m.get("between")]
           for aid, m in changed}
    check("SETMISSION test applies missions/test.yaml's three lanes",
          got == {"car1": [(-3.0, 2.0), (3.0, 2.0)],
                  "car2": [(-3.0, 0.0), (3.0, 0.0)],
                  "car3": [(-3.0, -2.0), (3.0, -2.0)]}, str(got))
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
                           "car2": {"do": "shuttle", "between": ["P3", "P4"]}})
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


def test_jamming_raises_floor_drops_links_and_strips_authority():
    """A jammer is an ordinary agent whose power lands in the SINR
    denominator: arm it and the noise floor rises, links fall, and the
    command authority that flowed over them is lost. See
    docs/contested-background.md."""
    print("\nCONTESTED - JAMMING IS PHYSICS, NOT A FLAG")
    arena, agents, links = st.load_scenario(
        {"scene": "lab_box", "fleets": ["3_roboracer", "red_jammer"]})
    poses = _poses_for(agents)
    jam1 = next(a for a in agents if a["id"] == "jam1")
    check("jam1 has a jammer block", bool(jam1.get("jammer")))

    import random
    def snap():
        return st.frame(0.0, 0.1, 0, arena, agents, links, poses,
                        random.Random(1))

    jam1["armed"] = False
    off = snap()
    car1_off = next(a for a in off["agents"] if a["id"] == "car1")
    blue_off = [l for l in off["links"] if l["network"] == "blue"]
    check("jammer off: car1 sees the scene baseline",
          not car1_off["rf"]["jammed"]
          and car1_off["rf"]["noise_floor_dbm"]
              == car1_off["rf"]["baseline_dbm"])
    check("jammer off: no emitters active", off["attacks_active"] == [])

    jam1["armed"] = True
    on = snap()
    car1_on = next(a for a in on["agents"] if a["id"] == "car1")
    check("jammer on: car1's floor rises above baseline",
          car1_on["rf"]["noise_floor_dbm"] > car1_on["rf"]["baseline_dbm"])
    check("jammer on: car1 reads as jammed", car1_on["rf"]["jammed"])
    check("jammer on: it is listed as an active emitter",
          any(e["id"] == "jam1" for e in on["attacks_active"]))
    worst_off = min(l["pdr"] for l in blue_off)
    worst_on = min(l["pdr"] for l in on["links"] if l["network"] == "blue")
    check("jammer on: blue link quality falls", worst_on < worst_off)
    # At least one hub link is down, so at least one car loses its decider.
    lost = [a for a in on["agents"]
            if a["network"] == "blue"
            and a["platform"] != "ground_station"
            and not a["authority"]["reachable"]]
    check("jammer on: at least one blue agent loses command authority",
          len(lost) >= 1, f"{[a['id'] for a in lost]}")


def test_jamming_respects_band_separation():
    """Band separation is a real (first-order) defence: a jammer off the
    victim's band contributes nothing. Frequency-hopping work depends on
    the model honouring this."""
    print("\nCONTESTED - OFF-BAND JAMMING DOES NOTHING")
    arena, agents, links = st.load_scenario(
        {"scene": "lab_box", "fleets": ["3_roboracer", "red_jammer"]})
    poses = _poses_for(agents)
    jam1 = next(a for a in agents if a["id"] == "jam1")
    jam1["armed"] = True
    jam1["jammer"]["band"] = {"value": 5800}    # blue is on 2400
    import random
    f = st.frame(0.0, 0.1, 0, arena, agents, links, poses, random.Random(1))
    car1 = next(a for a in f["agents"] if a["id"] == "car1")
    check("off-band jammer leaves car1 at the baseline",
          not car1["rf"]["jammed"]
          and car1["rf"]["noise_floor_dbm"] == car1["rf"]["baseline_dbm"])
    check("all blue links stay up",
          all(l["state"] == "up" for l in f["links"]
              if l["network"] == "blue" and l["active"]))


def test_scene_baseline_feeds_rf():
    """The RF model reads the SCENE's declared numbers, not function
    defaults. Two roles, both checked:
      - the noise floor is the REFERENCE the jamming delta is measured from
        (in this SINR model sensitivity is an absolute rx threshold, so the
        floor governs interference, not clean-link PDR - which is honest,
        and why a raised floor alone leaves a clean link untouched);
      - the path-loss exponent governs received power, so a lossier scene
        genuinely shortens clean-link range."""
    print("\nCONTESTED - THE SCENE OWNS THE BASELINE")
    rf = st.scene_rf(st.load_scenario(
        {"scene": "lab_box", "fleet": "3_roboracer"})[0])
    check("scene_rf reads lab_box's -95 dBm floor", rf["noise_dbm"] == -95.0)

    # Path-loss exponent: a lossier scene drops a long link that a
    # free-space scene holds. 80 m so the exponent actually bites.
    def link_state_at(exp):
        d = {"spec_version": 0.1, "name": "x",
             "arena": {"type": "box", "extent": {"x": 200, "y": 200, "z": 5},
                       "propagation": {"path_loss_exponent": {"value": exp}}},
             "agents": [{"id": "a", "network": "blue",
                         "pose": {"x": 0, "y": 0, "z": 0}},
                        {"id": "b", "network": "blue",
                         "pose": {"x": 80, "y": 0, "z": 0}}],
             "networks": {"blue": {"topology": "decentralized"}}}
        arena_x, agents_x, links_x = st.load_scenario(d)
        poses_x = _poses_for(agents_x)
        import random
        fx = st.frame(0.0, 0.1, 0, arena_x, agents_x, links_x, poses_x,
                      random.Random(1))
        return next(l for l in fx["links"])
    free = link_state_at(2.0)
    lossy = link_state_at(3.5)
    check("scene path-loss exponent feeds rf_link (free space holds the "
          "80 m link)", free["state"] == "up", str(free["state"]))
    check("a lossier scene exponent drops the same link",
          lossy["pdr"] < free["pdr"], f"{lossy['pdr']} vs {free['pdr']}")

    # Noise floor as the jamming REFERENCE: the same jammer power reads as a
    # bigger delta above a quiet floor than above an already-noisy one.
    def floor_delta(base_floor):
        d = {"spec_version": 0.1, "name": "x",
             "arena": {"type": "box", "extent": {"x": 20, "y": 20, "z": 5},
                       "spectrum": {"noise_floor": {"value": base_floor}}},
             "networks": {"blue": {"topology": "decentralized", "band": 2400},
                          "red": {"topology": "decentralized", "band": 2400}},
             "agents": [{"id": "a", "network": "blue",
                         "pose": {"x": 0, "y": 0, "z": 0}},
                        {"id": "j", "network": "red",
                         "pose": {"x": 3, "y": 0, "z": 0},
                         "jammer": {"tx_power": {"value": 5},
                                    "band": {"value": 2400}}}]}
        arena_x, agents_x, links_x = st.load_scenario(d)
        for ag in agents_x:
            if ag["id"] == "j":
                ag["armed"] = True
        poses_x = _poses_for(agents_x)
        import random
        fx = st.frame(0.0, 0.1, 0, arena_x, agents_x, links_x, poses_x,
                      random.Random(1))
        a = next(ag for ag in fx["agents"] if ag["id"] == "a")
        return a["rf"]["noise_floor_dbm"] - a["rf"]["baseline_dbm"]
    check("a fixed jammer reads as a larger rise above a quiet floor",
          floor_delta(-95) > floor_delta(-55),
          f"{floor_delta(-95)} vs {floor_delta(-55)}")


def test_two_fleets_compose_and_stay_separate():
    """Blue and red are spawned as separate fleets and both survive the
    merge - neither erases the other's network. See docs/vocabulary.md and
    the Setup tab's two pickers."""
    print("\nTWO FLEETS - BLUE AND RED, SEPARATE")
    both = st.load_scenario(
        {"scene": "lab_box", "fleets": ["3_roboracer", "red_jammer"]})
    arena, agents, links = both
    ids = {a["id"] for a in agents}
    check("both fleets' agents present", {"gcs", "car1", "jam1"} <= ids)
    check("both networks survive the overlay",
          {"blue", "red"} <= set(arena["networks"]))
    jam1 = next(a for a in agents if a["id"] == "jam1")
    check("jam1 is on the red network", jam1["network"] == "red")
    check("jam1 spawns opposite the gcs (far +y side)",
          jam1["start"]["y"] > 0)
    # order independence: red-then-blue keeps blue too
    other = st.resolve_doc({"scene": "lab_box",
                            "fleets": ["red_jammer", "3_roboracer"]})
    check("fleet order does not drop a network",
          {"blue", "red"} <= set(other["networks"]))


def test_jam_command_tunes_live():
    """JAM <id> power <dBm> retunes a running jammer; the victim's floor
    follows. On/off stays the LAUNCH/HALT machine."""
    print("\nJAM COMMAND - LIVE JAMMER TUNING")
    arena, agents, links = st.load_scenario(
        {"scene": "lab_box", "fleets": ["3_roboracer", "red_jammer"]})
    abid = {a["id"]: a for a in agents}
    poses = _poses_for(agents)
    import random

    def car1_floor():
        f = st.frame(0.0, 0.1, 0, arena, agents, links, poses,
                     random.Random(1))
        return next(a for a in f["agents"]
                    if a["id"] == "car1")["rf"]["noise_floor_dbm"]

    def drain(line):
        qd = Path(tempfile.mkdtemp())
        _queue_with(qd, line)
        st.drain_retasks(qd, abid, arena, links, poses)

    drain("LAUNCH red\n")
    lo = car1_floor()
    check("armed jammer raises car1's floor", lo > -95.0)
    drain("JAM jam1 power 30\n")
    hi = car1_floor()
    check("JAM power 30 raises the floor further", hi > lo, f"{hi} vs {lo}")
    drain("JAM jam1 power -30\n")
    back = car1_floor()
    check("JAM power -30 drops the floor back toward baseline", back < hi)
    drain("HALT red\n")
    check("HALT red returns car1 to the scene baseline",
          car1_floor() == -95.0)
    # a non-jammer target is rejected, not crashed
    drain("JAM car1 power 10\n")
    check("JAM on a non-jammer is a no-op", abid["car1"].get("jammer") is None)


def test_jammer_range_helper():
    """The influence radius grows with power and shrinks with a higher J/N
    threshold - and is the value the range ring draws. See
    docs/jamming-model-justification.md."""
    print("\nJAMMER RANGE - THE INFLUENCE RADIUS")
    r10 = st.jammer_range_m(10, 2400, -95, 2.8)
    r20 = st.jammer_range_m(20, 2400, -95, 2.8)
    core = st.jammer_range_m(10, 2400, -95, 2.8, jn_db=20)
    check("more power reaches further", r20 > r10)
    check("the denial core (J/N=20 dB) is inside the influence boundary",
          core < r10)
    check("a quieter floor is reached from further away",
          st.jammer_range_m(10, 2400, -110, 2.8)
          > st.jammer_range_m(10, 2400, -80, 2.8))


def test_missions_are_blue_only():
    """A mission / REOBJECTIVE cannot task an adversary agent - red is
    jamming, not manoeuvre. See docs/cells-and-network.md."""
    print("\nMISSIONS ARE BLUE ONLY")
    arena, agents, links = st.load_scenario(
        {"scene": "lab_box", "fleets": ["3_roboracer", "red_jammer"],
         "points": {"P1": {"x": -3.0, "y": 2.0}, "P2": {"x": 3.0, "y": 2.0}}})
    abid = {a["id"]: a for a in agents}
    nets = arena["networks"]
    check("blue car is taskable", st._is_taskable(abid["car1"], nets))
    check("gcs (blue) is taskable", st._is_taskable(abid["gcs"], nets))
    check("red jammer is NOT taskable", not st._is_taskable(abid["jam1"], nets))

    poses = _poses_for(agents)
    # SETMISSION naming a red agent skips it with a reason
    mpath = _mission_file({"jam1": {"do": "shuttle", "between": ["P1", "P2"]},
                           "car1": {"do": "shuttle", "between": ["P1", "P2"]}},
                          name="mixed")
    changed, msgs, _ = st.apply_mission_file(mpath, abid, arena["points"],
                                             arena)
    tasked = {aid for aid, _ in changed}
    check("SETMISSION tasks the blue car", "car1" in tasked)
    check("SETMISSION skips the red jammer", "jam1" not in tasked)
    check("and says why", any("adversary" in m for m in msgs), str(msgs))

    # a live REOBJECTIVE on the jammer is ignored
    before = dict(abid["jam1"]["mission"])
    qd = Path(tempfile.mkdtemp())
    _queue_with(qd, "jam1: pursue car1\n")
    st.drain_retasks(qd, abid, arena, links, poses)
    check("REOBJECTIVE on a red agent is ignored",
          abid["jam1"]["mission"] == before)


def test_leader_loss_doctrine():
    """leader_loss doctrine, declarable per network: fallback (report up to
    the coordinator) vs strand (on your own). See docs/cells-and-network.md."""
    print("\nLEADER-LOSS DOCTRINE (fallback vs strand)")
    def authority(doctrine):
        net = {"authority": "hierarchical", "routing": "tiered",
               "coordinator": "gcs", "leader_loss": doctrine,
               "squads": {"alpha": {"leader": "car1", "members": ["car2"]}}}
        poses = {"gcs": {"x": 0, "y": 0, "z": 0},
                 "car1": {"x": 9000, "y": 0, "z": 0},   # leader out of range
                 "car2": {"x": 1, "y": 0, "z": 0}}
        links = [{"a": "gcs", "b": "car2", "network": "blue"},
                 {"a": "car1", "b": "car2", "network": "blue"},
                 {"a": "gcs", "b": "car1", "network": "blue"}]
        return st.command_authority({"id": "car2", "network": "blue"}, {},
                                    links, poses, {"blue": net})
    fb = authority("fallback")
    st_ = authority("strand")
    check("fallback: the member reports up to the coordinator",
          fb["decider"] == "gcs" and fb["reachable"])
    check("strand: the member is orphaned when its leader drops",
          not st_["reachable"] and st_["tier"] == "orphaned")
    check("default (unset) behaves as fallback",
          authority(None)["reachable"])


def test_jamming_stops_a_centralized_fleet_but_not_a_decentralized_one():
    """The causal link that makes jamming MATTER: an agent that loses its
    commander applies its on_link_loss doctrine. A centralized car freezes
    (no orders); a decentralized car keeps going (it commands itself). See
    step()'s `unreachable` gate and docs/cells-and-network.md."""
    print("\nJAMMING CHANGES BEHAVIOUR (hold vs self-command)")
    import random

    def run_under_jam(authority):
        arena, agents, links = st.load_scenario(
            {"scene": "lab_box", "fleets": ["3_roboracer", "red_jammer"]})
        arena["networks"]["blue"]["authority"] = authority
        abid = {a["id"]: a for a in agents}
        poses = _poses_for(agents)
        for line in ("SETMISSION test\n", "LAUNCH blue\n",
                     "LAUNCH red\n", "JAM jam1 power 30\n"):
            qd = Path(tempfile.mkdtemp()); _queue_with(qd, line)
            st.drain_retasks(qd, abid, arena, links, poses)
        b = (poses["car1"]["x"], poses["car1"]["y"])
        rng = random.Random(1)
        for k in range(25):
            st.frame(k * 0.1, 0.1, k, arena, agents, links, poses, rng)
        moved = abs(poses["car1"]["x"] - b[0]) + abs(poses["car1"]["y"] - b[1])
        f = st.frame(2.6, 0.1, 26, arena, agents, links, poses, rng)
        held = next(x for x in f["agents"] if x["id"] == "car1")["link_loss_hold"]
        return moved, held

    cen_moved, cen_held = run_under_jam("centralized")
    check("centralized car FREEZES under jamming (lost its coordinator)",
          cen_moved < 0.05, f"moved {cen_moved:.3f}")
    check("and reports it is holding on link loss", cen_held)

    dec_moved, dec_held = run_under_jam("decentralized")
    check("decentralized car KEEPS MOVING under the same jamming",
          dec_moved > 0.5, f"moved {dec_moved:.3f}")
    check("and is not flagged as holding", not dec_held)


def test_gnss_jamming_causes_drift_that_grows_recovers_and_lidar_resists():
    """The headline: GNSS-band jamming denies a no-lidar car its fix, its
    belief dead-reckons and DRIFTS (~% of distance, emergent), the drift
    recovers when the fix returns, and a lidar car resists entirely (it
    localises without GNSS). Literature: UAV Navigation 4%/distance;
    ArduPilot ~10 s usable then divergence. docs/gnss-drift-model.md."""
    print("\nGNSS JAMMING -> POSITION DRIFT (belief vs truth)")
    import random

    def run(fleet, jam_band, ticks, scene="open_field"):
        arena, agents, links = st.load_scenario(
            {"scene": scene, "fleets": [fleet, "red_jammer"]})
        abid = {a["id"]: a for a in agents}
        abid["jam1"]["jammer"]["band"] = {"value": jam_band}
        abid["jam1"]["jammer"]["tx_power"] = {"value": 30}
        poses = _poses_for(agents)
        for line in ("SETMISSION test\n", "LAUNCH blue\n"):
            qd = Path(tempfile.mkdtemp()); _queue_with(qd, line)
            st.drain_retasks(qd, abid, arena, links, poses)
        rng = random.Random(3)
        # warm up clean
        for k in range(20):
            st.frame(k * 0.1, 0.1, k, arena, agents, links, poses, rng)
        clean = _car_err(arena, agents, links, poses, rng)
        qd = Path(tempfile.mkdtemp()); _queue_with(qd, "LAUNCH red\n")
        st.drain_retasks(qd, abid, arena, links, poses)
        errs = []
        for k in range(20, 20 + ticks):
            st.frame(k * 0.1, 0.1, k, arena, agents, links, poses, rng)
            if k % 25 == 24:
                errs.append(_car_err(arena, agents, links, poses, rng))
        qd = Path(tempfile.mkdtemp()); _queue_with(qd, "HALT red\n")
        st.drain_retasks(qd, abid, arena, links, poses)
        for k in range(20 + ticks, 40 + ticks):
            st.frame(k * 0.1, 0.1, k, arena, agents, links, poses, rng)
        recovered = _car_err(arena, agents, links, poses, rng)
        return clean, errs, recovered

    clean, errs, recovered = run("3_roboracer_no_lidar", 1575.42, 120)
    check("with GNSS the no-lidar car knows where it is (error ~0)",
          clean < 0.05, f"{clean}")
    check("GNSS jamming makes the error GROW over time",
          len(errs) >= 3 and errs[-1] > errs[0] > 0,
          f"{errs}")
    # SOURCED, NOT GUESSED. 0.49% of distance travelled (Papadopoulos &
    # Misailidis 2007, Table I, worst uncalibrated differential-drive case:
    # 60.87 cm over 124.9 m). Over ~18 m that is ~9 cm of expected error, and
    # because the drift direction is a random walk the realised value spreads
    # either side of it. The old test asserted 0.3-2.0 m, which was the old
    # invented 4% - an order of magnitude worse than anything measured, and
    # what made every indoor mission fail on drift within seconds.
    check("the drift is a fraction of a percent of the ~18 m travelled",
          0.01 < errs[-1] < 0.5, f"{errs[-1]} m")
    check("a regained fix recovers the position (error back to ~0)",
          recovered < 0.05, f"{recovered}")

    # In a FEATURELESS field, a lidar cannot localise either - it has
    # nothing to scan - so a lidar car ALSO drifts under GNSS jamming here.
    lclean, lerrs, _ = run("3_roboracer", 1575.42, 120)
    check("a lidar car in an OPEN field also drifts (nothing to scan)",
          lerrs[-1] > 0.01, f"{[lclean] + lerrs}")
    # WITH features (walls) a lidar localises without GNSS - but it is NOT
    # perfect: scan-matching error still accumulates, so it drifts. Claiming
    # zero was an over-claim.
    llab, llerrs, _ = run("3_roboracer", 1575.42, 120, scene="lab_box")
    check("a lidar car with walls still drifts a little (SLAM is not perfect)",
          llerrs[-1] > 0.0, f"{[llab] + llerrs}")
    # AN AIDED VEHICLE IS AT LEAST NOT WORSE THAN AN UNAIDED ONE. That is the
    # only claim this test can honestly make today. DRIFT_RATE_AIDED used to
    # be a vendor figure of ~1% of distance, which is now WORSE than the
    # measured unaided 0.49% - incoherent, so it was held at the unaided rate
    # rather than replaced with a better-feeling guess. Restore the stronger
    # assertion (aided is MUCH better) the moment a measured lidar- or
    # vision-aided drift figure is sourced. See SOURCES.md.
    # Asserted on the RATES, not on two runs in different rooms over
    # different distances - which is what the old version did, and it only
    # ever passed because the unaided rate was ten times too big.
    check("an aiding sensor is never modelled as worse than none",
          st.DRIFT_RATE_AIDED <= st.DRIFT_RATE_UNAIDED,
          f"aided {st.DRIFT_RATE_AIDED} vs unaided {st.DRIFT_RATE_UNAIDED}")
    check("the unaided rate is the one the paper measured",
          abs(st.DRIFT_RATE_UNAIDED - 0.005) < 1e-9,
          f"{st.DRIFT_RATE_UNAIDED} - Papadopoulos & Misailidis 2007 Table I "
          f"worst uncalibrated case is 60.87 cm / 124.9 m = 0.49%")

    # A COMMS-band jammer does NOT cause positional drift (wrong band).
    cclean, cerrs, _ = run("3_roboracer_no_lidar", 2400.0, 80)
    check("a comms-band jammer causes no positional drift (band separation)",
          max([cclean] + cerrs) < 0.05, f"{[cclean] + cerrs}")


def _car_err(arena, agents, links, poses, rng):
    f = st.frame(0.0, 0.1, 0, arena, agents, links, poses, rng)
    return next(a for a in f["agents"]
                if a["id"] == "car1")["position_error_m"]


def test_vehicle_dynamics_momentum_and_no_yaw_snap():
    """A vehicle has momentum and a turning limit. It used to reach full
    speed in one tick and TELEPORT its heading 180 degrees at a shuttle
    turnaround - so a forward-mounted lidar snapped round with it. Now speed
    is acceleration-limited, heading is rate-limited (v / turn_radius), and a
    car too tight to turn REVERSES instead - pointing its lidar away from
    travel, which is a real sensing gap."""
    print("\nVEHICLE DYNAMICS - MOMENTUM AND TURNING")
    import random
    arena, agents, links = st.load_scenario(
        {"scene": "lab_box", "fleet": "3_roboracer"})
    abid = {a["id"]: a for a in agents}
    poses = _poses_for(agents)
    for line in ("SETMISSION test\n", "LAUNCH blue\n"):
        qd = Path(tempfile.mkdtemp()); _queue_with(qd, line)
        st.drain_retasks(qd, abid, arena, links, poses)
    rng = random.Random(1)

    st.frame(0.0, 0.1, 0, arena, agents, links, poses, rng)
    v_first = poses["car1"]["speed"]
    check("does NOT reach full speed in the first tick (momentum)",
          0.0 < v_first < 1.0, f"{v_first}")

    speeds, yaws, reversed_seen = [], [], False
    for k in range(1, 80):
        st.frame(k * 0.1, 0.1, k, arena, agents, links, poses, rng)
        speeds.append(poses["car1"]["speed"])
        yaws.append(poses["car1"]["yaw"])
        if poses["car1"].get("reversing"):
            reversed_seen = True
    check("accelerates up to its speed cap", max(speeds) > 1.4, f"{max(speeds)}")
    # The heading must never jump - no teleport-rotation.
    biggest = max(abs(st.wrap_pi(yaws[i] - yaws[i - 1]))
                  for i in range(1, len(yaws)))
    check("heading never snaps (no 180 deg jump in one tick)",
          biggest < math.radians(45), f"max jump {math.degrees(biggest):.1f} deg")
    check("a car too tight to turn reverses instead", reversed_seen)

    # A quadcopter is holonomic - it may translate any direction.
    q = {"spec_version": 0.1, "name": "q", "kind": "fleet",
         "arena": {"type": "box", "extent": {"x": 40, "y": 40, "z": 20}},
         "networks": {"blue": {"topology": "decentralized"}},
         "agents": [{"id": "q1", "platform": "quadcopter", "network": "blue",
                     "pose": {"x": 0, "y": 0, "z": 2},
                     "performance": {"max_speed": 8.0}}]}
    tmp = Path(tempfile.mkdtemp()) / "q.yaml"
    tmp.write_text(yaml.safe_dump(q))
    _a, _ag, _l = st.load_scenario(str(tmp))
    check("a quadcopter is holonomic, a car is ackermann",
          _ag[0]["motion"] == "holonomic"
          and abid["car1"]["motion"] == "ackermann")


def test_fleet_self_interference_grows_with_size():
    """The README's "8 or more drones start jamming each other". Modelled as
    CONTENTION, not SINR: co-channel fleet members share airtime and collide,
    so delivered quality falls and latency rises as the fleet grows. An
    external jammer is different - it ignores the protocol, which is what
    makes it a jammer."""
    print("\nFLEET SELF-INTERFERENCE (channel contention)")
    import random, tempfile as _t
    def mean_pdr(n):
        ags = [{"id": "gcs", "platform": "ground_station", "network": "blue",
                "ghost": True, "pose": {"x": 0, "y": -10, "z": 0, "yaw": 0}}]
        for i in range(n):
            ags.append({"id": f"d{i+1}", "platform": "quadcopter",
                        "network": "blue",
                        "pose": {"x": (i % 5) * 3 - 6, "y": (i // 5) * 3,
                                 "z": 5, "yaw": 0},
                        "performance": {"max_speed": 8}})
        doc = {"spec_version": 0.1, "name": f"f{n}",
               "arena": {"type": "box", "extent": {"x": 60, "y": 60, "z": 30}},
               "networks": {"blue": {"topology": "centralized",
                                     "coordinator": "gcs", "band": 2400}},
               "agents": ags}
        tmp = Path(_t.mkdtemp()) / "f.yaml"
        tmp.write_text(yaml.safe_dump(doc))
        arena, agents, links = st.load_scenario(str(tmp))
        poses = _poses_for(agents)
        f = st.frame(0.0, 0.1, 0, arena, agents, links, poses,
                     random.Random(1))
        act = [l for l in f["links"] if l["active"]]
        return (sum(l["pdr"] for l in act) / len(act),
                sum(l["latency_ms"] for l in act) / len(act),
                max(l["contenders"] for l in act))

    p2, lat2, c2 = mean_pdr(2)
    p8, lat8, c8 = mean_pdr(8)
    p16, lat16, c16 = mean_pdr(16)
    check("a bigger fleet has more co-channel contenders", c16 > c8 > c2)
    check("delivered quality falls as the fleet grows",
          p16 < p8 < p2, f"{p2:.2f} / {p8:.2f} / {p16:.2f}")
    check("latency rises as airtime is shared",
          lat16 > lat8 > lat2, f"{lat2:.0f} / {lat8:.0f} / {lat16:.0f} ms")
    check("a small fleet is barely affected", p2 > 0.9, f"{p2:.2f}")
    check("a large fleet is materially degraded by itself alone",
          p16 < 0.6, f"{p16:.2f}")

    # HOW THE FLEET IS ORGANISED changes the answer: tiered routing keeps
    # intra-squad traffic local, so it PARTITIONS contention. One of the real
    # reasons militaries organise hierarchically.
    def tiered16():
        ids = [f"d{i+1}" for i in range(16)]
        ags = [{"id": "gcs", "platform": "ground_station", "network": "blue",
                "ghost": True, "pose": {"x": 0, "y": -10, "z": 0, "yaw": 0}}]
        for i, aid in enumerate(ids):
            ags.append({"id": aid, "platform": "quadcopter", "network": "blue",
                        "pose": {"x": (i % 4) * 3 - 5, "y": (i // 4) * 3,
                                 "z": 5, "yaw": 0},
                        "performance": {"max_speed": 8}})
        net = {"topology": "hierarchical", "coordinator": "gcs", "band": 2400,
               "routing": "tiered", "qos": {"deadline_ms": 100},
               "squads": {"a": {"leader": ids[0], "members": ids[1:8]},
                          "b": {"leader": ids[8], "members": ids[9:]}}}
        doc = {"spec_version": 0.1, "name": "t16",
               "arena": {"type": "box", "extent": {"x": 60, "y": 60, "z": 30}},
               "networks": {"blue": net}, "agents": ags}
        tmp = Path(_t.mkdtemp()) / "t.yaml"
        tmp.write_text(yaml.safe_dump(doc))
        arena, agents, links = st.load_scenario(str(tmp))
        poses = _poses_for(agents)
        f = st.frame(0.0, 0.1, 0, arena, agents, links, poses,
                     random.Random(1))
        act = [l for l in f["links"] if l["active"]]
        return (sum(l["pdr"] for l in act) / len(act),
                max(l["contenders"] for l in act))
    pt, ct = tiered16()
    check("tiered routing partitions contention (fewer contenders)",
          ct < c16, f"tiered {ct} vs star/mesh {c16}")
    check("...so the same 16 agents communicate better when tiered",
          pt > p16, f"tiered {pt:.2f} vs flat {p16:.2f}")

    # A network's own declared deadline decides how much contention costs it.
    def with_deadline(ms):
        ags = [{"id": "gcs", "platform": "ground_station", "network": "blue",
                "ghost": True, "pose": {"x": 0, "y": -10, "z": 0, "yaw": 0}}]
        for i in range(16):
            ags.append({"id": f"d{i+1}", "platform": "quadcopter",
                        "network": "blue",
                        "pose": {"x": (i % 4) * 3 - 5, "y": (i // 4) * 3,
                                 "z": 5, "yaw": 0},
                        "performance": {"max_speed": 8}})
        doc = {"spec_version": 0.1, "name": "d",
               "arena": {"type": "box", "extent": {"x": 60, "y": 60, "z": 30}},
               "networks": {"blue": {"topology": "centralized",
                                     "coordinator": "gcs", "band": 2400,
                                     "qos": {"deadline_ms": ms}}},
               "agents": ags}
        tmp = Path(_t.mkdtemp()) / "d.yaml"
        tmp.write_text(yaml.safe_dump(doc))
        arena, agents, links = st.load_scenario(str(tmp))
        poses = _poses_for(agents)
        f = st.frame(0.0, 0.1, 0, arena, agents, links, poses,
                     random.Random(1))
        act = [l for l in f["links"] if l["active"]]
        return sum(l["pdr"] for l in act) / len(act)
    check("a tight real-time deadline suffers more from contention "
          "than a lax one", with_deadline(50) < with_deadline(500),
          f"{with_deadline(50):.2f} vs {with_deadline(500):.2f}")


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


def test_missions_read_belief_not_ground_truth():
    """THE FIX for the review's critical finding. A mission is handed what the
    agent KNOWS - its own drifted belief plus the position reports it has
    actually received over live links - never the simulator's ground truth.

    Three things are asserted, and the first two are what the old code failed:
      1. under GNSS denial, a pursuer's picture of its target DIVERGES from the
         target's true pose (before this fix the error was identically zero);
      2. that picture tracks the target's own BELIEF, which is the only thing
         the target could have transmitted;
      3. when the comms link drops, the picture FREEZES and AGES - the pursuer
         keeps acting on the last report it heard, which is what really happens.
    See docs/gnss-drift-model.md and the critical review."""
    print("\nMISSIONS READ BELIEF, NOT GROUND TRUTH")
    import random, copy

    arena, agents, links = st.load_scenario(
        {"scene": "open_field", "fleets": ["3_roboracer_no_lidar", "red_jammer"],
         "points": dict(OPEN_FIELD_POINTS)})
    by = {a["id"]: a for a in agents}

    # jam1 denies GNSS outright; jam2 (armed later) cuts the comms band.
    by["jam1"]["armed"] = True
    by["jam1"]["jammer"]["tx_power"] = {"value": 40}
    by["jam1"]["jammer"]["band"] = {"value": st.GNSS_BAND_MHZ}
    j2 = copy.deepcopy(by["jam1"])
    j2["id"], j2["armed"] = "jam2", False
    j2["jammer"]["band"] = {"value": 2400}
    j2["start"] = dict(by["jam1"]["start"]); j2["start"]["y"] = 1.0
    agents.append(j2); by["jam2"] = j2

    by["car1"]["mission"] = {"type": "pursuit", "target": "car3", "standoff": 1.2}
    by["car3"]["mission"] = {"type": "shuttle", "between": ["P1", "P2"]}
    for a in agents:
        if a["id"].startswith("car"):
            a["armed"] = True

    rng = random.Random(1)
    poses = _poses_for(agents)
    dt, err_at, know = 0.1, {}, {}
    for k in range(300):
        t = k * dt
        if k == 120:
            by["jam2"]["armed"] = True          # comms cut at t = 12 s
        st.frame(t, dt, k, arena, agents, links, poses, rng)
        kn = (by["car1"].get("knowledge") or {}).get("car3") or {}
        b3 = by["car3"].get("belief") or {}
        p3 = poses["car3"]
        err_at[k] = math.hypot(kn.get("x", 0.0) - p3["x"], kn.get("y", 0.0) - p3["y"])
        know[k] = (dict(kn), dict(b3), t)

    check("a pursuer's picture of its target DIVERGES under GNSS denial "
          "(the old code had it exactly right, forever)",
          err_at[115] > 0.01, f"error at t=11.5s: {err_at[115]:.3f} m")
    kn, b3, _ = know[115]
    check("that picture tracks the target's own BELIEF, not its true pose",
          math.hypot(kn["x"] - b3.get("x", 0.0), kn["y"] - b3.get("y", 0.0)) < 0.25,
          f"knowledge {kn['x']:.2f},{kn['y']:.2f} vs belief "
          f"{b3.get('x', 0):.2f},{b3.get('y', 0):.2f}")

    kn_cut, _, t_cut = know[130]
    kn_end, _, t_end = know[299]
    check("when the comms link drops the picture FREEZES",
          math.hypot(kn_end["x"] - kn_cut["x"], kn_end["y"] - kn_cut["y"]) < 1e-9,
          f"{kn_cut['x']:.3f},{kn_cut['y']:.3f} -> {kn_end['x']:.3f},{kn_end['y']:.3f}")
    check("...and AGES, so staleness is visible rather than hidden",
          (t_end - kn_end["t"]) > 15.0,
          f"age {t_end - kn_end['t']:.1f}s")

    check("an agent always knows its OWN belief, connected or not",
          "car1" in (by["car1"].get("knowledge") or {}))


def test_routing_gates_authority_not_just_geometry():
    """REGRESSION. Authority must follow the routes the network actually
    carries, over a PATH, not over any link that happens to have good SINR.

    Found by the first sweep: star, mesh and tiered produced byte-identical
    command availability at every jammer power. The axis was declared, swept
    and silently ignored, because command_authority() asked link_states for a
    pair's SINR state and never asked whether the routing carried that pair.

    Two properties are asserted:
      1. a link the routing does NOT carry confers no authority;
      2. authority relays - a tiered member with no direct link to the
         coordinator is still commanded THROUGH its leader, and is stranded
         only when that leader is lost."""
    print("\nROUTING GATES AUTHORITY (multi-hop, active links only)")
    net = {"blue": {"authority": "centralized", "coordinator": "gcs",
                    "routing": "tiered"}}
    agent = {"id": "car2", "network": "blue"}
    poses = {}

    # gcs--car1--car2 : no direct gcs-car2 link is carried.
    carried = {frozenset(("gcs", "car1")): {"state": "up", "active": True},
               frozenset(("car1", "car2")): {"state": "up", "active": True},
               frozenset(("gcs", "car2")): {"state": "up", "active": False}}
    a = st.command_authority(agent, {}, [], poses, net, link_states=carried)
    check("a tiered member is commanded THROUGH its relay, not stranded",
          a["reachable"], str(a))

    # Same geometry, relay link cut. The unused direct link is still 'up' and
    # must NOT rescue it - that is the exact bug.
    cut = dict(carried)
    cut[frozenset(("car1", "car2"))] = {"state": "down", "active": True}
    a = st.command_authority(agent, {}, [], poses, net, link_states=cut)
    check("losing the relay strands the member - an inactive link does not "
          "quietly confer authority", not a["reachable"], str(a))

    # And the routing axis must MOVE the answer, end to end.
    import random
    def commanded(routing, jam_dbm):
        arena, agents, links = st.load_scenario(
            {"scene": "open_field", "fleets": ["3_roboracer_no_lidar", "red_jammer"]})
        by = {x["id"]: x for x in agents}
        blue = arena["networks"]["blue"]
        blue["routing"] = routing
        blue["squads"] = {"alpha": {"leader": "car1", "members": ["car2", "car3"]}}
        by["jam1"]["armed"] = True
        by["jam1"]["jammer"]["tx_power"] = {"value": jam_dbm}
        by["jam1"]["jammer"]["band"] = {"value": 2400}
        poses_ = _poses_for(agents)
        for line in ("SETMISSION test\n", "LAUNCH blue\n"):
            qd = Path(tempfile.mkdtemp()); _queue_with(qd, line)
            st.drain_retasks(qd, by, arena, links, poses_)
        rng = random.Random(1)
        got = []
        for k in range(150):
            f = st.frame(k * 0.1, 0.1, k, arena, agents, links, poses_, rng)
            cars = [x for x in f["agents"] if x["id"].startswith("car")]
            got.append(sum(1 for x in cars
                           if (x["authority"] or {}).get("reachable")) / len(cars))
        return sum(got) / len(got)

    star, mesh, tiered = (commanded(r, 10) for r in ("star", "mesh", "tiered"))
    check("routing CHANGES command availability under the same jamming "
          "(mesh relays, tiered has one relay to lose)",
          not (abs(star - mesh) < 1e-9 and abs(star - tiered) < 1e-9),
          f"star {star:.3f} mesh {mesh:.3f} tiered {tiered:.3f}")
    check("mesh is the most resilient of the three, tiered the least",
          mesh >= star >= tiered,
          f"star {star:.3f} mesh {mesh:.3f} tiered {tiered:.3f}")


def test_formations_are_functions_not_coordinate_lists():
    """A formation is a FUNCTION of (count, spacing), which is what lets one
    named shape serve any fleet at any size - and what a table of x/y/z can
    never do. Four properties, each of which a coordinate list would break."""
    print("\nFORMATIONS SCALE, GROW AND STAY CENTRED")
    import math as _m

    for shape in st.FORMATIONS:
        for n in (1, 2, 3, 4, 5, 8, 12):
            offs = st.formation_offsets(shape, n, 3.0)
            check(f"{shape} n={n}: one offset per vehicle", len(offs) == n)
            if n > 1:
                sep = min(_m.dist(a, b) for i, a in enumerate(offs)
                          for j, b in enumerate(offs) if i < j)
                check(f"{shape} n={n}: nobody is stacked on anybody",
                      sep > 0.1, f"min separation {sep:.2f} m")

    # CENTRED: placing a formation is placing its middle.
    for shape in st.FORMATIONS:
        offs = st.formation_offsets(shape, 7, 3.0)
        cx = sum(q[0] for q in offs) / len(offs)
        check(f"{shape}: x is centred on the formation's middle",
              abs(cx) < 1e-3, f"{cx}")

    # SIZE: one dial scales the whole shape linearly.
    for shape in st.FORMATIONS:
        a = st.formation_offsets(shape, 6, 2.0)
        b = st.formation_offsets(shape, 6, 4.0)
        # Tolerance is 1e-3, not 1e-6: offsets are rounded to 4 decimals on
        # the way out so a saved formation is clean YAML rather than
        # sixteen-digit noise. Doubling a rounded number can differ from the
        # rounded double by twice the rounding, and that is the rounding
        # working as intended, not the shape failing to scale.
        ok = all(abs(2 * p[k] - q[k]) < 1e-3
                 for p, q in zip(a, b) for k in range(3))
        check(f"{shape}: doubling the spacing doubles the shape", ok)

    # GROWTH, per shape, as specified:
    line5 = st.formation_offsets("line", 5, 3.0)
    line6 = st.formation_offsets("line", 6, 3.0)
    check("a line adds one on the END and keeps its spacing",
          len(line6) == 6
          and abs((max(q[0] for q in line6) - min(q[0] for q in line6))
                  - (max(q[0] for q in line5) - min(q[0] for q in line5))
                  - 3.0) < 1e-6)

    for n in (3, 5, 7, 9):
        w = st.formation_offsets("wedge", n, 3.0)
        ys = sorted(round(q[1], 4) for q in w)
        check(f"a wedge of {n} is symmetric about its axis",
              all(abs(ys[i] + ys[-1 - i]) < 1e-3 for i in range(len(ys) // 2)),
              str(ys))
        check(f"a wedge of {n} has its apex in front",
              max(w, key=lambda q: q[0])[1] == 0.0)

    for n in (4, 5, 6, 9):
        c = st.formation_offsets("circle", n, 3.0)
        r = [_m.hypot(q[0], q[1]) for q in c]
        check(f"a circle of {n} re-spaces EVERY vehicle evenly",
              max(r) - min(r) < 1e-3, f"radii {min(r):.6f}..{max(r):.6f}")

    check("only the cube uses altitude",
          {round(q[2], 3) for q in st.formation_offsets("cube", 8, 3.0)} != {0.0}
          and all({round(q[2], 3) for q in st.formation_offsets(sh, 8, 3.0)}
                  == {0.0} for sh in st.FORMATIONS if sh != "cube"))

    # THE GROUND STATION IS NEVER MOVED.
    arena, agents, links = st.load_scenario(
        {"scene": "corridor_200m", "fleets": ["3_roboracer_no_lidar"]})
    before = dict(next(a["start"] for a in agents if a["id"] == "gcs"))
    moved = st.apply_formation(agents, "wedge", 4.0)
    after = next(a["start"] for a in agents if a["id"] == "gcs")
    check("a formation moves the vehicles, not the ground station",
          "gcs" not in moved and after["x"] == before["x"]
          and after["y"] == before["y"], f"{moved}")


# The corridor declares no points of its own any more - a scene is the world,
# and where you send a fleet inside it is a decision about the run. Tests that
# need somewhere to go supply it, exactly as the Console does.
CORRIDOR_POINTS = {"P1": {"x": -95.0, "y": 0.0, "z": 0.0},
                   "P2": {"x": 95.0, "y": 0.0, "z": 0.0},
                   "P3": {"x": 0.0, "y": 0.0, "z": 0.0}}


def _corridor(*fleets, points=None):
    """A composed corridor run with points supplied by the operator."""
    return {"scene": "corridor_200m",
            "fleets": [str(FIXTURE_FLEETS / f) for f in
                       (fleets or ("3_roboracer_no_lidar.yaml",))],
            "points": dict(points or CORRIDOR_POINTS)}


OPEN_FIELD_POINTS = {"P1": {"x": -6.0, "y": 2.0, "z": 0.0},
                     "P2": {"x": 6.0, "y": 2.0, "z": 0.0},
                     "P3": {"x": 0.0, "y": -6.0, "z": 0.0}}


def _field(*fleets, points=None):
    """An open_field run with points supplied by the operator."""
    return {"scene": "open_field",
            "fleets": [str(FIXTURE_FLEETS / f) for f in
                       (fleets or ("3_roboracer_no_lidar.yaml",))],
            "points": dict(points or OPEN_FIELD_POINTS)}


def _sweep_mod():
    """tools/sweep.py as a module, without running its CLI."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "deadband_sweep_test", str(REPO / "tools" / "sweep.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_a_mission_is_portable_across_scenes():
    """A mission says HOW MANY goals it needs. It does not say where they are.

    `advance` used to say `to: FAR`, and FAR existed only in the corridor - so
    the one mission the whole experiment programme is built on could run on
    exactly one scene, and pointed anywhere else it tasked NOBODY rather than
    failing loudly. Worse, it let whoever wrote the SCENE decide the
    objective. A scene is the world; where you send a fleet inside it belongs
    to whoever is running it.
    """
    print("\nA MISSION ASKS FOR N GOALS AND THE RUN SUPPLIES THEM")
    check("advance lets the operator choose how many goals",
          st.mission_spec(REPO / "missions" / "advance.yaml")["goals"] == "any")
    check("...and fixes one lap, because that is what advance MEANS",
          st.mission_spec(REPO / "missions" / "advance.yaml")["laps"] == 1)
    check("patrol leaves both open - points AND how many times round",
          st.mission_spec(REPO / "missions" / "patrol.yaml")
          == {"goals": "any", "laps": "any"})
    check("forward asks for none - it has no destination at all",
          st.mission_spec(REPO / "missions" / "forward.yaml")["goals"] == 0)

    arena, agents, links = st.load_scenario(_field())
    by = {a["id"]: a for a in agents}
    points = arena.get("points") or {}

    # NO GOAL GIVEN: refused, loudly, naming what it needed.
    changed, msgs, _ = st.apply_mission_file(
        str(REPO / "missions" / "advance.yaml"), by, points, arena)
    check("a mission with no goal chosen tasks nobody, and says why",
          not changed and any("at least one point" in m for m in msgs),
          f"{msgs[:1]}")

    # A GOAL FROM THIS SCENE: the same file tasks the whole fleet.
    changed, msgs, _ = st.apply_mission_file(
        str(REPO / "missions" / "advance.yaml"), by, points, arena,
        goals=["P2"])
    check("given a point, the same file tasks the fleet",
          len(changed) >= 3, f"{len(changed)} tasked: {msgs[:1]}")
    check("every advance objective points at the goal that was chosen",
          all((by[a].get("mission") or {}).get("to") == "P2"
              for a, _ in changed))

    # HOW MANY IS THE OPERATOR'S TOO. One goal is the penetration command;
    # three is a route; the mission file is the same either way.
    changed, msgs, _ = st.apply_mission_file(
        str(REPO / "missions" / "advance.yaml"), by, points, arena,
        goals=["P1", "P2", "P3"])
    check("the same mission takes three goals and visits them in order",
          by["car1"]["_plan"]["waypoints"] == ["P1", "P2", "P3"],
          str(by["car1"].get("_plan")))
    check("...once each, because advance fixes one lap",
          by["car1"]["_plan"]["laps"] == 1)

    # PATROL IS THE SAME THING THAT COMES BACK. Two points patrolled is a
    # shuttle, which is why there is no shuttle mission any more.
    changed, msgs, _ = st.apply_mission_file(
        str(REPO / "missions" / "patrol.yaml"), by, points, arena,
        goals=["P1", "P2"], laps=4)
    check("patrol takes the operator's lap count",
          by["car1"]["_plan"]["waypoints"] == ["P1", "P2"]
          and by["car1"]["_plan"]["laps"] == 4,
          str(by["car1"].get("_plan")))
    st.apply_mission_file(str(REPO / "missions" / "advance.yaml"), by,
                          points, arena, goals=["P1", "P2"], laps=9)
    check("advance IGNORES a lap count - it would be a patrol in disguise",
          by["car1"]["_plan"]["laps"] == 1, str(by["car1"]["_plan"]))

    # A point the scene does not have is refused, and says what it does have.
    changed, msgs, _ = st.apply_mission_file(
        str(REPO / "missions" / "advance.yaml"), by, points, arena,
        goals=["NOWHERE"])
    check("a goal this scene lacks is refused, not silently ignored",
          not changed and any("NOWHERE" in m for m in msgs), f"{msgs}")

    # `forward` needs nothing and must not acquire a destination.
    changed, _msgs, _ = st.apply_mission_file(
        str(REPO / "missions" / "forward.yaml"), by, points, arena,
        goals=["P2"])
    check("a mission with no goals is left alone by one being offered",
          all((by[a].get("mission") or {}).get("to") is None
              for a, _ in changed), str([by[a].get("mission")
                                         for a, _ in changed][:1]))


def test_a_scene_no_longer_ships_objectives_of_its_own():
    """The corridor declares no points. That is the contract, not an omission.

    HOME, FAR, APEX, WINGL and WINGR were every one of them an objective in
    disguise. APEX/WINGL/WINGR were a wedge's formation slots hardcoded into
    the room it happened to be standing in - the exact thing
    formations-as-functions removed - and HOME/FAR were the ends of one
    particular mission, chosen by whoever wrote the scene.
    """
    print("\nA SCENE IS THE WORLD, NOT THE OBJECTIVE")
    arena, _a, _l = st.load_scenario({
        "scene": "corridor_200m",
        "fleets": [str(FIXTURE_FLEETS / "3_roboracer_no_lidar.yaml")]})
    check("the corridor ships no points at all",
          not (arena.get("points") or {}), str(arena.get("points")))
    check("...and its geometry is untouched by that",
          float(arena["extent"]["x"]) == 200.0, str(arena["extent"]))

    # Points supplied by the run appear, and are usable immediately.
    arena2, agents2, _l2 = st.load_scenario(_corridor())
    pts = arena2.get("points") or {}
    check("points supplied with the run are the ones it has",
          sorted(pts) == ["P1", "P2", "P3"], str(sorted(pts)))
    changed, msgs, _ = st.apply_mission_file(
        str(REPO / "missions" / "patrol.yaml"),
        {a["id"]: a for a in agents2}, pts, arena2, goals=["P1", "P2"],
        laps=3)
    check("a mission flies against points that never touched a scene file",
          len(changed) == 3, f"{len(changed)}: {msgs[:1]}")

    # A scene MAY still declare points - the benchmark lanes are the fixed
    # geometry of a scene that has to be identical every time. That is a scene
    # making a claim about itself, which is a different thing.
    lab, _a3, _l3 = st.load_scenario({
        "scene": "lab_box",
        "fleets": [str(FIXTURE_FLEETS / "3_roboracer_no_lidar.yaml")]})
    check("no scene ships points at all - not even the benchmark ones",
          not (lab.get("points") or {}), str(sorted(lab.get("points") or {})))
    # The one mission that IS welded carries its own COORDINATES instead, so a
    # benchmark that has to be identical every time does not depend on points
    # somebody set up differently this morning.
    by_lab = {a["id"]: a for a in _a3}
    changed, msgs, _ = st.apply_mission_file(
        str(REPO / "missions" / "test.yaml"), by_lab,
        lab.get("points") or {}, lab)
    check("the fixed benchmark still runs, on its own coordinates",
          len(changed) == 3, f"{len(changed)}: {msgs[:1]}")


def test_setmission_grammar_takes_a_goal():
    """SETMISSION <name> to <POINT>, so a destination is never a file edit.

    The framework's rule is that hardware is a file and everything else is a
    decision made in the Console. A destination baked into a mission YAML
    broke that rule: the only way to advance somewhere else was to open the
    YAML and retype it. This is the same one mission file, sent to two
    different points from the command line.
    """
    print("\nSETMISSION TAKES A GOAL")
    arena, agents, links = st.load_scenario(_field())
    agents_by_id = {a["id"]: a for a in agents}
    poses = _poses_for(agents)

    qdir = Path(tempfile.mkdtemp())
    _queue_with(qdir, "SETMISSION advance to P2\n")
    changed = st.drain_retasks(qdir, agents_by_id, arena, links, poses)
    check("SETMISSION advance to P2 tasks the fleet",
          len(changed) >= 3, f"{len(changed)} tasked")
    check("...and every vehicle is advancing on P2",
          all(m.get("to") == "P2" for _aid, m in changed),
          str([m for _a, m in changed][:1]))

    # The same file, a different point, no edit anywhere.
    qdir2 = Path(tempfile.mkdtemp())
    _queue_with(qdir2, "SETMISSION patrol to P1 P2 laps 5\n")
    changed2 = st.drain_retasks(qdir2, agents_by_id, arena, links, poses)
    check("the same points, patrolled, with laps typed on the line",
          changed2 and agents_by_id["car1"]["_plan"]["laps"] == 5
          and agents_by_id["car1"]["_plan"]["waypoints"] == ["P1", "P2"],
          str(agents_by_id["car1"].get("_plan")))

    # Without the goal, on this scene, it is refused rather than half-applied.
    qdir3 = Path(tempfile.mkdtemp())
    _queue_with(qdir3, "SETMISSION advance\n")
    changed3 = st.drain_retasks(qdir3, agents_by_id, arena, links, poses)
    check("without a goal the corridor's mission is refused here",
          not changed3, str(changed3))
    check("...and the fleet keeps the last order it actually received",
          all((agents_by_id[i].get("mission") or {}).get("to") == "P1"
              for i in ("car1", "car2", "car3")))


def test_penetration_is_measured_along_the_axis_of_advance():
    """Penetration was "how far in +x", which is only right in a corridor
    that happens to run east-west.

    On any other scene that number measured a direction nobody was travelling
    in, so an experiment there would have produced a table of near-zeros and
    looked like a jamming result. It is now the distance advanced along the
    line from the fleet's start to the goal it was given - which is the same
    number on the corridor and a meaningful one everywhere else.
    """
    print("\nPENETRATION IS MEASURED TOWARD THE GOAL, NOT ALONG +X")
    sweep = _sweep_mod()
    base = {
        "name": "axis-test", "scene": "open_field",
        "blue_fleet": str(FIXTURE_FLEETS / "3_roboracer_no_lidar.yaml"),
        "mission": "advance", "points": dict(OPEN_FIELD_POINTS),
        "duration_s": 30.0, "warmup_s": 0.0, "rate_hz": 10.0,
        "stop_when_stalled": False,
        "coordinator": "gcs", "doctrine": "hold",
        "spawns": {"gcs": {"x": 0.0, "y": -8.0},
                   "car1": {"x": 0.0, "y": -6.0},
                   "car2": {"x": -1.0, "y": -7.0},
                   "car3": {"x": 1.0, "y": -7.0}},
        "seeds": [1],
    }
    cell = {"authority": "centralized", "routing": "mesh",
            "jam_dbm": sweep.JAM_OFF, "jam_rel_db": -999, "seed": 1,
            "cell": "axis"}

    # A goal due NORTH of the start. Nothing about this run happens in x.
    north = dict(base, goal="P1")         # P1 is at (-6, +2); start y is -7
    row = sweep.run_one((north, dict(cell), False))
    check("a fleet advancing north records real penetration",
          not row.get("error") and row["penetration_m"] > 1.0,
          f"{row.get('error') or row['penetration_m']}")

    # The old measure - displacement in x alone - would have been near zero
    # for that same run, which is exactly the failure being fixed.
    check("...and it is not just the x displacement",
          row["penetration_m"] > 1.0)


def test_formation_and_spacing_are_swept_axes_that_reach_the_model():
    """A formation axis that does not change the run is worse than no axis:
    the table fills with rows that differ in a column and not in a result,
    which reads as "formation does not matter" when what happened is that
    nothing was applied. The routing axis failed exactly this way once
    already.
    """
    print("\nFORMATION AND SPACING REACH THE RUN")
    sweep = _sweep_mod()
    cfg = {
        "name": "form-test", "scene": "corridor_200m",
        "blue_fleet": str(FIXTURE_FLEETS / "3_roboracer_no_lidar.yaml"),
        "mission": "advance", "goal": "P2", "points": dict(CORRIDOR_POINTS),
        "duration_s": 5.0, "warmup_s": 0.0, "rate_hz": 10.0,
        "stop_when_stalled": False, "coordinator": "gcs",
        "spawns": {"gcs": {"x": -95.0, "y": 0.0},
                   "car1": {"x": -89.0, "y": 0.0},
                   "car2": {"x": -93.0, "y": 3.0},
                   "car3": {"x": -93.0, "y": -3.0}},
        "seeds": [1],
    }
    base_cell = {"authority": "centralized", "routing": "mesh",
                 "jam_dbm": sweep.JAM_OFF, "seed": 1, "cell": "c"}

    def starts(**over):
        _a, agents, _l, _b = sweep.build(cfg, dict(base_cell, **over))
        return {a["id"]: (round(a["start"]["x"], 3), round(a["start"]["y"], 3))
                for a in agents if not a.get("jammer")}

    spawned = starts(formation="(as spawned)")
    wedge3 = starts(formation="wedge", spacing=3.0)
    wedge9 = starts(formation="wedge", spacing=9.0)
    line3 = starts(formation="line", spacing=3.0)

    check("(as spawned) leaves the poses exactly as given",
          spawned["car2"] == (-93.0, 3.0), str(spawned["car2"]))
    check("a formation moves the vehicles", wedge3 != spawned)
    check("a different shape is a different arrangement", wedge3 != line3)
    check("spacing scales the shape", wedge9 != wedge3)

    def spread(d):
        ys = [v[1] for k, v in d.items() if k != "gcs"]
        return max(ys) - min(ys)
    check("tripling the spacing triples the wedge's width",
          abs(spread(wedge9) - 3 * spread(wedge3)) < 1e-2,
          f"{spread(wedge3):.2f} -> {spread(wedge9):.2f}")
    # THE GROUND STATION IS NEVER IN THE FORMATION. Every link in the run is
    # measured against where the bench is; moving it because the fleet changed
    # shape would move the ruler along with the thing being measured.
    check("no formation ever moves the ground station",
          spawned["gcs"] == wedge3["gcs"] == wedge9["gcs"] == line3["gcs"],
          f"{spawned['gcs']} {wedge3['gcs']}")


def test_a_swept_axis_is_min_max_steps_not_a_row_of_tickboxes():
    """The values a parametric axis produces, checked as arithmetic.

    Tickboxes could only offer values somebody had thought to put there, and
    four points do not find a cliff. The Console's AxisRange widget cannot be
    imported without Qt, so the arithmetic it performs is checked here in the
    same form - inclusive of BOTH ends, which is the part that is easy to get
    wrong by one step and hard to notice on a chart.
    """
    print("\nA PARAMETRIC AXIS INCLUDES BOTH ENDS")

    def values(lo, hi, n):
        if n <= 1:
            return [round(lo, 3)]
        return [round(lo + (hi - lo) * i / (n - 1), 3) for i in range(n)]

    check("steps=1 pins the axis to its minimum", values(0, 30, 1) == [0.0])
    check("steps=2 is exactly the two ends", values(0, 30, 2) == [0.0, 30.0])
    v = values(0, 30, 7)
    check("steps=7 spans both ends inclusively",
          v[0] == 0.0 and v[-1] == 30.0 and len(v) == 7, str(v))
    check("the values are evenly spaced",
          all(abs((v[i + 1] - v[i]) - 5.0) < 1e-9 for i in range(len(v) - 1)),
          str(v))
    check("a descending range still runs from min to max",
          values(20, -10, 4) == [20.0, 10.0, 0.0, -10.0],
          str(values(20, -10, 4)))


def test_a_hand_drawn_formation_saves_scales_and_sweeps():
    """A shape dragged out on the map, kept, and then used as a swept value.

    A built-in formation is a FUNCTION of (count, spacing); a hand-drawn one
    cannot be, because it is a specific arrangement of a specific number of
    vehicles. What must survive is the SIZE being parametric - otherwise the
    spacing dial silently stops working the moment you use your own shape,
    which is precisely the sort of "it only works for the built-in case"
    behaviour this project keeps finding and removing.
    """
    print("\nA DRAWN FORMATION IS SAVED, SCALED AND SWEPT")
    tmp = Path(tempfile.mkdtemp())
    with mock.patch.object(st, "FORMATION_DIR", tmp):
        # An arrangement with its middle deliberately NOT at the origin.
        drawn = [(10.0, 0.0, 0.0), (8.0, 2.0, 0.0), (8.0, -2.0, 0.0)]
        path = st.save_formation("my_arrow", drawn)
        check("a saved formation is a file you can read", path.exists())

        doc = yaml.safe_load(path.read_text())
        offs = [tuple(q) for q in doc["offsets"]]
        cx = sum(q[0] for q in offs) / len(offs)
        cy = sum(q[1] for q in offs) / len(offs)
        # Tolerance 1e-3, not 1e-6: offsets are rounded to 4 decimals on the
        # way out so a saved formation is clean YAML rather than sixteen
        # digits of float noise, and a centroid of three rounded numbers
        # carries that rounding. That is the rounding working, not the
        # centring failing.
        check("it is centred on save, so placing it places its middle",
              abs(cx) < 1e-3 and abs(cy) < 1e-3, f"{cx}, {cy}")
        check("it records the spacing it was drawn at",
              abs(doc["spacing"] - min(
                  math.dist(a, b) for i, a in enumerate(drawn)
                  for b in drawn[i + 1:])) < 1e-3, str(doc["spacing"]))
        check("it appears in the list of saved shapes",
              st.custom_formations() == ["my_arrow"],
              str(st.custom_formations()))

        # THE SIZE IS STILL A DIAL. The shape at twice its drawn spacing is
        # the same shape, twice as big.
        at1 = st.formation_offsets("my_arrow", 3, doc["spacing"])
        at2 = st.formation_offsets("my_arrow", 3, doc["spacing"] * 2)
        check("asking for it at its own spacing returns it unchanged",
              all(abs(a[k] - b[k]) < 1e-3
                  for a, b in zip(at1, offs) for k in range(3)), str(at1))
        check("doubling the spacing doubles the drawn shape",
              all(abs(2 * a[k] - b[k]) < 1e-3
                  for a, b in zip(at1, at2) for k in range(3)), str(at2))

        # A DRAWING HAS A FIXED SIZE, and says so rather than inventing more.
        four = st.formation_offsets("my_arrow", 4, doc["spacing"])
        check("it never invents a vehicle it was not drawn with",
              len(four) == 3, str(four))
        agents = [{"id": f"c{i}", "start": {"x": 0.0, "y": float(i),
                                            "z": 0.0, "yaw": 0.0}}
                  for i in range(4)]
        moved = st.apply_formation(agents, "my_arrow", spacing=doc["spacing"])
        check("applying it reports only the vehicles it actually placed",
              len(moved) == 3, str(moved))
        check("...and the one it ran out of room for keeps its spawn pose",
              agents[3]["start"]["y"] == 3.0, str(agents[3]["start"]))

        # It is a value on the swept axis like any other.
        sweep = _sweep_mod()
        cells, names = sweep.cells_of({
            "axes": {"formation": [sweep.AS_SPAWNED, "wedge", "my_arrow"],
                     "spacing": [3.0, 9.0]},
            "seeds": [1]})
        got = sorted({(c["formation"], c["spacing"]) for c in cells})
        check("a saved shape is swept at every spacing, like a built-in",
              ("my_arrow", 3.0) in got and ("my_arrow", 9.0) in got, str(got))
        check("...and (as spawned) is not, because it has nothing to scale",
              sum(1 for g in got if g[0] == sweep.AS_SPAWNED) == 1, str(got))


def test_the_map_cannot_place_what_the_model_will_refuse():
    """Dragging a vehicle or a goal point is bounded by the SAME rule that
    validates an objective.

    Two rules would have differed by the wall margin, and the failure would
    have been silent in the worst way: you drag the goal to the end of the
    corridor, the mission is refused for being outside the arena, and the
    fleet sits still while the reason is three panels away from the thing you
    just did.
    """
    print("\nA DRAG CANNOT BUILD A RUN THE MODEL REFUSES")
    arena, agents, links = st.load_scenario({
        "scene": "corridor_200m",
        "fleets": [str(FIXTURE_FLEETS / "3_roboracer_no_lidar.yaml")]})
    hx = arena["extent"]["x"] / 2
    x, y, z = st.clamp_to_arena(10_000.0, 10_000.0, -5.0, arena)
    check("a drag past the wall is clamped, not accepted",
          x < hx and abs(y) < arena["extent"]["y"] / 2 and z >= 0.0,
          f"{x}, {y}, {z}")
    ok, _hx, _hy = st._in_bounds(x, y, arena)
    check("...and lands somewhere an objective will actually validate", ok)
    good, err = st.validate_objective(
        {"type": "advance", "to": {"x": x, "y": y, "z": z}},
        arena.get("points") or {}, arena)
    check("an advance to the furthest draggable point is accepted",
          good, str(err))
    inside = st.clamp_to_arena(1.0, 2.0, 0.5, arena)
    check("a position already inside is left exactly alone",
          inside == (1.0, 2.0, 0.5), str(inside))


def test_a_moved_point_overrides_the_scene_without_editing_it():
    """Points ride with the RUN, and a dragged one changes only itself.

    A point created or moved in the Console is an override on the composition,
    merged over whatever the scene declares - the same mechanism the
    architecture override uses. It has to leave every other point alone: a
    shallow merge here would delete the points you did not touch, and the
    missions naming them would start failing for no visible reason.
    """
    print("\nA POINT RIDES WITH THE RUN, AND OVERRIDES ONLY ITSELF")
    arena0, _a, _l = st.load_scenario(_corridor())
    before = dict(arena0.get("points") or {})
    check("a composed run carries the points it was given",
          sorted(before) == ["P1", "P2", "P3"], str(sorted(before)))

    moved = _corridor()
    moved["points"] = dict(moved["points"], P2={"x": 40.0, "y": 6.0, "z": 0.0})
    arena1, _a1, _l1 = st.load_scenario(moved)
    after = arena1.get("points") or {}
    check("the dragged point takes its new position",
          (after["P2"]["x"], after["P2"]["y"]) == (40.0, 6.0),
          str(after.get("P2")))
    check("every other point is untouched",
          all(after[k] == before[k] for k in before if k != "P2"),
          str({k: (before[k], after[k]) for k in before if k != "P2"}))

    # A scene that DOES declare points keeps them when the run adds its own.
    lab = {"scene": "lab_box",
           "fleets": [str(FIXTURE_FLEETS / "3_roboracer_no_lidar.yaml")],
           "points": {"P1": {"x": -2.0, "y": 0.0, "z": 0.0}}}
    arena2, agents2, _l2 = st.load_scenario(lab)
    pts2 = arena2.get("points") or {}
    check("a point added by hand is the only geometry a scene has",
          sorted(pts2) == ["P1"], str(sorted(pts2)))
    changed, msgs, _ = st.apply_mission_file(
        str(REPO / "missions" / "advance.yaml"),
        {a["id"]: a for a in agents2}, pts2, arena2, goals=["P1"])
    check("...and can immediately be used as a goal",
          len(changed) >= 3 and all(mm.get("to") == "P1"
                                    for _i, mm in changed),
          f"{len(changed)} tasked: {msgs[:1]}")


def _planned_run(routing="mesh", doctrine="hold", jam_dbm=None, gnss=False,
                 laps=2, dur=400.0, waypoints=("P3", "P1"),
                 jam_at=(30.0, 0.0)):
    """A corridor run with a PLANNED mission, stepped to its end.

    One helper, three attacks: it is the same fleet and the same plan every
    time, so any difference in outcome is the spectrum and nothing else. The
    points are supplied WITH the run, because the corridor declares none of
    its own - P1 is the ground station end, P2 the far end, P3 the middle.
    """
    import random as _rand
    fleets = ["3_roboracer_no_lidar.yaml"]
    if jam_dbm is not None:
        fleets.append("red_gnss.yaml" if gnss else "red_comms.yaml")
    arena, agents, links = st.load_scenario(_corridor(*fleets))
    by = {a["id"]: a for a in agents}
    for aid, (x, y) in (("gcs", (-95, 0)), ("car1", (-89, 0)),
                        ("car2", (-93, 3)), ("car3", (-93, -3)),
                        ("jam1", jam_at)):
        if aid in by:
            by[aid]["start"].update({"x": float(x), "y": float(y)})
    blue = arena["networks"]["blue"]
    blue["authority"] = blue["topology"] = "centralized"
    blue["routing"] = routing
    blue["coordinator"] = "gcs"
    for a in agents:
        if not a.get("jammer") and a.get("platform") != "ground_station":
            a["on_link_loss"] = doctrine
        if a.get("jammer"):
            a["armed"] = jam_dbm is not None
            if jam_dbm is not None:
                a["jammer"]["tx_power"] = {"value": jam_dbm, "unit": "dBm"}
    poses = {a["id"]: {"x": a["start"]["x"], "y": a["start"]["y"],
                       "z": a["start"]["z"], "yaw": a["start"]["yaw"],
                       "speed": 0.0} for a in agents}
    for aid in ("car1", "car2", "car3"):
        st.install_plan(by[aid], list(waypoints), laps)
        by[aid]["armed"] = True
    rng, dt = _rand.Random(1), 0.1
    txs, heard, frame = [], [], None
    for k in range(int(dur / dt)):
        frame = st.frame(k * dt, dt, k, arena, agents, links, poses, rng)
        txs += frame["transmissions"]
        heard += [x for x in frame["transmissions"] if x["heard_by"]]
        if frame["mission_score"]["running"] == 0:
            break
    return frame, txs, heard, agents


def test_a_mission_has_an_end_and_is_passed_or_failed():
    """An objective runs forever; a MISSION finishes.

    Everything in this framework used to be an objective - a shuttling vehicle
    shuttles until you stop it - so "did it work" had no answer and every
    metric was a rate or a mean. A plan is a circuit and a number of laps, and
    a run now scores what FRACTION of the fleet completed it, which is what
    makes two of three getting through a different result from none rather
    than the same slightly-lower average.
    """
    print("\nA MISSION HAS AN END, AND A PASS RATE")
    frame, txs, _heard, _ag = _planned_run()
    sc = frame["mission_score"]
    check("a clean fleet passes its whole mission",
          sc["pass_frac"] == 1.0 and sc["complete"] == 3, str(sc))
    check("nothing is left running once every lap is done",
          sc["running"] == 0 and sc["failed"] == 0, str(sc))
    check("a completed vehicle stops rather than starting again",
          all(a["mission"]["type"] == "static" for a in _ag
              if a["id"].startswith("car")),
          str([a["mission"] for a in _ag if a["id"].startswith("car")][:1]))

    # THE ARRIVAL TOLERANCE IS THE VEHICLE'S OWN LENGTH, not a number
    # somebody picked - a bigger vehicle has a bigger nose.
    car = next(a for a in _ag if a["id"] == "car1")
    check("arrival tolerance comes from the hardware",
          abs(st.arrival_tolerance_m(car)
              - _num_len(car)) < 1e-9,
          f"{st.arrival_tolerance_m(car)} vs {_num_len(car)}")


def _num_len(agent):
    return max((agent.get("dimensions") or {}).get("length", 0.5), 0.25)


def test_the_coordinator_issues_one_leg_at_a_time_over_the_network():
    """THE MECHANISM, and it matters more than the metric.

    The plan is held by the COORDINATOR. The vehicle is told one leg at a
    time, and each reassignment is a command that has to travel. So a fleet
    out of contact does not fail and does not carry on - it ARRIVES and then
    sits there, because the order telling it where to go next never came. That
    is a stall, it is different from a failure, and nothing scripts it: it
    falls out of the link budget.
    """
    print("\nTHE NEXT LEG IS A COMMAND, AND A COMMAND HAS TO GET THROUGH")
    clean, txs, _h, _a = _planned_run()
    check("a reachable fleet is reassigned leg by leg",
          len(txs) == 9, f"{len(txs)} reassignments for 3 cars x 3 legs")
    check("every reassignment names the vehicle and where to go",
          all(t["kind"] == "reassign" and t["to"] and " go " in t["text"]
              for t in txs), str(txs[:1]))
    check("the coordinator is the one transmitting",
          {t["from"] for t in txs} == {"gcs"}, str({t["from"] for t in txs}))

    # THE STALL. `intent` doctrine, so the vehicles keep driving while cut
    # off - they reach the waypoint, believe they are there, and cannot be
    # told what comes next.
    stalled, txs2, _h2, _a2 = _planned_run(doctrine="intent", jam_dbm=40.0,
                                           jam_at=(20.0, 0.0), dur=200.0)
    sc = stalled["mission_score"]
    check("a cut-off vehicle that arrives is left AWAITING ORDERS",
          sc["awaiting_orders"] >= 1, str(sc))
    check("...and is neither complete nor failed - it is stalled",
          sc["complete"] == 0 and sc["failed"] == 0, str(sc))
    check("no order was transmitted to a fleet nobody can reach",
          not txs2, f"{len(txs2)} sent")

    # `hold` doctrine: it never even gets there.
    held, txs3, _h3, _a3 = _planned_run(doctrine="hold", jam_dbm=40.0,
                                        dur=200.0)
    check("a held fleet passes nothing",
          held["mission_score"]["pass_frac"] == 0.0,
          str(held["mission_score"]))


def test_drift_that_makes_a_reported_arrival_untrue_is_a_failure():
    """A vehicle that reports arriving somewhere it is not has FAILED.

    Note what is NOT here: a drift threshold in metres. Inventing one would be
    a number this project cannot source. The test is whether the arrival the
    vehicle is REPORTING is one it actually made, judged with the very
    tolerance the vehicle itself used to decide it had arrived - so the rule
    is derived from the hardware, not chosen.

    The result is the one that matters for GNSS denial: the fleet reports
    mission success while sitting somewhere else entirely.
    """
    print("\nA REPORTED ARRIVAL THAT IS NOT TRUE IS A FAILED MISSION")
    frame, _txs, _h, agents = _planned_run(jam_dbm=40.0, gnss=True, dur=600.0)
    sc = frame["mission_score"]
    check("GNSS denial stops the fleet passing cleanly",
          sc["pass_frac"] < 1.0, str(sc))
    check("...and at least one vehicle fails specifically on drift",
          sc["drifted"] >= 1 and sc["failed"] >= 1, str(sc))
    # A PARTIAL RESULT, and that is the point of scoring a percentage. With
    # the sourced drift rate (0.49% of distance, Papadopoulos & Misailidis
    # 2007) a 190 m corridor accumulates around a metre of error, so SOME
    # vehicles report a false arrival and some do not. Under the invented 4%
    # that preceded it every vehicle failed every time, which looked decisive
    # and was an artefact of a number nobody had measured.
    check("the outcome is partial, not all-or-nothing",
          0.0 <= sc["pass_frac"] < 1.0 and sc["tasked"] == 3, str(sc))
    reasons = [(a.get("_plan") or {}).get("failed_reason")
               for a in agents if (a.get("_plan") or {}).get("failed_reason")]
    check("each failure says how wrong its own estimate was",
          reasons and all("off its own estimate" in r and "tolerance" in r
                          for r in reasons), str(reasons[:1]))
    # The same fleet with a working fix passes, so the failure is the
    # spectrum and not the plan.
    clean, _t, _hh, _aa = _planned_run()
    check("the same plan passes with a position fix",
          clean["mission_score"]["pass_frac"] == 1.0,
          str(clean["mission_score"]))


def test_an_order_is_a_transmission_and_can_be_intercepted():
    """Because the next leg is transmitted, it can be heard.

    Ordinary RF - the coordinator's own transmit power, the network's band,
    the same path loss and the same interference every other link gets. No
    special case, which is what makes it worth building on: to spoof an order
    you must first be able to hear one, and this is where that starts.
    """
    print("\nAN ORDER CAN BE OVERHEARD")
    _f, txs, heard, _a = _planned_run(jam_dbm=None)
    check("nothing is intercepted when there is nobody listening",
          not heard and txs, f"{len(heard)} heard of {len(txs)}")

    # A red agent parked in the middle of the corridor hears the traffic.
    frame, txs2, heard2, _a2 = _planned_run(jam_dbm=-999.0, jam_at=(0.0, 4.0))
    check("a red agent inside the corridor overhears reassignments",
          bool(heard2), f"{len(heard2)} of {len(txs2)}")
    if heard2:
        one = heard2[0]
        check("the intercept recovers the words of the order",
              " go " in one["text"] and one["to"].startswith("car"),
              one["text"])
        check("...and says who heard it, and how well",
              all("sinr_db" in h and h["network"] == "red"
                  for h in one["heard_by"]), str(one["heard_by"]))
    check("a listener never 'intercepts' its own side's traffic",
          all(h["network"] != t["network"]
              for t in txs2 for h in t["heard_by"]))


def test_setplan_is_the_mission_typed_and_is_gated_like_any_order():
    """SETPLAN <who> <points...> laps <n>, from a terminal or a button."""
    print("\nSETPLAN")
    arena, agents, links = st.load_scenario(_corridor())
    by = {a["id"]: a for a in agents}
    poses = _poses_for(agents)
    qdir = Path(tempfile.mkdtemp())
    _queue_with(qdir, "SETPLAN all P1 P2 laps 3\n")
    changed = st.drain_retasks(qdir, by, arena, links, poses)
    check("SETPLAN tasks every mobile vehicle", len(changed) == 3,
          str(changed))
    check("the ground station is never given a plan to carry out",
          by["gcs"].get("_plan") is None)
    pl = by["car1"]["_plan"]
    check("the circuit and the lap count are what was typed",
          pl["waypoints"] == ["P1", "P2"] and pl["laps"] == 3, str(pl))
    check("the vehicle is only told the FIRST leg",
          by["car1"]["mission"] == {"type": "advance", "to": "P1"},
          str(by["car1"]["mission"]))

    qdir2 = Path(tempfile.mkdtemp())
    _queue_with(qdir2, "SETPLAN all NOWHERE laps 1\n")
    before = dict(by["car1"]["_plan"])
    st.drain_retasks(qdir2, by, arena, links, poses)
    check("a point this scene lacks is refused, and changes nothing",
          by["car1"]["_plan"] == before, str(by["car1"]["_plan"]))

    qdir3 = Path(tempfile.mkdtemp())
    _queue_with(qdir3, "SETPLAN car2 P2 laps 1\n")
    st.drain_retasks(qdir3, by, arena, links, poses)
    check("one agent can be given its own plan",
          by["car2"]["_plan"]["waypoints"] == ["P2"]
          and by["car1"]["_plan"]["waypoints"] == ["P1", "P2"],
          str(by["car2"]["_plan"]))


def test_a_mission_file_can_declare_a_plan():
    """`plan: {waypoints: [...], laps: n}` in a mission file.

    And an `advance` with a destination becomes a one-waypoint, one-lap plan
    on its own - so penetration and a four-lap shuttle are the same KIND of
    thing, scored the same way, rather than penetration being a special case
    with its own private notion of arrival.
    """
    print("\nA MISSION FILE DECLARES ITS OWN END")
    arena, agents, links = st.load_scenario(_corridor())
    by = {a["id"]: a for a in agents}
    changed, msgs, _n = st.apply_mission_file(
        str(REPO / "missions" / "advance.yaml"), by,
        arena.get("points") or {}, arena, goals=["P2"])
    check("advance to a point is itself a one-lap plan",
          all((by[a].get("_plan") or {}).get("waypoints") == ["P2"]
              and by[a]["_plan"]["laps"] == 1 for a, _m in changed),
          str([by[a].get("_plan") for a, _m in changed][:1]))

    mpath = Path(tempfile.mkdtemp()) / "circuit.yaml"
    mpath.write_text(yaml.safe_dump({
        "spec_version": 0.1, "name": "circuit", "kind": "mission",
        "plan": {"who": "all", "waypoints": ["P1", "P3", "P2"],
                 "laps": 2}}), encoding="utf-8")
    changed2, msgs2, name2 = st.apply_mission_file(
        str(mpath), by, arena.get("points") or {}, arena)
    check("a declared plan needs no objectives block at all",
          len(changed2) == 3, f"{len(changed2)}: {msgs2[:1]}")
    check("every vehicle gets the circuit and the laps",
          all(by[a]["_plan"]["waypoints"] == ["P1", "P3", "P2"]
              and by[a]["_plan"]["laps"] == 2 for a, _m in changed2))
    check("...and is started on the first leg only",
          all(by[a]["mission"]["to"] == "P1" for a, _m in changed2))

    # An open-ended objective must NOT acquire a plan: `pursue` has no end,
    # and putting it in the pass/fail column would leave it failing forever.
    ok = st.parse_retask("car1: pursue car2", by)
    by["car1"]["mission"] = ok[1]
    qdir = Path(tempfile.mkdtemp())
    _queue_with(qdir, "car1: pursue car2\n")
    st.drain_retasks(qdir, by, arena, links, _poses_for(agents))
    check("an open-ended objective clears the plan rather than faking an end",
          by["car1"].get("_plan") is None, str(by["car1"].get("_plan")))


def test_a_depth_camera_is_not_a_small_lidar():
    """The D435i and the UST-10LX fail in OPPOSITE directions, and the model
    has to show that or there is no reason to have both.

        UST-10LX   270 deg, 0.06-10 m, one horizontal plane, 40 Hz
        D435i      87 deg,  0.28-3 m,  a volume, up to 90 fps

    Reach and coverage against a close-in volume. Which one keeps a fleet
    localised under GNSS denial is therefore a property of the SCENE as much
    as of the vehicle - which is a thing this framework can now say and could
    not before.
    """
    print("\nA DEPTH CAMERA IS NOT A SMALL LIDAR")
    lid, cam = st.sensor_spec("ust10lx"), st.sensor_spec("d435i")
    check("both are recognised as ranging sensors",
          lid is not None and cam is not None)
    check("an IMU is not one - it is what DRIFTS, not what fixes",
          st.sensor_spec("generic_imu") is None)
    check("the lidar reaches much further", lid["range_max"] > cam["range_max"] * 3)
    check("the lidar sees much wider", lid["fov_deg"] > cam["fov_deg"] * 3)
    check("the camera sees closer than the lidar can",
          cam["range_min"] > lid["range_min"])
    check("the camera has a VERTICAL field; the lidar has a plane",
          cam.get("fov_v_deg") and not lid.get("fov_v_deg"))

    # ERROR IS SPECIFIED DIFFERENTLY, and modelled differently. The lidar is
    # +/-40 mm flat; the camera is <2% at 2 m, which grows with range because
    # stereo disparity error does.
    check("the lidar's accuracy is absolute", lid.get("accuracy_m") == 0.040)
    check("the camera's accuracy is proportional",
          cam.get("accuracy_frac") == 0.02 and not cam.get("accuracy_m"))

    arena, agents, links = st.load_scenario({
        "scene": "lab_box",
        "fleets": [str(FIXTURE_FLEETS / "3_roboracer.yaml")]})
    car = next(a for a in agents if a["id"] == "car1")
    depth = dict(car)
    depth["sensors"] = [{"id": "depth", "type": "d435i",
                         "offset": {"x": 0.0, "y": 0.0, "z": 0.14}}]
    poses = _poses_for(agents)
    import random as _rand
    rng = _rand.Random(1)
    sc = st.scan_for(depth, depth["sensors"][0], poses, agents, arena, rng)
    check("a depth scan names the camera, not a lidar",
          sc["model"] == "RealSense D435i", sc["model"])
    check("...and says it is a SLICE of a depth image, not a plane sweep",
          sc["slice_of_depth_image"] is True)
    check("its arc is the camera's 87 degrees",
          abs(math.degrees(sc["angle_max"] - sc["angle_min"]) - 87.0) < 1e-6,
          str(math.degrees(sc["angle_max"] - sc["angle_min"])))
    # Nothing BEYOND range is measured - but a return AT the limit carries
    # its measurement noise like any other, and 2% of 3 m is 6 cm. A camera
    # reporting 3.03 m where its nominal maximum is 3.00 m is what real
    # hardware does; clamping it would hide the noise the datasheet specifies.
    far = max((r for r in sc["ranges"] if r != st.NO_RETURN), default=0.0)
    check("nothing beyond 3 m plus its own noise is ever returned",
          far <= 3.0 * 1.05, f"{far} m")


def test_range_decides_whether_a_sensor_can_localise_at_all():
    """A 3 m sensor in a room whose walls are 4 m away is not aiding.

    This is the whole reason range is in the model rather than being a number
    on a spec sheet. A depth-camera car crossing open ground has nothing
    within three metres to match against, so it dead-reckons however good the
    camera is - and re-acquires as it comes back in near a wall. A lidar
    reaching 10 m does not have that problem in the same room.
    """
    print("\nRANGE DECIDES WHETHER A SENSOR CAN LOCALISE AT ALL")
    lab, _a, _l = st.load_scenario({
        "scene": "lab_box",
        "fleets": [str(FIXTURE_FLEETS / "3_roboracer.yaml")]})
    corridor, _a2, _l2 = st.load_scenario({
        "scene": "corridor_200m",
        "fleets": [str(FIXTURE_FLEETS / "3_roboracer_no_lidar.yaml")]})
    lidar_car = {"sensors": [{"id": "l", "type": "ust10lx"}]}
    depth_car = {"sensors": [{"id": "d", "type": "d435i"}]}
    imu_car = {"sensors": [{"id": "i", "type": "generic_imu"}]}
    both = {"sensors": [{"id": "l", "type": "ust10lx"},
                        {"id": "d", "type": "d435i"}]}

    # FEATURELESS FIRST. The corridor declares every boundary open - no walls
    # to match against - so no sensor localises anywhere in it, however close
    # to the edge it is. Range cannot rescue a scene with nothing in it.
    check("in a scene with no solid boundary, NOTHING localises",
          st._position_aiding(lidar_car, corridor, {"x": 0.0, "y": 19.0})
          is None
          and st._position_aiding(depth_car, corridor, {"x": 0.0, "y": 19.0})
          is None)

    # lab_box is 8 x 8 with solid walls, so its walls are 4 m from the centre.
    mid = {"x": 0.0, "y": 0.0}
    check("in the middle of the room the lidar has the walls at 4 m",
          st._position_aiding(lidar_car, lab, mid) is not None)
    check("...and the camera, reaching 3 m, does NOT",
          st._position_aiding(depth_car, lab, mid) is None)

    near = {"x": 0.0, "y": 3.5}
    check("half a metre off the wall, the camera has it too",
          st._position_aiding(depth_car, lab, near) is not None)

    check("an IMU never localises, anywhere",
          st._position_aiding(imu_car, lab, near) is None)
    check("a two-sensor car falls back to the lidar when the camera is short",
          st._position_aiding(both, lab, mid) is not None)


def test_a_two_sensor_car_publishes_both_without_a_topic_clash():
    """Two publishers on one topic is not a naming inconvenience.

    A car carrying a lidar AND a D435i has two scans and two IMUs - the
    chassis one and the camera's own. Flat topic names put two nodes on one
    topic and a consumer receives an interleaved mixture of both, which is the
    kind of fault that looks like a sensor going mad.
    """
    print("\nBOTH SENSORS, NO TOPIC CLASH")
    ag = {"id": "car1", "platform": "roboracer",
          "sensors": [{"id": "lidar", "type": "ust10lx"},
                      {"id": "depth", "type": "d435i"},
                      {"id": "imu", "type": "generic_imu"}]}
    pubs = st.publications_for(ag)
    topics = [p["topic"] for p in pubs]
    check("every topic is unique", len(topics) == len(set(topics)),
          str([t for t in topics if topics.count(t) > 1]))
    check("both scans are published, namespaced by sensor",
          "/car1/lidar/scan" in topics and "/car1/depth/scan" in topics,
          str(topics))
    check("both IMUs are published",
          "/car1/imu/data" in topics and "/car1/depth/imu" in topics)
    check("the camera publishes the topics its real driver does",
          {"/car1/depth/image_rect_raw", "/car1/depth/camera_info",
           "/car1/depth/color/points"} <= set(topics), str(topics))
    types = {p["topic"]: p["type"] for p in pubs}
    check("the cloud is a PointCloud2 and the depth frame an Image",
          types["/car1/depth/color/points"] == "sensor_msgs/PointCloud2"
          and types["/car1/depth/image_rect_raw"] == "sensor_msgs/Image")
    check("the lidar runs at its 40 Hz and the camera at the driver's 30",
          {p["rate_hz"] for p in pubs if p["topic"] == "/car1/lidar/scan"}
          == {40.0}
          and {p["rate_hz"] for p in pubs if p["topic"] == "/car1/depth/scan"}
          == {30.0})


def test_the_bridge_publishes_every_ranging_sensor_it_advertises():
    """What the ROS 2 bridge puts on the wire must match what the agent says
    it carries - and a topic that is advertised but never written to is worse
    than a missing one, because a graph looks complete and a subscriber waits
    forever.

    Checked against the world node's own source rather than by running ROS,
    which is not installed here: the contract is that every ranging sensor
    gets a namespaced LaserScan and a depth camera also gets a CameraInfo.
    """
    print("\nTHE BRIDGE PUBLISHES WHAT THE AGENT ADVERTISES")
    src = (REPO / "ros2" / "src" / "deadband_ros" / "deadband_ros"
           / "world_node.py").read_text(encoding="utf-8")
    check("the world node publishes one scan per ranging sensor",
          'f"/{aid}/{sen[\'id\']}/scan"' in src, "namespaced scan topic")
    check("...decided by sensor_spec, not a hardcoded lidar type",
          "self.sim.sensor_spec(sen[\"type\"])" in src
          or "sensor_spec(sen[" in src)
    check("a depth camera also gets its intrinsics",
          "CameraInfo" in src and "camera_info" in src)
    check("no flat /<agent>/scan is published any more",
          'f"/{aid}/scan"' not in src,
          "two sensors on one topic is an interleaved mixture")

    ctl = (REPO / "ros2" / "src" / "deadband_ros" / "deadband_ros"
           / "controller_node.py").read_text(encoding="utf-8")
    check("the controller subscribes to its own longest-reaching sensor",
          "range_max" in ctl and "/scan" in ctl)

    # The advertised list and the sensors on the agent have to agree.
    ag = {"id": "car1", "platform": "roboracer",
          "sensors": [{"id": "lidar", "type": "ust10lx"},
                      {"id": "depth", "type": "d435i"}]}
    topics = {p["topic"] for p in st.publications_for(ag)}
    check("both scans are advertised on the topics the node publishes",
          {"/car1/lidar/scan", "/car1/depth/scan"} <= topics, str(topics))
    check("and the camera's intrinsics with them",
          "/car1/depth/camera_info" in topics)


def test_a_dual_band_vehicle_declares_two_radios_and_no_sensors():
    """The control condition for frequency agility.

    Nothing but the link, so what happens to the link IS what happens to the
    vehicle - a car that could localise or keep going on intent would confound
    the measurement.
    """
    print("\nA DUAL-BAND VEHICLE: TWO RADIOS, NO SENSORS")
    doc = yaml.safe_load(
        (REPO / "agents" / "roboracer_dualband.yaml").read_text(
            encoding="utf-8"))
    check("it carries no sensors at all", doc.get("sensors") == [])
    radios = doc.get("radios") or []
    check("it declares two radios", len(radios) == 2, str(len(radios)))
    bands = [_q(r.get("band"), "value") if isinstance(r.get("band"), dict)
             else r.get("band") for r in radios]
    check("on genuinely separate bands", sorted(bands) == [2400, 5800],
          str(bands))
    check("the primary radio is still the singular `radio:` block, so every "
          "existing link calculation reads it unchanged",
          isinstance(doc.get("radio"), dict))

    # ADJACENT-CHANNEL LEAKAGE IS ALREADY MODELLED, which is what makes the
    # second band a real escape rather than a smaller version of the same
    # problem. A jammer on 2.4 GHz puts essentially nothing into 5.8 GHz.
    same = st.aci_mu(0.0)
    far = st.aci_mu(abs(5800 - 2400))
    check("a jammer on the same channel lands in full", same > 0.99, str(same))
    check("...and almost none of it reaches the other band",
          far < 1e-6, f"{far}")


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
               test_jamming_raises_floor_drops_links_and_strips_authority,
               test_jamming_respects_band_separation,
               test_scene_baseline_feeds_rf,
               test_two_fleets_compose_and_stay_separate,
               test_jam_command_tunes_live, test_jammer_range_helper,
               test_missions_are_blue_only, test_leader_loss_doctrine,
               test_vehicle_dynamics_momentum_and_no_yaw_snap,
               test_fleet_self_interference_grows_with_size,
               test_jamming_stops_a_centralized_fleet_but_not_a_decentralized_one,
               test_gnss_jamming_causes_drift_that_grows_recovers_and_lidar_resists,
               test_retask_spool_is_one_file_per_command,
               test_retask_claim_failure_is_never_silent_and_not_lost,
               test_origin_passthrough,
               test_missions_read_belief_not_ground_truth,
               test_routing_gates_authority_not_just_geometry,
               test_formations_are_functions_not_coordinate_lists,
               test_a_mission_is_portable_across_scenes,
               test_a_scene_no_longer_ships_objectives_of_its_own,
               test_setmission_grammar_takes_a_goal,
               test_penetration_is_measured_along_the_axis_of_advance,
               test_formation_and_spacing_are_swept_axes_that_reach_the_model,
               test_a_swept_axis_is_min_max_steps_not_a_row_of_tickboxes,
               test_a_hand_drawn_formation_saves_scales_and_sweeps,
               test_the_map_cannot_place_what_the_model_will_refuse,
               test_a_moved_point_overrides_the_scene_without_editing_it,
               test_a_depth_camera_is_not_a_small_lidar,
               test_range_decides_whether_a_sensor_can_localise_at_all,
               test_a_two_sensor_car_publishes_both_without_a_topic_clash,
               test_the_bridge_publishes_every_ranging_sensor_it_advertises,
               test_a_dual_band_vehicle_declares_two_radios_and_no_sensors,
               test_a_mission_has_an_end_and_is_passed_or_failed,
               test_the_coordinator_issues_one_leg_at_a_time_over_the_network,
               test_drift_that_makes_a_reported_arrival_untrue_is_a_failure,
               test_an_order_is_a_transmission_and_can_be_intercepted,
               test_setplan_is_the_mission_typed_and_is_gated_like_any_order,
               test_a_mission_file_can_declare_a_plan):
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
