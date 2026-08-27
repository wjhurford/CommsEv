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

Each of those would have been caught in under a second by a test.
"""

import math
import sys
import traceback
from pathlib import Path

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
    for folder in ("scenarios", "missions", "scenes"):
        for f in sorted((REPO / folder).glob("*.yaml")):
            try:
                doc = st.resolve_mission(str(f))
                ok = isinstance(doc, dict) and bool(doc.get("agents"))
                check(f"{folder}/{f.name}", ok,
                      "no agents after merge" if not ok else "")
            except Exception as exc:
                check(f"{folder}/{f.name}", False, repr(exc))


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


if __name__ == "__main__":
    for fn in (test_rf, test_topology, test_two_squad_hierarchy,
               test_authority_modes, test_blast_radius, test_files_load,
               test_mission_contract, test_retask_grammar):
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
