# =============================================================================
# Deadband SWEEP — headless parameter sweep over scene + fleet + mission
# =============================================================================
# Runs the same composition many times with one or more parameters varied, and
# writes one CSV row per run. Nothing is drawn and nothing waits on a clock:
# frames are stepped at a fixed dt as fast as the CPU allows, so a run of 60
# simulated seconds takes a fraction of a second and a full grid takes less
# than a coffee.
#
# THREE PROPERTIES THIS TOOL GUARANTEES, because an experiment is worthless
# without them:
#
#   DETERMINISM   Every run is a pure function of (config, seed). The clock is
#                 synthetic (k * dt), the RNG is seeded per run, and no global
#                 state crosses between runs - each worker reloads the
#                 scenario from the YAML. Re-running the grid reproduces it
#                 exactly.
#
#   REPLAYABILITY Every row carries the cell key that produced it. Feed that
#                 key back with --replay and the tool re-runs THAT run alone
#                 and writes the full frame stream as JSONL, which the Console
#                 plays back. You never have to watch 315 runs to look at the
#                 one that was strange.
#
#   PARALLELISM   Runs are independent, so they go out to a process pool. The
#                 grid is embarrassingly parallel; the only serial part is
#                 writing the CSV.
#
# WHAT IT MEASURES, and why each one is defensible:
#
#   commanded_fraction  Mean share of blue vehicles whose decider is reachable,
#                       over the jammed window. This is the framework's
#                       headline: jamming does not stop a robot directly, it
#                       severs the authority that was telling it what to do.
#
#   held_fraction       Mean share of vehicles frozen by their on_link_loss
#                       doctrine. The behavioural consequence of the line above.
#
#   distance_m          Mean true ground distance covered per vehicle over the
#                       jammed window. The task itself, with no tolerance
#                       parameter to argue about: a fleet that is holding is
#                       not travelling.
#
#   track_err_m         Mean distance between a vehicle's TRUE pose and where
#                       its mission wanted it to be. Scored on ground truth,
#                       which the analyst has and the agent does not.
#
#   belief_err_m        Mean distance between a vehicle's own position estimate
#                       and its true pose. Zero with a GNSS fix; grows under
#                       GNSS denial. This is what the fleet does not know it
#                       does not know.
#
#   worst_sinr_db       The fleet's worst active link, meaned over the window.
#                       Max-min fairness: capability is governed by the worst
#                       link, not the average (Zhou et al. 2020).
#
# NOT MEASURED, and deliberately so: anything that would need a threshold or
# a fitted constant this project cannot yet source. See SOURCES.md.
# =============================================================================
from __future__ import annotations

import argparse
import copy
import csv
import itertools
import json
import math
import multiprocessing as mp
import os
import random
import sys
import time
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO))

import yaml  # noqa: E402

import stub_telemetry as st  # noqa: E402

JAM_OFF = -999.0        # sentinel power meaning "the emitter is silent"
# The fleet's own transmit power. rf_link()'s default, and the reference every
# relative jammer power is measured against: jam_rel_db = P_j - P_t.
FLEET_TX_DBM = 20.0
# How close counts as arrived now comes from the VEHICLE - st.arrival_
# tolerance_m(agent), its own length. The constant that used to live here
# quietly asserted that every vehicle in every fleet is the same size, which
# stops being harmless the moment a fleet mixes a rover with a quadcopter.


# ---------------------------------------------------------------------------
# Building one run
# ---------------------------------------------------------------------------
def build(cfg, cell):
    """Compose scene + fleets, then apply this cell's overrides.

    Returns (arena, agents, links, blue_ids). Everything is loaded fresh from
    the YAML so no state leaks between runs in the same worker process.
    """
    # THE COMPOSITION. Either named files (an experiment written as a YAML),
    # or `compose`: a complete run dict handed straight in - which is what the
    # Console does, so a sweep can use the fleet you built in Setup, spawns
    # and per-agent doctrine included, without any of it having to exist as a
    # file first. Nothing is typed into a YAML to run an experiment.
    if cfg.get("compose"):
        base = copy.deepcopy(cfg["compose"])
    else:
        fleets = [cfg["blue_fleet"]]
        if cfg.get("red_fleet"):
            fleets.append(cfg["red_fleet"])
        base = {"scene": cfg["scene"], "fleets": fleets}
    # POINTS THE EXPERIMENT DECLARES. A scene is the world; where you send a
    # fleet inside it is a decision about this run, so an experiment file may
    # carry its own geometry and a Console-composed run carries whatever was
    # placed on the map. Merged one level, so naming one point leaves any the
    # scene does declare alone.
    if cfg.get("points"):
        base.setdefault("points", {})
        base["points"] = {**cfg["points"], **(base.get("points") or {})}
    arena, agents, links = st.load_scenario(base)
    by = {a["id"]: a for a in agents}

    # --- AUTHORITY and ROUTING: the two axes, set independently ------------
    # This is the whole point of the experiment. `authority` decides WHO
    # decides; `routing` decides HOW packets travel. They are separate fields
    # and this sets them separately.
    blue = (arena.get("networks") or {}).get("blue") or {}
    blue["authority"] = cell["authority"]
    blue["routing"] = cell["routing"]
    blue["coordinator"] = cfg.get("coordinator", "gcs")
    if cell["authority"] == "hierarchical" or cell["routing"] == "tiered":
        # A tiered ROUTING needs squads to partition on, and a hierarchical
        # AUTHORITY needs squads to find a leader in. They are declared once
        # in the experiment file and applied to whichever axis needs them -
        # which is itself the separation on show: the same squad structure can
        # carry the routing without carrying the authority, and vice versa.
        blue["squads"] = copy.deepcopy(cfg.get("squads") or {})
        blue["leader_loss"] = cfg.get("leader_loss", "fallback")
    else:
        blue.pop("squads", None)
    # `topology` is the legacy alias command_authority() falls back on. Leaving
    # a stale value there would silently override the axis under test, so it is
    # kept in step with the authority rather than left to rot.
    blue["topology"] = cell["authority"]

    # --- THE JAMMER --------------------------------------------------------
    # One red emitter per declared band, co-located at the fleet file's jammer
    # pose. Two co-located emitters is not a modelling trick: it is exactly the
    # configuration of a single vehicle carrying two transmitters, and it is
    # the honest way to attack two bands at full power rather than pretending
    # one emitter's power covers both for free.
    # SPAWNS and DOCTRINE from the experiment file. Both are decisions, so
    # they live with the experiment rather than in a fleet.
    for aid, xy in (cfg.get("spawns") or {}).items():
        a = by.get(aid)
        if a is not None:
            a["start"].update({k: float(v) for k, v in xy.items()})
    # FORMATION, as a swept axis. Applied to the MOBILE vehicles only - the
    # ground station keeps the position it was deliberately given, because
    # every link in the run is measured against it and shuffling it because
    # the fleet changed shape would move the ruler.
    shape = cell.get("formation") or cfg.get("formation")
    if shape and shape != AS_SPAWNED:
        st.apply_formation(
            agents, shape,
            spacing=float(cell.get("spacing", cfg.get("spacing", 3.0))))

    doct = cfg.get("doctrine")
    if doct:
        for a in agents:
            if not a.get("jammer") and a.get("platform") != "ground_station":
                a["on_link_loss"] = doct

    # P_j / P_t. The sweep parameter is the jammer's power RELATIVE to the
    # fleet's own radios, which is the dimensionless quantity the physics
    # actually depends on. Absolute dBm is derived from it here and nowhere
    # else, so no result is ever expressed in units that only mean something
    # for this particular radio.
    jam_cfg = cfg.get("jammer") or {}
    base_id = jam_cfg.get("id") or next(
        (a["id"] for a in agents if a.get("jammer")), "jam1")
    bands = list(jam_cfg.get("bands_mhz") or [])
    if not bands:
        _b = ((by.get(base_id) or {}).get("jammer") or {}).get("band")
        bands = [st._qty(_b, 2400.0)]
    base = by.get(base_id)
    if base is not None:
        armed = cell["jam_dbm"] > JAM_OFF / 2
        base["armed"] = armed
        base["jammer"]["tx_power"] = {"value": cell["jam_dbm"], "unit": "dBm",
                                      "source": "swept parameter"}
        base["jammer"]["band"] = {"value": bands[0], "unit": "MHz",
                                  "source": "experiment"}
        for i, b in enumerate(bands[1:], start=2):
            extra = copy.deepcopy(base)
            extra["id"] = f"{base_id}_b{i}"
            extra["jammer"]["band"] = {"value": b, "unit": "MHz",
                                       "source": "experiment"}
            agents.append(extra)

    # --- THE MISSION -------------------------------------------------------
    # Applied through the same code path the Console's SETMISSION uses, so the
    # sweep cannot drift away from what a human running the app would get.
    blue_ids = [a["id"] for a in agents
                if a.get("network") == "blue" and not a.get("ghost")
                and not a.get("jammer")]
    mref = str(cfg.get("mission") or "advance")
    mpath = Path(mref)
    if mpath.parent == Path(".") and not mpath.suffix:
        mpath = REPO / "missions" / f"{mref}.yaml"
    # THE GOAL, RE-POINTED AT THIS SCENE. A mission names a point ("advance
    # to FAR"); a scene defines the points. missions/advance.yaml was written
    # against the corridor, so it only ever ran on the one scene with a point
    # by that name - which is why an experiment could not move to another
    # scene at all. The rewrite lives in apply_mission_file, so the sweep, the
    # sandbox and `SETMISSION <name> to <POINT>` all obey ONE rule rather than
    # three that can drift.
    # THE GOALS THIS SWEEP CHOSE. A mission says how many points it needs;
    # `goals` (or the single `goal`, for the penetration path) says where they
    # are. One mission file, any scene.
    changed, messages, _ = st.apply_mission_file(
        str(mpath), by, arena.get("points") or {}, arena,
        goals=(list(cfg.get("goals") or [])
               or ([cfg["goal"]] if cfg.get("goal") else [])))
    if not changed:
        raise RuntimeError("mission tasked no agents: "
                           + ("; ".join(messages) or str(mpath)))

    for a in agents:
        if a["id"] in blue_ids:
            a["armed"] = True
    return arena, agents, links, blue_ids


def _target_on_truth(agent, t, poses, arena):
    """Where the mission WANTED this agent to be, scored on ground truth.

    The agent steers from its own belief - that is the model, and it is what
    makes drift matter. The analyst, unlike the agent, has ground truth, and
    scoring the task against it is what turns a wrong belief into a measurable
    task failure rather than a number in a debug panel.
    """
    try:
        return st.mission_target(agent, t, poses, arena)
    except Exception:
        p = poses.get(agent["id"]) or {}
        return (p.get("x", 0.0), p.get("y", 0.0))


def run_one(args):
    """Execute one cell. Pure function of (cfg, cell) - safe to run in a pool.

    Returns a metrics dict; on failure returns the dict with `error` set rather
    than raising, so one bad cell cannot take down a grid that has been running
    for an hour.
    """
    cfg, cell, want_frames = args
    t_start = time.time()
    # P_j / P_t -> absolute dBm, once, here. The sweep parameter stays the
    # dimensionless ratio everywhere else.
    if "jam_rel_db" in cell and "jam_dbm" not in cell:
        cell = dict(cell)
        cell["jam_dbm"] = FLEET_TX_DBM + float(cell["jam_rel_db"])
    try:
        arena, agents, links, blue_ids = build(cfg, cell)
    except Exception as exc:                       # a broken cell is reported
        return {**cell, "error": f"build: {exc}"}

    by = {a["id"]: a for a in agents}
    poses = {a["id"]: {"x": a["start"]["x"], "y": a["start"]["y"],
                       "z": a["start"]["z"], "yaw": a["start"]["yaw"],
                       "speed": 0.0} for a in agents}
    rng = random.Random(cell["seed"])

    dt = 1.0 / float(cfg.get("rate_hz", 10.0))
    warm = float(cfg.get("warmup_s", 0.0))
    dur = float(cfg.get("duration_s", 60.0))
    n = int(round(dur / dt))
    n_warm = int(round(warm / dt))

    # The jammer stays silent through the warm-up, then transmits. Scoring only
    # the jammed window means the metrics are not diluted by clean seconds.
    jammer_ids = [a["id"] for a in agents if a.get("jammer")]
    on = cell["jam_dbm"] > JAM_OFF / 2
    for jid in jammer_ids:
        by[jid]["armed"] = False

    prev = {i: (poses[i]["x"], poses[i]["y"]) for i in blue_ids}
    acc = {"cmd": [], "held": [], "dist": 0.0, "track": [], "belief": [],
           "sinr": [], "pdr": []}
    frames = []

    # PENETRATION. The outcome metric: how far each vehicle actually got,
    # measured from where it started, as the furthest it ever reached rather
    # than where it ended - a vehicle that advanced and was then pushed back
    # by a collision still got there.
    # ALONG THE AXIS OF ADVANCE, not along x. Penetration used to be "how far
    # did it get in +x", which is only the same thing in a corridor that
    # happens to run east-west - so an experiment on any other scene measured
    # a quantity that meant nothing there. It is now the distance advanced
    # along the line from where the fleet started to the goal it was given,
    # which reduces EXACTLY to the old number on the corridor (the axis is
    # +x there) and is the same idea on any scene.
    #
    # A run with no goal keeps +x, because "advance until something stops
    # you" has no destination to point an axis at.
    gname = cfg.get("goal")
    gpt = (arena.get("points") or {}).get(gname) or {} if gname else {}
    ux, uy = 1.0, 0.0
    if gpt and blue_ids:
        sx = sum(poses[i]["x"] for i in blue_ids) / len(blue_ids)
        sy = sum(poses[i]["y"] for i in blue_ids) / len(blue_ids)
        dx, dy = float(gpt.get("x", 0.0)) - sx, float(gpt.get("y", 0.0)) - sy
        norm = math.hypot(dx, dy)
        if norm > 1e-6:
            ux, uy = dx / norm, dy / norm

    def along(i):
        return poses[i]["x"] * ux + poses[i]["y"] * uy

    x0 = {i: along(i) for i in blue_ids}
    deepest = dict(x0)
    # The goal as a DISTANCE ALONG THAT AXIS. Arrival is therefore "crossed
    # the line through the goal, perpendicular to the advance" - which is what
    # it has always been, and is the only test a formation can pass: a wedge's
    # wingmen are metres to the side of the goal point and have plainly
    # arrived.
    goal_x = None
    if gpt:
        goal_x = float(gpt.get("x", 0.0)) * ux + float(gpt.get("y", 0.0)) * uy
    stalled_since = None
    ended_at = None

    for k in range(n):
        t = k * dt
        if k == n_warm and on:
            for jid in jammer_ids:
                by[jid]["armed"] = True
        f = st.frame(t, dt, k, arena, agents, links, poses, rng)
        if want_frames:
            frames.append(f)
        if k < n_warm:
            for i in blue_ids:
                prev[i] = (poses[i]["x"], poses[i]["y"])
            continue

        blue_out = [a for a in f["agents"] if a["id"] in blue_ids]
        if blue_out:
            acc["cmd"].append(sum(1 for a in blue_out
                                  if (a["authority"] or {}).get("reachable"))
                              / len(blue_out))
            acc["held"].append(sum(1 for a in blue_out if a["link_loss_hold"])
                               / len(blue_out))
            acc["belief"].append(sum(a["position_error_m"] for a in blue_out)
                                 / len(blue_out))
        for i in blue_ids:
            px, py = prev[i]
            acc["dist"] += math.hypot(poses[i]["x"] - px, poses[i]["y"] - py)
            prev[i] = (poses[i]["x"], poses[i]["y"])
            tx, ty = _target_on_truth(by[i], t + dt, poses, arena)
            acc["track"].append(math.hypot(poses[i]["x"] - tx,
                                           poses[i]["y"] - ty))
        w = f.get("worst_link")
        if w:
            acc["sinr"].append(w["sinr_db"])
            acc["pdr"].append(w["pdr"])

        for i in blue_ids:
            if along(i) > deepest[i]:
                deepest[i] = along(i)

        # END THE RUN WHEN THERE IS NOTHING LEFT TO MEASURE. Every vehicle has
        # either arrived or lost its commander, so nobody can advance further.
        # A grace period avoids stopping on a single frame's flicker as a
        # marginal link drops in and out.
        if cfg.get("stop_when_stalled", True):
            done = True
            for a in blue_out:
                arrived = (goal_x is not None
                           and along(a["id"]) >= goal_x
                           - st.arrival_tolerance_m(by[a["id"]]))
                cut = not (a["authority"] or {}).get("reachable", True)
                if not (arrived or cut):
                    done = False
                    break
            if done and blue_out:
                if stalled_since is None:
                    stalled_since = t
                elif t - stalled_since >= float(cfg.get("stall_grace_s", 5.0)):
                    ended_at = t
                    break
            else:
                stalled_since = None

    def mean(xs):
        return round(sum(xs) / len(xs), 4) if xs else None

    pen = {i: deepest[i] - x0[i] for i in blue_ids}
    reach = ([i for i in blue_ids
              if goal_x is not None
              and deepest[i] >= goal_x - st.arrival_tolerance_m(by[i])]
             if goal_x is not None else [])
    full = 0.0
    if goal_x is not None and blue_ids:
        full = max((goal_x - x0[i]) for i in blue_ids) or 1.0

    score = st.mission_score(agents)
    row = {
        **cell,
        "runs_agents": len(blue_ids),
        # THE OUTCOME, AS A PERCENTAGE OF THE FLEET. Penetration says how far
        # they got; this says whether the job was done. Two of three getting
        # through is a different result from none, and both are different from
        # all - a mean distance hides that and a pass rate does not.
        "mission_pass_frac": score["pass_frac"],
        "mission_complete": score["complete"],
        "mission_failed": score["failed"],
        # STALLED, NOT FAILED: arrived at a waypoint and never told the next
        # leg, because the order could not get through. The number to watch -
        # a fleet that has not failed and cannot proceed.
        "mission_awaiting_orders": score["awaiting_orders"],
        # Reported an arrival it had not made. Under GNSS denial a fleet can
        # report success from somewhere else entirely.
        "mission_drifted": score["drifted"],
        # THE HEADLINE. How far the fleet advanced, in metres and as a
        # fraction of the corridor it was asked to cross. A complete success
        # is every vehicle at the goal: penetration_frac 1.0, arrived 3.
        "penetration_m": round(sum(pen.values()) / max(len(pen), 1), 2),
        "penetration_max_m": round(max(pen.values()) if pen else 0.0, 2),
        "penetration_min_m": round(min(pen.values()) if pen else 0.0, 2),
        "penetration_frac": round(
            (sum(pen.values()) / max(len(pen), 1)) / full, 4) if full else None,
        "arrived": len(reach),
        "complete_success": int(bool(blue_ids)
                                and len(reach) == len(blue_ids)),
        "ended_s": round(ended_at, 1) if ended_at is not None else None,
        "commanded_fraction": mean(acc["cmd"]),
        "held_fraction": mean(acc["held"]),
        "distance_m": round(acc["dist"] / max(len(blue_ids), 1), 3),
        "track_err_m": mean(acc["track"]),
        "belief_err_m": mean(acc["belief"]),
        "worst_sinr_db": mean(acc["sinr"]),
        "worst_pdr": mean(acc["pdr"]),
        "wall_s": round(time.time() - t_start, 3),
        "error": "",
    }
    return (row, frames) if want_frames else row


# ---------------------------------------------------------------------------
# The grid
# ---------------------------------------------------------------------------
# The formation value that means "leave the vehicles where they were placed".
# Shared with the Console, which offers it as the control every other shape is
# compared against.
AS_SPAWNED = "(as spawned)"


def cells_of(cfg):
    """Cartesian product of the declared axes x seeds, in a stable order.

    MINUS THE CELLS THAT ARE DUPLICATES BY CONSTRUCTION. Spacing scales a
    formation, so a cell with no formation - (as spawned) - has nothing for it
    to scale: sweeping five spacings against it produces five byte-identical
    runs, which is not a result, it is a wait, and five identical rows in the
    table that look like a suspiciously flat finding. One of them is kept, at
    the first spacing on the axis, so the control is still there to compare
    against.
    """
    axes = cfg.get("axes") or {}
    names = list(axes.keys())
    spacings = list(axes.get("spacing") or [])
    out = []
    for combo in itertools.product(*(axes[n] for n in names)):
        cell0 = dict(zip(names, combo))
        if (spacings and cell0.get("formation") == AS_SPAWNED
                and cell0.get("spacing") != spacings[0]):
            continue
        for seed in cfg.get("seeds", [1]):
            cell = dict(cell0)
            cell["seed"] = seed
            cell["cell"] = key_of(cell, names)
            out.append(cell)
    return out, names


def key_of(cell, names):
    """The replay key: a stable, human-typable identifier for one run."""
    return ",".join(f"{n}={cell[n]}" for n in list(names) + ["seed"])


def main(argv=None):
    ap = argparse.ArgumentParser(description="Deadband headless sweep")
    ap.add_argument("experiment", help="path to an experiment YAML")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2)),
                    help="parallel worker processes (default: every core)")
    ap.add_argument("--out", default=None, help="output directory")
    ap.add_argument("--replay", metavar="CELL",
                    help="re-run ONE cell by its key and write its frames as "
                         "JSONL for Console playback")
    args = ap.parse_args(argv)

    cfg = yaml.safe_load(Path(args.experiment).read_text())
    cells, names = cells_of(cfg)

    # ---- replay: one cell, full frame stream --------------------------------
    if args.replay:
        want = args.replay.strip()
        match = [c for c in cells if c["cell"] == want]
        if not match:
            sys.exit(f"no such cell: {want}\nfirst few:\n  " +
                     "\n  ".join(c["cell"] for c in cells[:5]))
        row, frames = run_one((cfg, match[0], True))
        out = Path(args.out or REPO / "runs") / (
            f"replay_{cfg['name']}_{want.replace(',', '_').replace('=', '')}"
            ".jsonl")
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w") as fh:
            for f in frames:
                fh.write(json.dumps(f) + "\n")
        print(f"replayed {want}")
        for k, v in row.items():
            print(f"  {k:20} {v}")
        print(f"\n{len(frames)} frames -> {out}")
        return 0

    # ---- the grid -----------------------------------------------------------
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = Path(args.out or REPO / "runs" / f"sweep_{cfg['name']}_{stamp}")
    outdir.mkdir(parents=True, exist_ok=True)

    shape = " x ".join("%s:%d" % (n, len(cfg["axes"][n])) for n in names)
    # `cells` may be fewer than that product: see cells_of.
    print("%s: %d runs (%s x seed:%d) on %d workers"
          % (cfg["name"], len(cells), shape, len(cfg.get("seeds", [1])),
             args.jobs))
    t0 = time.time()
    payload = [(cfg, c, False) for c in cells]
    if args.jobs > 1:
        with mp.Pool(args.jobs) as pool:
            rows = []
            for i, r in enumerate(pool.imap_unordered(run_one, payload), 1):
                rows.append(r)
                if i % 25 == 0 or i == len(cells):
                    print(f"  {i}/{len(cells)}  ({time.time() - t0:.1f}s)")
    else:
        rows = [run_one(p) for p in payload]
    rows.sort(key=lambda r: r.get("cell", ""))

    fields = (names + ["seed", "cell", "runs_agents",
                       "mission_pass_frac", "mission_complete",
                       "mission_failed", "mission_awaiting_orders",
                       "mission_drifted",
                       "penetration_m", "penetration_frac",
                       "penetration_max_m", "penetration_min_m",
                       "arrived", "complete_success", "ended_s",
                       "commanded_fraction",
                       "held_fraction", "distance_m", "track_err_m",
                       "belief_err_m", "worst_sinr_db", "worst_pdr",
                       "wall_s", "error"])
    csv_path = outdir / "results.csv"
    with csv_path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)

    # The experiment file is copied beside its results. Without it a CSV is an
    # orphan: you cannot say what was run, only what came out.
    (outdir / "experiment.yaml").write_text(Path(args.experiment).read_text())
    (outdir / "provenance.json").write_text(json.dumps({
        "experiment": cfg["name"],
        "started": stamp,
        "runs": len(rows),
        "failed": sum(1 for r in rows if r.get("error")),
        "wall_s": round(time.time() - t0, 2),
        "python": sys.version.split()[0],
        "rate_hz": cfg.get("rate_hz"),
        "duration_s": cfg.get("duration_s"),
        "warmup_s": cfg.get("warmup_s"),
        "replay": "python3 tools/sweep.py <experiment> --replay '<cell>'",
    }, indent=2))

    bad = [r for r in rows if r.get("error")]
    print(f"\n{len(rows)} runs in {time.time() - t0:.1f}s"
          + (f"  ({len(bad)} FAILED)" if bad else ""))
    for r in bad[:5]:
        print(f"  FAILED {r.get('cell')}: {r['error']}")
    print(f"-> {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
