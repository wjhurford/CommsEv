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


# ---------------------------------------------------------------------------
# Building one run
# ---------------------------------------------------------------------------
def build(cfg, cell):
    """Compose scene + fleets, then apply this cell's overrides.

    Returns (arena, agents, links, blue_ids). Everything is loaded fresh from
    the YAML so no state leaks between runs in the same worker process.
    """
    fleets = [cfg["blue_fleet"]]
    if cfg.get("red_fleet"):
        fleets.append(cfg["red_fleet"])
    arena, agents, links = st.load_scenario(
        {"scene": cfg["scene"], "fleets": fleets})
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
    jam_cfg = cfg.get("jammer") or {}
    base_id = jam_cfg.get("id", "jam1")
    bands = list(jam_cfg.get("bands_mhz") or [2400])
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
    st.apply_mission_file(str(REPO / "missions" / f"{cfg['mission']}.yaml"),
                          by, arena.get("points") or {}, arena)
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

    def mean(xs):
        return round(sum(xs) / len(xs), 4) if xs else None

    row = {
        **cell,
        "runs_agents": len(blue_ids),
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
def cells_of(cfg):
    """Cartesian product of the declared axes x seeds, in a stable order."""
    axes = cfg.get("axes") or {}
    names = list(axes.keys())
    out = []
    for combo in itertools.product(*(axes[n] for n in names)):
        for seed in cfg.get("seeds", [1]):
            cell = dict(zip(names, combo))
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

    fields = (names + ["seed", "cell", "runs_agents", "commanded_fraction",
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
