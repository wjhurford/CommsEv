#!/usr/bin/env python3
"""Print the network picture for a scene or mission, in one command.

    python3 tools/netcheck.py                        # default_run.yaml
    python3 tools/netcheck.py default_run.yaml
    python3 tools/netcheck.py --budget               # link budget vs distance

Exists because the alternative was pasting multi-line python -c into bash,
where the leading indentation of a nicely-formatted snippet makes Python throw
IndentationError. This is the same information, one argument, no quoting.

It shows what the Console currently does NOT: command authority, measured
topology, and which links are actually carrying traffic.
"""

import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "tools"))


def budget():
    from stub_telemetry import rf_link
    a = {"x": 0, "y": 0, "z": 0}
    print("Link budget: 2.4 GHz, 20 dBm tx, path loss exponent 2.8\n")
    print(f"{'dist':>6}  {'rx dBm':>8}  {'SINR dB':>8}  {'PDR':>6}  state")
    for d in (1, 2, 5, 10, 20, 50, 100, 200, 300, 500):
        r = rf_link(a, {"x": d, "y": 0, "z": 0})
        print(f"{d:>5}m  {r['rx_dbm']:>8.1f}  {r['sinr_db']:>8.1f}  "
              f"{r['pdr']:>6.3f}  {r['state']}")
    print("\nNOTE: path loss exponent 2.8 and noise floor -95 dBm are "
          "defensible\ndefaults, NOT measurements. See SOURCES.md.")


def first_frame(scenario):
    """Run the sim just long enough to catch one frame."""
    proc = subprocess.Popen(
        [sys.executable, str(REPO / "tools" / "stub_telemetry.py"),
         "--scenario", str(scenario)],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    try:
        for line in proc.stdout:
            line = line.strip()
            if line.startswith("{"):
                return json.loads(line)
    finally:
        proc.kill()
    return None


def report(scenario):
    f = first_frame(scenario)
    if not f:
        sys.exit(f"no frame from {scenario}")

    print(f"=== {scenario} ===\n")

    top = f.get("topology") or {}
    print("MEASURED TOPOLOGY")
    print(f"  shape            {top.get('shape')}")
    print(f"  max betweenness  {top.get('max_betweenness')}")
    print(f"  hub              {top.get('hub')}")
    print(f"  nodes / edges    {top.get('nodes')} / {top.get('edges')}")
    print("  (measured from the graph, not read from the file - if this")
    print("   disagrees with what the scene declares, that is the finding)\n")

    print("COMMAND AUTHORITY  (who decides for each agent)")
    print(f"  {'agent':<8}{'decider':<10}{'tier':<14}reachable")
    for a in f.get("agents", []):
        au = a.get("authority") or {}
        flag = "yes" if au.get("reachable") else "NO  <-- orphaned"
        print(f"  {a['id']:<8}{str(au.get('decider')):<10}"
              f"{str(au.get('tier')):<14}{flag}")
    print()

    print("LINKS  (active = carries traffic under this routing)")
    print(f"  {'pair':<16}{'state':<11}{'PDR':<8}{'SINR':<9}use")
    for l in f.get("links", []):
        pair = f"{l['a']}-{l['b']}"
        use = "ACTIVE" if l.get("active") else "spare"
        print(f"  {pair:<16}{l['state']:<11}{l['pdr']:<8}"
              f"{l.get('sinr_db', '?'):<9}{use}")
    active = sum(1 for l in f.get("links", []) if l.get("active"))
    spare = len(f.get("links", [])) - active
    print(f"\n  {active} active, {spare} spare (reachable but unused).")
    print("  Spare links are the capacity a mesh can route damage around;")
    print("  a star has none. The viewport draws both the same for now.")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:]]
    if "--budget" in args:
        budget()
    else:
        scn = args[0] if args else "default_run.yaml"
        report(scn)
