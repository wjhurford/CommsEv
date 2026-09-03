# =============================================================================
# Deadband — render a sweep's results to a PNG, with no Console
# =============================================================================
# Two jobs, and the second is the reason it exists:
#
#   1. A chart you can put in a slide, from any results.csv.
#   2. A DIAGNOSTIC. It uses the Console's own ResultPlot renderer, so if this
#      produces a correct PNG then the chart code is sound and any blankness
#      in the Console is a window/layout problem, not a drawing one. Two
#      rounds of guessing at a blank widget is what this exists to prevent.
#
#     python3 tools/plot_results.py                      # newest sweep
#     python3 tools/plot_results.py runs/<dir>/results.csv
#     python3 tools/plot_results.py <csv> --db 0 10 20   # which powers
#
# Writes <results>.png beside the CSV and prints the path.
# =============================================================================
import csv
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "console"))

# Offscreen: no window is created, so this runs over ssh, in CI, or beside a
# Console that is already open.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


def newest_csv():
    runs = sorted((REPO / "runs").glob("sweep_*/results.csv"),
                  key=lambda p: p.stat().st_mtime)
    if not runs:
        sys.exit("no sweep results found under runs/ - run an experiment first")
    return runs[-1]


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    path = Path(args[0]) if args else newest_csv()
    if not path.exists():
        sys.exit(f"no such file: {path}")

    keep = None
    if "--db" in argv:
        i = argv.index("--db")
        keep = {float(v) for v in argv[i + 1:] if not v.startswith("--")}

    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if keep is not None:
        rows = [r for r in rows if float(r.get("jam_rel_db", "nan")) in keep]
    if not rows:
        sys.exit("no rows to plot (check --db)")

    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    import app as console_app                      # console/app.py

    plot = console_app.ResultPlot()
    plot.rows = rows
    pm = plot.render_chart(size=(1400, 720))

    out = path.with_suffix(".png")
    pm.save(str(out), "PNG")

    # Say what was actually drawn, so a blank PNG is never a mystery either.
    series = plot.series()
    print(f"{len(rows)} rows -> {len(series)} series")
    for (auth, route), pts in sorted(series.items()):
        vals = "  ".join(f"{x:+.0f}dB:{y:.1f}m" for x, y in pts)
        print(f"  {auth:14} {route:7} {vals}")
    print(f"\nwrote {out}")
    del app
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
