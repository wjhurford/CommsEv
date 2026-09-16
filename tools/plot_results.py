# =============================================================================
# CommsEv — plot a sweep's results. NO DEPENDENCIES AT ALL.
# =============================================================================
# Writes an SVG (open it in any browser) and prints the numbers as a table.
#
# Pure standard library, deliberately. The first version of this drove the
# Console's own Qt renderer, which meant the tool you reach for when the chart
# is broken needed the very thing you were trying to diagnose - and it fell
# over on a machine where PySide6 lives under a different Python. A diagnostic
# has to work when nothing else does.
#
#     python3 tools/plot_results.py                       # newest sweep
#     python3 tools/plot_results.py runs/<dir>/results.csv
#     python3 tools/plot_results.py <csv> --db 0 20       # only these powers
#     python3 tools/plot_results.py <csv> --metric commanded_fraction
#
# TWO VISUAL CHANNELS FOR TWO INDEPENDENT VARIABLES:
#   colour = command authority (who decides)
#   dash   = routing           (how packets travel)
# They are independent axes in the model, so they get independent channels.
# =============================================================================
import csv
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

AUTH_COLOUR = {"centralized": "#D2694F",
               "decentralized": "#4FA3D1",
               "hierarchical": "#6FAE7E"}
ROUTE_DASH = {"star": "none", "mesh": "9 5", "tiered": "2 5"}


def newest_csv():
    runs = sorted((REPO / "runs").glob("sweep_*/results.csv"),
                  key=lambda q: q.stat().st_mtime)
    if not runs:
        sys.exit("no sweep results under runs/ - run an experiment first")
    return runs[-1]


def series_of(rows, metric):
    acc = {}
    for r in rows:
        try:
            x, y = float(r.get("jam_rel_db")), float(r.get(metric))
        except (TypeError, ValueError):
            continue
        acc.setdefault((r.get("authority"), r.get("routing")), {}) \
           .setdefault(x, []).append(y)
    return {k: sorted((x, sum(v) / len(v)) for x, v in d.items())
            for k, d in acc.items()}


def svg(series, metric, W=1200, H=680):
    L, R, T, B = 88, 250, 34, 74
    w, h = W - L - R, H - T - B
    xs = [x for pts in series.values() for x, _ in pts]
    ys = [y for pts in series.values() for _, y in pts]
    x0, x1 = min(xs), max(xs)
    if x1 - x0 < 1e-9:
        x0, x1 = x0 - 1.0, x1 + 1.0
    y1 = (max(ys) or 1.0) * 1.10

    def sx(v):
        return L + (v - x0) / (x1 - x0) * w

    def sy(v):
        return T + h - (v / y1) * h

    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
         f'viewBox="0 0 {W} {H}" font-family="Consolas,DejaVu Sans Mono,'
         f'monospace">',
         f'<rect width="{W}" height="{H}" fill="#181C1F"/>']
    for i in range(6):
        v = y1 * i / 5.0
        o.append(f'<line x1="{L}" y1="{sy(v):.1f}" x2="{L+w}" y2="{sy(v):.1f}"'
                 f' stroke="#2C3236" stroke-width="1"/>')
        o.append(f'<text x="{L-10}" y="{sy(v)+4:.1f}" fill="#8A969E" '
                 f'font-size="12" text-anchor="end">{v:.0f}</text>')
    o.append(f'<line x1="{L}" y1="{T}" x2="{L}" y2="{T+h}" stroke="#4A5257"/>')
    o.append(f'<line x1="{L}" y1="{T+h}" x2="{L+w}" y2="{T+h}" '
             f'stroke="#4A5257"/>')
    for x in sorted(set(xs)):
        o.append(f'<text x="{sx(x):.1f}" y="{T+h+22}" fill="#8A969E" '
                 f'font-size="12" text-anchor="middle">{x:+.0f} dB</text>')
    o.append(f'<text x="{L}" y="{T+h+48}" fill="#8A969E" font-size="12">'
             f'JAMMER ADVANTAGE  P_j / P_t  &#8212; dimensionless: the '
             f'absolute powers cancel</text>')
    o.append(f'<text transform="translate(24,{T+h}) rotate(-90)" '
             f'fill="#8A969E" font-size="12">{metric}</text>')

    ly = T + 6
    for (auth, route), pts in sorted(series.items()):
        col = AUTH_COLOUR.get(auth, "#AAAAAA")
        dash = ROUTE_DASH.get(route, "none")
        pl = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in pts)
        o.append(f'<polyline points="{pl}" fill="none" stroke="{col}" '
                 f'stroke-width="2.4" stroke-dasharray="{dash}"/>')
        for x, y in pts:
            o.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="3.4" '
                     f'fill="{col}"/>')
        o.append(f'<line x1="{L+w+18}" y1="{ly}" x2="{L+w+56}" y2="{ly}" '
                 f'stroke="{col}" stroke-width="2.4" '
                 f'stroke-dasharray="{dash}"/>')
        o.append(f'<text x="{L+w+64}" y="{ly+4}" fill="#8A969E" '
                 f'font-size="12">{auth} / {route}</text>')
        ly += 22
    o.append(f'<text x="{L+w+18}" y="{ly+18}" fill="#6B7378" font-size="11">'
             f'colour = authority   dash = routing</text>')
    o.append("</svg>")
    return "\n".join(o)


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    path = Path(args[0]) if args else newest_csv()
    if not path.exists():
        sys.exit(f"no such file: {path}")

    metric = "penetration_m"
    if "--metric" in argv:
        metric = argv[argv.index("--metric") + 1]
    keep = None
    if "--db" in argv:
        rest = argv[argv.index("--db") + 1:]
        keep = {float(v) for v in rest if not v.startswith("--")}

    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if keep is not None:
        rows = [r for r in rows
                if r.get("jam_rel_db") not in (None, "")
                and float(r["jam_rel_db"]) in keep]
    if not rows:
        sys.exit("no rows to plot (check --db)")
    if metric not in rows[0]:
        sys.exit(f"no column '{metric}'. Columns: {', '.join(rows[0])}")

    series = series_of(rows, metric)
    if not series:
        sys.exit(f"'{metric}' had no numeric values in any row")

    powers = sorted({x for pts in series.values() for x, _ in pts})
    print(f"{len(rows)} rows -> {len(series)} series   metric: {metric}\n")
    print(f"{'authority':14} {'routing':8}" +
          "".join(f"{p:+10.0f}dB" for p in powers))
    for (auth, route), pts in sorted(series.items()):
        d = dict(pts)
        print(f"{auth:14} {route:8}" +
              "".join(f"{d.get(p, float('nan')):12.1f}" for p in powers))

    out = path.with_suffix(".svg")
    out.write_text(svg(series, metric), encoding="utf-8")
    print(f"\nwrote {out}")
    print("open it in any browser")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
