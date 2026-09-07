"""
Deadband Console — the application.

This is the thing you double-click. Everything else runs behind it: the scenario
file is loaded and checked by deadband.spec, and the telemetry source is a
background process whose output feeds the views. No terminal, ever.

Structure of this file:

    Viewport      the 2D world view, three fixed orthographic projections
    SensorView    what one agent's lidar sees, drawn as a polar plot
    Console       the main window: docks, toolbar, process control, editing

Deliberately absent: physics, radio modelling, and any 3D engine. Three
orthographic projections of axis-aligned boxes is a 2D drawing problem, which is
why this whole file fits in one sitting.
"""

from __future__ import annotations

import copy
import json
import re
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# The same repo as WSL sees it: C:\Users\x -> /mnt/c/Users/x.
#
# Written against the string rather than pathlib's drive attribute, because
# pathlib only parses Windows drives when it is running ON Windows - so the
# pathlib version silently did nothing anywhere it could be tested.
def _wsl_path(p):
    import re
    text = str(p).replace("\\", "/")
    m = re.match(r"^([A-Za-z]):/(.*)$", text)
    return f"/mnt/{m.group(1).lower()}/{m.group(2)}" if m else text


REPO_WSL_PATH = _wsl_path(REPO_ROOT)

# Where the bag recorder writes its own pid, inside WSL. See stop_ros_stack.
BAG_PIDFILE = "/tmp/deadband_bag.pid"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from PySide6.QtCore import (
    QMimeData, QPointF, QProcess, QRectF, Qt, QThread, Signal,
)
from PySide6.QtGui import (
    QAction, QBrush, QColor, QDrag, QFont, QPainter, QPen, QPixmap,
    QPolygonF,
)
from PySide6.QtWidgets import (
    QButtonGroup,
    QApplication, QDockWidget, QFileDialog, QHBoxLayout, QLabel, QMainWindow,
    QMessageBox, QPlainTextEdit, QPushButton, QStackedWidget, QTabWidget,
    QTableWidget, QTableWidgetItem, QTreeWidget, QTreeWidgetItem,
    QComboBox, QDialog, QInputDialog, QLineEdit, QMenu, QSizePolicy,
    QSlider, QSplitter, QProgressBar, QCheckBox,
    QVBoxLayout, QWidget,
)

from deadband import spec

# The Console shows the same merged map+mission the simulator runs, so a split
# mission (agents in the map, objectives in the mission file) displays its
# agents rather than an empty tree. Reuse the sim's own resolver so there is one
# merge rule, not two that can drift.
try:
    sys.path.insert(0, str(REPO_ROOT / "tools"))
    from stub_telemetry import resolve_mission as _resolve_mission
    from stub_telemetry import jammer_range_m as _jammer_range_m
    # Same tokenizer the sim's own retask grammar uses, so a parenthesized
    # coordinate typed here and re-parsed there is the same one line, not
    # two tokenizers that can drift apart.
    from stub_telemetry import _tokenize_args
except Exception:
    _resolve_mission = None
    _jammer_range_m = None
    _tokenize_args = None

# Round-trip YAML keeps the comments in a scenario file alive across an edit.
# Those comments are half the value of the file as a research artefact, so
# losing them silently would be worse than not editing at all.
try:
    from ruamel.yaml import YAML
    _RT = YAML()
    _RT.preserve_quotes = True
    _RT.width = 100
    # Match the hand-written indentation of the scenario files. Without this
    # ruamel reindents every list on save and a one-field edit shows up in git
    # as a hundred changed lines, which makes the diff useless for review.
    _RT.indent(mapping=2, sequence=4, offset=2)
except ImportError:
    _RT = None

# ---------------------------------------------------------------------------
# Appearance. One accent; semantic colour reserved for status.
# ---------------------------------------------------------------------------

C_BG, C_PANEL, C_LINE = "#1E2225", "#252A2E", "#343B40"
C_TEXT, C_DIM, C_ACCENT = "#D6DCE0", "#8A969E", "#3E9AA8"
C_WARN, C_ERR, C_GRID = "#C99A3E", "#C4685A", "#2C3236"
C_OK = "#6FAE7E"

# Colour identifies the NETWORK, line style identifies the STATE. Using colour
# for state meant a degraded blue link and a healthy red one would eventually be
# the same hue, which stops working the moment adversarial agents arrive.
NETWORK_COLOURS = {"blue": "#4FA3D1", "red": "#C4685A", "green": "#6FAE7E"}


def network_colour(name):
    if name in NETWORK_COLOURS:
        return NETWORK_COLOURS[name]
    palette = list(NETWORK_COLOURS.values())
    return palette[hash(str(name)) % len(palette)]

STYLE = f"""
QMainWindow, QWidget {{ background: {C_BG}; color: {C_TEXT};
    font-family: 'Segoe UI', sans-serif; font-size: 12px; }}
QDockWidget::title {{ background: {C_PANEL}; padding: 2px 8px;
    border-bottom: 1px solid {C_LINE}; text-transform: uppercase;
    letter-spacing: 1px; color: {C_DIM}; }}
QTreeWidget, QTableWidget, QPlainTextEdit {{ background: {C_PANEL};
    border: 1px solid {C_LINE}; selection-background-color: {C_ACCENT};
    selection-color: #12171A; }}
QTreeWidget::item, QTableWidget::item {{ padding: 3px; }}
QHeaderView::section {{ background: {C_BG}; color: {C_DIM}; border: none;
    border-bottom: 1px solid {C_LINE}; padding: 2px 4px; font-size: 10px;
    text-transform: uppercase; letter-spacing: 1px; }}
QPushButton {{ background: {C_PANEL}; border: 1px solid {C_LINE};
    padding: 5px 12px; color: {C_TEXT}; }}
QPushButton:hover {{ border-color: {C_ACCENT}; color: {C_ACCENT}; }}
QPushButton:checked {{ background: {C_ACCENT}; color: #12171A;
    border-color: {C_ACCENT}; }}
QPushButton#tool {{ font-size: 14px; padding: 0px; }}
QComboBox {{ background: {C_PANEL}; border: 1px solid {C_LINE}; padding: 4px 8px; }}
QComboBox QAbstractItemView {{ background: {C_PANEL}; border: 1px solid {C_LINE};
    selection-background-color: {C_ACCENT}; selection-color: #12171A; }}
QTabWidget::pane {{ border: 1px solid {C_LINE}; }}
QTabBar::tab {{ background: {C_BG}; color: {C_DIM}; padding: 3px 10px;
    border: 1px solid {C_LINE}; font-size: 10px;
    text-transform: uppercase; letter-spacing: 1px; }}
QTabBar::tab:selected {{ background: {C_PANEL}; color: {C_ACCENT}; }}
QTabBar::tab:west {{ padding: 10px 4px; margin: 0px; border-right: none; }}
QTabWidget#chrome > QTabBar::tab {{ background: {C_PANEL}; color: {C_DIM};
    padding: 3px 12px; border: none; border-bottom: 1px solid {C_LINE};
    font-size: 10px; letter-spacing: 1px; }}
QTabWidget#chrome > QTabBar::tab:selected {{ color: {C_ACCENT};
    border-bottom: 1px solid {C_ACCENT}; }}
QTabBar::tab:west:selected {{ border-left: 2px solid {C_ACCENT}; }}
QStatusBar {{ background: {C_PANEL}; border-top: 1px solid {C_LINE};
    color: {C_DIM}; }}
QMenuBar {{ background: {C_PANEL}; }}
QMenuBar::item:selected, QMenu::item:selected {{ background: {C_ACCENT};
    color: #12171A; }}
QMenu {{ background: {C_PANEL}; border: 1px solid {C_LINE}; }}
QLabel#hint {{ color: {C_DIM}; padding: 14px; }}
QWidget#bottomframe {{ background: {C_PANEL}; border-top: 1px solid {C_LINE}; }}
"""


def _num(v, default=0.0):
    """Accept either a bare number or a {value, unit, source} quantity."""
    if isinstance(v, dict):
        v = v.get("value")
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _point_label(p):
    """A waypoint as it should read to a human: a point name as-is, or a
    literal {x,y[,z]} as 'x,y' / 'x,y,z' rather than a dict repr - the
    (x,y[,z]) retask grammar and REMISSION decomposition both produce
    literals now, not just named points."""
    if isinstance(p, dict):
        x, y, z = p.get("x", 0), p.get("y", 0), p.get("z", 0)
        if z:
            return f"{x:g},{y:g},{z:g}"
        return f"{x:g},{y:g}"
    return str(p)


def _objective_label(obj, armed=None):
    """A one-line label for an objective, for the tree.

    e.g. {'type': 'shuttle', 'between': ['A','B']} -> "shuttle A-B"

    `armed` is the assign/inspect/launch state - see docs/PATCH-07-CHECKS.md
    - and is appended as a tag so the tree row itself answers "is this what
    I meant, and will it move": None (not known - a file that hasn't run
    yet) omits the tag, False shows "assigned, not launched", True shows
    "armed".
    """
    if not isinstance(obj, dict):
        label = "static"
    else:
        kind = obj.get("type", "static")
        if kind == "shuttle":
            bt = obj.get("between")
            if isinstance(bt, (list, tuple)) and len(bt) == 2:
                label = f"shuttle {_point_label(bt[0])}-{_point_label(bt[1])}"
            else:
                label = "shuttle"
        elif kind == "pursuit":
            label = f"pursue {obj.get('target', '?')}"
        elif kind == "patrol":
            label = "patrol"
        elif kind == "orbit":
            label = f"orbit r={obj.get('radius', '?')}"
        elif kind == "script":
            f = str(obj.get("file", "")).split("/")[-1]
            label = f"script {f}" if f else "script"
        else:
            label = kind
    if armed is True:
        return f"{label}  [armed]"
    if armed is False:
        return f"{label}  [assigned, not launched]"
    return label


def _objective_description(agent):
    """Plain-language description of what an agent is doing, for the popup.

    Resolves point names to coordinates where it can, so "shuttle between A and
    B" reads as the actual metres too. Kept to a couple of short sentences: this
    is a glance, not a manual.
    """
    obj = (agent or {}).get("mission") or {"type": "static"}
    kind = obj.get("type", "static")
    aid = agent.get("id", "this agent")
    armed = agent.get("armed")
    armed_note = ""
    if armed is False:
        armed_note = " It is assigned but not launched, so it will not move yet."
    elif armed is True:
        armed_note = " It is armed and acting on this now."

    if kind == "static":
        return f"{aid} holds position. It is not tasked to move."
    if kind == "shuttle":
        bt = obj.get("between")
        if isinstance(bt, (list, tuple)) and len(bt) == 2:
            return (f"{aid} drives back and forth between "
                    f"{_point_label(bt[0])} and {_point_label(bt[1])}, "
                    f"repeating until retasked.{armed_note}")
        return (f"{aid} shuttles between two points, repeating until "
                f"retasked.{armed_note}")
    if kind == "pursuit":
        return (f"{aid} chases {obj.get('target', 'another agent')}, holding a "
                f"standoff of {obj.get('standoff', 1.2)} m behind it.{armed_note}")
    if kind == "patrol":
        return f"{aid} loops a fixed circuit of waypoints until retasked.{armed_note}"
    if kind == "orbit":
        return (f"{aid} circles the arena centre at radius "
                f"{obj.get('radius', 2.0)} m.{armed_note}")
    if kind == "script":
        return (f"{aid} runs the mission script {obj.get('file', '?')}. It "
                f"decides its own target each tick - open the file to see "
                f"how.{armed_note}")
    return f"{aid}: {kind}.{armed_note}"


# ---------------------------------------------------------------------------
# Viewport
# ---------------------------------------------------------------------------

class Viewport(QWidget):
    """The world, in one of three fixed orthographic projections.

    World coordinates are metres everywhere. Exactly one function turns metres
    into pixels, which is what keeps the three views consistent with each other
    and makes the scale bar honest.
    """

    TOP, FRONT, SIDE, ISO = "TOP", "FRONT", "SIDE", "ISO"

    def __init__(self):
        super().__init__()
        self.mode = self.TOP
        self.arena = None
        self.agents = []
        # Scene RF baseline (noise floor dBm, path-loss exponent) for the
        # jammer range ring. Set by the Console on load; defaults match
        # rf_link()'s own until a scene declares otherwise.
        self.scene_rf = (-95.0, 2.8)
        # Band filter: None = all links; a float MHz = only that band's links;
        # "gnss" = dim comms links so the positioning/drift is foregrounded.
        self.band_filter = None
        self.links = []
        self.selected = set()   # highlighted agent ids
        self.scan_overlay = None   # (agent_id, scan) drawn in world coordinates
        self.show_axes = False
        # The key is collapsed by default: once you know it, it is clutter
        # sitting on top of the arena. Click "key" in the corner to open it.
        self.show_key = False
        self._key_hit = None      # QRectF of the clickable header
        self.zoom = 1.0
        self.pan = QPointF(0, 0)
        self._drag = None
        self.setMinimumSize(420, 320)

    def project(self, x, y, z):
        """World metres -> plan coordinates in metres, with v pointing UP.

        The 'v points up' convention matters. to_screen negates v because screen
        y grows downward; if a projection returns a screen-down value here, the
        two negations cancel and the view comes out mirrored. That is exactly
        the bug the ISO view had: altitude ran backwards and the whole thing
        looked like it was being viewed from underneath.
        """
        if self.mode == self.TOP:
            return x, y
        if self.mode == self.FRONT:
            return x, z          # looking along -Y: the long axis of the room
        if self.mode == self.SIDE:
            return y, z          # looking along +X: across the room
        # Isometric, viewed from above: increasing x and y come toward the
        # viewer (down the screen); increasing z goes up it.
        return (x - y) * 0.8660, z - (x + y) * 0.5

    def _extent(self):
        if not self.arena:
            return 10.0
        e = self.arena.get("extent", {})
        return max(_num(e.get("x"), 10), _num(e.get("y"), 10),
                   _num(e.get("z"), 10)) * 1.6

    def scale(self):
        return (min(self.width(), self.height()) / self._extent()) * self.zoom

    def to_screen(self, x, y, z):
        u, v = self.project(x, y, z)
        s = self.scale()
        return QPointF(self.width() / 2 + u * s + self.pan.x(),
                       self.height() / 2 - v * s + self.pan.y())

    # -- interaction --------------------------------------------------------

    def wheelEvent(self, ev):
        self.zoom = max(0.15, min(12.0, self.zoom * 1.0015 ** ev.angleDelta().y()))
        self.update()

    # Set by the Console so a click on the map selects the agent everywhere.
    on_pick = None

    def mousePressEvent(self, ev):
        # The key header is a control, not scenery: a click there toggles it
        # and does NOT start a pan or fall through to agent selection.
        if self._key_hit is not None and self._key_hit.contains(ev.position()):
            self.show_key = not self.show_key
            self._drag = None
            self._press_at = None
            self.update()
            return
        self._drag = ev.position()
        self._press_at = ev.position()

    def mouseMoveEvent(self, ev):
        if self._drag is not None and ev.buttons():
            self.pan += ev.position() - self._drag
            self._drag = ev.position()
            self.update()

    def mouseReleaseEvent(self, ev):
        # A click that did not drag is a selection, not a pan.
        start = getattr(self, "_press_at", None)
        if start is not None and self.on_pick:
            moved = math.hypot(ev.position().x() - start.x(),
                               ev.position().y() - start.y())
            if moved < 4:
                picked = self.agent_at(ev.position())
                if picked:
                    self.on_pick(picked)
        self._drag = None

    def reset_view(self):
        self.zoom, self.pan = 1.0, QPointF(0, 0)
        self.update()

    # -- painting -----------------------------------------------------------

    # A paint failure used to blank the whole canvas with no explanation: Qt
    # swallows the traceback, so a broken draw call looks exactly like "no data".
    # One report, then the message stays on screen instead of an empty window.
    _paint_error = None

    def paintEvent(self, _):
        p = QPainter(self)
        try:
            p.setRenderHint(QPainter.Antialiasing)
            p.fillRect(self.rect(), QColor(C_BG))
            if self.arena:
                self._floor(p)
                self._grid(p)
                self._bounds(p)
            self._scan_fan(p)
            self._range_rings(p)
            self._jammer_affect_lines(p)
            self._belief_ghosts(p)
            self._link_lines(p)
            for a in self.agents:
                self._agent(p, a)
            if self.show_axes:
                self._axes(p)
            self._overlay(p)
        except Exception as exc:
            if Viewport._paint_error is None:
                import traceback
                Viewport._paint_error = f"{type(exc).__name__}: {exc}"
                traceback.print_exc()
            p.setPen(QPen(QColor(C_ERR)))
            p.setFont(QFont("Consolas", 10))
            p.drawText(16, 40, "Viewport draw failed - see console\\last_error.log")
            p.drawText(16, 58, Viewport._paint_error or "")
        finally:
            p.end()

    def _grid(self, p):
        """One line per metre across the arena footprint."""
        e = self.arena.get("extent", {})
        hx, hy = _num(e.get("x"), 8) / 2, _num(e.get("y"), 8) / 2
        p.setPen(QPen(QColor(C_GRID), 1))
        for i in range(-int(hx), int(hx) + 1):
            p.drawLine(self.to_screen(i, -hy, 0), self.to_screen(i, hy, 0))
        for i in range(-int(hy), int(hy) + 1):
            p.drawLine(self.to_screen(-hx, i, 0), self.to_screen(hx, i, 0))

    def _floor(self, p):
        """Fill the arena footprint so the room reads as a volume, not an outline."""
        e = self.arena.get("extent", {})
        hx, hy = _num(e.get("x"), 8) / 2, _num(e.get("y"), 8) / 2
        hz = _num(e.get("z"), 3)
        if self.mode == self.FRONT:
            quad = [(-hx, 0, 0), (hx, 0, 0), (hx, 0, hz), (-hx, 0, hz)]
        elif self.mode == self.SIDE:
            quad = [(0, -hy, 0), (0, hy, 0), (0, hy, hz), (0, -hy, hz)]
        else:
            quad = [(-hx, -hy, 0), (hx, -hy, 0), (hx, hy, 0), (-hx, hy, 0)]
        p.setBrush(QBrush(QColor(37, 42, 46)))
        p.setPen(Qt.NoPen)
        p.drawPolygon(QPolygonF([self.to_screen(*c) for c in quad]))

    def _bounds(self, p):
        """Arena walls. Solid boundaries are drawn as real surfaces; open ones
        as a dashed line, because an open boundary is an absence of wall and
        should not look like one."""
        e = self.arena.get("extent", {})
        b = self.arena.get("boundaries", {})
        hx, hy = _num(e.get("x"), 8) / 2, _num(e.get("y"), 8) / 2
        hz = _num(e.get("z"), 3)

        if self.mode == self.FRONT:
            pts = [(-hx, 0, 0), (hx, 0, 0), (hx, 0, hz), (-hx, 0, hz)]
            faces = ["z_min", "x_max", "z_max", "x_min"]
        elif self.mode == self.SIDE:
            pts = [(0, -hy, 0), (0, hy, 0), (0, hy, hz), (0, -hy, hz)]
            faces = ["z_min", "y_max", "z_max", "y_min"]
        else:
            pts = [(-hx, -hy, 0), (hx, -hy, 0), (hx, hy, 0), (-hx, hy, 0)]
            faces = ["y_min", "x_max", "y_max", "x_min"]

        # In ISO the vertical walls are drawn as translucent planes, so the room
        # looks like a room. Drawn before the agents so they stay readable.
        if self.mode == self.ISO:
            for i in range(4):
                if b.get(faces[i], "solid") != "solid":
                    continue
                a, c = pts[i], pts[(i + 1) % 4]
                quad = QPolygonF([
                    self.to_screen(a[0], a[1], 0), self.to_screen(c[0], c[1], 0),
                    self.to_screen(c[0], c[1], hz), self.to_screen(a[0], a[1], hz)])
                p.setBrush(QBrush(QColor(214, 220, 224, 16)))
                p.setPen(QPen(QColor(C_LINE), 1))
                p.drawPolygon(quad)

        p.setBrush(Qt.NoBrush)
        for i in range(4):
            solid = b.get(faces[i], "solid") == "solid"
            pen = QPen(QColor(C_TEXT if solid else C_DIM), 3 if solid else 1)
            if not solid:
                pen.setStyle(Qt.DashLine)
            p.setPen(pen)
            p.drawLine(self.to_screen(*pts[i]), self.to_screen(*pts[(i + 1) % 4]))

    def _scan_fan(self, p):
        """Draw the selected agent's lidar return in world coordinates.

        This exists to answer 'is the sensor panel telling the truth'. Drawn on
        the map, a correct scan hugs the walls and notches around the other
        agents; a wrong one is obvious instantly. It is the cheapest correctness
        check in the whole application.

        Only in the plan views. The scan is a horizontal planar slice, so
        drawing it in the FRONT elevation would be a straight line pretending
        to be information.
        """
        if not self.scan_overlay or self.mode in (self.FRONT, self.SIDE):
            return
        agent_id, scan = self.scan_overlay
        if not scan or not scan.get("ranges"):
            return
        agent = next((a for a in self.agents if a.get("id") == agent_id), None)
        if not agent:
            return

        pose = agent.get("pose", {})
        x, y, z = _num(pose.get("x")), _num(pose.get("y")), _num(pose.get("z"))
        yaw = _num(pose.get("yaw"))
        a0 = float(scan.get("angle_min", -2.356))
        a1 = float(scan.get("angle_max", 2.356))
        ranges = scan["ranges"]
        n = len(ranges)

        rmax = float(scan.get("range_max", 10.0))

        # Drawn as individual return points, the way RViz and the RoboRacer sim
        # show a LaserScan. A filled wedge looked like a torch beam and made it
        # impossible to see where the beam actually stopped; a scatter of points
        # lands ON the wall, so the wall is unmistakable and a ray that found
        # nothing simply has no point.
        pts = []
        for i, rng in enumerate(ranges):
            if rng is None:
                continue          # no return: nothing to draw, which is the point
            a = yaw + a0 + (a1 - a0) * i / (n - 1)
            pts.append(self.to_screen(x + float(rng) * math.cos(a),
                                      y + float(rng) * math.sin(a), z))

        # A faint hull first, so the swept region still reads at a glance.
        if pts:
            hull = QPolygonF([self.to_screen(x, y, z)] + pts + [self.to_screen(x, y, z)])
            p.setBrush(QBrush(QColor(62, 154, 168, 20)))
            p.setPen(Qt.NoPen)
            p.drawPolygon(hull)

        p.setBrush(QBrush(QColor(120, 214, 226)))
        p.setPen(Qt.NoPen)
        for pt in pts:
            p.drawEllipse(pt, 1.7, 1.7)

        # The sensor's reach, drawn as a ring. Without it "no return" is
        # ambiguous: you cannot tell an empty room from a sensor at its limit.
        # TWO rings, because one number was misleading. The outer is the
        # datasheet maximum against a white target; the inner is what the sensor
        # actually reaches against this room's surfaces. The gap between them is
        # the whole reason a scan can look short in a small room.
        reff = float(scan.get("range_effective", rmax))
        rp = self.to_screen(x, y, z)
        p.setBrush(Qt.NoBrush)
        p.setFont(QFont("Consolas", 8))
        for radius, alpha, style, label in (
                (rmax, 70, Qt.DotLine, f"rated {rmax:.0f} m"),
                (reff, 150, Qt.DashLine, f"reaches {reff:.1f} m")):
            edge = self.to_screen(x + radius, y, z)
            px = abs(edge.x() - rp.x())
            if px < 5:
                continue
            pen = QPen(QColor(62, 154, 168, alpha), 1)
            pen.setStyle(style)
            p.setPen(pen)
            p.drawEllipse(rp, px, px)
            p.setPen(QPen(QColor(62, 154, 168, alpha + 60)))
            p.drawText(rp + QPointF(px - 62, -4), label)

        # The no-return count is on the sensor panel, where the rest of this
        # agent's numbers are. Printing it on the map as well put a second copy
        # over the top of the arena for no benefit.

    def _link_lines(self, p):
        by_id = {a.get("id"): a for a in self.agents}
        bf = self.band_filter
        for link in self.links:
            # Band filter: a comms band shows only its own links; GNSS hides
            # the comms lines entirely (GNSS has no links - the drift ghosts
            # are the story on that band).
            if bf == "gnss":
                continue
            if isinstance(bf, (int, float)) and \
                    abs(_num(link.get("band_mhz"), 2400.0) - bf) > 0.5:
                continue
            a, b = by_id.get(link.get("a")), by_id.get(link.get("b"))
            if not a or not b:
                continue
            # No link at all beyond range: an absent line says "cannot hear
            # each other" far more clearly than a line drawn in a warning colour.
            if float(link.get("quality", 1.0)) <= 0.0:
                continue
            state = link.get("state", "up")
            # CARRIED vs SPARE. `active` is set by apply_routing: it is whether
            # THIS ROUTING uses this pair, not whether the pair could hear each
            # other. Drawing all six lines whatever routing was picked made a
            # star look exactly like a mesh - the map was showing geometry, not
            # the network, which is the same mistake command_authority() was
            # making before it was fixed to walk only carried links.
            #
            # Spare links are drawn, faintly, rather than hidden. The gap
            # between what a topology COULD use and what it DOES use is the
            # whole star-vs-mesh finding: hiding the spares would hide the
            # capacity a star is choosing not to spend. Full weight = the
            # network is using it; hairline = it exists and is going unused.
            carried = bool(link.get("active", True))
            if not carried:
                spare = QColor(C_DIM)
                spare.setAlpha(70)
                pen = QPen(spare, 0.8)
                pen.setStyle(Qt.DotLine)
                p.setPen(pen)
                pa, pb = a.get("pose", {}), b.get("pose", {})
                p.drawLine(
                    self.to_screen(_num(pa.get("x")), _num(pa.get("y")),
                                   _num(pa.get("z"))),
                    self.to_screen(_num(pb.get("x")), _num(pb.get("y")),
                                   _num(pb.get("z"))))
                continue
            # A DOWN link is drawn ORANGE (not the network colour) so a broken
            # link is unmistakable at a glance; up/degraded stay the network
            # colour, solid vs dashed.
            colour = (QColor("#E08A3C") if state == "down"
                      else QColor(network_colour(link.get("network"))))
            pen = QPen(colour, 1.6 if state == "down" else 1.4)
            pen.setStyle({"up": Qt.SolidLine,
                          "degraded": Qt.DashLine}.get(state, Qt.DashDotLine))
            p.setPen(pen)
            pa, pb = a.get("pose", {}), b.get("pose", {})
            p.drawLine(self.to_screen(_num(pa.get("x")), _num(pa.get("y")), _num(pa.get("z"))),
                       self.to_screen(_num(pb.get("x")), _num(pb.get("y")), _num(pb.get("z"))))

    def _agent(self, p, agent):
        pose, dims = agent.get("pose", {}), agent.get("dimensions", {})
        x, y, z = _num(pose.get("x")), _num(pose.get("y")), _num(pose.get("z"))
        yaw = _num(pose.get("yaw"))
        L, W, H = _num(dims.get("length"), .4), _num(dims.get("width"), .4), _num(dims.get("height"), .2)

        # TWO INDEPENDENT CHANNELS, because there are two independent failures
        # and one mark cannot carry both:
        #
        #   FILL    = command state. Is anyone telling this thing what to do?
        #             Network colour when its decider is reachable, ORANGE when
        #             it is not - whatever the agent then DOES about it (hold or
        #             act on intent) is the label underneath, not the fill.
        #   GHOST   = position knowledge. Does it know where it is? Drawn
        #             separately in _belief_ghosts as the dotted offset.
        #
        # The OUTLINE always stays the agent's own colour, so a cut-off blue car
        # still reads as blue force at a glance - it is in trouble, it has not
        # changed sides.
        #
        # The four combinations are exactly the experiment's four conditions:
        #   blue  + no ghost  nothing jammed
        #   blue  + ghost     GNSS jammed - commanded, obeying, and wrong. The
        #                     dangerous one: no alarm anywhere says so.
        #   orange + no ghost comms jammed - cut off, knows exactly where it is
        #   orange + ghost    both - cut off AND lost
        base = QColor(agent.get("colour") or "#2E6FB0")
        cut_off = not (agent.get("authority") or {}).get("reachable", True)
        colour = QColor("#E08A3C") if cut_off else base
        selected = agent.get("id") in self.selected
        p.setBrush(QBrush(colour))
        # A cut-off agent gets a thicker outline so the identity ring stays
        # legible against the orange rather than being lost in it.
        p.setPen(QPen(QColor(C_TEXT) if selected else base.lighter(150),
                      2 if (selected or cut_off) else 1))

        cy, sy = math.cos(yaw), math.sin(yaw)

        def corner(dx, dy, dz):
            return self.to_screen(x + dx * cy - dy * sy,
                                  y + dx * sy + dy * cy, z + dz)

        if self.mode == self.FRONT:
            p.drawRect(QRectF(self.to_screen(x - L / 2, y, z),
                              self.to_screen(x + L / 2, y, z + H)).normalized())
        elif self.mode == self.SIDE:
            p.drawRect(QRectF(self.to_screen(x, y - W / 2, z),
                              self.to_screen(x, y + W / 2, z + H)).normalized())
        elif self.mode == self.TOP:
            p.drawPolygon(QPolygonF([corner(L / 2, W / 2, 0), corner(L / 2, -W / 2, 0),
                                     corner(-L / 2, -W / 2, 0), corner(-L / 2, W / 2, 0)]))
        else:
            # ISO: a real box. Two side faces shaded darker than the top so the
            # form reads as solid rather than as a flat card lying on the floor.
            fl, fr = (L / 2, W / 2), (L / 2, -W / 2)
            bl, br = (-L / 2, W / 2), (-L / 2, -W / 2)
            top = QPolygonF([corner(*fl, H), corner(*fr, H),
                             corner(*br, H), corner(*bl, H)])
            side_a = QPolygonF([corner(*fl, 0), corner(*fr, 0),
                                corner(*fr, H), corner(*fl, H)])
            side_b = QPolygonF([corner(*fr, 0), corner(*br, 0),
                                corner(*br, H), corner(*fr, H)])
            p.setBrush(QBrush(colour.darker(150)))
            p.drawPolygon(side_a)
            p.setBrush(QBrush(colour.darker(125)))
            p.drawPolygon(side_b)
            p.setBrush(QBrush(colour))
            p.drawPolygon(top)

        # Sensor mounts, at their real offsets. Shown only for the selected
        # agent and labelled: two anonymous dots on every car told you nothing.
        if selected:
            p.setFont(QFont("Consolas", 7))
            for sen in agent.get("sensors") or []:
                o = sen.get("offset") or {}
                pt = self.to_screen(x + _num(o.get("x")) * cy - _num(o.get("y")) * sy,
                                    y + _num(o.get("x")) * sy + _num(o.get("y")) * cy,
                                    z + _num(o.get("z")))
                is_lidar = sen.get("type") == "ust10lx"
                p.setBrush(QBrush(QColor(C_ACCENT if is_lidar else C_TEXT)))
                p.setPen(Qt.NoPen)
                p.drawEllipse(pt, 2.6 if is_lidar else 1.8, 2.6 if is_lidar else 1.8)
                p.setPen(QPen(QColor(C_DIM)))
                p.drawText(pt + QPointF(5, 3), str(sen.get("id", "")))
        else:
            p.setBrush(QBrush(QColor(C_TEXT)))
            p.setPen(Qt.NoPen)
            for sen in agent.get("sensors") or []:
                o = sen.get("offset") or {}
                p.drawEllipse(self.to_screen(x + _num(o.get("x")), y + _num(o.get("y")),
                                             z + _num(o.get("z"))), 1.6, 1.6)

        p.setPen(QPen(QColor(C_TEXT if selected else C_DIM)))
        p.setFont(QFont("Consolas", 8))
        p.drawText(self.to_screen(x, y, z) + QPointF(9, -7), str(agent.get("id", "")))
        # WHAT IT IS DOING ABOUT IT. The fill says nobody is commanding this
        # vehicle; this says which doctrine it is following in response, which
        # is the variable the whole experiment turns on:
        #
        #   held    on_link_loss: hold - frozen until the link returns. Without
        #           this label a stopped car reads as "arrived", not "cut off".
        #   intent  on_link_loss: intent - still executing the objective it was
        #           already given, on the picture it already has (AJP-3 3.8,
        #           3.11). This was INVISIBLE: a car acting on delegated intent
        #           looked exactly like a normally commanded one, which is the
        #           single most important state the doctrine experiment is
        #           about.
        # A JAMMER SAYS WHAT IT IS DOING, always - not only when selected.
        # Its power is the single most important number on the map during an
        # experiment replay, and "is this actually jamming?" should never need
        # a click to answer. It is the question that hid a wrong replay.
        jam = agent.get("jammer")
        if jam:
            on = jam.get("on")
            p.setPen(QPen(QColor(NETWORK_COLOURS["red"] if on else C_DIM)))
            p.setFont(QFont("Consolas", 7))
            p.drawText(self.to_screen(x, y, z) + QPointF(9, 6),
                       f"{_num(jam.get('tx_dbm')):.0f} dBm @ "
                       f"{_num(jam.get('band_mhz')):.0f} MHz"
                       + ("" if on else "  (silent)"))
        note = ("⊘ held (no commander)" if agent.get("link_loss_hold")
                else "→ intent (no commander)" if agent.get("on_intent")
                else None)
        if note:
            p.setPen(QPen(QColor("#E08A3C")))
            p.setFont(QFont("Consolas", 7))
            p.drawText(self.to_screen(x, y, z) + QPointF(9, 6), note)

    def _jammer_range(self, agent):
        """Nominal influence radius (m) of a jammer agent, from whichever
        jammer-power shape is present (live frame: tx_dbm/band_mhz; pre-run
        fleet: tx_power/band quantities)."""
        if _jammer_range_m is None:
            return None
        j = agent.get("jammer")
        if not j:
            return None
        def val(*keys, default=None):
            for k in keys:
                v = j.get(k)
                if isinstance(v, dict):
                    v = v.get("value")
                if v is not None:
                    try:
                        return float(v)
                    except (TypeError, ValueError):
                        pass
            return default
        tx = val("tx_dbm", "tx_power", default=20.0)
        band = val("band_mhz", "band", default=2400.0)
        noise, plexp = self.scene_rf
        return _jammer_range_m(tx, band, noise, plexp)

    def _jammer_affect_lines(self, p):
        """Select a jammer and see WHO it is currently hurting, on ITS band.

        A red line runs from the jammer to every agent it is actually degrading
        this tick, labelled with how far that agent's noise floor has been
        raised (dB) - or "GNSS denied" for a positioning jammer. Thickness
        follows severity. This answers "what is this emitter doing, right now,
        to whom" without reading a table."""
        for j in self.agents:
            if j.get("id") not in self.selected:
                continue
            jam = j.get("jammer")
            if not jam or not jam.get("on"):
                continue
            band = _num(jam.get("band_mhz"), 2400.0)
            is_gnss = abs(band - 1575.42) < 5.0
            # Respect the band strip: don't draw a comms jammer's effects
            # while looking at the GNSS band, or vice versa.
            bf = self.band_filter
            if bf == "gnss" and not is_gnss:
                continue
            if isinstance(bf, (int, float)) and abs(band - bf) > 0.5:
                continue
            jp = j.get("pose", {})
            jpt = self.to_screen(_num(jp.get("x")), _num(jp.get("y")),
                                 _num(jp.get("z")))
            for a in self.agents:
                if a.get("id") == j.get("id"):
                    continue
                if is_gnss:
                    if not a.get("gnss_denied"):
                        continue
                    label, sev = "GNSS denied", 12.0
                else:
                    rf = a.get("rf") or {}
                    if not rf.get("jammed"):
                        continue
                    sev = _num(rf.get("noise_floor_dbm")) - \
                        _num(rf.get("baseline_dbm"))
                    label = f"+{sev:.0f} dB"
                ap = a.get("pose", {})
                apt = self.to_screen(_num(ap.get("x")), _num(ap.get("y")),
                                     _num(ap.get("z")))
                width = max(1.0, min(3.4, 0.8 + sev / 15.0))
                col = QColor(NETWORK_COLOURS["red"])
                p.setPen(QPen(col, width, Qt.SolidLine))
                p.drawLine(jpt, apt)
                mid = QPointF((jpt.x() + apt.x()) / 2.0,
                              (jpt.y() + apt.y()) / 2.0)
                p.setFont(QFont("Consolas", 7))
                p.setPen(QPen(col))
                p.drawText(mid + QPointF(4, -3), label)

    def _belief_ghosts(self, p):
        """For any agent whose estimate has drifted from truth (GNSS denied),
        draw a faint ghost where it THINKS it is, joined to its true position.
        TOP view only - it is a 2-D position error. This is the cost of GNSS
        jamming, drawn: the gap between the solid car and its ghost."""
        if self.mode != self.TOP:
            return
        gnss_view = (self.band_filter == "gnss")
        for a in self.agents:
            err = _num(a.get("position_error_m"))
            bel = a.get("believed") or {}
            if "x" not in bel:
                continue
            # Normally only a meaningful drift is worth drawing. On the GNSS
            # band the drift IS the subject, so show every agent's, however
            # small, with the distance spelled out.
            if not gnss_view and err <= 0.15:
                continue
            pose = a.get("pose", {})
            tp = self.to_screen(_num(pose.get("x")), _num(pose.get("y")), 0)
            bp = self.to_screen(_num(bel.get("x")), _num(bel.get("y")), 0)
            # THREE RULES, NO EXCEPTIONS - so a glance decodes without a key:
            #   SOLID  = truth (the vehicle)      HOLLOW = belief (the ghost)
            #   COLOUR = command state, on both. Network colour when the agent's
            #            decider is reachable, orange when it is not.
            # The ghost used to be orange ALWAYS. Once orange started meaning
            # "cut off" on the fill, a blue commanded car with an orange ghost
            # read as a contradiction. It is the same fact drawn twice, so it
            # must be the same colour twice: a commanded-but-lost car is a solid
            # blue car with a hollow BLUE ghost - which is the GNSS-only
            # condition, and the one with no alarm anywhere.
            col = (QColor("#E08A3C")
                   if not (a.get("authority") or {}).get("reachable", True)
                   else QColor(a.get("colour") or "#2E6FB0").lighter(130))
            pen = QPen(col, 1.2, Qt.DotLine)
            p.setPen(pen)
            p.drawLine(tp, bp)
            # THE GHOST IS THE SAME VEHICLE, HOLLOW. It was a small circle,
            # which said "a point over there" - but what it means is "this
            # vehicle, as it believes itself to be", including its heading.
            # Drawing the actual footprint at the believed pose makes the
            # error read as a displaced CAR rather than an abstract marker,
            # and it matches the hollow swatch in the key.
            dims = a.get("dimensions") or {}
            L = _num(dims.get("length"), 0.4)
            W = _num(dims.get("width"), 0.4)
            byaw = _num((bel.get("yaw") if "yaw" in bel
                         else pose.get("yaw")))
            cy_, sy_ = math.cos(byaw), math.sin(byaw)
            bx, by = _num(bel.get("x")), _num(bel.get("y"))

            def gcorner(dx, dy):
                return self.to_screen(bx + dx * cy_ - dy * sy_,
                                      by + dx * sy_ + dy * cy_, 0)

            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(col, 1.4))
            p.drawPolygon(QPolygonF([gcorner(L / 2, W / 2),
                                     gcorner(L / 2, -W / 2),
                                     gcorner(-L / 2, -W / 2),
                                     gcorner(-L / 2, W / 2)]))
            p.setFont(QFont("Consolas", 7))
            p.setPen(QPen(col))
            p.drawText(bp + QPointF(10, -6),
                       (f"{a.get('id')}: believes it is {err:.2f} m from here"
                        if gnss_view else f"thinks: {err:.1f} m off"))

    def _comms_ring(self, p, agent, s):
        """The distance at which this agent's own transmissions fall to the
        noise floor - a NOMINAL omni contour, like the jammer's, and honest
        about being one: real coverage is ragged and this ignores the far
        end's power entirely. Its use is comparative, against the jammer
        rings drawn beside it."""
        if _jammer_range_m is None:
            return
        node = (agent.get("radio") or {}).get("tx_power")
        tx = (_num(node.get("value")) if isinstance(node, dict)
              else (_num(node) if isinstance(node, (int, float)) else None))
        if tx is None:
            return
        noise, plexp = self.scene_rf
        try:
            r = _jammer_range_m(tx, 2400.0, noise, plexp)
        except Exception:                              # noqa: BLE001
            return
        if not r or r <= 0:
            return
        pose = agent.get("pose", {})
        c = self.to_screen(_num(pose.get("x")), _num(pose.get("y")),
                           _num(pose.get("z")))
        col = QColor(agent.get("colour") or "#2E6FB0").lighter(120)
        pen = QPen(col, 1.1, Qt.DashDotLine)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(c, r * s, r * s)
        p.setFont(QFont("Consolas", 7))
        p.setPen(QPen(col))
        p.drawText(c + QPointF(r * s * 0.7, r * s * 0.7),
                   f"{agent.get('id')} reach ~{r:.0f} m @ {tx:.0f} dBm")

    def _range_rings(self, p):
        """Ring the influence area of a SELECTED jammer (TOP view only - a
        2-D contour only reads on the plan). Two rings: the J/N=0 influence
        boundary (dashed) and the J/N=20 dB denial core (solid). A NOMINAL
        omni contour - real jammed areas are ragged (sensors 2024)."""
        if self.mode != self.TOP:
            return
        s = self.scale()
        for a in self.agents:
            if a.get("id") not in self.selected:
                continue
            if not a.get("jammer"):
                # NOT A JAMMER: draw its own COMMS REACH instead. Every agent
                # with a radio has one, and the ground station's is the one
                # that matters most - it is the difference between a star that
                # covers the fleet and a star that does not. It was invisible,
                # which meant "why did that link drop" had no answer you could
                # see. Same computation as a jammer's contour, driven by this
                # agent's own declared transmit power, so the two are directly
                # comparable on the map: where the rings overlap is where the
                # jammer is winning.
                self._comms_ring(p, a, s)
                continue
            r0 = self._jammer_range(a)
            if not r0 or r0 <= 0:
                continue
            pose = a.get("pose", {})
            c = self.to_screen(_num(pose.get("x")), _num(pose.get("y")),
                               _num(pose.get("z")))
            col = QColor(NETWORK_COLOURS["red"])
            # denial core (J/N=20 dB): a tenth the radius per 20 dB / (10*n)...
            # recompute directly for honesty rather than scaling.
            core = self._core_range(a)
            if core and core > 0:
                p.setPen(QPen(col, 1.4))
                p.setBrush(QBrush(QColor(col.red(), col.green(), col.blue(), 40)))
                p.drawEllipse(c, core * s, core * s)
            pen = QPen(col, 1.2, Qt.DashLine)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(c, r0 * s, r0 * s)
            p.setFont(QFont("Consolas", 7))
            p.setPen(QPen(col))
            p.drawText(c + QPointF(r0 * s * 0.7, -r0 * s * 0.7),
                       f"influence ~{r0:.0f} m (nominal)")

    def _core_range(self, agent):
        if _jammer_range_m is None or not agent.get("jammer"):
            return None
        j = agent.get("jammer")
        def val(*keys, default=None):
            for k in keys:
                v = j.get(k)
                if isinstance(v, dict):
                    v = v.get("value")
                if v is not None:
                    try:
                        return float(v)
                    except (TypeError, ValueError):
                        pass
            return default
        tx = val("tx_dbm", "tx_power", default=20.0)
        band = val("band_mhz", "band", default=2400.0)
        noise, plexp = self.scene_rf
        return _jammer_range_m(tx, band, noise, plexp, jn_db=20.0)

    def agent_at(self, pos):
        """Which agent is under this screen point? Nearest within a tolerance."""
        best, best_d = None, 22.0
        for a in self.agents:
            pose = a.get("pose", {})
            pt = self.to_screen(_num(pose.get("x")), _num(pose.get("y")),
                                _num(pose.get("z")))
            d = math.hypot(pt.x() - pos.x(), pt.y() - pos.y())
            if d < best_d:
                best, best_d = a.get("id"), d
        return best

    def _axes(self, p):
        """Measured axes with ticks, so distances can be read off the view.

        Each projection gets the two world axes it actually shows: no point
        drawing a Y axis on a front elevation, where Y is the direction you are
        looking along. In ISO all three are drawn from the floor corner, which is
        the only place they read unambiguously.
        """
        if not self.arena:
            return
        e = self.arena.get("extent", {})
        hx, hy = _num(e.get("x"), 8) / 2, _num(e.get("y"), 8) / 2
        hz = _num(e.get("z"), 3)
        p.setFont(QFont("Consolas", 8))

        centre = self.to_screen(0, 0, 0)

        def axis(a, b, label, values):
            """Draw one axis from world point a to b, ticked at `values`.

            Ticks and labels sit on whichever side faces AWAY from the middle of
            the arena, decided per axis. A fixed normal points inward for half of
            them, which is what put numbers on top of the floor in ISO.
            """
            pa, pb = self.to_screen(*a), self.to_screen(*b)
            p.setPen(QPen(QColor(C_TEXT), 1.2))
            p.drawLine(pa, pb)
            dx, dy = pb.x() - pa.x(), pb.y() - pa.y()
            length = math.hypot(dx, dy) or 1.0
            nx, ny = -dy / length * 6, dx / length * 6
            mid_x, mid_y = (pa.x() + pb.x()) / 2, (pa.y() + pb.y()) / 2
            if nx * (mid_x - centre.x()) + ny * (mid_y - centre.y()) < 0:
                nx, ny = -nx, -ny
            for v, point in values:
                pt = self.to_screen(*point)
                p.setPen(QPen(QColor(C_TEXT), 1))
                p.drawLine(QPointF(pt.x() - nx, pt.y() - ny),
                           QPointF(pt.x() + nx, pt.y() + ny))
                if int(v) % 2 == 0:
                    p.setPen(QPen(QColor(C_DIM)))
                    p.drawText(QPointF(pt.x() + nx * 2.4 - 5,
                                       pt.y() + ny * 2.4 + 4), f"{v:g}")
            p.setPen(QPen(QColor(C_ACCENT)))
            p.drawText(QPointF(pb.x() + nx * 1.6 + (dx / length) * 14,
                               pb.y() + ny * 1.6 + (dy / length) * 14), label)

        rng = lambda h: [v for v in range(-int(h), int(h) + 1)]

        if self.mode == self.TOP:
            axis((-hx, -hy, 0), (hx, -hy, 0), "X",
                 [(v, (v, -hy, 0)) for v in rng(hx)])
            axis((-hx, -hy, 0), (-hx, hy, 0), "Y",
                 [(v, (-hx, v, 0)) for v in rng(hy)])
        elif self.mode == self.FRONT:
            axis((-hx, 0, 0), (hx, 0, 0), "X",
                 [(v, (v, 0, 0)) for v in rng(hx)])
            axis((-hx, 0, 0), (-hx, 0, hz), "Z",
                 [(v, (-hx, 0, v)) for v in range(0, int(hz) + 1)])
        elif self.mode == self.SIDE:
            axis((0, -hy, 0), (0, hy, 0), "Y",
                 [(v, (0, v, 0)) for v in rng(hy)])
            axis((0, -hy, 0), (0, -hy, hz), "Z",
                 [(v, (0, -hy, v)) for v in range(0, int(hz) + 1)])
        else:
            axis((-hx, -hy, 0), (hx, -hy, 0), "X",
                 [(v, (v, -hy, 0)) for v in rng(hx)])
            axis((-hx, -hy, 0), (-hx, hy, 0), "Y",
                 [(v, (-hx, v, 0)) for v in rng(hy)])
            axis((-hx, -hy, 0), (-hx, -hy, hz), "Z",
                 [(v, (-hx, -hy, v)) for v in range(0, int(hz) + 1)])

    def _overlay(self, p):
        p.setPen(QPen(QColor(C_DIM)))
        p.setFont(QFont("Consolas", 9))
        p.drawText(10, 18, f"{self.mode}   1 m grid   zoom {self.zoom:.2f}x")

        # THE KEY. Collapsed by default - it is a reference, not a readout, and
        # once you know it, it sits on top of the arena. The header is a hit
        # target handled in mousePressEvent.
        nets = {l.get("network") for l in self.links} or {"blue"}
        base = network_colour(sorted(str(n) for n in nets)[0])
        head = ("\u25be  key" if self.show_key else "\u25b8  key")
        p.setPen(QPen(QColor(C_ACCENT)))
        p.setFont(QFont("Consolas", 9))
        p.drawText(10, 36, head)
        self._key_hit = QRectF(6, 24, 64, 16)
        if not self.show_key:
            s_ = self.scale()
            if s_ > 2:
                p.setPen(QPen(QColor(C_DIM)))
                p.drawLine(10, self.height() - 16, 10 + int(s_), self.height() - 16)
                p.drawText(14 + int(s_), self.height() - 12, "1 m")
            return

        # A panel behind it, so the key never has to compete with the arena
        # grid for legibility. Sized to its content rather than guessed.
        rows_link = (("up", Qt.SolidLine, QColor(base), 1.6),
                     ("degraded", Qt.DashLine, QColor(base), 1.6),
                     ("down", Qt.DashDotLine, QColor("#E08A3C"), 1.6))
        spare = QColor(C_DIM)
        spare.setAlpha(90)
        # No panel and no border: the key sits directly on the arena. A box
        # around it read as another object in the world, which is exactly what
        # a key must not look like.
        LH, x_sw, x_tx = 15, 12, 46
        y = 52
        p.setBrush(Qt.NoBrush)

        def section(title):
            nonlocal y
            p.setPen(QPen(QColor(C_DIM).darker(115)))
            p.setFont(QFont("Consolas", 7))
            p.drawText(x_sw, y, title)
            y += LH - 2

        def row(label, draw):
            nonlocal y
            draw(y - 4)
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(QColor(C_DIM)))
            p.setFont(QFont("Consolas", 8))
            p.drawText(x_tx, y, label)
            y += LH

        section("LINKS")
        for label, style, col, w in rows_link:
            def d(yy, style=style, col=col, w=w):
                pen = QPen(col, w)
                pen.setStyle(style)
                p.setPen(pen)
                p.drawLine(x_sw, yy, x_sw + 26, yy)
            row(label, d)

        def d_spare(yy):
            pen = QPen(spare, 0.9)
            pen.setStyle(Qt.DotLine)
            p.setPen(pen)
            p.drawLine(x_sw, yy, x_sw + 26, yy)
        row("spare (routing does not carry it)", d_spare)
        row("no line = out of range", lambda yy: None)

        # VEHICLES. Two independent channels, and every combination of them is
        # a real condition, so all four swatches are drawn rather than leaving
        # the reader to infer that hollow-orange exists.
        section("VEHICLES   fill = command   hollow = its own estimate")

        def swatch(fill, solid):
            def d(yy):
                p.setBrush(QBrush(fill) if solid else Qt.NoBrush)
                p.setPen(QPen(QColor(base).lighter(150) if solid
                              else QColor(fill).lighter(130), 1))
                p.drawRect(QRectF(x_sw, yy - 5, 24, 10))
            return d

        blue_f, orange_f = QColor(base), QColor("#E08A3C")
        row("commanded", swatch(blue_f, True))
        row("commanded, but lost (GNSS)", swatch(blue_f, False))
        row("cut off - no reachable commander", swatch(orange_f, True))
        row("cut off AND lost", swatch(orange_f, False))

        section("DOCTRINE, once cut off")
        p.setPen(QPen(QColor("#E08A3C")))
        p.setFont(QFont("Consolas", 8))
        p.drawText(x_sw, y, "\u2298 held")
        p.setPen(QPen(QColor(C_DIM)))
        p.drawText(x_tx + 30, y, "frozen until the link returns")
        y += LH
        p.setPen(QPen(QColor("#E08A3C")))
        p.drawText(x_sw, y, "\u2192 intent")
        p.setPen(QPen(QColor(C_DIM)))
        p.drawText(x_tx + 30, y, "still executing its last order")

        s_ = self.scale()
        if s_ > 2:
            p.setPen(QPen(QColor(C_DIM)))
            p.drawLine(10, self.height() - 16, 10 + int(s_), self.height() - 16)
            p.drawText(14 + int(s_), self.height() - 12, "1 m")


# ---------------------------------------------------------------------------
# Sensor view
# ---------------------------------------------------------------------------

class SensorView(QWidget):
    """What the selected agent's lidar sees, as a polar plot.

    Drawn in the sensor's own frame — straight up is dead ahead — because that
    is how you read a scan when you are debugging a controller. Rotating it into
    the world frame makes it prettier and much less useful.
    """

    def __init__(self):
        super().__init__()
        self.scan = None
        self.agent_id = None
        self.state = None
        self.setMinimumHeight(240)

    def set_scan(self, agent_id, scan, state=None):
        self.agent_id, self.scan, self.state = agent_id, scan, state
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(C_BG))
        p.setPen(QPen(QColor(C_DIM)))
        p.setFont(QFont("Consolas", 9))

        # Position and speed first. Every moving agent has them whether or not
        # it carries a lidar, so this block is the generic part of the panel.
        top = 0
        if self.state:
            pose = self.state.get("pose") or {}
            p.setPen(QPen(QColor(C_TEXT)))
            p.drawText(10, 16, str(self.agent_id or ""))
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(10, 32, f"x {_num(pose.get('x')):7.2f}  "
                               f"y {_num(pose.get('y')):7.2f}  "
                               f"z {_num(pose.get('z')):7.2f}  m")
            p.drawText(10, 46, f"yaw {math.degrees(_num(pose.get('yaw'))):6.1f} deg   "
                               f"speed {_num(pose.get('speed')):5.2f} m/s")
            obj = self.state.get("objective")
            if isinstance(obj, dict):
                # The full objective - "pursue car3", not a bare "pursuit" -
                # tagged with armed state, since assigned-but-not-launched is
                # the whole point of the inspect step.
                p.drawText(10, 60, "objective  " +
                          _objective_label(obj, armed=self.state.get("armed")))
            elif self.state.get("mission"):
                p.drawText(10, 60, f"objective  {self.state.get('mission')}")
            # POSITION KNOWLEDGE - the belief-vs-truth story, per agent.
            # A GNSS-denied car is dead-reckoning; show how wrong its estimate
            # has become. This is what makes jamming legible on one agent.
            err = _num(self.state.get("position_error_m"))
            denied = self.state.get("gnss_denied")
            y = 74
            if denied:
                p.setPen(QPen(QColor("#E08A3C")))
                p.drawText(10, y, f"GNSS DENIED - dead reckoning")
                p.drawText(10, y + 14, f"est. position error  {err:5.2f} m")
            else:
                p.setPen(QPen(QColor(C_DIM)))
                p.drawText(10, y, "GNSS ok" +
                           (f"  (est. error {err:.2f} m)" if err > 0.05 else ""))
                y -= 0
            # The sensors this agent ACTUALLY carries - so the panel is the
            # agent's, not a fixed lidar view.
            sensors = self.state.get("sensors") or []
            names = ", ".join(sen.get("type", "?") for sen in sensors) or "none"
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(10, y + 28, f"sensors: {names}")
            top = y + 34
            p.setPen(QPen(QColor(C_LINE)))
            p.drawLine(8, top, self.width() - 8, top)

        if not self.scan or not self.scan.get("ranges"):
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(12, top + 20, "No lidar on this agent - nothing to plot."
                                     if self.agent_id else "Select an agent.")
            return

        ranges = self.scan["ranges"]
        rmax = float(self.scan.get("range_max", 10.0))
        a0 = float(self.scan.get("angle_min", -2.356))
        a1 = float(self.scan.get("angle_max", 2.356))

        cx = self.width() / 2
        cy = top + (self.height() - top) * 0.62
        radius = min(self.width() / 2, (self.height() - top) * 0.62) - 18
        if radius < 20:
            return
        scale = radius / rmax

        # Range rings, every 2 m.
        p.setPen(QPen(QColor(C_GRID)))
        step = 2.0
        r = step
        while r <= rmax:
            p.drawEllipse(QPointF(cx, cy), r * scale, r * scale)
            p.drawText(QPointF(cx + 3, cy - r * scale - 2), f"{r:.0f}m")
            r += step

        # The scan itself. Straight up on screen is straight ahead.
        n = len(ranges)
        poly = QPolygonF([QPointF(cx, cy)])
        for i, rng in enumerate(ranges):
            a = a0 + (a1 - a0) * i / (n - 1)
            d = (rmax if rng is None else min(float(rng), rmax)) * scale
            poly.append(QPointF(cx + d * math.sin(a), cy - d * math.cos(a)))
        poly.append(QPointF(cx, cy))
        p.setBrush(QBrush(QColor(62, 154, 168, 34)))
        p.setPen(Qt.NoPen)
        p.drawPolygon(poly)

        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(QColor(C_ACCENT), 1.4))
        run = QPolygonF()
        for i, rng in enumerate(ranges):
            if rng is None:
                if run.size() > 1:
                    p.drawPolyline(run)
                run = QPolygonF()
                continue
            a = a0 + (a1 - a0) * i / (n - 1)
            d = min(float(rng), rmax) * scale
            run.append(QPointF(cx + d * math.sin(a), cy - d * math.cos(a)))
        if run.size() > 1:
            p.drawPolyline(run)

        hits = [r for r in ranges if r is not None]
        misses = n - len(hits)
        p.setPen(QPen(QColor(C_TEXT)))
        p.drawText(10, top + 16, f"{n} rays  ·  "
                           f"min {min(hits):.2f} m" if hits else f"{self.agent_id}  ·  no returns")
        if misses:
            p.setPen(QPen(QColor(C_WARN)))
            p.drawText(10, top + 30, f"{misses} beyond {rmax:.0f} m - no return")
        p.setPen(QPen(QColor(C_DIM)))
        p.drawText(10, self.height() - 8,
                   f"fan {math.degrees(a1 - a0):.0f}°   up = forward")


# ---------------------------------------------------------------------------
# Results
#
# A DELIBERATELY SMALL plotter. PlotJuggler already exists, is open source, and
# is better at this than anything worth building here — so the job is not to
# replace it. The job is to answer the questions you ask constantly ("did PDR
# drop when the drone crossed the room?") without leaving the app, and to
# record runs in a format PlotJuggler and pandas can both open when a question
# needs more than this.
# ---------------------------------------------------------------------------

def discover_series(frame):
    """Every numeric time series a telemetry frame offers, as dotted paths."""
    out = []
    for a in frame.get("agents", []):
        aid = a.get("id")
        if a.get("platform") != "ground_station":
            # Turn rate is derived; speed is NOT - the agent reports its own,
            # the way an odometry topic does. Deriving it as well would put two
            # subtly different "speed" series in the tree.
            out.append(f"agents.{aid}.yaw_rate")
        for group in ("pose",):
            for k, v in (a.get(group) or {}).items():
                if isinstance(v, (int, float)):
                    out.append(f"agents.{aid}.{group}.{k}")
        scan = a.get("scan")
        if scan and scan.get("ranges"):
            # The useful scalar from a scan is the nearest return: it is the
            # collision-risk number and it plots meaningfully over time.
            out.append(f"agents.{aid}.scan.min_range")
        # THE EXPERIMENT METRICS. Pose alone cannot answer "did jamming stop
        # this fleet doing its job, and did it know where it was" - which is
        # the whole question. Exported as 0/1 where the source is a flag, so
        # a column means the same thing whether you average it or plot it.
        if a.get("platform") != "ground_station" and not a.get("jammer"):
            out += [f"agents.{aid}.commanded",        # decider reachable
                    f"agents.{aid}.held",             # frozen by doctrine
                    f"agents.{aid}.on_intent",        # acting on delegated intent
                    f"agents.{aid}.gnss_denied",
                    f"agents.{aid}.position_error_m",  # belief vs truth
                    f"agents.{aid}.noise_floor_dbm"]
        # ANY numeric field an agent carries. Written generically rather than
        # as a list, because a SWEPT result arrives as synthetic agents whose
        # fields are metrics (penetration_m, adaptability, ...) rather than
        # poses - and hardcoding the names would mean a new metric could never
        # be plotted without editing this function.
        for k, v in a.items():
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                path = f"agents.{aid}.{k}"
                if path not in out:
                    out.append(path)
    for l in frame.get("links", []):
        tag = f"{l.get('a')}-{l.get('b')}"
        for k, v in l.items():
            if isinstance(v, (int, float)):
                out.append(f"links.{tag}.{k}")
    return out


# The curated set. Grounded in the internship plan's stated aims: test
# communication reliability and data exchange, and characterise the effect of an
# attack on a vehicle as "sudden braking, speed up, slow down, random turn".
# Link reliability and vehicle speed/turn-rate ARE those measurements. Anything
# outside this set is available in Custom, but it is not what the study is for.
KEY_METRICS = [
    ("Link reliability", ["pdr", "latency_ms", "quality", "sinr_db"]),
    ("Command", ["commanded", "held", "on_intent"]),
    ("Position knowledge", ["gnss_denied", "position_error_m"]),
    ("Vehicle response", ["pose.speed", "yaw_rate"]),
    ("Safety", ["scan.min_range"]),
]


def series_value(frames, i, path):
    """Pull one series' value out of frame i. None if it is not there.

    Takes the whole run rather than one frame because the two most important
    vehicle metrics - speed and turn rate - are derivatives, and a derivative
    needs its neighbour. Braking, speeding up and turning are exactly what the
    plan says an attack looks like from outside, so these are not conveniences.
    """
    frame = frames[i]
    parts = path.split(".")

    if parts[0] == "agents" and parts[2] == "yaw_rate":
        if i == 0:
            return 0.0
        try:
            prev, cur = frames[i - 1], frame
            dt = cur.get("sim_time_s", 0) - prev.get("sim_time_s", 0)
            if dt <= 1e-6:
                return 0.0
            a0 = next(a for a in prev["agents"] if a.get("id") == parts[1])["pose"]
            a1 = next(a for a in cur["agents"] if a.get("id") == parts[1])["pose"]
            d = (a1["yaw"] - a0["yaw"] + math.pi) % (2 * math.pi) - math.pi
            return round(d / dt, 4)
        except (KeyError, StopIteration, TypeError):
            return None

    # The experiment metrics. Flags become 0/1 so a column is numeric all the
    # way down and means the same thing averaged or plotted.
    _FLAGS = {"commanded": ("authority", "reachable"),
              "held": ("link_loss_hold",), "on_intent": ("on_intent",),
              "gnss_denied": ("gnss_denied",)}
    if parts[0] == "agents" and len(parts) == 3 and parts[2] in _FLAGS:
        try:
            agent = next(a for a in frame["agents"] if a.get("id") == parts[1])
            node = agent
            for k in _FLAGS[parts[2]]:
                node = node[k]
            return 1 if node else 0
        except (KeyError, StopIteration, TypeError):
            return None
    if parts[0] == "agents" and len(parts) == 3 and parts[2] == "noise_floor_dbm":
        try:
            agent = next(a for a in frame["agents"] if a.get("id") == parts[1])
            return agent["rf"]["noise_floor_dbm"]
        except (KeyError, StopIteration, TypeError):
            return None

    try:
        if parts[0] == "agents":
            agent = next(a for a in frame["agents"] if a.get("id") == parts[1])
            if len(parts) == 3:
                v = agent.get(parts[2])
                return v if isinstance(v, (int, float)) else None
            if parts[2] == "scan":
                hits = [r for r in agent["scan"]["ranges"] if r is not None]
                return min(hits) if hits else None
            return agent[parts[2]][parts[3]]
        if parts[0] == "links":
            a, b = parts[1].split("-")
            link = next(l for l in frame["links"]
                        if l.get("a") == a and l.get("b") == b)
            return link[parts[2]]
    except (KeyError, IndexError, StopIteration, TypeError, ValueError):
        return None
    return None


PALETTE = ["#3E9AA8", "#C99A3E", "#8FB84E", "#B57BD0", "#C4685A", "#5C8FD6"]


class SeriesTree(QTreeWidget):
    """The series list, draggable onto a plot.

    Carries the series path in the drag rather than the visible label, so a
    friendly name like "speed (m/s)" can be shown without the plot having to
    guess which series it came from.
    """

    def __init__(self):
        super().__init__()
        self.setDragEnabled(True)
        self.setDragDropMode(QTreeWidget.DragOnly)

    def startDrag(self, actions):
        item = self.currentItem()
        path = item.data(0, Qt.UserRole) if item else None
        if not path:
            return
        mime = QMimeData()
        mime.setText(path)
        drag = QDrag(self)
        drag.setMimeData(mime)
        drag.exec(Qt.CopyAction)


class PlotPane(QWidget):
    """One plot. Drop series on it; right-click to split it.

    Series dropped together share a pane and are drawn overlaid, each on its own
    scale with its range printed. Different units on a shared axis would either
    flatten one series or imply a comparison that is not real; what overlaying
    is genuinely good for is comparing shape and timing, and that survives.
    """

    def __init__(self, area):
        super().__init__()
        self.area = area
        self.series = []
        self.setAcceptDrops(True)
        self.setMinimumSize(160, 90)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    # -- drag and drop ------------------------------------------------------

    def dragEnterEvent(self, ev):
        if ev.mimeData().hasText():
            ev.acceptProposedAction()

    def dropEvent(self, ev):
        path = ev.mimeData().text()
        if path and path not in self.series:
            self.series.append(path)
            self.update()
        ev.acceptProposedAction()

    # -- splitting ----------------------------------------------------------

    def contextMenuEvent(self, ev):
        menu = QMenu(self)
        a = menu.addAction("Split horizontally")
        a.triggered.connect(lambda: self.area.split(self, Qt.Horizontal))
        a = menu.addAction("Split vertically")
        a.triggered.connect(lambda: self.area.split(self, Qt.Vertical))
        menu.addSeparator()
        if self.series:
            for pth in list(self.series):
                act = menu.addAction(f"Remove  {pth}")
                act.triggered.connect(lambda _, q=pth: self.remove_series(q))
            a = menu.addAction("Clear this plot")
            a.triggered.connect(lambda: (self.series.clear(), self.update()))
        a = menu.addAction("Close this plot")
        a.triggered.connect(lambda: self.area.close_pane(self))
        menu.exec(ev.globalPos())

    def remove_series(self, path):
        if path in self.series:
            self.series.remove(path)
            self.update()

    # -- painting -----------------------------------------------------------

    def paintEvent(self, _):
        p = QPainter(self)
        try:
            p.setRenderHint(QPainter.Antialiasing)
            p.fillRect(self.rect(), QColor(C_BG))
            p.setPen(QPen(QColor(C_LINE)))
            p.drawRect(0, 0, self.width() - 1, self.height() - 1)
            p.setFont(QFont("Consolas", 9))
            self._draw(p)
        finally:
            p.end()

    def _draw(self, p):
        frames = self.area.frames
        if self.area.live:
            p.setPen(QPen(QColor(C_WARN)))
            p.drawText(12, 22, "Run in progress - stop the run to read results")
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(12, 40, f"{len(frames)} frames recorded")
            return
        if not frames:
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(12, 22, "No run recorded. Press Run, then Stop.")
            return
        if not self.series:
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(12, 22, "Drag a series here from the left.")
            p.drawText(12, 40, "Right-click to split this plot.")
            return

        t = [f.get("sim_time_s", 0.0) for f in frames]
        t0, t1 = t[0], max(t[-1], t[0] + 1e-6)
        left, right, top, bot = 60, 10, 8, 20
        w, h = self.width() - left - right, self.height() - top - bot
        if w < 40 or h < 30:
            return

        p.setPen(QPen(QColor(C_GRID)))
        p.drawRect(QRectF(left, top, w, h))

        shared = getattr(self.area, "shared_scale", False)
        allpairs = {}
        for path in self.series:
            vals = [series_value(frames, i, path) for i in range(len(frames))]
            allpairs[path] = [(tt, v) for tt, v in zip(t, vals)
                              if isinstance(v, (int, float))]
        if shared:
            every = [v for prs in allpairs.values() for _, v in prs]
            g_lo, g_hi = (min(every), max(every)) if every else (0.0, 1.0)
            # A shared scale starts at zero when the data does not go
            # negative: a penetration of 12 m next to one of 87 m should LOOK
            # like a seventh, and a floating baseline hides exactly that.
            if g_lo > 0:
                g_lo = 0.0
        for idx, path in enumerate(self.series):
            pairs = allpairs[path]
            colour = QColor(PALETTE[idx % len(PALETTE)])
            if not pairs:
                p.setPen(QPen(colour))
                p.drawText(left + 6, top + 14 + idx * 14, f"{path}   no data")
                continue
            # DRAW on the shared scale, but LABEL with this series' own
            # range. Printing the shared range against every series made nine
            # different curves all read "0 .. 192", which looks exactly like
            # nine identical series - the opposite of what the label is for.
            own_lo = min(v for _, v in pairs)
            own_hi = max(v for _, v in pairs)
            if shared:
                lo, hi = g_lo, g_hi
            else:
                lo, hi = own_lo, own_hi
            span = (hi - lo) or 1.0
            poly = QPolygonF()
            for tt, v in pairs:
                poly.append(QPointF(left + w * (tt - t0) / (t1 - t0),
                                    top + h - (v - lo) / span * (h - 8) - 4))
            p.setPen(QPen(colour, 1.4))
            p.setBrush(Qt.NoBrush)
            p.drawPolyline(poly)
            # A COMPACT LEGEND. The full dotted path is how the series is
            # addressed, not what it is called: on a swept chart every entry
            # began "agents." and ended with the same metric, so the nine
            # names differed only in the middle and the legend was a wall.
            label = path
            if label.startswith("agents."):
                label = label[7:]
            if "." in label:
                who, metric = label.rsplit(".", 1)
                label = who.replace("_", " / ")
            p.setPen(QPen(colour))
            p.drawText(left + 6, top + 14 + idx * 13,
                       f"{label}   {own_lo:.3g} .. {own_hi:.3g}")

        cursor = self.area.cursor
        if cursor is not None and 0 <= cursor < len(t):
            cx = left + w * (t[cursor] - t0) / (t1 - t0)
            p.setPen(QPen(QColor(C_WARN), 1))
            p.drawLine(QPointF(cx, top), QPointF(cx, top + h))

        # X AXIS, labelled with whatever it actually is. Every distinct x
        # value gets a tick, so a sweep of three powers reads as three
        # readings rather than as a continuum.
        unit = getattr(self.area, "xlabel", "s")
        p.setPen(QPen(QColor(C_DIM)))
        p.setFont(QFont("Consolas", 7))
        for tt in sorted(set(t)):
            cx = left + w * (tt - t0) / (t1 - t0)
            p.drawLine(QPointF(cx, top + h), QPointF(cx, top + h + 3))
            p.drawText(QPointF(cx - 12, self.height() - 6), f"{tt:g}")
        p.drawText(QPointF(self.width() - right - 52, self.height() - 6), unit)
        if shared:
            # The shared scale is a number you can read off, not an implied
            # one - otherwise "they share a scale" is a claim, not a fact.
            p.drawText(QPointF(4, top + 10), f"{g_hi:.3g}")
            p.drawText(QPointF(4, top + h), f"{g_lo:.3g}")


class PlotArea(QWidget):
    """A tree of splitters holding plots — the PlotJuggler arrangement.

    One plot to begin with. Right-click any plot and split it, and the pane is
    replaced in place by a splitter holding the original and a new empty one, so
    the layout the researcher builds is whatever their question needs rather
    than whatever grid was decided in advance.
    """

    def __init__(self):
        super().__init__()
        self.frames = []
        self.cursor = None
        self.live = False
        # THE X AXIS IS NOT ALWAYS TIME. A swept result is a value per
        # PARAMETER, not per second, and an axis that says "s" under jammer
        # power is simply wrong. Set by whoever loads the frames.
        self.xlabel = "s"
        # SHARED SCALE. For time series, each series keeps its own scale on
        # purpose - different units on one axis either flatten one or imply a
        # comparison that is not real. For a SWEEP every series is the SAME
        # metric across architectures, and auto-scaling each one to its own
        # range would make them all fill the pane and look identical, which
        # destroys the only comparison the chart exists to make.
        self.shared_scale = False
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        self.root = PlotPane(self)
        lay.addWidget(self.root)

    def panes(self):
        out, stack = [], [self.root]
        while stack:
            node = stack.pop()
            if isinstance(node, PlotPane):
                out.append(node)
            elif isinstance(node, QSplitter):
                stack += [node.widget(i) for i in range(node.count())]
        return out

    def split(self, pane, orientation):
        new = PlotPane(self)
        splitter = QSplitter(orientation)
        splitter.setChildrenCollapsible(False)
        parent = pane.parentWidget()
        if pane is self.root:
            self.layout().removeWidget(pane)
            splitter.addWidget(pane)
            splitter.addWidget(new)
            self.root = splitter
            self.layout().addWidget(splitter)
        else:
            while parent is not None and not isinstance(parent, QSplitter):
                parent = parent.parentWidget()
            if parent is None:
                return
            index = parent.indexOf(pane)
            sizes = parent.sizes()
            splitter.addWidget(pane)
            splitter.addWidget(new)
            parent.insertWidget(index, splitter)
            parent.setSizes(sizes)
        self.refresh()

    def close_pane(self, pane):
        if pane is self.root:
            pane.series.clear()
            pane.update()
            return
        parent = pane.parentWidget()
        while parent is not None and not isinstance(parent, QSplitter):
            parent = parent.parentWidget()
        if parent is None:
            return
        pane.setParent(None)
        pane.deleteLater()
        # A splitter holding one plot is just that plot: collapse it away so
        # repeated splitting and closing does not leave dead scaffolding.
        if parent.count() == 1:
            survivor = parent.widget(0)
            grand = parent.parentWidget()
            while grand is not None and not isinstance(grand, (QSplitter, PlotArea)):
                grand = grand.parentWidget()
            if isinstance(grand, QSplitter):
                idx = grand.indexOf(parent)
                grand.insertWidget(idx, survivor)
                parent.setParent(None)
                parent.deleteLater()
            elif parent is self.root:
                self.layout().removeWidget(parent)
                self.root = survivor
                self.layout().addWidget(survivor)
                parent.setParent(None)
                parent.deleteLater()
        self.refresh()

    def add_to_first_empty(self, path):
        """Double-click fallback: fill the first empty plot, else the first."""
        panes = self.panes()
        if not panes:
            return
        target = next((q for q in panes if not q.series), panes[0])
        if path not in target.series:
            target.series.append(path)
        self.refresh()

    def refresh(self):
        for pane in self.panes():
            pane.update()


class ShellPanel(QWidget):
    """One Ubuntu shell inside the Console.

    Each panel keeps its own working directory and its own processes, so a
    long-running controller in one tab does not stop you inspecting topics in
    another — which is the whole reason there is more than one.

    Pipes, not a pseudo-terminal: fine for colcon, ros2, git and ls; not fine
    for anything that wants to talk back. sudo's password prompt and vim need a
    real Ubuntu window, and the placeholder text says so.
    """

    # Cell doctrine: which SIDE a cell may issue tactical commands to.
    # white = umpire, anything. blue/red = that side only. Bash is allowed in
    # every cell (they are still terminals); only mission/attack commands are
    # scoped. See docs/cells-and-network.md.
    CELL_SIDE = {"blue": "blue", "red": "red", "white": None}

    def __init__(self, repo_wsl_path, cwd=None, cell="white", console=None):
        super().__init__()
        self.repo = repo_wsl_path
        self.cwd = cwd or repo_wsl_path
        self.procs = []
        self.cell = cell
        self.console = console

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setFont(QFont("Consolas", 9))
        self.inp = QLineEdit()
        self.inp.setFont(QFont("Consolas", 9))
        remit = {"blue": "Blue cell — commands the BLUE network "
                         "(SETMISSION, launch/halt, REOBJECTIVE).",
                 "red": "Red cell — commands the RED network "
                        "(red launch/halt, JAM).",
                 "white": "White cell (umpire) — any command, any side, "
                          "plus the shell."}.get(self.cell, "")
        self.inp.setPlaceholderText(
            remit + "  Ubuntu commands also work; interactive tools "
            "(sudo, vim) need a real terminal.")
        self.inp.returnPressed.connect(self.run)
        lay.addWidget(self.out, 1)
        lay.addWidget(self.inp)
        self.out.appendPlainText(f"# working directory: {self.cwd}")

    def run(self):
        cmd = self.inp.text().strip()
        if not cmd:
            return
        self.inp.clear()
        # A pasted shell transcript often carries its own "$ " prompt. Strip
        # ONE leading occurrence so a paste of "$ REOBJECTIVE car1 shuttle
        # (-3,3,0) (3,3,0)" parses as the command, not as literal text that
        # sends "$" (and its parentheses) straight to bash.
        if cmd.startswith("$"):
            cmd = cmd[1:].lstrip()
            if not cmd:
                return
        self.out.appendPlainText(f"$ {cmd}")

        # Only a BARE cd. "cd ros2 && colcon build" is a compound command and
        # must reach the shell whole.
        if cmd.startswith("cd ") and not any(t in cmd for t in ("&&", "||", ";", "|")):
            self.cwd = cmd[3:].strip()
            self.out.appendPlainText(f"# working directory: {self.cwd}")
            return
        if cmd in ("clear", "cls"):
            self.out.clear()
            return

        # Mission commands - handled here rather than sent to the shell, all
        # writing one line to the run's retask queue, which the running sim
        # reads on its next tick:
        #     <network-or-agent> launch      arm - objective(s) start acting
        #     <network-or-agent> halt        un-arm - freezes at current pose
        #     REOBJECTIVE <agent> <objective> <args...>
        #         e.g.  REOBJECTIVE car1 pursue car3
        #               REOBJECTIVE car3 shuttle A B
        #               REOBJECTIVE car3 shuttle (-3,3,0) (3,3,0)
        #     SETMISSION <name>   - set the run's mission: applies
        #         missions/<name>.yaml's objectives (gated by command
        #         authority - an unreachable agent is not retasked) and
        #         titles the run. Do this after Play, before `blue launch`.
        # Tokenizing keeps a parenthesized "(-3, 3, 0)" as one token even with
        # the internal spaces - see docs/maps-missions-and-retasking.md.
        head = _tokenize_args(cmd) if _tokenize_args else cmd.split()
        if len(head) == 2 and head[1].lower() in ("launch", "halt"):
            if self._cell_allows("launch", scope=head[0]):
                self._launch_halt(head[1].lower(), head[0])
            return
        if head and head[0].upper() == "SETMISSION":
            if self._cell_allows("SETMISSION"):
                self._retask(head)
            return
        if head and head[0].upper() == "REOBJECTIVE":
            scope = head[1] if len(head) > 1 else None
            if self._cell_allows("REOBJECTIVE", scope=scope):
                self._retask(head)
            return
        if head and head[0].upper() == "JAM":
            scope = head[1] if len(head) > 1 else None
            if self._cell_allows("JAM", scope=scope):
                self._send_queue_line(cmd + "\n", cmd)
            return

        full = (f"cd {self.cwd} 2>/dev/null; "
                f"source /opt/ros/humble/setup.bash 2>/dev/null; "
                f"source {self.repo}/ros2/install/setup.bash 2>/dev/null; "
                f"{cmd}")
        proc = QProcess(self)
        self.procs.append(proc)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.readyReadStandardOutput.connect(
            lambda p=proc: self.out.appendPlainText(
                bytes(p.readAllStandardOutput()).decode(errors="replace").rstrip()))
        proc.finished.connect(
            lambda code, _st, p=proc: (
                self.out.appendPlainText(f"[exit {code}]") if code else None,
                self.procs.remove(p) if p in self.procs else None))
        proc.start("wsl.exe", ["-e", "bash", "-lc", full])
        if not proc.waitForStarted(3000):
            self.out.appendPlainText(
                "[cannot start wsl.exe - is WSL installed and on PATH?]")

    def _cell_allows(self, verb, scope=None):
        """Is this cell permitted to issue this command? White may do
        anything. Blue/red may act only on their own side. A refusal prints
        why and is NOT sent. SETMISSION is blue-only (missions are blue);
        JAM is red-only (jamming is the adversary's)."""
        remit = ShellPanel.CELL_SIDE.get(self.cell)   # None = white = all
        if remit is None:
            return True
        if verb == "SETMISSION":
            if remit != "blue":
                self.out.appendPlainText(
                    "[blocked: missions are a BLUE-cell action]")
                return False
            return True
        if verb == "JAM":
            if remit != "red":
                self.out.appendPlainText(
                    "[blocked: JAM is a RED-cell action]")
                return False
            return True
        # launch / halt / REOBJECTIVE: the scope must be on this cell's side.
        side = None
        if scope is not None and self.console is not None:
            side = self.console.side_of(scope)
        if side is None:
            self.out.appendPlainText(
                f"[blocked: {self.cell} cell cannot resolve '{scope}' on its "
                f"side — is it a {remit} agent/network?]")
            return False
        if side != remit:
            self.out.appendPlainText(
                f"[blocked: '{scope}' is on the {side} side; this is the "
                f"{remit} cell]")
            return False
        return True

    def _retask(self, tokens):
        """Write a retask/order command to the running sim's queue.

        tokens[0] is REOBJECTIVE, REMISSION or LOADMISSION (already
        upper-checked in run()):

            REOBJECTIVE <agent> <objective> <args...>
                -> "<agent>: <objective> <args>" - parse_retask's grammar,
                one agent.
            REMISSION <network> <objective> <args...>
                -> forwarded verbatim as "REMISSION <network> <objective>
                <args>" - a system order the running sim decomposes into
                per-agent objectives, gated by that network's authority. See
                docs/maps-missions-and-retasking.md.
            LOADMISSION <mission-file>
                -> forwarded as "LOADMISSION <path>" - that file's own
                per-agent objectives, applied verbatim, no decomposition.
        """
        verb = tokens[0].upper()
        if verb == "SETMISSION":
            if len(tokens) != 2:
                self.out.appendPlainText(
                    "[usage: SETMISSION <name>   e.g. SETMISSION test   "
                    "(missions/<name>.yaml; set it after Play, before "
                    "`blue launch`)]")
                return
            self._send_queue_line(f"SETMISSION {tokens[1]}\n",
                                  f"SETMISSION {tokens[1]}")
            return
        if len(tokens) < 3:
            self.out.appendPlainText(
                "[usage: REOBJECTIVE <agent> <objective> <args>   "
                "e.g. REOBJECTIVE car1 pursue car3]")
            return
        agent = tokens[1]
        rest = " ".join(tokens[2:])
        self._send_queue_line(f"{agent}: {rest}\n", f"{agent} -> {rest}")

    def _launch_halt(self, verb, scope):
        """`<scope> launch` / `<scope> halt` -> "LAUNCH <scope>" / "HALT
        <scope>" on the queue. Scope is a network name or an agent id; the
        running sim resolves which one - see "The state machine" in
        docs/PATCH-07-CHECKS.md."""
        self._send_queue_line(f"{verb.upper()} {scope}\n", f"{verb} {scope}")

    def _send_queue_line(self, line, summary):
        """Write one command as its own brand-new file in the sim's retask
        directory - the channel LAUNCH/HALT, REOBJECTIVE, REMISSION and
        LOADMISSION all share.

        Used to append this line to one shared "queue" file. On Windows
        that raced the sim's reader: Python's open() doesn't request
        FILE_SHARE_DELETE, so while this file handle was open (even
        briefly) the sim's attempt to claim the file by renaming it could
        fail outright - and that failure meant the sim read NOTHING that
        poll, not even the earlier commands already sitting in the file. A
        command then needing to be typed twice wasn't a second attempt
        succeeding where the first failed; it was the first attempt still
        sitting there, waiting for a poll that could finally get in.

        Fixed by never touching an existing path at all: each command gets
        its own new filename (monotonic, so read order matches send order),
        written to a temp name and atomically renamed into place, so the
        sim never sees a partial write either. Nothing else ever reopens
        this path once it exists, so there is nothing left for the sim's
        reader to contend with. See "Bug fix: the retask race" in
        docs/PATCH-08-CHECKS.md.
        """
        try:
            qdir = REPO_ROOT / "runs" / "retask"
            qdir.mkdir(parents=True, exist_ok=True)
            name = f"cmd_{time.monotonic_ns():020d}.txt"
            tmp = qdir / (name + ".tmp")
            tmp.write_text(line, encoding="utf-8")
            tmp.rename(qdir / name)
            self.out.appendPlainText(f"[sent: {summary}]")
        except OSError as exc:
            self.out.appendPlainText(f"[send failed: {exc}]")

    def stop_all(self):
        for proc in list(self.procs):
            proc.kill()
            proc.waitForFinished(1000)


# ---------------------------------------------------------------------------
# Custom fleet builder - compose a fleet in the Console, no YAML by hand
# ---------------------------------------------------------------------------

# Sensible defaults per platform, so a custom fleet is a real fleet: bodies,
# sensors and performance that the RF and dynamics models can actually use.
# The "build one now" entry that appears at the bottom of each fleet picker.
CUSTOM_FLEET_ENTRY = "+ Build custom fleet..."

def _load_agent_types():
    """Read agents/*.yaml - the HARDWARE catalogue the fleet builder offers.

    An agent file is a thing you could buy: size, performance, sensors,
    emitters. It carries no network, no authority, no doctrine and no pose,
    because those are DECISIONS and decisions are made in the Console. Adding
    a type is dropping a file in agents/ - no code change, and later an
    agent-creation console writes here.
    """
    out = {}
    folder = REPO_ROOT / "agents"
    if not folder.exists():
        return out
    for f in sorted(folder.glob("*.yaml")):
        try:
            import yaml as _y
            with open(f, encoding="utf-8") as fh:
                doc = _y.safe_load(fh) or {}
        except Exception:
            continue
        if doc.get("kind") != "agent":
            continue
        out[doc.get("name") or f.stem] = doc
    return out


PLATFORM_DEFAULTS = _load_agent_types()


class CustomFleetDialog(QDialog):
    """Build a fleet by hand: name it, pick a side, add agents with their
    platform and spawn pose. Writes a real fleets/<name>.yaml, so a fleet you
    invented in the sandbox is as diffable and re-runnable as a shipped one."""

    def __init__(self, side="blue", parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Build a custom {side} fleet")
        self.side = side
        lay = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addWidget(QLabel("Fleet name"))
        self.name = QLineEdit(f"custom_{side}")
        top.addWidget(self.name, 1)
        lay.addLayout(top)
        hint = QLabel("Add agents, set platform and spawn pose. Saved to "
                      "fleets/<name>.yaml and selected in Setup.")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        lay.addWidget(hint)

        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(
            ["id", "agent type", "x", "y", "z", "yaw"])
        self.table.verticalHeader().setVisible(False)
        lay.addWidget(self.table, 1)

        rowbtns = QHBoxLayout()
        add = QPushButton("Add agent")
        add.clicked.connect(self.add_row)
        rem = QPushButton("Remove selected")
        rem.clicked.connect(self.remove_row)
        rowbtns.addWidget(add)
        rowbtns.addWidget(rem)
        rowbtns.addStretch(1)
        lay.addLayout(rowbtns)

        row = QHBoxLayout()
        row.addStretch(1)
        cancel = QPushButton("Cancel")
        cancel.clicked.connect(self.reject)
        ok = QPushButton("Create fleet")
        ok.clicked.connect(self.accept)
        row.addWidget(cancel)
        row.addWidget(ok)
        lay.addLayout(row)

        # Seed with something sensible for the side.
        if side == "red":
            self.add_row(("jam1", "jam_single", 0.0, 3.0, 0.0, 0.0))
        else:
            self.add_row(("gcs", "ground_station", 0.0, -3.4, 0.0, 0.0))
            self.add_row(("car1", "roboracer", -3.0, 2.0, 0.0, 0.0))
            self.add_row(("car2", "roboracer", -3.0, 0.0, 0.0, 0.0))
            self.add_row(("car3", "roboracer", -3.0, -2.0, 0.0, 0.0))

    def add_row(self, preset=None):
        r = self.table.rowCount()
        self.table.insertRow(r)
        n = r + 1
        vals = preset if isinstance(preset, tuple) else (
            f"agent{n}", next(iter(PLATFORM_DEFAULTS), "roboracer"),
            0.0, 0.0, 0.0, 0.0)
        self.table.setItem(r, 0, QTableWidgetItem(str(vals[0])))
        combo = QComboBox()
        combo.addItems(list(PLATFORM_DEFAULTS))
        combo.setCurrentText(str(vals[1]))
        self.table.setCellWidget(r, 1, combo)
        for c, v in ((2, vals[2]), (3, vals[3]), (4, vals[4]), (5, vals[5])):
            self.table.setItem(r, c, QTableWidgetItem(str(v)))
        self.table.resizeColumnsToContents()

    def remove_row(self):
        r = self.table.currentRow()
        if r >= 0:
            self.table.removeRow(r)

    def fleet_doc(self):
        """The fleet YAML this dialog describes.

        A fleet is a COMPOSITION: which agents, with which ids, on which
        side. It deliberately declares NO authority, NO routing and NO
        doctrine - those are picked in the Setup tab and in the spawn dialog,
        and baking them in here is what produced seven fleet files describing
        three vehicles.

        The only network property written is the band, because which radio is
        fitted is hardware, and `coordinator`, because naming the ground
        station is structural rather than a policy: WHETHER anyone obeys it is
        the authority setting.
        """
        agents = []
        for r in range(self.table.rowCount()):
            def num(c):
                try:
                    return float(self.table.item(r, c).text())
                except (TypeError, ValueError):
                    return 0.0
            aid = (self.table.item(r, 0).text() or f"agent{r+1}").strip()
            w = self.table.cellWidget(r, 1)
            tname = w.currentText() if w else next(iter(PLATFORM_DEFAULTS), "")
            spec_doc = PLATFORM_DEFAULTS.get(tname) or {}
            a = {"id": aid,
                 "platform": spec_doc.get("platform", "roboracer"),
                 # Which agent FILE this came from, kept so a saved fleet says
                 # what hardware it is made of rather than only what shape.
                 "agent_type": tname,
                 "network": self.side,
                 "colour": spec_doc.get("colour", "#2E6FB0"),
                 "pose": {"x": num(2), "y": num(3), "z": num(4),
                          "yaw": num(5)}}
            for key in ("dimensions", "performance", "sensors", "ghost"):
                if spec_doc.get(key) is not None:
                    a[key] = copy.deepcopy(spec_doc[key])
            # EMITTERS. The agent file says how many radios are fitted and
            # what each defaults to; power and band are retuned live from the
            # red cell. One emitter collapses to the single `jammer:` block
            # the model already understands.
            em = spec_doc.get("emitters") or []
            if len(em) == 1:
                a["jammer"] = {"tx_power": copy.deepcopy(em[0]["tx_power"]),
                               "band": copy.deepcopy(em[0]["band"])}
            elif len(em) > 1:
                a["jammer"] = {"tx_power": copy.deepcopy(em[0]["tx_power"]),
                               "band": copy.deepcopy(em[0]["band"])}
                a["emitters"] = copy.deepcopy(em)
            agents.append(a)
        net = {"system": "adversary" if self.side == "red" else "friendly",
               "band": 2400}
        gcs = next((a["id"] for a in agents
                    if a["platform"] == "ground_station"), None)
        if gcs and self.side != "red":
            net["coordinator"] = gcs
        name = (self.name.text() or f"custom_{self.side}").strip()
        return name, {
            "spec_version": 0.1, "name": name, "kind": "fleet",
            "description": "Composed in the Console from agents/. Declares no "
                           "authority, routing or doctrine - those are picked "
                           "in Setup.",
            "networks": {self.side: net}, "agents": agents}



# ---------------------------------------------------------------------------
# Spawn dialog - where does this fleet start, in THIS scene?
# ---------------------------------------------------------------------------

class SpawnDialog(QDialog):
    """Asked the moment a fleet is chosen, with the scene already in view.

    A fleet file carries default poses, but those are coordinates from
    whatever scene it was written against - so placing the fleet is a
    per-setup decision, made looking at the actual room. Defaults are
    offered, not presumed.
    """

    def __init__(self, agents, parent=None, title="Spawn the fleet"):
        super().__init__(parent)
        self.setWindowTitle(title)
        lay = QVBoxLayout(self)
        lab = QLabel("Where does each agent start? Defaults are the fleet's "
                     "own. x/y in metres, yaw in radians.")
        lab.setWordWrap(True)
        lay.addWidget(lab)
        self.table = QTableWidget(len(agents), 6)
        self.table.setHorizontalHeaderLabels(
            ["Agent", "x", "y", "z", "yaw", "on link loss"])
        self.table.verticalHeader().setVisible(False)
        self._ids = []
        self._doctrine = {}
        for r, a in enumerate(agents):
            pose = a.get("pose") or {}
            aid = str(a.get("id", f"agent{r}"))
            self._ids.append(aid)
            item = QTableWidgetItem(aid)
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            self.table.setItem(r, 0, item)
            # z is a real column, not carried silently: an aerial fleet
            # spawns AT altitude, and the suspended-gcs trick (out of the
            # lidar scan plane) is exactly a z decision.
            for c, key in ((1, "x"), (2, "y"), (3, "z"), (4, "yaw")):
                self.table.setItem(
                    r, c, QTableWidgetItem(str(_num(pose.get(key)))))
            # DOCTRINE, PER AGENT. What this vehicle does when it cannot reach
            # whoever commands it. Per agent, not per fleet, because the
            # interesting configuration is a MIXED one - a leader that holds
            # while its members act on the commander's intent - and that
            # cannot be said with a fleet-wide setting at all.
            #
            # It is here rather than in the fleet file because it is a
            # DECISION, not hardware. A fleet file that hardcodes it forces a
            # near-duplicate file per doctrine; this is one dropdown.
            box = QComboBox()
            box.addItems(["hold", "intent"])
            box.setToolTip(
                "hold    freeze until the link returns - the conservative\n"
                "        default, and what most autopilot failsafes do.\n"
                "intent  keep executing the objective already assigned, from\n"
                "        the picture already held. NATO mission command:\n"
                "        centralized intent, decentralized execution\n"
                "        (AJP-3 Ed D V1, paras 3.8 and 3.11).")
            cur = str(a.get("on_link_loss") or "hold").lower()
            box.setCurrentText("intent" if cur in ("intent", "continue")
                               else "hold")
            # A jammer or a ground station has no commander to lose.
            if a.get("jammer") or a.get("platform") == "ground_station":
                box.setEnabled(False)
                box.setToolTip("Not applicable - this agent has no commander.")
            self._doctrine[aid] = box
            self.table.setCellWidget(r, 5, box)
        self.table.resizeColumnsToContents()
        lay.addWidget(self.table)
        row = QHBoxLayout()
        row.addStretch(1)
        ok = QPushButton("Spawn here")
        ok.clicked.connect(self.accept)
        cancel = QPushButton("Cancel")
        cancel.clicked.connect(self.reject)
        row.addWidget(cancel)
        row.addWidget(ok)
        lay.addLayout(row)

    def spawns(self):
        """{agent_id: {x, y, z, yaw}} - every component the operator's, with
        the fleet's own pose as the offered default."""
        out = {}
        for r, aid in enumerate(self._ids):
            def val(c, fallback=0.0):
                try:
                    return float(self.table.item(r, c).text())
                except (TypeError, ValueError):
                    return fallback
            out[aid] = {"x": val(1), "y": val(2), "z": val(3), "yaw": val(4)}
        return out

    def doctrines(self):
        """{agent_id: 'hold'|'intent'} for agents that have a commander."""
        return {aid: box.currentText()
                for aid, box in self._doctrine.items() if box.isEnabled()}


# ---------------------------------------------------------------------------
# Experiment runner and results viewer
# ---------------------------------------------------------------------------
# Deliberately a SEPARATE WINDOW rather than another tab. An experiment is a
# different mode of working from a sandbox run - many runs, headless, nothing
# to interfere with - and it needs the whole canvas for its result. It also
# means nothing here can break the live tabs.
#
# The flow is the one the demo needs, end to end:
#   Run       -> tools/sweep.py in a worker thread, progress on a bar
#   Results   -> the table appears the moment it finishes
#   Click     -> that one run is re-executed and played back in the viewport
#
# Nothing is stored to make playback work. Every run is a pure function of
# (config, seed), so clicking a row RE-RUNS that cell in a fraction of a second
# and the frames are identical to the ones the sweep saw. Storing recordings
# for hundreds of runs would cost gigabytes to save you nothing.


def _sweep_module():
    """tools/sweep.py, imported as a module.

    The sweep runs IN-PROCESS. It used to be a QThread driving a subprocess,
    which is machinery for a problem that does not exist: 81 runs take two
    seconds. What it bought instead was a whole class of failure - a worker
    thread, a pipe, cross-thread signals and a second Python interpreter - and
    it crashed the Console on Windows. A loop with processEvents() is simpler,
    cannot crash that way, and gives finer progress.
    """
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "deadband_sweep", str(REPO_ROOT / "tools" / "sweep.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class ResultPlot(QLabel):
    """The swept result: penetration against jammer advantage.

    RENDERED INTO A PIXMAP, not painted in a paintEvent. Two attempts at the
    custom-paint version produced a blank rectangle that could not be
    diagnosed from the outside - no border, no error text, nothing - so the
    mechanism was changed rather than debugged further. A QLabel showing a
    pixmap has no paint-path subtleties: if the pixmap has pixels in it, they
    appear. It also means the chart can be rendered and inspected without a
    screen at all, which is how it is now tested.

    TWO VISUAL CHANNELS FOR TWO INDEPENDENT VARIABLES:
        COLOUR = command authority   (who decides)
        DASH   = routing             (how packets travel)
    They are independent axes in the model, so they get independent channels.
    Nine hues would be indistinguishable AND would hide that the two are
    separable, which is the finding.
    """

    AUTH_COLOUR = {"centralized": "#D2694F",
                   "decentralized": "#4FA3D1",
                   "hierarchical": "#6FAE7E"}
    ROUTE_DASH = {"star": Qt.SolidLine, "mesh": Qt.DashLine,
                  "tiered": Qt.DotLine}

    def __init__(self, parent=None):
        super().__init__(parent)
        self.rows = []
        self.metric = "penetration_m"
        self.highlight = None       # (authority, routing) of the opened run
        self._last_size = None
        self._rendering = False
        self.setMinimumHeight(320)
        # IGNORED, NOT EXPANDING - and this is load-bearing, not a detail. A
        # QLabel takes its size hint FROM its pixmap, so a bigger pixmap asks
        # for a bigger label, which resizes, which renders a bigger pixmap.
        # That feedback loop is what crashed the Console. `Ignored` makes the
        # layout decide the size and the pixmap follow it, never the reverse.
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background:#181C1F;")
        self.setText("No results yet - press Run.")

    def set_rows(self, rows):
        self.rows = rows
        self.render_chart()

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        # Only on a REAL size change, and never re-entrantly. Belt and braces
        # against the same feedback loop the size policy already breaks.
        size = (self.width(), self.height())
        if size != self._last_size:
            self._last_size = size
            self.render_chart()

    def series(self):
        """{(authority, routing): [(x, mean y), ...]} - seeds averaged."""
        acc = {}
        for r in self.rows:
            try:
                x = float(r.get("jam_rel_db"))
                y = float(r.get(self.metric))
            except (TypeError, ValueError):
                continue
            acc.setdefault((r.get("authority"), r.get("routing")), {}) \
               .setdefault(x, []).append(y)
        return {k: sorted((x, sum(v) / len(v)) for x, v in d.items())
                for k, d in acc.items()}

    def render_chart(self, size=None):
        """Draw the chart into a pixmap and show it. Returns the pixmap, so a
        test can render one with no window on screen and check its pixels."""
        if self._rendering:
            return None
        self._rendering = True
        try:
            return self._render(size)
        finally:
            self._rendering = False

    def _render(self, size=None):
        # Two pixels inside the widget, so the pixmap can never be the thing
        # that decides how big the widget wants to be.
        w = int(size[0] if size else max(self.width() - 2, 320))
        h = int(size[1] if size else max(self.height() - 2, 240))
        w, h = min(w, 4000), min(h, 3000)
        pm = QPixmap(w, h)
        pm.fill(QColor("#181C1F"))
        p = QPainter(pm)
        try:
            self._draw(p, w, h)
        except Exception as exc:                       # noqa: BLE001
            p.setPen(QPen(QColor("#D2694F")))
            p.setFont(QFont("Consolas", 9))
            p.drawText(14, 26, f"chart failed: {type(exc).__name__}: {exc}")
        p.end()
        self.setPixmap(pm)
        return pm

    def _draw(self, p, W, H):
        p.setRenderHint(QPainter.Antialiasing)
        series = self.series()
        L, R, T, B = 70, 24, 22, 46
        w, h = W - L - R, H - T - B
        if w < 60 or h < 40:
            p.setPen(QPen(QColor(C_DIM)))
            p.setFont(QFont("Consolas", 9))
            p.drawText(10, 24, f"too small: {W}x{H}")
            return
        if not series:
            p.setPen(QPen(QColor(C_DIM)))
            p.setFont(QFont("Consolas", 9))
            p.drawText(L, T + 30, "No rows selected - tick a P_j/P_t value.")
            return

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

        p.setFont(QFont("Consolas", 8))
        for i in range(5):
            v = y1 * i / 4.0
            p.setPen(QPen(QColor("#2C3236")))
            p.drawLine(int(L), int(sy(v)), int(L + w), int(sy(v)))
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(8, int(sy(v)) + 4, f"{v:6.0f}")
        p.setPen(QPen(QColor("#4A5257")))
        p.drawLine(int(L), int(T), int(L), int(T + h))
        p.drawLine(int(L), int(T + h), int(L + w), int(T + h))
        for x in sorted(set(xs)):
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(int(sx(x)) - 14, int(T + h + 18), f"{x:+.0f} dB")
        p.drawText(int(L), int(T + h + 38),
                   "JAMMER ADVANTAGE  P_j / P_t   (dimensionless: the "
                   "absolute powers cancel)")
        p.save()
        p.translate(18, T + h)
        p.rotate(-90)
        p.drawText(0, 0, "PENETRATION (m advanced)")
        p.restore()

        for (auth, route), pts in sorted(series.items()):
            # THE RUN YOU OPENED IS ORANGE. Double-click a row and its line
            # lights up, so "the one I am watching" is answerable by looking.
            lit = self.highlight == (auth, route)
            col = (QColor("#E08A3C") if lit
                   else QColor(self.AUTH_COLOUR.get(auth, "#AAAAAA")))
            pen = QPen(col, 3.4 if lit else 2.0)
            pen.setStyle(self.ROUTE_DASH.get(route, Qt.SolidLine))
            p.setPen(pen)
            for i in range(len(pts) - 1):
                p.drawLine(int(sx(pts[i][0])), int(sy(pts[i][1])),
                           int(sx(pts[i + 1][0])), int(sy(pts[i + 1][1])))
            p.setPen(QPen(col, 1))
            p.setBrush(QBrush(col))
            for x, y in pts:
                p.drawEllipse(QPointF(sx(x), sy(y)), 3.0, 3.0)
        p.setBrush(Qt.NoBrush)


class ExperimentWindow(QDialog):
    """Run a sweep, see the table, click a row to watch that run."""

    def __init__(self, console, parent=None):
        super().__init__(parent)
        self.console = console
        self.setWindowTitle("Experiment")
        self.resize(1100, 760)
        self.rows = []
        self.outdir = None
        lay = QVBoxLayout(self)

        top = QHBoxLayout()
        top.addWidget(QLabel("Experiment"))
        self.exp_combo = QComboBox()
        # THE SETUP TAB IS THE DEFAULT SOURCE, and the only one you need. A
        # saved experiment YAML is still selectable, for re-running something
        # from months ago exactly as it was - but nothing has to be written to
        # a file to run an experiment now.
        self.exp_combo.addItem("From the Setup tab", None)
        for f in sorted((REPO_ROOT / "experiments").glob("*.yaml")):
            self.exp_combo.addItem(f"file: {f.stem}", str(f))
        top.addWidget(self.exp_combo, 1)
        self.run_btn = QPushButton("Run")
        self.run_btn.clicked.connect(self.start)
        top.addWidget(self.run_btn)
        lay.addLayout(top)

        mrow = QHBoxLayout()
        mrow.addWidget(QLabel("Show"))
        self.metric_combo = QComboBox()
        # THE METRICS WORTH A HEADLINE, in the order the argument runs:
        # how far did it get, was it commanded, did it stop, did it know where
        # it was, and what was the radio doing. Everything else is in Results.
        for key, lab in (
                ("penetration_m", "penetration (m advanced)"),
                ("penetration_frac", "penetration (fraction of the corridor)"),
                ("commanded_fraction", "commanded fraction"),
                ("held_fraction", "held fraction (frozen by doctrine)"),
                ("belief_err_m", "position error (belief vs truth, m)"),
                ("track_err_m", "tracking error (off the ordered path, m)"),
                ("worst_sinr_db", "worst link SINR (dB)"),
                ("worst_pdr", "worst link packet delivery"),
                ("arrived", "vehicles that arrived"),
                ("ended_s", "run length (s)")):
            self.metric_combo.addItem(lab, key)
        self.metric_combo.setToolTip(
            "Which metric the headline chart draws against jammer advantage. "
            "Every one of them, per architecture, is also in the Results tab "
            "where panes can be split and series overlaid.")
        self.metric_combo.activated.connect(
            lambda _i: self._draw_summary(getattr(self, "_shown", self.rows)))
        mrow.addWidget(self.metric_combo, 1)
        svgb = QPushButton("SVG")
        svgb.setFixedWidth(64)
        svgb.setToolTip("Write this chart as an SVG beside the results and "
                        "open it in a browser - the version for a slide.")
        svgb.clicked.connect(self.open_svg)
        mrow.addWidget(svgb)
        lay.addLayout(mrow)

        # A LOAD BAR, NOT AN ESTIMATE. An estimate is a guess you then have to
        # defend; a bar is a fact, and it cannot be wrong.
        self.bar = QProgressBar()
        self.bar.setTextVisible(True)
        self.bar.setFormat("idle")
        lay.addWidget(self.bar)

        # THE HEADLINE CHART, HERE, drawn by PlotArea - the same widget the
        # Results tab uses, which has worked all along. Four bespoke charts
        # failed in this slot for one reason: PlotPane draws against
        # sim_time_s, and a sweep handed over without a time gets an axis of
        # zero width, which renders as nothing at all and says nothing about
        # why. Give each swept power a frame whose sim_time_s IS that power
        # and everything works.
        #
        # This one opens on penetration, the answer to the question the
        # experiment asks. Every other metric is in the Results tab, where
        # there is room to split panes and overlay them.
        self.plot = ResultPlot()          # kept only for its colour map
        self.plot.hide()
        self.summary = PlotArea()
        self.summary.setMinimumHeight(280)
        lay.addWidget(self.summary, 2)
        note = QLabel("Penetration against jammer advantage. The x axis is "
                      "P_j/P_t in dB, not seconds. Every other metric - "
                      "commanded fraction, belief error, worst SINR - is in "
                      "the Results tab, where panes can be split and series "
                      "overlaid.")
        note.setObjectName("hint")
        note.setWordWrap(True)
        lay.addWidget(note)
        # THE CHART, GUARANTEED. Three different in-widget drawing mechanisms
        # have failed to appear in this dialog - a custom paintEvent, the same
        # wrapped in a try/except that could not even print its own failure,
        # and a QLabel showing a pixmap. The drawing code is demonstrably
        # sound: tools/plot_results.py renders the identical chart to SVG with
        # pure standard library and it is correct every time.
        #
        # So rather than debug a fourth attempt blind, this writes that SVG
        # and opens it. It is one click, it works in any browser, and it is
        # the file you would put in a slide anyway. If the embedded chart
        # above ever starts working it is a convenience; this is the chart.


        # The power tickboxes: which jammer advantages are drawn. Opens on ONE
        # of them, because nine lines is a chart and twenty-seven is a wall.
        self.filt = QHBoxLayout()
        self.filt.addWidget(QLabel("Show P_j/P_t:"))
        self.filt.addStretch(1)
        lay.addLayout(self.filt)
        self._boxes = []

        leg = QLabel(
            "colour = authority   centralized / decentralized / hierarchical"
            "        dash = routing   — star   – – mesh   "
            "··· tiered")
        leg.setObjectName("hint")
        lay.addWidget(leg)

        self.table = QTableWidget(0, 0)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.cellDoubleClicked.connect(self.replay_row)
        lay.addWidget(self.table, 3)

        hint = QLabel("Double-click a row to re-run that exact cell and watch "
                      "it in the viewport. Nothing is stored to make this "
                      "work — every run is a pure function of its config "
                      "and seed, so the replay is the same run, not a "
                      "recording of it.")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        lay.addWidget(hint)

    # -- running -----------------------------------------------------------
    def start(self):
        """Run the sweep here and now, one cell at a time, updating the bar.

        No thread, no subprocess. Every failure is an ordinary Python
        exception that lands in the log with a traceback instead of taking the
        Console down with it.
        """
        path = self.exp_combo.currentData()
        cfg = None
        if path is None:
            cfg, why = (self.console._experiment_cfg()
                        if self.console is not None else (None, "no console"))
            if cfg is None:
                self.bar.setFormat(why)
                return
        self.run_btn.setEnabled(False)
        self.bar.setRange(0, 1)
        self.bar.setValue(0)
        self.bar.setFormat("loading...")
        QApplication.processEvents()
        try:
            self._run_sweep(Path(path) if path else None, cfg=cfg)
        except Exception as exc:                       # noqa: BLE001
            import traceback
            self.bar.setFormat(f"failed: {exc}")
            if self.console is not None:
                self.console.say("EXPERIMENT FAILED\n" + traceback.format_exc())
        finally:
            self.run_btn.setEnabled(True)

    def _run_sweep(self, exp_path, cfg=None):
        import csv as _csv
        import yaml as _yaml
        from datetime import datetime
        sweep = _sweep_module()
        if cfg is None:
            cfg = _yaml.safe_load(exp_path.read_text(encoding="utf-8"))
        cells, names = sweep.cells_of(cfg)
        n = len(cells)
        self.bar.setRange(0, n)
        self._cfg, self._names, self._exp_path = cfg, names, exp_path

        rows = []
        for i, cell in enumerate(cells, 1):
            rows.append(sweep.run_one((cfg, cell, False)))
            self.bar.setValue(i)
            self.bar.setFormat(f"%v / %m runs")
            if i % 3 == 0 or i == n:
                QApplication.processEvents()
        rows.sort(key=lambda r: r.get("cell", ""))

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        outdir = REPO_ROOT / "runs" / f"sweep_{cfg['name']}_{stamp}"
        outdir.mkdir(parents=True, exist_ok=True)
        fields = list(dict.fromkeys(
            [k for r in rows for k in r.keys()]))
        csv_path = outdir / "results.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as fh:
            w = _csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)
        # The experiment file goes with its results. A CSV without the
        # configuration that produced it is an orphan.
        # THE CONFIGURATION GOES WITH ITS RESULTS, always. You never open
        # this - Setup is where you configure - but a CSV without the run that
        # produced it is an orphan, and this one is complete enough to re-run
        # the whole grid months later, fleet and spawns included.
        (outdir / "experiment.yaml").write_text(
            exp_path.read_text(encoding="utf-8") if exp_path
            else _yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
        bad = [r for r in rows if r.get("error")]
        if self.console is not None:
            self.console.say(f"{len(rows)} runs -> {csv_path}"
                             + (f"  ({len(bad)} FAILED)" if bad else ""))
            for r in bad[:5]:
                self.console.say(f"  FAILED {r.get('cell')}: {r['error']}")
        self.outdir = outdir
        self._csv_path = csv_path
        self.load_csv(csv_path)

    # -- results -----------------------------------------------------------
    def load_csv(self, path):
        self._csv_path = path
        import csv as _csv
        try:
            with open(path, newline="", encoding="utf-8") as fh:
                self.rows = list(_csv.DictReader(fh))
        except OSError as exc:
            self.bar.setFormat(f"cannot read results: {exc}")
            return
        self.bar.setFormat(f"{len(self.rows)} runs")
        self._build_filter()
        self._fill_table()
        self._apply_filter()
        # The chart lives in the RESULTS TAB now, drawn by the plotting the
        # Console already has and which demonstrably works. Four bespoke
        # widgets failed in this dialog; this one reuses the tab whose whole
        # job is plotting, and brings split panes and series selection with
        # it for free.
        if self.console is not None:
            try:
                self.console.show_sweep_results(self.rows)
            except Exception as exc:                   # noqa: BLE001
                import traceback
                self.console.say("could not chart the sweep:\n"
                                 + traceback.format_exc())

    FILTER_COLS = ("authority", "routing", "jam_rel_db")

    def _build_filter(self):
        """One tick-list per filtered column, behind a button.

        Three columns are worth filtering and no more: the two architecture
        axes and the jammer power. Everything else in the table is an OUTPUT,
        and filtering on an output is how you fool yourself.
        """
        for w in self._boxes:
            w.setParent(None)
        self._boxes, self._filter = [], {}
        for col in self.FILTER_COLS:
            vals = sorted({r.get(col) for r in self.rows
                           if r.get(col) not in (None, "")},
                          key=lambda v: (self._numish(v), str(v)))
            if not vals:
                continue
            self._filter[col] = set(vals)
            btn = QPushButton(f"{col} \u25be")
            menu = QMenu(btn)
            for v in vals:
                act = QAction(str(v), menu)
                act.setCheckable(True)
                act.setChecked(True)
                act.toggled.connect(
                    lambda on, c=col, val=v: self._toggle_filter(c, val, on))
                menu.addAction(act)
            menu.addSeparator()
            allact = QAction("all / none", menu)
            allact.triggered.connect(
                lambda _c=False, mn=menu, cc=col: self._toggle_all(mn, cc))
            menu.addAction(allact)
            btn.setMenu(menu)
            btn.setToolTip(f"Show only these {col} values.")
            self.filt.insertWidget(self.filt.count() - 1, btn)
            self._boxes.append(btn)

    @staticmethod
    def _numish(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return float("inf")

    def _toggle_filter(self, col, val, on):
        keep = self._filter.setdefault(col, set())
        keep.add(val) if on else keep.discard(val)
        self._apply_filter()

    def _toggle_all(self, menu, col):
        acts = [a for a in menu.actions() if a.isCheckable()]
        want = not all(a.isChecked() for a in acts)
        for a in acts:
            a.setChecked(want)


    def _apply_filter(self, *_):
        """Redraw the table and the chart from the ticked values only."""
        rows = [r for r in self.rows
                if all(r.get(c) in keep
                       for c, keep in (self._filter or {}).items())]
        self._shown = rows
        self._fill_table(rows)
        if self.console is not None:
            try:
                self.console.show_sweep_results(rows)
            except Exception:                          # noqa: BLE001
                pass
        self._draw_summary(rows)
        self.bar.setFormat(f"{len(rows)} of {len(self.rows)} runs shown")


    def _fill_table(self, rows=None):
        rows = self.rows if rows is None else rows
        self._table_rows = rows
        # THE COLUMNS THAT MATTER. The CSV keeps everything; this table shows
        # the four that answer the question - what was configured, how far it
        # got, and whether it finished - because twenty columns of scrolling
        # is not a result you can read.
        cols = [c for c in ("authority", "routing", "jam_rel_db",
                            "penetration_m", "penetration_frac", "arrived",
                            "commanded_fraction", "ended_s")
                if rows and c in rows[0]]
        self.table.setColumnCount(len(cols))
        self.table.setHorizontalHeaderLabels(cols)
        self.table.setRowCount(len(rows))
        for r, row in enumerate(rows):
            for c, key in enumerate(cols):
                it = QTableWidgetItem(str(row.get(key, "")))
                it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                if key == "authority":
                    it.setForeground(QBrush(QColor(
                        ResultPlot.AUTH_COLOUR.get(row.get(key), "#AAAAAA"))))
                self.table.setItem(r, c, it)
        self.table.resizeColumnsToContents()


    def _draw_summary(self, rows):
        """Load penetration into this window's own plot, one series per
        architecture, already selected - so the answer is on screen without
        anyone having to build a chart first."""
        if not rows or self.console is None:
            return
        try:
            frames = self.console.sweep_frames(rows)
            self.summary.frames = frames
            self.summary.live = False
            self.summary.cursor = None
            # NOT SECONDS. The axis is the jammer's advantage over the
            # fleet's own radios, and every series is the same metric across
            # architectures, so they share one scale or the comparison the
            # chart exists to make is destroyed.
            self.summary.xlabel = "dB   P_j / P_t"
            self.summary.shared_scale = True
            metric = (self.metric_combo.currentData()
                      if hasattr(self, "metric_combo") else "penetration_m")
            paths = sorted({f"agents.{a['id']}.{metric}"
                            for f in frames for a in f["agents"]
                            if metric in a})
            for pane in self.summary.panes():
                pane.series = list(paths)
            self.summary.refresh()
        except Exception as exc:                       # noqa: BLE001
            if self.console is not None:
                self.console.say(f"summary chart: {exc}")

    def open_svg(self):
        """Render the current results to SVG and open it."""
        import webbrowser
        csvp = getattr(self, "_csv_path", None)
        if not csvp:
            self.bar.setFormat("run an experiment first")
            return
        try:
            import importlib.util
            spec = importlib.util.spec_from_file_location(
                "deadband_plot", str(REPO_ROOT / "tools" / "plot_results.py"))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            # The filter applies here too: the chart you export is the chart
            # you are looking at.
            rows = list(getattr(self, "_shown", None) or self.rows)
            metric = (self.metric_combo.currentData()
                      if hasattr(self, "metric_combo") else "penetration_m")
            series = mod.series_of(rows, metric)
            out = Path(csvp).with_suffix(f".{metric}.svg")
            out.write_text(mod.svg(series, metric), encoding="utf-8")
        except Exception as exc:                       # noqa: BLE001
            import traceback
            self.bar.setFormat(f"could not write the chart: {exc}")
            if self.console is not None:
                self.console.say(traceback.format_exc())
            return
        webbrowser.open(out.as_uri())
        self.bar.setFormat(f"chart -> {out.name}")
        if self.console is not None:
            self.console.say(f"chart written to {out}")

    # -- playback ----------------------------------------------------------
    def replay_row(self, row, _col):
        """Open this cell AS A LIVE RUN, not as a recording.

        It used to compute all the frames and then scrub them, which showed
        the outcome and hid the mechanism - and the mechanism is the whole
        point: vehicles advancing, one dropping out and going held, the rest
        pushing past it. Now the cell's exact configuration - its authority,
        its routing, its jammer power, its seed - is composed into a run and
        handed to the ordinary sandbox path, so it plays out in front of you
        with every panel live and every agent clickable, exactly like a run
        you set up by hand. There is one run path in this application, not
        two, and this is it.
        """
        shown = getattr(self, "_table_rows", None) or self.rows
        if row < 0 or row >= len(shown):
            return
        r = shown[row]
        cfg = getattr(self, "_cfg", None)
        if cfg is None:
            self.bar.setFormat("run an experiment first")
            return
        try:
            compose = copy.deepcopy(cfg.get("compose") or {})
            if not compose:
                self.bar.setFormat(
                    "this experiment came from a file, so there is no "
                    "composition to replay live - run it from the Setup tab")
                return
            nets = compose.setdefault("networks", {})
            blue = nets.setdefault("blue", {})
            blue["authority"] = blue["topology"] = r.get("authority")
            blue["routing"] = r.get("routing")
            if cfg.get("squads"):
                blue["squads"] = copy.deepcopy(cfg["squads"])
                blue["leader_loss"] = cfg.get("leader_loss", "fallback")
            # THE JAMMER'S POWER. This looked right and was wrong: a composed
            # run's `agents:` list holds only OVERRIDES - an id, a pose, a
            # doctrine - and never a `jammer` block, which lives in the fleet
            # layer underneath. So testing `if a.get("jammer")` matched
            # nothing, the override was never written, and the replay ran at
            # whatever the fleet file declared instead of the cell's power.
            # The cars advanced almost unimpeded and the run looked unjammed,
            # because it very nearly was.
            #
            # Resolve the composition first to find out which agents ARE
            # jammers, then write the power as an overlay entry keyed by id -
            # which _overlay merges into the fleet's own block.
            sweep = _sweep_module()
            dbm = sweep.FLEET_TX_DBM + float(r.get("jam_rel_db") or 0.0)
            import stub_telemetry as _st
            _a, _agents, _l = _st.load_scenario(copy.deepcopy(compose))
            jam_ids = [x["id"] for x in _agents if x.get("jammer")]
            by_id = {x.get("id"): x for x in compose.setdefault("agents", [])}
            for jid in jam_ids:
                entry = by_id.get(jid)
                if entry is None:
                    entry = {"id": jid}
                    compose["agents"].append(entry)
                entry["jammer"] = {
                    "tx_power": {"value": dbm, "unit": "dBm",
                                 "source": "experiment cell"}}
            if not jam_ids:
                self.bar.setFormat("this cell has no jammer to set")
        except Exception as exc:                       # noqa: BLE001
            self.bar.setFormat(f"could not compose that cell: {exc}")
            return

        self.plot.highlight = (r.get("authority"), r.get("routing"))
        self.plot.render_chart()
        if self.console is not None:
            self.console.run_composition(
                compose, mission=cfg.get("mission") or "advance",
                title=r.get("cell", ""), seed=int(r.get("seed") or 1),
                warmup_s=float(cfg.get("warmup_s") or 0.0),
                jam_dbm=dbm)
        self.bar.setFormat(f"running {r.get('cell')} live in the Console")



# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class Console(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Deadband Console")
        self.resize(1420, 880)
        self.report = None
        self.doc = None          # the round-trip document we edit and save
        self.path = None
        self.doc = None          # nothing loaded yet - Setup composes a run
        self.resolved = None
        self.report = None
        self.proc = None
        self.ws = None
        self.dirty = False
        self.selected_agent = None
        self._mission_name = None   # None -> Mission tree header reads UNASSIGNED
        self.latest = {}         # newest telemetry frame, by agent id
        self.frames = []         # every frame of the current run, for Results
        self._known_series = []
        self._buf = ""

        self.viewport = Viewport()
        self.viewport.on_pick = self.select_agent_by_id
        self._build_centre()
        self._build_docks()
        self._build_menu()

        # Deliberately BLANK at startup. A run is COMPOSED, not opened:
        # Setup tab -> choose a scene (the world appears) -> choose a fleet
        # (a spawn dialog places the agents) -> press Play -> in the
        # terminal, SETMISSION <name> -> blue launch. File > Open remains
        # for legacy self-contained files only.
        self._setup_scene = None
        self._setup_fleet = None       # blue fleet name
        self._setup_red = None         # red fleet name
        self._doctrines = {}           # {agent_id: hold|intent} from Setup
        self._fleet_docs = {}          # {side: fleet dict} built, NOT saved
        self._spawns = {}              # {agent_id: {x,y,z,yaw}} across sides
        self._blue_ids = set()
        self._red_ids = set()
        self.statusBar().showMessage(
            "Setup: choose a scene, then a fleet")

    # -- construction -------------------------------------------------------

    def _build_centre(self):
        bar = QWidget()
        row = QHBoxLayout(bar)
        row.setContentsMargins(6, 6, 6, 6)
        row.setSpacing(4)

        # View is a dropdown rather than three buttons: it is one choice from a
        # closed set, which is what a combo box is for, and it frees the width.
        row.addWidget(QLabel("View"))
        self.view_combo = QComboBox()
        self.view_combo.addItems([Viewport.TOP, Viewport.FRONT,
                                  Viewport.SIDE, Viewport.ISO])
        self.view_combo.currentTextChanged.connect(self.set_view)
        self.view_combo.setMinimumWidth(90)
        self.view_combo.setToolTip(
            "TOP looking down, FRONT and SIDE elevations, ISO from above")
        row.addWidget(self.view_combo)

        def tool(symbol, tip, slot):
            b = QPushButton(symbol)
            b.setToolTip(tip)
            b.setFixedSize(30, 26)
            b.setObjectName("tool")
            if slot is not None:
                b.clicked.connect(slot)
            row.addWidget(b)
            return b

        row.addSpacing(10)
        tool("\u2b1a", "Fit the arena to the window", self.viewport.reset_view)
        axes_btn = tool("\u22a2", "Show measured axes with metre ticks", None)
        axes_btn.setCheckable(True)
        axes_btn.clicked.connect(
            lambda on: (setattr(self.viewport, "show_axes", on),
                        self.viewport.update()))
        row.addSpacing(16)
        self.run_button = tool(
            "\u25b6", "Run. With the ROS 2 source this also starts the nodes "
            "and records a bag.", self.toggle_run)
        tool("\u21bb", "Reset: clear the run and wait for play", self.restart_run)
        row.addSpacing(16)
        tool("\u2913", "Save the scenario file, keeping its comments",
             self.save_scenario)

        row.addSpacing(24)
        row.addWidget(QLabel("Source"))
        self.source_combo = QComboBox()
        self.source_combo.addItems(["Stub (no ROS)", "ROS 2 bridge"])
        self.source_combo.setMinimumWidth(130)
        self.source_combo.setToolTip(
            "Stub: a local process, nothing to install.\n"
            "ROS 2 bridge: connect to deadband_ros running in WSL.")
        row.addWidget(self.source_combo)
        row.addStretch(1)

        # The centre swaps between the world view and the plots, driven by
        # which build tab is open. Results is a different question, so it gets
        # the whole canvas rather than being squeezed into a corner.
        self.plots = PlotArea()
        self.stack = QStackedWidget()
        self.stack.addWidget(self.viewport)
        self.stack.addWidget(self.plots)

        # Timeline. Scrubbing drives BOTH the plot cursor and the world view,
        # so you can find a moment on a graph and immediately see where every
        # agent was when it happened. That pairing is the point; a scrubber that
        # only moved a line on a chart would not be worth the space.
        tl = QWidget()
        tlrow = QHBoxLayout(tl)
        tlrow.setContentsMargins(8, 4, 8, 6)
        self.time_label = QLabel("no run")
        self.time_label.setMinimumWidth(150)
        self.timeline = QSlider(Qt.Horizontal)
        self.timeline.setEnabled(False)
        self.timeline.setToolTip(
            "Scrub the recorded run. The world view follows the cursor.")
        self.timeline.valueChanged.connect(self.on_scrub)
        self.play_button = QPushButton("\u25b6")
        self.play_button.setToolTip("Play back the recorded run")
        self.play_button.setFixedSize(30, 24)
        self.play_button.setCheckable(True)
        self.play_button.clicked.connect(self.toggle_playback)

        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["0.25x", "0.5x", "1x", "2x", "4x",
                           "8x", "16x"])
        self.speed_combo.setCurrentText("1x")
        self.speed_combo.setToolTip("Playback speed")
        self.speed_combo.setFixedWidth(66)

        self.live_button = QPushButton("Live")
        self.live_button.setCheckable(True)
        self.live_button.setChecked(True)
        self.live_button.setToolTip("Follow the run as it happens")
        self.live_button.clicked.connect(self.on_live_toggled)
        tlrow.addWidget(QLabel("Timeline"))
        tlrow.addWidget(self.timeline, 1)
        tlrow.addWidget(self.time_label)
        tlrow.addWidget(self.play_button)
        tlrow.addWidget(self.speed_combo)
        tlrow.addWidget(self.live_button)

        # BAND STRIP - one button per frequency in the scene. Selecting a band
        # shows only that band's links (or, for GNSS, foregrounds the drift);
        # "All" shows everything. A band button turns ORANGE while a jammer is
        # emitting on it, so what is being jammed, and when, is evident at a
        # glance. See docs/band-strip.md.
        self.band_strip = QWidget()
        bsrow = QHBoxLayout(self.band_strip)
        bsrow.setContentsMargins(8, 2, 8, 2)
        bsrow.setSpacing(4)
        bsrow.addWidget(QLabel("Band"))
        self.band_group = QButtonGroup(self)
        self.band_group.setExclusive(True)
        self._band_buttons = {}          # key -> QPushButton
        self._bandrow = bsrow
        bsrow.addStretch(1)
        self._rebuild_band_strip()

        holder = QWidget()
        lay = QVBoxLayout(holder)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(bar)
        lay.addWidget(self.stack, 1)
        lay.addWidget(self.band_strip)
        lay.addWidget(tl)
        self.setCentralWidget(holder)

    def _build_docks(self):
        # Left: the three-tab workflow. Left to right is the order you build a
        # study in: where it happens, what is in it, what goes wrong.
        self.tab_env = QTreeWidget()
        # No header label: the selected sidebar tab already names the panel,
        # so repeating it wastes a row of a narrow panel.
        self.tab_env.setHeaderHidden(True)
        self.tab_scn = QTreeWidget()
        self.tab_scn.setHeaderHidden(True)
        # Two questions, two trees. OVERVIEW answers "what is each agent" -
        # system, network, agents, and each agent's equipment (sensors). MISSION
        # answers "what is each agent doing" - the same hierarchy down to agents,
        # then their objectives, and nothing about hardware.
        self.tab_msn = QTreeWidget()
        self.tab_msn.setHeaderHidden(True)
        for t in (self.tab_env, self.tab_scn, self.tab_msn):
            t.itemSelectionChanged.connect(self.on_select)
            # The default indent stacks five levels deep off the right edge of a
            # narrow panel. Networks > blue > Agents > car1 > lidar has to fit.
            t.setIndentation(12)
        # Double-click an agent or its objective row -> a short popup describing
        # what that agent is doing right now.
        self.tab_scn.itemDoubleClicked.connect(self.on_overview_double_click)
        self.tab_msn.itemDoubleClicked.connect(self.on_overview_double_click)

        # SETUP - where a run is composed. Scene first (the world appears),
        # then fleet (a spawn dialog places the agents). The mission is NOT
        # set here: a mission is a COMMAND, issued from the terminal
        # (SETMISSION <name>) once the run is up, before `blue launch`.
        setup = QWidget()
        slay = QVBoxLayout(setup)
        slay.setContentsMargins(6, 6, 6, 6)
        slay.setSpacing(4)

        # THE TAB STARTS BLANK. One question, asked first, because the answer
        # decides what everything else on this tab MEANS - a sandbox picks one
        # architecture and issues its mission live; an experiment sweeps
        # architectures and must decide everything before it starts. Showing
        # both sets of controls at once, greyed, made it look as though the
        # difference were cosmetic. It is not.
        slay.addWidget(QLabel("Run"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems([
            "(choose a run type)",
            "Sandbox - one run, watched live",
            "Experiment - many runs, swept"])
        self.mode_combo.setToolTip(
            "Sandbox    one run you watch and can interfere with: retask\n"
            "           mid-run, tune the jammer live, and issue the mission\n"
            "           from the terminal - where whether the order can even\n"
            "           get through is part of what you are watching.\n"
            "Experiment many runs, headless, nothing interactive. Every\n"
            "           choice is made here, before it starts.")
        self.mode_combo.activated.connect(self._on_mode_chosen)
        slay.addWidget(self.mode_combo)

        # ---- COMPOSE: shown for both run types -----------------------------
        self.compose_box = QWidget()
        clay = QVBoxLayout(self.compose_box)
        clay.setContentsMargins(0, 6, 0, 0)
        clay.setSpacing(4)
        clay.addWidget(QLabel("Scene"))
        self.scene_combo = QComboBox()
        self.scene_combo.setToolTip("The world: arena, radio background, "
                                    "named points. scenes/*.yaml")
        self.scene_combo.activated.connect(self.on_scene_chosen)
        clay.addWidget(self.scene_combo)
        clay.addWidget(QLabel("Blue fleet (friendly)"))
        self.fleet_combo = QComboBox()
        self.fleet_combo.setToolTip(
            "Composed from agents/ hardware. Choosing one asks where to spawn "
            "them and what doctrine each follows.")
        self.fleet_combo.setEnabled(False)
        self.fleet_combo.activated.connect(
            lambda i: self.on_fleet_chosen(i, "blue"))
        clay.addWidget(self.fleet_combo)
        clay.addWidget(QLabel("Red fleet (adversary)"))
        self.red_combo = QComboBox()
        self.red_combo.setToolTip("Jammers and, later, spoofers. Spawned "
                                  "separately from blue. Optional.")
        self.red_combo.setEnabled(False)
        self.red_combo.activated.connect(
            lambda i: self.on_fleet_chosen(i, "red"))
        clay.addWidget(self.red_combo)
        savrow = QHBoxLayout()
        for _side, _lab in (("blue", "Save blue fleet as..."),
                            ("red", "Save red fleet as...")):
            b = QPushButton(_lab)
            b.setToolTip("Write the fleet built this session to "
                         "fleets/<name>.yaml. Nothing is saved until you "
                         "press this.")
            b.clicked.connect(lambda _c=False, sd=_side: self.save_fleet_as(sd))
            savrow.addWidget(b)
        clay.addLayout(savrow)
        slay.addWidget(self.compose_box)
        self.compose_box.setVisible(False)

        # ---- ARCHITECTURE: both run types, but different meanings ----------
        # SWEEP_ALL is the fourth option. In a sandbox it is not offered -
        # you cannot watch three authorities at once. In an experiment it is
        # the default, because the 3x3 IS the framework's claim.
        self.arch_box = QWidget()
        alay = QVBoxLayout(self.arch_box)
        alay.setContentsMargins(0, 6, 0, 0)
        alay.setSpacing(4)
        alay.addWidget(QLabel("Command authority (who decides)"))
        self.auth_combo = QComboBox()
        self.auth_combo.setToolTip(
            "centralized    one coordinator decides for everyone.\n"
            "decentralized  every agent decides for itself.\n"
            "hierarchical   agents answer to a squad leader, leaders up.\n"
            "all            sweep all three (experiment only).")
        self.auth_combo.activated.connect(lambda _i: self._on_arch_chosen())
        alay.addWidget(self.auth_combo)
        alay.addWidget(QLabel("Routing (how packets travel)"))
        self.route_combo = QComboBox()
        self.route_combo.setToolTip(
            "star    every agent to the hub, no peer links.\n"
            "mesh    everything to everything; authority can relay.\n"
            "tiered  the squad meshes internally, only the leader talks up.\n"
            "all     sweep all three (experiment only).")
        self.route_combo.activated.connect(lambda _i: self._on_arch_chosen())
        alay.addWidget(self.route_combo)
        self.lbl_arch = QLabel("")
        self.lbl_arch.setObjectName("hint")
        self.lbl_arch.setWordWrap(True)
        alay.addWidget(self.lbl_arch)
        slay.addWidget(self.arch_box)
        self.arch_box.setVisible(False)

        # ---- SANDBOX ONLY --------------------------------------------------
        self.sandbox_box = QWidget()
        sblay = QVBoxLayout(self.sandbox_box)
        sblay.setContentsMargins(0, 6, 0, 0)
        sblay.setSpacing(4)
        self.lbl_mission = QLabel("Mission: UNASSIGNED")
        self.lbl_mission.setToolTip(
            "Issued from the terminal once the run is started:\n"
            "    SETMISSION <name>\n"
            "then `blue launch`. Results are titled by this name.")
        sblay.addWidget(self.lbl_mission)
        hint = QLabel("Press Play, then in the terminal:\n"
                      "  SETMISSION <name>\n  blue launch")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        sblay.addWidget(hint)
        slay.addWidget(self.sandbox_box)
        self.sandbox_box.setVisible(False)

        # ---- EXPERIMENT ONLY -----------------------------------------------
        self.exp_box = QWidget()
        elay = QVBoxLayout(self.exp_box)
        elay.setContentsMargins(0, 6, 0, 0)
        elay.setSpacing(4)
        elay.addWidget(QLabel("Mission"))
        self.exp_mission = QComboBox()
        self.exp_mission.setToolTip(
            "Issued to every vehicle when each run starts. There is nobody "
            "to type at a headless run, so it is chosen here.")
        elay.addWidget(self.exp_mission)
        elay.addWidget(QLabel("Jammer advantage P_j/P_t to sweep (dB)"))
        prow = QHBoxLayout()
        self.exp_powers = []
        for db, on in ((0, True), (10, True), (20, True), (30, False)):
            b = QCheckBox(f"{db:+d}")
            b.setChecked(on)
            b.setToolTip(
                "The jammer's power RELATIVE to the fleet's own radios. A "
                "ratio, so the absolute powers cancel out of the physics and "
                "the result holds for any radio at any scale.")
            b.stateChanged.connect(lambda *_: self._refresh_run_count())
            prow.addWidget(b)
            self.exp_powers.append((db, b))
        prow.addStretch(1)
        elay.addLayout(prow)
        self.lbl_runs = QLabel("")
        self.lbl_runs.setObjectName("hint")
        self.lbl_runs.setWordWrap(True)
        elay.addWidget(self.lbl_runs)
        expb = QPushButton("Run the experiment...")
        expb.setToolTip("Sweep, then double-click any result to watch that "
                        "exact run play out.")
        expb.clicked.connect(self.open_experiment)
        elay.addWidget(expb)
        slay.addWidget(self.exp_box)
        self.exp_box.setVisible(False)

        # The scene tree (arena + background conditions) lives under the
        # pickers - it describes what Setup composed.
        slay.addWidget(self.tab_env, 1)
        self._refresh_setup_lists()

        # CONTESTED - everything degrading the spectrum, in one tree:
        # the scene's declared BASELINE, the EMITTERS transmitting into it
        # right now (jammers as agents), and the noise floor each agent
        # ACTUALLY experiences. The gap between baseline and experienced is
        # jamming, as a number. Attack injection (spoofing, mobile jammers)
        # hangs off this next. See docs/contested-background.md.
        # NETWORK - command structure, live: who decides for each agent and
        # whether they can be reached (command_authority), and what the
        # topology MEASURES as versus what it was declared (observed_topology
        # vs the routing field). This is netcheck.py's tables, in the GUI -
        # the thing that has been computed every frame but invisible.
        self.tab_network = QTreeWidget()
        self.tab_network.setHeaderHidden(True)
        self.tab_network.setIndentation(12)
        network = self.tab_network

        self.tab_contested = QTreeWidget()
        self.tab_contested.setHeaderHidden(True)
        self.tab_contested.setIndentation(12)
        self.tab_contested.itemDoubleClicked.connect(self.on_contested_edit)
        contested = self.tab_contested

        # Results: tick series to plot them. Populated from the recorded run.
        results = QWidget()
        rlay = QVBoxLayout(results)
        rlay.setContentsMargins(0, 0, 0, 0)
        rlay.setSpacing(0)

        # Two lists, deliberately. KEY shows only what the study is actually
        # measuring; ALL is everything, for when a hunch needs chasing. A
        # results page that offers thirty equally-weighted series is a page that
        # has no opinion about what the research is for.
        top = QWidget()
        toprow = QHBoxLayout(top)
        toprow.setContentsMargins(4, 4, 4, 4)
        toprow.setSpacing(4)
        self.series_mode = QComboBox()
        self.series_mode.addItems(["Key metrics", "All series"])
        self.series_mode.currentTextChanged.connect(lambda _: self.rebuild_series_tree())
        toprow.addWidget(self.series_mode, 1)
        rlay.addWidget(top)

        self.series_tree = SeriesTree()
        self.series_tree.setHeaderLabels(["Series"])
        self.series_tree.setIndentation(12)
        self.series_tree.itemDoubleClicked.connect(
            lambda it, _c: self.plots.add_to_first_empty(it.data(0, Qt.UserRole)))
        rlay.addWidget(self.series_tree, 1)
        hint = QLabel("Drag a series onto a plot, or double-click it.\n"
                      "Right-click a plot to split it.")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        rlay.addWidget(hint)

        export = QPushButton("Export run to CSV")
        export.clicked.connect(self.export_csv)
        rlay.addWidget(export)

        # Comms. Two questions a researcher asks constantly and the map cannot
        # answer once there are more than a handful of agents: what is on the
        # air, and which links are actually carrying traffic. Modelled on how
        # spectrum monitoring is presented in the open literature - an emitter
        # list (who is transmitting, on what band, at what power) beside a link
        # state table - rather than on any one product.
        comms = QWidget()
        clay = QVBoxLayout(comms)
        clay.setContentsMargins(0, 0, 0, 0)
        clay.setSpacing(0)

        clay.addWidget(QLabel("  Emitters"))
        self.emitters = QTableWidget(0, 4)
        self.emitters.setHorizontalHeaderLabels(["Agent", "Band", "Tx", "Link type"])
        self.emitters.horizontalHeader().setStretchLastSection(True)
        self.emitters.verticalHeader().setVisible(False)
        clay.addWidget(self.emitters, 1)

        clay.addWidget(QLabel("  Links"))
        self.linktable = QTableWidget(0, 5)
        self.linktable.setHorizontalHeaderLabels(["From", "To", "Dist m", "Quality", "State"])
        self.linktable.horizontalHeader().setStretchLastSection(True)
        self.linktable.verticalHeader().setVisible(False)
        clay.addWidget(self.linktable, 1)

        self.tabs = QTabWidget()
        # The sidebar renders in the order added, top to bottom: Setup
        # (compose the run: scene, fleet, spawns), Overview (what things
        # are), Mission (what they are doing), then the analysis tabs
        # Comms, Contested, Results.
        self.tabs.addTab(setup, "Setup")
        self.tabs.addTab(self.tab_scn, "Overview")
        self.tabs.addTab(self.tab_msn, "Mission")
        self.tabs.addTab(network, "Network")
        self.tabs.addTab(comms, "Comms")
        self.tabs.addTab(contested, "Contested")
        self.tabs.addTab(results, "Results")
        self.tabs.currentChanged.connect(self.on_tab_changed)
        # Tabs down the left edge rather than across the top: the labels stack
        # vertically, the panel stays narrow, and the section you are in is the
        # only one taking horizontal space.
        self.tabs.setTabPosition(QTabWidget.West)
        d = QDockWidget("Build")
        d.setWidget(self.tabs)
        d.setMinimumWidth(150)
        d.setMaximumWidth(400)
        self.addDockWidget(Qt.LeftDockWidgetArea, d)
        self.resizeDocks([d], [215], Qt.Horizontal)

        # Right: properties above, sensor view below.
        self.props = QTableWidget(0, 4)
        self.props.setHorizontalHeaderLabels(["Property", "Value", "Unit", "Source"])
        self.props.horizontalHeader().setStretchLastSection(True)
        self.props.verticalHeader().setVisible(False)
        self.props.itemChanged.connect(self.on_prop_edited)
        d_props = QDockWidget("Properties — no agent selected")
        self.d_props = d_props
        d_props.setWidget(self.props)
        self.addDockWidget(Qt.RightDockWidgetArea, d_props)

        self.sensor = SensorView()
        d_sensor = QDockWidget("Sensor view")
        d_sensor.setWidget(self.sensor)
        self.addDockWidget(Qt.RightDockWidgetArea, d_sensor)
        self.splitDockWidget(d_props, d_sensor, Qt.Vertical)

        # Bottom: log and publications side by side.
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setFont(QFont("Consolas", 9))

        # A flat table of every topic is unreadable past a few agents, and this
        # framework exists to have many. One expandable folder per agent.
        self.pubs = QTreeWidget()
        self.pubs.setHeaderLabels(["Topic", "Type", "Rate Hz"])
        self.pubs.setIndentation(12)

        # Outputs are produced by the run and cannot be closed; terminals are
        # opened by the user and can be. Mixing them in one strip put a close
        # button on the Log, which invited exactly the wrong action.
        outputs = QTabWidget()
        outputs.addTab(self.log, "Log")
        outputs.addTab(self.pubs, "Publications")

        self.terminals = QTabWidget()
        self.terminals.setTabsClosable(True)
        self.terminals.tabCloseRequested.connect(self.close_terminal)
        self.terminals.setContextMenuPolicy(Qt.CustomContextMenu)
        self.terminals.customContextMenuRequested.connect(self.terminal_menu)
        self.shells = []
        self._cells_built = False

        bottom = QTabWidget()
        bottom.addTab(outputs, "Output")
        bottom.addTab(self.terminals, "Terminals")
        self.bottom = bottom
        self._build_cells()
        outputs.setDocumentMode(True)
        self.terminals.setDocumentMode(True)
        bottom.setDocumentMode(True)
        bottom.setObjectName("chrome")
        # One framed block, so the divide between the views above and the
        # output below reads as a section rather than as loose tabs.
        frame = QWidget()
        frame.setObjectName("bottomframe")
        framelay = QVBoxLayout(frame)
        framelay.setContentsMargins(1, 1, 1, 1)
        framelay.setSpacing(0)
        framelay.addWidget(bottom)

        d = QDockWidget("Output")
        d.setWidget(frame)
        # The dock's own title bar just repeated the word "Output" above a tab
        # that already said it. Replacing it with an empty widget reclaims the
        # row and lets the tabs act as the title.
        d.setTitleBarWidget(QWidget())
        self.addDockWidget(Qt.BottomDockWidgetArea, d)

    def _build_menu(self):
        m = self.menuBar().addMenu("&File")
        for label, fn in (("&Open file (advanced)...", self.open_dialog),
                          ("&Save", self.save_scenario),
                          ("&Reload", self.reload_scenario)):
            a = QAction(label, self)
            a.triggered.connect(fn)
            m.addAction(a)
        m.addSeparator()
        a = QAction("E&xit", self)
        a.triggered.connect(self.close)
        m.addAction(a)

        # EXPERIMENT is its own window, not another tab: many runs, headless,
        # nothing to interfere with, and it needs the whole canvas for the
        # result. Ctrl+E from anywhere.
        e = self.menuBar().addMenu("&Experiment")
        a = QAction("&Run an experiment...", self)
        a.setShortcut("Ctrl+E")
        a.triggered.connect(self.open_experiment)
        e.addAction(a)

    # -- setup: compose a run ----------------------------------------------

    def _run_stem(self):
        """The title every kept result carries: scene_fleet_mission - all of
        what the run WAS, so a results folder needs no decoder. Fields that
        do not exist yet are simply absent; characters Windows filenames
        cannot hold are replaced. The caller appends the date-time."""
        doc = self.doc or {}
        scene = self._setup_scene or doc.get("scene") or doc.get("map") \
            or (self.path.stem if self.path else None)
        fleet = self._setup_fleet or doc.get("fleet")
        mission = self._mission_name
        if mission == "assigned (live)":
            mission = None
        parts = [str(x) for x in (scene, fleet, mission) if x]
        stem = "_".join(parts) or "run"
        return "".join(c if c.isalnum() or c in "-_" else "_" for c in stem)

    def _lock_setup(self, locked):
        """The Setup tab is how a run is COMPOSED; while one is actually
        running, recomposing under it is a crash waiting to happen (the sim
        holds the old world, the Console loads a new one). Lock the tab for
        the duration; everything else stays live."""
        for i in range(self.tabs.count()):
            if self.tabs.tabText(i) == "Setup":
                if locked and self.tabs.currentIndex() == i:
                    self.tabs.setCurrentIndex(i + 1)   # step off it first
                self.tabs.setTabEnabled(i, not locked)
                self.tabs.setTabToolTip(
                    i, "Locked while a run is up - Stop to recompose"
                       if locked else "")
                break

    def _refresh_setup_lists(self):
        """(Re)list scenes/ and fleets/ into the Setup dropdowns."""
        for combo, folder, placeholder in (
                (self.scene_combo, "scenes", "(choose a scene)"),
                (self.fleet_combo, "fleets", "(choose a blue fleet)"),
                (self.red_combo, "fleets", "(none)")):
            current = combo.currentText()
            combo.blockSignals(True)
            combo.clear()
            combo.addItem(placeholder)
            for f in sorted((REPO_ROOT / folder).glob("*.yaml")):
                combo.addItem(f.stem)
            if folder == "fleets":
                combo.addItem(CUSTOM_FLEET_ENTRY)
            i = combo.findText(current)
            if i > 0:
                combo.setCurrentIndex(i)
            combo.blockSignals(False)

    def on_scene_chosen(self, index):
        if index <= 0:
            return
        self._setup_scene = self.scene_combo.currentText()
        # A new scene invalidates any placed fleet - spawns are coordinates
        # in the OLD world. Ask again rather than silently carrying them.
        self._setup_fleet = None
        self._setup_red = None
        self._spawns = {}
        self.fleet_combo.setEnabled(True)
        self.fleet_combo.setCurrentIndex(0)
        self.red_combo.setEnabled(True)
        self.red_combo.setCurrentIndex(0)
        self._compose_setup()
        self.statusBar().showMessage(
            f"Scene {self._setup_scene} - now choose a blue fleet")

    def on_fleet_chosen(self, index, side):
        combo = self.red_combo if side == "red" else self.fleet_combo
        if not self._setup_scene:
            return
        if index <= 0:                      # "(none)" - clear this side
            self._clear_side(side)
            self._compose_setup()
            return
        fleet = combo.currentText()
        built = None
        if fleet == CUSTOM_FLEET_ENTRY:
            fleet = self._build_custom_fleet(side)
            if not fleet:
                combo.setCurrentIndex(0)
                return
            # It exists only in memory, so it is not in the dropdown list. Show
            # it as the current text without pretending it is a saved file.
            built = self._fleet_docs.get(side)
            combo.setEditable(True)
            combo.setEditText(f"{fleet}  (unsaved)")
            combo.setEditable(False)
        if side == "red" and fleet == self._setup_fleet:
            self.say("Red and blue fleets must be different files.")
            combo.setCurrentIndex(0)
            return
        defaults = []
        if built is not None:
            defaults = built.get("agents") or []
        elif _resolve_mission is not None:
            try:
                fdoc = _resolve_mission(
                    str(REPO_ROOT / "fleets" / f"{fleet}.yaml"))
                defaults = fdoc.get("agents") or []
            except Exception as exc:
                self.say(f"cannot read fleet {fleet}: {exc}")
                return
        dlg = SpawnDialog(defaults, self, title=f"Spawn the {side} fleet")
        if dlg.exec() != QDialog.Accepted:
            combo.setCurrentIndex(0)
            return
        # Replace this side's spawns; keep the other side's.
        self._clear_side(side)
        if built is None:
            # A saved fleet was picked - drop whatever was built in memory for
            # this side, so the run never silently uses the wrong one.
            self._fleet_docs.pop(side, None)
        new_spawns = dlg.spawns()
        self._doctrines.update(dlg.doctrines())
        if side == "red":
            self._setup_red = fleet
            self._red_ids = set(new_spawns)
        else:
            self._setup_fleet = fleet
            self._blue_ids = set(new_spawns)
        self._spawns.update(new_spawns)
        self._compose_setup()
        both = " + ".join(x for x in (self._setup_fleet, self._setup_red) if x)
        self.statusBar().showMessage(
            f"{self._setup_scene} + {both} - press Play, then "
            f"SETMISSION <name> and blue launch")

    def _build_custom_fleet(self, side):
        """Open the fleet builder and hold the result IN MEMORY.

        NOTHING IS WRITTEN TO DISK. A fleet you are still experimenting with is
        not a fleet you have decided to keep, and a builder that saves on every
        press fills fleets/ with abandoned attempts - which is the exact
        problem emptying that folder was meant to solve. The composition is
        inlined into the run instead, and only "Save fleet as..." writes a
        file.

        Returns the display name, or None if cancelled.
        """
        dlg = CustomFleetDialog(side=side, parent=self)
        if dlg.exec() != QDialog.Accepted:
            return None
        name, doc = dlg.fleet_doc()
        if not doc.get("agents"):
            self.say("A fleet needs at least one agent.")
            return None
        self._fleet_docs[side] = doc
        self.say(f"Built {name} in memory ({len(doc['agents'])} agents, "
                 f"{side}) - NOT saved. Use 'Save fleet as...' to keep it.")
        return name

    def save_fleet_as(self, side="blue"):
        """Write the in-memory fleet for this side to fleets/<name>.yaml.

        The ONLY thing that creates a file in fleets/. Explicit, on purpose:
        that folder should contain exactly the fleets you decided were worth
        keeping and nothing else.
        """
        doc = (self._fleet_docs or {}).get(side)
        if not doc:
            self.say(f"No {side} fleet built in this session to save. "
                     f"Build one first (Setup -> {side} fleet -> "
                     f"{CUSTOM_FLEET_ENTRY}).")
            return
        import yaml as _yaml
        suggested = str(REPO_ROOT / "fleets" / f"{doc.get('name', side)}.yaml")
        path, _ = QFileDialog.getSaveFileName(
            self, f"Save {side} fleet", suggested, "Fleet (*.yaml)")
        if not path:
            return
        out = dict(doc)
        out["name"] = Path(path).stem
        try:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            Path(path).write_text(
                "# Saved from the Console's fleet builder.\n"
                "# A COMPOSITION of agents/ hardware: which vehicles, their\n"
                "# ids and where they start. It declares no authority, no\n"
                "# routing and no doctrine - those are picked in Setup, every\n"
                "# run, and are never baked in here.\n"
                "# Dimensions and performance come from agents/; anything\n"
                "# marked nominal is still nominal.\n"
                + _yaml.safe_dump(out, sort_keys=False), encoding="utf-8")
        except OSError as exc:
            self.say(f"cannot write fleet: {exc}")
            return
        self._refresh_setup_lists()
        self.say(f"Saved {Path(path).name} ({len(out['agents'])} agents)")


    def _clear_side(self, side):
        ids = getattr(self, "_red_ids" if side == "red" else "_blue_ids", set())
        for aid in ids:
            self._spawns.pop(aid, None)
        if side == "red":
            self._setup_red = None
            self._red_ids = set()
        else:
            self._setup_fleet = None
            self._blue_ids = set()
        # Doctrine is a per-agent Setup decision, so it is cleared with the
        # spawns it was taken alongside - a stale doctrine for an agent that
        # is no longer in the run is exactly the kind of ghost setting that
        # makes a result impossible to explain.
        for aid in list(getattr(self, "_doctrines", {})):
            if aid in ids:
                self._doctrines.pop(aid, None)


    def play_frames(self, frames, title=""):
        """Load a finished run into the viewport and scrub it on the timeline.

        Used by the experiment window: double-clicking a result row re-runs
        that cell and hands the frames here. It replaces the live stream's
        buffer, so it behaves exactly like a run you watched happen - the same
        timeline, the same map, the same plots.
        """
        if not frames:
            return
        self.frames = list(frames)
        self.plots.frames = self.frames
        self.plots.xlabel = "s"
        self.plots.shared_scale = False
        self.refresh_series_tree(self.frames[0])
        if hasattr(self, "live_button"):
            self.live_button.setChecked(False)
        self.timeline.blockSignals(True)
        self.timeline.setEnabled(True)
        self.timeline.setMaximum(len(self.frames) - 1)
        self.timeline.blockSignals(False)
        # LAND ON THE OUTCOME, NOT ON FRAME ZERO. Frame zero is before the
        # jammer is even armed, so every vehicle is blue, commanded and
        # unlabelled - which looks exactly like a replay that did not load.
        # The last frame is where the run ENDED: who was cut off, who was
        # held, how far each got. Scrub backwards to watch it happen.
        # START AT THE BEGINNING AND PLAY. The point of opening a run is to
        # WATCH it - the vehicles advancing, one dropping out, the rest
        # pushing past it. Landing on the last frame showed the outcome and
        # hid the mechanism, which is the interesting half.
        self.timeline.setValue(0)
        self.on_scrub(0)
        # SPEED IT UP, AND FIT THE VIEW. A 200 m advance at 1.5 m/s is a run
        # of well over a minute, and at 1x the vehicles creep - which reads as
        # "nothing is happening" rather than as a slow advance. Pick a rate
        # that plays the whole thing in roughly fifteen seconds, and reset the
        # view so the corridor is on screen rather than wherever the last run
        # left the pan.
        try:
            span = (self.frames[-1].get("sim_time_s", 0.0)
                    - self.frames[0].get("sim_time_s", 0.0))
            want = span / 15.0
            best = min((float(self.speed_combo.itemText(i).rstrip("x")), i)
                       for i in range(self.speed_combo.count())
                       if float(self.speed_combo.itemText(i).rstrip("x")) >= want) \
                if any(float(self.speed_combo.itemText(i).rstrip("x")) >= want
                       for i in range(self.speed_combo.count())) else None
            if best is not None:
                self.speed_combo.setCurrentIndex(best[1])
        except Exception:                              # noqa: BLE001
            pass
        self.viewport.reset_view()
        if hasattr(self, "play_button"):
            self.play_button.setChecked(True)
            self.toggle_playback(True)
        # The result lives in the main window, so bring it to the front - the
        # experiment window is usually covering it.
        self.raise_()
        self.activateWindow()
        if title:
            self.say(f"Opened {title} - {len(self.frames)} frames, showing the "
                     f"playing from the start. Drag the timeline to scrub.")

    # Metrics a swept result offers as plottable series. Ordered so the ones
    # people actually ask for come first in the tree.
    SWEEP_METRICS = ("penetration_m", "penetration_frac", "commanded_fraction",
                     "held_fraction", "belief_err_m", "track_err_m",
                     "worst_sinr_db", "worst_pdr", "arrived", "distance_m",
                     "ended_s")

    def sweep_frames(self, rows, xkey="jam_rel_db"):
        """Turn swept results into frames the existing plotting can draw.

        A sweep has the same SHAPE as a run - a value per step - so each
        swept parameter value becomes a frame whose sim_time_s IS that value,
        and each architecture becomes an agent whose fields are its metrics.
        That one substitution is what makes every plot in this application
        work on a sweep without a line of new drawing code.
        """
        xs = sorted({float(r[xkey]) for r in rows
                     if r.get(xkey) not in (None, "")})
        frames = []
        for x in xs:
            agents = []
            for r in rows:
                try:
                    if float(r.get(xkey)) != x:
                        continue
                except (TypeError, ValueError):
                    continue
                aid = f"{r.get('authority', '?')}_{r.get('routing', '?')}"
                body = {"id": aid, "platform": "result", "network": "blue",
                        "colour": ResultPlot.AUTH_COLOUR.get(
                            r.get("authority"), "#AAAAAA")}
                for k in self.SWEEP_METRICS:
                    if r.get(k) in (None, ""):
                        continue
                    try:
                        body[k] = float(r[k])
                    except (TypeError, ValueError):
                        pass
                agents.append(body)
            frames.append({"seq": len(frames), "sim_time_s": x,
                           "run_state": "sweep", "mission": "sweep",
                           "agents": agents, "links": [],
                           "arena": self.viewport.arena or {}})
        return frames

    def show_sweep_results(self, rows, xkey="jam_rel_db",
                           xlabel="P_j/P_t (dB)"):
        """Put a SWEPT result into the Results tab, using the plotting the
        Console already has.

        THE X AXIS WAS THE WHOLE PROBLEM. PlotPane draws against sim_time_s,
        so anything handed to it without a time is drawn on an axis of zero
        width - which renders as nothing at all, with no error, which is
        exactly what four attempts at a bespoke chart widget produced.

        The fix is not another widget. A sweep has the same SHAPE as a run -
        a value per step - so each swept parameter value becomes a frame whose
        `sim_time_s` IS that value, and each architecture becomes an agent
        whose fields are its metrics. Everything the Results tab can already
        do then works unchanged: right-click to split a pane, drag series in,
        overlay them, scrub the cursor.

        Read the x axis as the swept parameter, not as seconds. That is the
        one honest compromise, and it is worth it to reuse plotting that
        demonstrably works rather than to maintain a fifth version that does
        not.
        """
        if not rows:
            return
        frames = self.sweep_frames(rows, xkey)
        if not frames:
            return
        self.frames = frames
        self.plots.frames = frames
        self.plots.live = False
        self.plots.cursor = 0
        self.plots.xlabel = xlabel
        # Results holds many different metrics at once, so each keeps its own
        # scale here - the shared scale belongs to the headline chart, which
        # shows one metric across architectures.
        self.plots.shared_scale = False
        self._known_series = []            # force the tree to rebuild
        self.refresh_series_tree(frames[0])
        if hasattr(self, "series_mode"):
            self.series_mode.setCurrentText("All series")
        self.plots.refresh()
        self.timeline.blockSignals(True)
        self.timeline.setEnabled(True)
        self.timeline.setMaximum(max(0, len(frames) - 1))
        self.timeline.setValue(0)
        self.timeline.blockSignals(False)
        self.time_label.setText(f"swept: {xlabel}")
        # Results is a different question from the world, so it takes the
        # whole canvas - the same swap the tab already does.
        for i in range(self.tabs.count()):
            if self.tabs.tabText(i) == "Results":
                self.tabs.setCurrentIndex(i)
                break
        self.say(f"Swept result in Results: {len(rows)} runs, "
                 f"x axis is {xlabel}. Drag a series onto the plot; "
                 f"right-click a plot to split it.")

    def run_composition(self, compose, mission="advance", title="", seed=1,
                        warmup_s=0.0, jam_dbm=None):
        """Start a LIVE run of a composition handed in from elsewhere.

        This is how an experiment cell is opened: the sweep's own composition,
        with that cell's authority, routing, jammer power and seed, is written
        out and started through the ordinary run path. The result is a live
        run - every panel updating, every agent clickable, the timeline
        filling as it happens - not a recording being scrubbed. There is one
        run path in this application, and both the sandbox and an experiment
        replay go down it.
        """
        import yaml as _yaml
        self.stop_run()
        self._run_seed = int(seed)
        path = REPO_ROOT / "runs" / "replay_setup.yaml"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            "# Written by the Console to replay an experiment cell live.\n"
            "# Regenerated every time; safe to delete.\n"
            + _yaml.safe_dump(compose, sort_keys=False), encoding="utf-8")
        self.load_scenario(path)
        self.start_run()
        # The order goes in once the sim is up. It travels the same retask
        # spool a human types into, so a replayed run is commanded exactly the
        # way a sandbox run is - including being refused if the fleet cannot
        # be reached, which is itself part of what an experiment measures.
        from PySide6.QtCore import QTimer
        red = any(a.get("jammer") for a in (compose.get("agents") or []))

        def _order():
            self._send_setup_orders(mission, red=False)
        QTimer.singleShot(900, _order)
        # THE JAMMER ARMS AFTER THE WARM-UP, exactly as it does in the sweep.
        # Arming it at t=0 instead would have jammed the fleet before it had
        # moved, which is a different experiment from the one the table
        # scored - and the whole point of opening a cell is to watch THAT run.
        if red:
            QTimer.singleShot(int(900 + max(warmup_s, 0.0) * 1000),
                              lambda: self._send_setup_orders(None, red=True))
        self.say(f"Running {title or 'composition'} live - mission {mission}, "
                 f"seed {seed}"
                 + (f", jammer {jam_dbm:.0f} dBm arming at t={warmup_s:.0f}s"
                    if jam_dbm is not None else "") + ".")

    def _send_setup_orders(self, mission, red):
        """SETMISSION, then launch, down the ordinary command channel."""
        lines = ([] if mission is None
                 else [f"SETMISSION {mission}", "LAUNCH blue"])
        if red:
            lines.append("LAUNCH red")
        d = getattr(self, "_retask_dir", None) or (REPO_ROOT / "runs" / "retask")
        try:
            d.mkdir(parents=True, exist_ok=True)
            for i, ln in enumerate(lines):
                (d / f"cmd_{time.time_ns()}_{i}.txt").write_text(
                    ln + "\n", encoding="utf-8")
        except OSError as exc:
            self.say(f"could not issue the mission: {exc}")

    def open_experiment(self):
        """The experiment window: run a sweep, read the table, watch a run."""
        if getattr(self, "_expwin", None) is None:
            self._expwin = ExperimentWindow(self, self)
        self._expwin.show()
        self._expwin.raise_()

    SWEEP_ALL = "all  (sweep all three)"

    def _on_mode_chosen(self, _index):
        """Reveal only the controls this run type actually uses.

        The tab starts blank because the run type decides what everything else
        MEANS. A sandbox picks one architecture and issues its mission live,
        at the terminal, where whether the order can get through is part of
        what you are watching. An experiment sweeps architectures and has
        nobody to type at it, so the mission and the powers are decided here.
        Showing both sets greyed made that difference look cosmetic.
        """
        mode = self.mode_combo.currentIndex()      # 0 none, 1 sandbox, 2 exp
        self.compose_box.setVisible(mode > 0)
        self.arch_box.setVisible(mode > 0)
        self.sandbox_box.setVisible(mode == 1)
        self.exp_box.setVisible(mode == 2)
        if mode == 0:
            return
        experiment = mode == 2
        if experiment:
            self._refresh_missions()
        # `all` only exists in an experiment: you cannot watch three
        # authorities at once, and in a sweep it is the default because that
        # 3x3 is the framework's whole claim.
        for combo, opts in ((self.auth_combo, ["centralized", "decentralized",
                                               "hierarchical"]),
                            (self.route_combo, ["star", "mesh", "tiered"])):
            keep = combo.currentText()
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(opts + ([self.SWEEP_ALL] if experiment else []))
            i = combo.findText(keep)
            combo.setCurrentIndex(i if i >= 0
                                  else (combo.count() - 1 if experiment else 0))
            combo.blockSignals(False)
        self._on_arch_chosen()
        self._refresh_run_count()


    def _refresh_missions(self):
        cur = self.exp_mission.currentText()
        self.exp_mission.blockSignals(True)
        self.exp_mission.clear()
        for f in sorted((REPO_ROOT / "missions").glob("*.yaml")):
            self.exp_mission.addItem(f.stem)
        i = self.exp_mission.findText(cur or "advance")
        self.exp_mission.setCurrentIndex(max(i, 0))
        self.exp_mission.blockSignals(False)

    def _refresh_run_count(self):
        """The grid size, live, as you tick. Not a time estimate - an estimate
        is a guess you then have to defend; this is arithmetic."""
        if not hasattr(self, "lbl_runs") or self.mode_combo.currentIndex() != 2:
            return
        na = 3 if self.auth_combo.currentText() == self.SWEEP_ALL else 1
        nr = 3 if self.route_combo.currentText() == self.SWEEP_ALL else 1
        powers = [db for db, b in self.exp_powers if b.isChecked()]
        n = na * nr * len(powers)
        self.lbl_runs.setText(
            f"{n} runs  =  {na} authority x {nr} routing x {len(powers)} "
            f"power{'s' if len(powers) != 1 else ''}"
            if n else "Tick at least one power.")


    def _experiment_cfg(self):
        """The sweep configuration, built ENTIRELY from this Setup tab.

        No experiment YAML is written or read. The composition handed to the
        sweep is the same dict the sandbox would have run - scene, fleet,
        spawns and per-agent doctrine included - so an experiment cannot
        quietly differ from what you set up, which is the failure mode a
        separate config file invites.
        """
        composed = getattr(self, "_composed", None)
        if not composed:
            return None, "Compose a run in Setup first (scene, then fleets)."
        powers = [db for db, b in self.exp_powers if b.isChecked()]
        if not powers:
            return None, "Tick at least one jammer power to sweep."
        # Squads for the hierarchical and tiered cells. The first blue vehicle
        # leads - stated here rather than assumed silently, and replaced by a
        # squad column in the fleet table when that lands.
        ids = [a.get("id") for a in (composed.get("agents") or [])
               if a.get("network") == "blue" and not a.get("ghost")
               and not a.get("jammer")]
        squads = ({"alpha": {"leader": ids[0], "members": ids[1:]}}
                  if len(ids) >= 2 else {})
        name = "_".join(x for x in (self._setup_scene,
                                    self.exp_mission.currentText()) if x)
        return {
            "name": name or "experiment",
            "compose": copy.deepcopy(composed),
            "mission": self.exp_mission.currentText() or "advance",
            "goal": "FAR",
            "duration_s": 400.0, "warmup_s": 5.0, "rate_hz": 10.0,
            "stop_when_stalled": True, "stall_grace_s": 5.0,
            "squads": squads, "coordinator": "gcs",
            "leader_loss": "fallback",
            # Each axis is swept if its dropdown says `all`, and pinned to
            # the single value otherwise. One place to look, and the run count
            # above is the same arithmetic.
            "axes": {
                "authority": (["centralized", "decentralized", "hierarchical"]
                              if self.auth_combo.currentText() == self.SWEEP_ALL
                              else [self.auth_combo.currentText()]),
                "routing": (["star", "mesh", "tiered"]
                            if self.route_combo.currentText() == self.SWEEP_ALL
                            else [self.route_combo.currentText()]),
                "jam_rel_db": powers},
            # One seed: measured, not assumed. Nothing is stochastic unless a
            # GNSS-band jammer or a lidar fleet is in play, and then this
            # should grow. See experiments/penetration.yaml.
            "seeds": [1],
        }, None

    def _on_arch_chosen(self):
        """An architecture override, applied and reported.

        Recomposes the run immediately so the map redraws with the new
        topology - a routing change you cannot see is a routing change you
        cannot trust - and warns about the one combination that silently
        degenerates."""
        auth = self.auth_combo.currentText()
        route = self.route_combo.currentText()
        note = []
        if self.mode_combo.currentIndex() == 2:
            self._refresh_run_count()
        # A hierarchy over a star is not a hierarchy. Star carries no peer
        # links, so every squad member reaches its leader VIA the coordinator,
        # and its fallback goes to the same coordinator - which is identical
        # to centralized. It is a legitimate thing to configure and a real
        # finding that it collapses, so it is allowed and explained rather
        # than forbidden.
        if auth == "hierarchical" and route == "star":
            note.append("hierarchical over star: no squad links exist, so "
                        "every member reaches its leader via the coordinator. "
                        "This will behave as centralized.")
        self.lbl_arch.setText("  ".join(note))
        if self._setup_scene:
            self._compose_setup()

    def _arch_override(self):
        """{"blue": {...}} of whatever the operator has overridden, or {}."""
        over = {}
        auth = getattr(self, "auth_combo", None)
        route = getattr(self, "route_combo", None)
        # ALWAYS written, never "whatever the file said". A fleet declares no
        # command policy at all now, so if these were not written the model
        # would fall back to an invisible default - and an invisible default
        # is the thing that made the routing axis silently do nothing for a
        # whole sweep. What the dropdown shows is what runs.
        # `all` is a SWEEP instruction, never a value: it must never reach
        # the model, which would not know what to do with it.
        if auth is not None and auth.currentText() != self.SWEEP_ALL:
            # `topology` is the legacy alias command_authority() falls back on.
            over["authority"] = over["topology"] = auth.currentText()
        if route is not None and route.currentText() != self.SWEEP_ALL:
            over["routing"] = route.currentText()
        if over.get("authority") == "hierarchical" or \
                over.get("routing") == "tiered":
            # A hierarchy needs squads to find a leader in, and tiered routing
            # needs squads to partition on. Until the fleet table can express
            # squad membership, the first blue vehicle leads the rest - stated
            # here rather than assumed silently.
            ids = [a for a in sorted(self._blue_ids or [])
                   if a not in ("gcs",)]
            if len(ids) >= 2:
                over["squads"] = {"alpha": {"leader": ids[0],
                                            "members": ids[1:]}}
                over.setdefault("leader_loss", "fallback")
        return {"blue": over} if over else {}

    def _compose_setup(self):
        """Write the composed run (scene + blue fleet + red fleet + spawn
        overrides) to runs/current_setup.yaml and load it. A FILE,
        deliberately: the whole existing pipeline (Play, ROS launch, save,
        reload) takes a path, and a composed run should be as diffable and
        re-runnable as any other."""
        import yaml as _yaml
        # A fleet is either a SAVED FILE (referenced by name) or one BUILT IN
        # MEMORY and never written (inlined here in full). Inlining keeps the
        # composed run completely self-describing either way: current_setup
        # always says what actually ran, whether or not you chose to keep the
        # fleet afterwards.
        named, inline_agents, inline_nets = [], [], {}
        for side, fname in (("blue", self._setup_fleet),
                            ("red", self._setup_red)):
            if not fname:
                continue
            built = (self._fleet_docs or {}).get(side)
            if built is not None:
                inline_agents += copy.deepcopy(built.get("agents") or [])
                inline_nets.update(copy.deepcopy(built.get("networks") or {}))
            else:
                named.append(fname)
        parts = [self._setup_scene] + [f for f in (self._setup_fleet,
                                                   self._setup_red) if f]
        doc = {"spec_version": 0.1, "name": " + ".join(p for p in parts if p),
               "scene": self._setup_scene}
        if named:
            doc["fleets"] = named
        # ARCHITECTURE OVERRIDE from the Setup pickers. Written into the
        # composed run, so it is diffable, re-runnable and lands in the CSV's
        # meta sidecar. _overlay merges one level into each network, so blue
        # keeps its coordinator, band and radios.
        arch = self._arch_override()
        nets = dict(inline_nets)
        for k, v in (arch or {}).items():
            nets[k] = {**(nets.get(k) or {}), **v}
        if nets:
            doc["networks"] = nets
        if self._spawns:
            # DOCTRINE rides with the spawn, because both are per-agent
            # decisions taken in the same dialog. _overlay merges agent
            # entries by id, so this adds on_link_loss without disturbing the
            # hardware the fleet declared.
            bodies = {a.get("id"): a for a in inline_agents}
            doc["agents"] = [
                dict(copy.deepcopy(bodies.get(aid, {})),
                     **dict({"id": aid, "pose": pose},
                            **({"on_link_loss": self._doctrines[aid]}
                               if aid in getattr(self, "_doctrines", {})
                               else {})))
                for aid, pose in self._spawns.items()]
        elif inline_agents:
            doc["agents"] = inline_agents
        # Kept so the experiment builder can hand the EXACT composition the
        # sandbox would have run to the sweep - same scene, same fleet, same
        # spawns, same per-agent doctrine. One source of truth, so an
        # experiment can never quietly differ from what you set up.
        self._composed = copy.deepcopy(doc)
        path = REPO_ROOT / "runs" / "current_setup.yaml"
        path.parent.mkdir(parents=True, exist_ok=True)
        header = ("# Composed by the Console's Setup tab - scene + fleets + "
                  "spawn overrides.\n# Regenerated on every Setup change; "
                  "safe to delete.\n")
        path.write_text(header + _yaml.safe_dump(doc, sort_keys=False),
                        encoding="utf-8")
        self.load_scenario(path)

    # -- scenario -----------------------------------------------------------

    def open_dialog(self):
        # Open at the repo root, not inside scenarios/ - a run is now a SCENE
        # plus a MISSION, and both folders (plus legacy scenarios/) need to be
        # one click away rather than up-a-level.
        path, _ = QFileDialog.getOpenFileName(
            self, "Open mission or scene", str(REPO_ROOT),
            "Deadband files (*.yaml *.yml)")
        if path:
            self.load_scenario(Path(path))

    def reload_scenario(self):
        if self.path:
            self.load_scenario(self.path)

    def load_scenario(self, path: Path):
        self.path = path
        # A new scenario is a fresh run. Old output sitting there - retask
        # confirmations, LAUNCH/HALT/REMISSION log lines, errors - from
        # whatever was open before is actively misleading once it's next to
        # a different scenario, not just clutter: it reads as evidence about
        # THIS run when it's actually about the last one. Clear the Log and
        # every open Terminal tab before loading.
        self.log.clear()
        for shell in getattr(self, "shells", []):
            shell.out.clear()
        # A fresh file means nothing has been ASSIGNED yet - even if the
        # file bakes objectives straight onto its agents (every legacy
        # scenarios/ file does), that isn't a runtime assignment. The
        # Mission tree header reads UNASSIGNED until SETMISSION names it.
        self._mission_name = None
        if hasattr(self, "lbl_mission"):
            self.lbl_mission.setText("Mission: UNASSIGNED")
        # Validate with the framework's own loader...
        self.report = spec.load(path)
        # ...but hold an editable copy that keeps the file's comments intact.
        if _RT:
            with open(path, encoding="utf-8") as fh:
                self.doc = _RT.load(fh)
        else:
            self.doc = getattr(self.report, "doc", None)
            self.say("NOTE   ruamel.yaml not installed - saving will drop the "
                     "comments from this file. Install it with: py -m pip install ruamel.yaml")

        self.say(f"Loaded {path.name}")
        # The RESOLVED view: map merged with mission, agents filled in, the same
        # thing the simulator runs. The tree and the scene draw from this so a
        # split mission is not an empty screen. Edits still go to self.doc (the
        # raw file); this is display-only and rebuilt on every load.
        self.resolved = None
        if _resolve_mission is not None:
            try:
                self.resolved = _resolve_mission(str(path))
            except Exception as exc:
                self.say(f"  NOTE   could not resolve map/mission: {exc}")
        if not self.resolved:
            self.resolved = self.doc
        for e in self.report.errors:
            self.say(f"  ERROR  {e}")
        for w in self.report.warnings:
            self.say(f"  WARN   {w}")

        self.dirty = False
        self.populate_trees()
        self.show_static_scene()
        self.update_status()

    def save_scenario(self):
        if not self.path or self.doc is None:
            return
        try:
            if _RT:
                with open(self.path, "w", encoding="utf-8") as fh:
                    _RT.dump(self.doc, fh)
            else:
                import yaml
                with open(self.path, "w", encoding="utf-8") as fh:
                    yaml.safe_dump(self.doc, fh, sort_keys=False)
        except OSError as exc:
            self.say(f"ERROR  could not save: {exc}")
            return
        self.dirty = False
        self.say(f"Saved {self.path.name}")
        # Re-validate: an edit can break a cross-reference.
        self.report = spec.load(self.path)
        for e in self.report.errors:
            self.say(f"  ERROR  {e}")
        self.update_status()

    def populate_trees(self):
        self.tab_env.clear()
        self.tab_scn.clear()
        self.tab_msn.clear()
        if not self.doc:
            return

        # Two things a researcher sets separately: the SCENE is the geometry
        # they are working in, BACKGROUND is the conditions inside it. Changing
        # the room and changing the weather are different jobs.
        arena = self._view().get("arena") or {}

        # Name the SCENE that is loaded. A mission names a scene; this is where
        # you confirm which world you are actually in.
        view = self._view()
        scene_name = (view.get("name") if not (self.doc or {}).get("scene")
                      else (self.doc or {}).get("scene"))
        scene = QTreeWidgetItem(self.tab_env, [f"Scene: {scene_name or 'inline'}"])
        room = QTreeWidgetItem(scene, [f"{arena.get('type', 'box')}  "
                                       f"({_num((arena.get('extent') or {}).get('x')):.0f}"
                                       f" x {_num((arena.get('extent') or {}).get('y')):.0f}"
                                       f" x {_num((arena.get('extent') or {}).get('z')):.0f} m)"])
        room.setData(0, Qt.UserRole, ("node", ["arena"]))

        # BACKGROUND - the contested-environment conditions the SCENE owns.
        # These live on the scene (not the fleet, not an attack) because they
        # are properties of the world that exist before anyone hostile does:
        # the propagation the walls impose, the spectrum's resting state, the
        # sky's GNSS view, the air the airframes push against. The Contested
        # tab will later act ON these; the scene declares their baseline.
        # Undeclared ones are shown greyed so a scene author can see what a
        # scene CAN declare - see docs/contested-background.md.
        background = QTreeWidgetItem(self.tab_env, ["Background"])
        for key in ("propagation", "spectrum", "gnss", "wind"):
            if key in arena:
                n = QTreeWidgetItem(background, [key])
                n.setData(0, Qt.UserRole, ("node", ["arena", key]))
            else:
                n = QTreeWidgetItem(background, [f"{key}   (not declared)"])
                n.setDisabled(True)
        self.tab_env.expandAll()

        # OVERVIEW and MISSION share one hierarchy - system > network > agents -
        # and differ only in what hangs off each agent. Overview shows the
        # agent's EQUIPMENT (its sensors); Mission shows its OBJECTIVE. Building
        # both from one helper keeps them from drifting apart.
        self._build_agent_tree(self.tab_scn, leaf="equipment")
        self._build_agent_tree(self.tab_msn, leaf="objective")
        self._build_contested_baseline()
        self._build_network_declared()
        if hasattr(self, "_band_buttons"):
            self._rebuild_band_strip()

    def _systems(self, networks):
        """Group networks into systems. A SYSTEM is a side - one or more
        networks that belong together (blue+green = friendly, red = adversary).

        A network may name its system with `system: friendly`. Until maps do
        that, everything falls into one 'friendly' system, so the tier is
        present and correct now and simply gains siblings when red arrives."""
        groups = {}
        for name, net in (networks or {}).items():
            sys_name = (net or {}).get("system", "friendly")
            groups.setdefault(sys_name, []).append((name, net))
        return groups

    def _build_agent_tree(self, tree, leaf):
        """Fill one tree with Mission > system > network > agents > <leaf>.

        leaf = 'equipment' hangs each agent's sensors under it (the Overview
        answer: what the agent IS). leaf = 'objective' hangs the agent's current
        objective under it (the Mission answer: what the agent is DOING). Nothing
        about hardware appears in the objective tree, and no objective appears in
        the equipment tree."""
        view = self.resolved or self.doc
        if leaf == "objective":
            self._objective_rows = {}
            self._history_rows = {}
            self._last_objective_label = {}
        mission_name = view.get("name") or (self.path.stem if self.path else "")
        if leaf == "objective":
            # Mission is tasking, not the file - a scenario can carry
            # baked-in objectives (every legacy scenarios/ file does) and
            # this still reads UNASSIGNED, because nothing has been
            # ASSIGNED at runtime yet. See _refresh_live_objectives(),
            # which is what actually flips this once something changes.
            root = QTreeWidgetItem(tree, [f"Mission: {self._mission_name or 'UNASSIGNED'}"])
            self._mission_root_item = root
        else:
            # Overview is equipment - what's in this scenario, not what it's
            # tasked to do. Was headed "Mission: X" too, which is backwards:
            # tasking has no business labelling the equipment view.
            root = QTreeWidgetItem(tree, [f"Scenario: {mission_name or 'Custom'}"])
        agents = view.get("agents") or []
        networks = view.get("networks") or {}
        placed = set()

        for sys_name, members in self._systems(networks).items():
            sys_item = QTreeWidgetItem(root, [f"system: {sys_name}"])
            sys_item.setData(0, Qt.UserRole, ("system", [sys_name]))
            for name, net in members:
                n = QTreeWidgetItem(sys_item, [name])
                n.setData(0, Qt.UserRole, ("node", ["networks", name]))
                hub = (net or {}).get("coordinator")
                folder = QTreeWidgetItem(n, ["Agents"])
                for i, agent in enumerate(agents):
                    if agent.get("network") != name:
                        continue
                    placed.add(i)
                    label = agent.get("id", "?")
                    if label == hub:
                        label += "   (coordinator)"
                    a = QTreeWidgetItem(folder, [label])
                    a.setData(0, Qt.UserRole, ("agent", ["agents", i]))
                    if leaf == "objective":
                        obj = agent.get("mission") or {"type": "static"}
                        o = QTreeWidgetItem(a, [f"objective: {_objective_label(obj)}"])
                        o.setData(0, Qt.UserRole, ("objective", ["agents", i]))
                        # Keep a handle so a retask can update this row live,
                        # rather than the tree showing what the FILE said while
                        # the agent is doing something else entirely.
                        self._objective_rows[agent.get("id")] = o
                        # History: every objective this agent has been given,
                        # with the sim time it took effect. A retask is a
                        # command decision, and a run you cannot reconstruct the
                        # decisions of is not reviewable.
                        h = QTreeWidgetItem(a, ["history"])
                        h.setData(0, Qt.UserRole, ("history", ["agents", i]))
                        self._history_rows[agent.get("id")] = h
                        self._last_objective_label[agent.get("id")] = _objective_label(obj)
                        first = QTreeWidgetItem(
                            h, [f"t=0.0  {_objective_label(obj)}   (initial)"])
                        first.setDisabled(True)
                    else:  # equipment
                        for j, sen in enumerate(agent.get("sensors") or []):
                            it = QTreeWidgetItem(
                                a, [f"{sen.get('id')}  ({sen.get('type')})"])
                            it.setData(0, Qt.UserRole,
                                       ("node", ["agents", i, "sensors", j]))

        loose = [i for i in range(len(agents)) if i not in placed]
        if loose:
            orphan = QTreeWidgetItem(root, ["Agents (no network)"])
            for i in loose:
                a = QTreeWidgetItem(orphan, [agents[i].get("id", "?")])
                a.setData(0, Qt.UserRole, ("agent", ["agents", i]))

        # Radios belong to the equipment view only - they are hardware, not
        # tasking.
        if leaf == "equipment":
            radios = QTreeWidgetItem(tree, ["Radios"])
            for name in (view.get("radios") or {}):
                r = QTreeWidgetItem(radios, [name])
                r.setData(0, Qt.UserRole, ("node", ["radios", name]))

        tree.expandAll()

    def show_static_scene(self):
        """The scene as the file defines it, before any run.

        Drawn from the RESOLVED view (map + mission), so a split mission - whose
        agents live in its map - still shows its cars rather than an empty box.
        """
        view = getattr(self, "resolved", None) or self.doc
        if not view:
            return
        self.viewport.arena = view.get("arena")
        self.viewport.agents = [
            {"id": a.get("id"), "colour": a.get("colour"),
             "dimensions": a.get("dimensions", {}), "pose": a.get("pose", {}),
             "sensors": a.get("sensors", []),
             "platform": a.get("platform"), "jammer": a.get("jammer")}
            for a in (view.get("agents") or [])
        ]
        self._push_scene_rf()
        self.viewport.links = []
        self.viewport.update()
        self.fill_publications_from_scenario()
        self.fill_comms()

    def on_contested_edit(self, item, _col):
        """Double-click an active emitter to tune its transmit power live.
        Writes a JAM command on the same channel the terminal uses, so the
        jammer is edited from the Console, not from a file."""
        data = item.data(0, Qt.UserRole)
        if not (isinstance(data, tuple) and data[0] == "jammer"):
            return
        if not getattr(self, "_retask_dir", None):
            self.say("Start the run first, then double-click to tune a jammer.")
            return
        jid = data[1]
        cur = 20.0
        live = (self.latest or {}).get(jid) or {}
        if live.get("jammer"):
            cur = _num(live["jammer"].get("tx_dbm"), 20.0)
        val, ok = QInputDialog.getDouble(
            self, "Tune jammer", f"{jid} transmit power (dBm):", cur,
            -30.0, 60.0, 1)
        if ok:
            line = f"JAM {jid} power {val}\n"
            self._send_queue_line_console(line, f"JAM {jid} power {val}")

    def _send_queue_line_console(self, line, summary):
        """Write one command file into the running sim's retask spool - the
        Console's own copy of the terminal's _send_queue_line, for controls
        that are not the terminal (the Contested editor)."""
        try:
            qdir = self._retask_dir
            qdir.mkdir(parents=True, exist_ok=True)
            name = f"cmd_{time.monotonic_ns():020d}.txt"
            tmp = qdir / (name + ".tmp")
            tmp.write_text(line, encoding="utf-8")
            tmp.rename(qdir / name)
            self.say(f"[sent: {summary}]")
        except OSError as exc:
            self.say(f"[send failed: {exc}]")

    def side_of(self, token):
        """Which side ('blue'/'red') a scope token belongs to - a network
        name or an agent id - by its network's `system` (friendly->blue,
        adversary->red). None if unknown. Used by the cell terminals to
        refuse cross-side commands."""
        view = getattr(self, "resolved", None) or self.doc or {}
        nets = view.get("networks") or {}
        def side_of_net(name):
            sysname = ((nets.get(name) or {}).get("system") or "friendly")
            return "red" if sysname == "adversary" else "blue"
        if token in nets:
            return side_of_net(token)
        for a in (view.get("agents") or []):
            if a.get("id") == token:
                return side_of_net(a.get("network"))
        return None

    def _rebuild_band_strip(self):
        """(Re)list the scene's distinct bands as buttons: All + each comms
        band + GNSS. Called on load. Keys: "all", a float MHz for a comms
        band, or "gnss"."""
        for b in list(self._band_buttons.values()):
            self.band_group.removeButton(b)
            b.setParent(None)
            b.deleteLater()
        self._band_buttons = {}
        # collect comms bands from the networks
        bands = []
        for net in (self._view().get("networks") or {}).values():
            v = (net or {}).get("band")
            if isinstance(v, dict):
                v = v.get("value")
            if v is not None and float(v) not in bands:
                bands.append(float(v))
        entries = [("all", "All")]
        for b in sorted(bands):
            entries.append((b, f"{b/1000:.1f} GHz" if b >= 1000
                            else f"{b:.0f} MHz"))
        entries.append(("gnss", "GNSS 1575"))
        insert_at = self._bandrow.count() - 1   # before the stretch
        for key, label in entries:
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setFixedHeight(20)
            btn.setToolTip("Show only this band's links" if key != "all"
                           else "Show every band")
            btn.clicked.connect(lambda _c=False, k=key: self._on_band(k))
            self.band_group.addButton(btn)
            self._bandrow.insertWidget(insert_at, btn)
            insert_at += 1
            self._band_buttons[key] = btn
        if "all" in self._band_buttons:
            self._band_buttons["all"].setChecked(True)
        self._on_band("all")

    def _on_band(self, key):
        """Set the viewport's band filter. None=all; a float=that comms band;
        'gnss'=foreground positioning (dim the comms links)."""
        self.viewport.band_filter = None if key == "all" else key
        self.viewport.update()

    def _refresh_band_strip(self, frame):
        """Colour a band button orange while a jammer emits on it - the live
        'what am I jamming' cue. GNSS counts a jammer within ~5 MHz of L1."""
        emitters = frame.get("attacks_active") or []
        jammed = set()
        for e in emitters:
            bm = e.get("band_mhz")
            if bm is None:
                continue
            if abs(bm - 1575.42) < 5.0:
                jammed.add("gnss")
            else:
                jammed.add(float(bm))
        for key, btn in self._band_buttons.items():
            if key == "all":
                btn.setStyleSheet("QPushButton { color: %s; }"
                                  % ("#E08A3C" if jammed else ""))
                continue
            on = key in jammed
            btn.setStyleSheet(
                "QPushButton { color: #E08A3C; font-weight: bold; }" if on
                else "")

    def _push_scene_rf(self):
        """Hand the viewport the scene's noise floor and path-loss exponent so
        it can size the jammer range ring. Read from the resolved background,
        falling back to rf_link()'s defaults."""
        bg = ((getattr(self, "resolved", None) or self.doc or {})
              .get("arena") or {})
        def q(sec, key, default):
            v = ((bg.get(sec) or {}).get(key))
            if isinstance(v, dict):
                v = v.get("value")
            try:
                return float(v)
            except (TypeError, ValueError):
                return default
        self.viewport.scene_rf = (q("spectrum", "noise_floor", -95.0),
                                  q("propagation", "path_loss_exponent", 2.8))

    def on_overview_double_click(self, item, _column):
        """Popup a short description of an agent's objective on double-click."""
        data = item.data(0, Qt.UserRole)
        if not data:
            return
        kind, path = data
        if kind not in ("agent", "objective"):
            return
        view = getattr(self, "resolved", None) or self.doc
        try:
            idx = path[1]
            agent = (view.get("agents") or [])[idx]
        except (IndexError, KeyError, TypeError):
            return
        QMessageBox.information(
            self, f"{agent.get('id', 'agent')} — objective",
            _objective_description(agent))

    # -- selection ----------------------------------------------------------

    def _view(self):
        """The merged map+mission for DISPLAY. Edits still target self.doc; this
        is only for showing agents/arena/radios, which a split mission keeps in
        its map rather than in the file the Console is editing."""
        return getattr(self, "resolved", None) or self.doc or {}

    def _at(self, path):
        node = self.doc
        try:
            for key in path:
                node = node[key]
            return node
        except (KeyError, IndexError, TypeError):
            # A split mission keeps its agents in the map, not in self.doc, so a
            # path into agents[] misses. Fall back to the merged view for
            # DISPLAY. Editing these is guarded in on_prop_edited - you cannot
            # yet edit a map-owned agent from a mission file, and it says so.
            node = self._view()
            for key in path:
                node = node[key]
            return node

    def _set_props_title(self, agent_id):
        """Name what the Properties panel is showing, so a table of values is
        never ambiguous about whose values they are."""
        dock = getattr(self, "d_props", None)
        if dock is not None:
            dock.setWindowTitle(f"Properties \u2014 {agent_id}" if agent_id
                                else "Properties \u2014 no agent selected")

    def on_select(self):
        tree = self.tabs.currentWidget()
        if not isinstance(tree, QTreeWidget):
            return
        items = tree.selectedItems()
        if not items or not self.doc:
            return
        data = items[0].data(0, Qt.UserRole)
        if not data:
            self.props.setRowCount(0)
            self._set_props_title(None)
            return
        kind, path = data
        # A 'system' row is a UI grouping, not a node in the file - selecting it
        # highlights every agent on its networks and shows no properties.
        if kind == "system":
            view = self._view()
            sys_name = path[0]
            net_names = {n for n, net in (view.get("networks") or {}).items()
                         if (net or {}).get("system", "friendly") == sys_name}
            self.viewport.selected = {
                a.get("id") for a in (view.get("agents") or [])
                if a.get("network") in net_names}
            self.selected_agent = None
            self.viewport.update()
            self.props.setRowCount(0)
            self._set_props_title(None)
            return
        # An 'objective' row points at its agent; show that agent selected.
        if kind == "objective":
            kind, path = "agent", path

        try:
            node = self._at(path)
        except (KeyError, IndexError, TypeError):
            return

        if kind == "agent":
            self.selected_agent = node.get("id")
            self.viewport.selected = {self.selected_agent}
            self._set_props_title(self.selected_agent)
        elif kind == "node" and len(path) == 2 and path[0] == "networks":
            # Selecting a network highlights everything on it. When red agents
            # arrive this is how you see the two sides apart at a glance.
            name = path[1]
            self.viewport.selected = {
                a.get("id") for a in (self.doc.get("agents") or [])
                if a.get("network") == name}
            self.selected_agent = None
        else:
            self.viewport.selected = set()
        self.viewport.update()
        self.refresh_sensor_view()
        self.show_props(node, path)

    def show_props(self, obj, base_path):
        rows = []
        _flatten(obj, "", base_path, rows)
        self.props.blockSignals(True)
        self.props.setRowCount(len(rows))
        for i, r in enumerate(rows):
            name = QTableWidgetItem(r["label"])
            name.setFlags(name.flags() & ~Qt.ItemIsEditable)
            self.props.setItem(i, 0, name)

            val = QTableWidgetItem("" if r["value"] is None else str(r["value"]))
            val.setData(Qt.UserRole, ("value", r["path"]))
            self.props.setItem(i, 1, val)

            unit = QTableWidgetItem(r["unit"])
            unit.setFlags(unit.flags() & ~Qt.ItemIsEditable)
            self.props.setItem(i, 2, unit)

            src = QTableWidgetItem(r["source"] if r["source"] else
                                   ("no source" if r["quantity"] else ""))
            if r["quantity"]:
                src.setData(Qt.UserRole, ("source", r["path"]))
                if not r["source"]:
                    src.setForeground(QBrush(QColor(C_WARN)))
            else:
                src.setFlags(src.flags() & ~Qt.ItemIsEditable)
            self.props.setItem(i, 3, src)
        self.props.blockSignals(False)
        self.props.resizeColumnsToContents()

    def on_prop_edited(self, item):
        """Write an edit straight back into the document.

        The file stays the source of truth. The GUI is a structured editor for
        it, not a replacement — which is what keeps scenarios diffable in git
        and sweepable from a script.
        """
        meta = item.data(Qt.UserRole)
        if not meta or self.doc is None:
            return
        field, path = meta
        text = item.text().strip()
        if field == "source" and text == "no source":
            text = ""

        # If this path does not exist in the raw file (only in the merged view),
        # it is owned by the MAP, not by the mission file we are editing. Writing
        # it would change nothing on disk. Say so once, and stop - editing map-
        # owned agents from a mission is a separate feature, still on the list.
        raw = self.doc
        try:
            for key in path:
                raw = raw[key]
        except (KeyError, IndexError, TypeError):
            self.say("This field comes from the map, not this mission file - "
                     "open the map to change it. (In-app map editing is coming.)")
            return

        try:
            node = self._at(path)
        except (KeyError, IndexError, TypeError):
            return

        is_quantity = isinstance(node, dict) and "source" in node and "value" in node
        if is_quantity:
            # path points at the {value, unit, source} dict itself.
            node["value" if field == "value" else "source"] = (
                _coerce(text) if field == "value" else text)
        else:
            # path points at the leaf, so the holder is one level up.
            if field != "value":
                return
            try:
                parent = self._at(path[:-1])
            except (KeyError, IndexError, TypeError):
                return
            parent[path[-1]] = _coerce(text)
        self.dirty = True
        self.show_static_scene()
        self.update_status()

    # -- publications -------------------------------------------------------

    def fill_publications_from_scenario(self):
        """Before a run, show what each agent is *configured* to publish."""
        rows = []
        for a in (self._view().get("agents") or []):
            aid = a.get("id")
            rows.append((aid, f"/{aid}/odom", "nav_msgs/Odometry", "-"))
            rows.append((aid, f"/{aid}/state", "deadband/AgentState", "-"))
            for s in a.get("sensors") or []:
                t = {"ust10lx": ("scan", "sensor_msgs/LaserScan"),
                     "generic_imu": ("imu", "sensor_msgs/Imu"),
                     "generic_gnss": ("navsat", "sensor_msgs/NavSatFix")}.get(s.get("type"))
                if t:
                    rows.append((aid, f"/{aid}/{t[0]}", t[1], "-"))
        self._set_pubs(rows)

    def fill_publications_from_telemetry(self, agents):
        rows = []
        for a in agents:
            for pub in a.get("publishes") or []:
                rows.append((a.get("id"), pub.get("topic"), pub.get("type"),
                             f"{pub.get('rate_hz', 0):.0f}"))
        self._set_pubs(rows)

    def _set_pubs(self, rows):
        """rows: (agent, topic, type, rate). Grouped by agent, with expansion
        remembered so a live refresh does not collapse the tree underneath you.

        The tree is rebuilt on every live frame, so whatever the user had open
        (or closed) has to be restored afterwards. The trap: "nothing is
        expanded" is ambiguous - it is true both on the very first frame, when
        we want everything open, AND after the user has deliberately collapsed
        everything, when we must leave it collapsed. A flag distinguishes them:
        expand-all happens once, on first populate, and never fights the user
        again."""
        expanded = {self.pubs.topLevelItem(i).text(0)
                    for i in range(self.pubs.topLevelItemCount())
                    if self.pubs.topLevelItem(i).isExpanded()}
        first_populate = not getattr(self, "_pubs_populated", False)
        self.pubs.clear()
        groups = {}
        for agent, topic, type_name, rate in rows:
            if agent not in groups:
                groups[agent] = QTreeWidgetItem(self.pubs, [str(agent)])
            QTreeWidgetItem(groups[agent], [str(topic), str(type_name), str(rate)])
        for name, item in groups.items():
            item.setExpanded(first_populate or name in expanded)
        if groups:
            self._pubs_populated = True
        for column in range(3):
            self.pubs.resizeColumnToContents(column)

    def toggle_playback(self, checked):
        """Step the timeline automatically, so a recorded run can be watched.

        Playback follows the recording's own timestamps rather than a fixed tick,
        so what you watch runs at the speed it happened.
        """
        from PySide6.QtCore import QTimer
        timer = getattr(self, "_play_timer", None)
        if not checked or not self.frames:
            # Pausing has to STOP the timer. Un-checking the button and
            # returning left it ticking, so pause looked broken while scrubbing
            # appeared to work - scrubbing just fought the timer for the value.
            self.stop_playback()
            return
        self.live_button.setChecked(False)
        self.play_button.setText("\u23f8")
        if self.timeline.value() >= self.timeline.maximum():
            self.timeline.setValue(0)
        self._play_clock = None
        if not hasattr(self, "_play_timer"):
            self._play_timer = QTimer(self)
            self._play_timer.timeout.connect(self.advance_playback)
        self._play_timer.start(50)

    def advance_playback(self):
        """Move a playback clock, then show the frame nearest to it.

        Driven by an accumulated clock rather than by stepping frames: forcing
        at least one frame per timer tick meant the tick rate, not the speed
        setting, decided how fast playback ran - so 0.5x and 2x looked identical.
        A clock also plays a run recorded at any rate at its true speed.
        """
        if not self.frames:
            return
        rate = float(self.speed_combo.currentText().rstrip("x"))
        self._play_clock = getattr(self, "_play_clock", None)
        if self._play_clock is None:
            i = max(0, self.timeline.value())
            self._play_clock = self.frames[i].get("sim_time_s", 0.0)
        self._play_clock += 0.05 * rate

        end = self.frames[-1].get("sim_time_s", 0.0)
        if self._play_clock >= end:
            self._advancing = True
            self.timeline.setValue(self.timeline.maximum())
            self._advancing = False
            self.stop_playback()
            return

        # The frame that was showing at this instant of the recording.
        i = self.timeline.value()
        while (i + 1 <= self.timeline.maximum()
               and self.frames[i + 1].get("sim_time_s", 0.0) <= self._play_clock):
            i += 1
        if i != self.timeline.value():
            self._advancing = True
            self.timeline.setValue(i)
            self._advancing = False

    def stop_playback(self):
        timer = getattr(self, "_play_timer", None)
        if timer is not None:
            timer.stop()
        self._play_clock = None
        self.play_button.setChecked(False)
        self.play_button.setText("\u25b6")

    def on_live_toggled(self, checked):
        if checked and self.frames:
            self.timeline.setValue(len(self.frames) - 1)

    def on_scrub(self, index):
        """Render the world as it was at frame `index`."""
        if not getattr(self, "_advancing", False):
            timer = getattr(self, "_play_timer", None)
            if timer is not None and timer.isActive():
                self.stop_playback()   # a human moved it: playback yields
        if not self.frames or not (0 <= index < len(self.frames)):
            return
        if index != len(self.frames) - 1:
            self.live_button.setChecked(False)
        f = self.frames[index]
        self.plots.cursor = index
        self.plots.refresh()
        agents = f.get("agents", [])
        self.latest = {a.get("id"): a for a in agents}
        self.viewport.agents = agents
        self.viewport.links = f.get("links", [])
        self.viewport.arena = f.get("arena") or self.viewport.arena
        self.viewport.update()
        self.refresh_sensor_view()
        # EVERY PANEL, NOT JUST THE MAP. Scrubbing used to move the vehicles
        # and leave the Comms, Network, Contested and Publications trees
        # showing whatever the last LIVE frame had put there - so a replayed
        # run looked half-dead, and clicking an agent told you nothing. A
        # frame is a frame: a scrubbed one deserves the same treatment as a
        # streamed one, which is the whole point of the run being replayable.
        try:
            self.fill_publications_from_telemetry(agents)
            self.fill_comms(f.get("links", []))
            self._refresh_contested(f)
            self._refresh_network(f)
            self._refresh_band_strip(f)
            self._refresh_live_objectives()
        except Exception as exc:                       # noqa: BLE001
            self.say(f"scrub: panel refresh failed: {exc}")
        self.time_label.setText(
            f"t = {f.get('sim_time_s', 0):.1f} s   frame {index + 1}/{len(self.frames)}")
        down = sum(1 for l in self.viewport.links if l.get("state") != "up")
        self.statusBar().showMessage(
            f"REPLAY   t={f.get('sim_time_s', 0):.1f}s   "
            f"frame {index + 1}/{len(self.frames)}   "
            f"agents {len(agents)}   "
            f"links {len(self.viewport.links)} ({down} degraded)")

    _CELL_COLOUR = {"blue": "#2E6FB0", "red": "#C4685A", "white": "#B8C0C6"}

    def _build_cells(self):
        """The three cells, in order: Blue (friendly command), Red (adversary),
        White (umpire + shell). This is the command structure as a UI - a blue
        operator, a red operator, and the umpire who sees and does everything.
        See docs/cells-and-network.md."""
        for cell in ("blue", "red", "white"):
            shell = ShellPanel(REPO_WSL_PATH, cell=cell, console=self)
            self.shells.append(shell)
            idx = self.terminals.addTab(shell, f"{cell.capitalize()} cell")
            self.terminals.tabBar().setTabTextColor(
                idx, QColor(self._CELL_COLOUR[cell]))
        self._cells_built = True

    def new_terminal(self, focus=False):
        """Open another WHITE (umpire) shell tab - an extra plain terminal.
        The three cells are made once at startup by _build_cells()."""
        self._terminal_count = getattr(self, "_terminal_count", 0) + 1
        shell = ShellPanel(REPO_WSL_PATH, cell="white", console=self)
        self.shells.append(shell)
        index = self.terminals.addTab(shell, f"Terminal {self._terminal_count}")
        self.terminals.tabBar().setTabTextColor(
            index, QColor(self._CELL_COLOUR["white"]))
        if focus:
            self.bottom.setCurrentIndex(1)
            self.terminals.setCurrentIndex(index)
            shell.inp.setFocus()
        return shell

    def terminal_menu(self, pos):
        menu = QMenu(self)
        menu.addAction("New terminal").triggered.connect(
            lambda: self.new_terminal(focus=True))
        index = self.terminals.tabBar().tabAt(pos)
        if index >= 0 and self.terminals.count() > 1:
            menu.addAction(f"Close {self.terminals.tabText(index)}").triggered.connect(
                lambda: self.close_terminal(index))
        menu.exec(self.terminals.mapToGlobal(pos))

    def close_terminal(self, index):
        widget = self.terminals.widget(index)
        if not isinstance(widget, ShellPanel):
            return
        widget.stop_all()
        if widget in self.shells:
            self.shells.remove(widget)
        self.terminals.removeTab(index)
        widget.deleteLater()
        if not self.shells:
            self.new_terminal()

    def _unused_run_shell(self):
        """Run one command in WSL and show its output.

        Each command is its own process, so `cd` does not persist between them.
        The working directory is tracked here instead and prepended, which is
        what makes the panel behave the way a shell appears to.
        """
        cmd = self.term_in.text().strip()
        if not cmd:
            return
        self.term_in.clear()
        self.term_out.appendPlainText(f"$ {cmd}")

        # Only a bare `cd`. "cd ros2 && colcon build" is a compound command and
        # must go to the shell whole - treating everything after "cd " as a
        # directory name swallowed the rest of the line.
        if cmd.startswith("cd ") and not any(t in cmd for t in ("&&", "||", ";", "|")):
            self._shell_cwd = cmd[3:].strip()
            self.term_out.appendPlainText(f"(working directory: {self._shell_cwd})")
            return
        if cmd in ("clear", "cls"):
            self.term_out.clear()
            return

        cwd = getattr(self, "_shell_cwd", None) or str(REPO_WSL_PATH)
        # Source ROS every time: a fresh process inherits nothing from the last.
        full = (f"cd {cwd} 2>/dev/null; "
                f"source /opt/ros/humble/setup.bash 2>/dev/null; "
                f"source {REPO_WSL_PATH}/ros2/install/setup.bash 2>/dev/null; "
                f"{cmd}")

        proc = QProcess(self)
        self._shell_procs = getattr(self, "_shell_procs", [])
        self._shell_procs.append(proc)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.readyReadStandardOutput.connect(
            lambda p=proc: self.term_out.appendPlainText(
                bytes(p.readAllStandardOutput()).decode(errors="replace").rstrip()))
        proc.finished.connect(
            lambda code, _st, p=proc: (
                self.term_out.appendPlainText(f"[exit {code}]") if code else None,
                self._shell_procs.remove(p) if p in self._shell_procs else None))
        proc.start("wsl.exe", ["-e", "bash", "-lc", full])
        if not proc.waitForStarted(3000):
            self.term_out.appendPlainText(
                "[cannot start wsl.exe - is WSL installed and on PATH?]")

    def select_agent_by_id(self, agent_id):
        """Selecting on the map selects everywhere: tree, properties, sensor."""
        if not agent_id or not self.doc:
            return
        for i, a in enumerate(self.doc.get("agents") or []):
            if a.get("id") != agent_id:
                continue
            self.selected_agent = agent_id
            self.viewport.selected = {agent_id}
            self.viewport.update()
            self.refresh_sensor_view()
            self.show_props(a, ["agents", i])
            self.tabs.setCurrentIndex(1)
            it = self._find_tree_item(self.tab_scn, ("agent", ["agents", i]))
            if it:
                self.tab_scn.blockSignals(True)
                self.tab_scn.setCurrentItem(it)
                self.tab_scn.blockSignals(False)
            return

    @staticmethod
    def _find_tree_item(tree, data):
        stack = [tree.topLevelItem(i) for i in range(tree.topLevelItemCount())]
        while stack:
            it = stack.pop()
            if it.data(0, Qt.UserRole) == data:
                return it
            stack += [it.child(i) for i in range(it.childCount())]
        return None

    def on_tab_changed(self, index):
        """Results shows the plots; every other tab shows the world."""
        # COLLAPSE THE TREES. A tab whose trees are all expanded is metres
        # long and you scroll past the thing you came for. Opening a branch is
        # one click; closing thirty is not.
        w = self.tabs.widget(index)
        for tree in (w.findChildren(QTreeWidget) if w is not None else []):
            tree.collapseAll()
        if self.tabs.tabText(index) == "Setup":
            self._refresh_setup_lists()
        self.stack.setCurrentIndex(1 if self.tabs.tabText(index) == "Results" else 0)

    def refresh_series_tree(self, frame):
        """Rebuild the series list when the shape of telemetry changes."""
        found = discover_series(frame)
        if found == self._known_series:
            return
        self._known_series = found
        self.rebuild_series_tree()

    def _is_key(self, path):
        leaf = path.split(".", 2)[2] if path.count(".") >= 2 else path
        for _, leaves in KEY_METRICS:
            if leaf in leaves:
                return True
        return False

    def rebuild_series_tree(self):
        found = self._known_series
        if self.series_mode.currentText() == "Key metrics":
            found = [pth for pth in found if self._is_key(pth)]
        self.series_tree.blockSignals(True)
        self.series_tree.clear()
        # Shaped like the ROS graph, the way PlotJuggler lays out a bag:
        # agent -> topic -> field. That is the hierarchy a researcher already
        # has in their head, and it is the one the real system will publish.
        TOPIC_OF = {"yaw_rate": "odom", "pose": "odom", "scan": "scan"}
        groups, subs = {}, {}
        for path in found:
            parts = path.split(".")
            if parts[0] == "agents":
                head, leaf = parts[1], ".".join(parts[2:])
                topic = TOPIC_OF.get(parts[2], parts[2])
            else:
                head, leaf, topic = parts[1], ".".join(parts[2:]), "link"
            if head not in groups:
                groups[head] = QTreeWidgetItem(self.series_tree, [head])
            key = (head, topic)
            if key not in subs:
                subs[key] = QTreeWidgetItem(groups[head], [f"/{topic}"])
            groups[path] = subs[key]
            leaf = path.split(".", 2)[2]
            leaf = leaf.replace("pose.", "")
            pretty = {"distance_m": "distance (m)", "latency_ms": "latency (ms)",
                      "pdr": "packet delivery ratio", "quality": "link quality",
                      "scan.min_range": "nearest lidar return (m)",
                      "speed": "speed (m/s)", "yaw_rate": "turn rate (rad/s)",
                      "pose.speed": "speed (m/s)",
                      "pose.x": "x (m)", "pose.y": "y (m)", "pose.z": "z (m)",
                      "pose.yaw": "yaw (rad)"}.get(leaf, leaf)
            item = QTreeWidgetItem(groups[path], [pretty])
            item.setData(0, Qt.UserRole, path)
        self.series_tree.expandAll()
        self.series_tree.blockSignals(False)

    def export_csv(self):
        """Wide CSV: one row per frame, one column per discovered series."""
        if not self.frames:
            self.say("Nothing recorded yet.")
            return
        stem = self._run_stem()
        from datetime import datetime
        stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
        path, _ = QFileDialog.getSaveFileName(
            self, "Export run",
            str(REPO_ROOT / "runs" / f"{stem}_{stamp}.csv"), "CSV (*.csv)")
        if not path:
            return
        import csv
        cols = self._known_series or discover_series(self.frames[0])
        try:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w", newline="", encoding="utf-8") as fh:
                w = csv.writer(fh)
                w.writerow(["sim_time_s"] + cols)
                for i, f in enumerate(self.frames):
                    w.writerow([f.get("sim_time_s")] +
                               [series_value(self.frames, i, c) for c in cols])
            # A results file without the configuration that produced it is an
            # orphan. The sidecar says what was composed, what doctrine the
            # blue fleet declared and what every jammer was emitting - so a
            # CSV can be read months later, or by somebody else.
            import json as _json
            meta = {
                "run": Path(path).stem,
                "scene": getattr(self, "_setup_scene", None),
                "blue_fleet": getattr(self, "_setup_fleet", None),
                "red_fleet": getattr(self, "_setup_red", None),
                "mission": (self.frames[-1] or {}).get("mission"),
                "frames": len(self.frames),
                "sim_time_s": (self.frames[-1] or {}).get("sim_time_s"),
                "agents": [
                    {"id": a.get("id"), "network": a.get("network"),
                     "platform": a.get("platform"),
                     "on_link_loss": a.get("on_link_loss"),
                     "jammer": a.get("jammer")}
                    for a in (self.frames[-1] or {}).get("agents", [])],
                "networks": ((self.frames[-1] or {}).get("arena")
                             or {}).get("networks"),
            }
            Path(path).with_suffix(".meta.json").write_text(
                _json.dumps(meta, indent=2), encoding="utf-8")
        except OSError as exc:
            self.say(f"ERROR  export failed: {exc}")
            return
        self.say(f"Exported {len(self.frames)} frames to {path}")

    def write_run_log(self):
        """Every frame as JSON lines, so PlotJuggler or pandas can take over
        for any question this panel is too small to answer."""
        if not self.frames:
            return
        out = REPO_ROOT / "runs"
        try:
            out.mkdir(parents=True, exist_ok=True)
            stamp = int(self.frames[0].get("wall_time", 0))
            target = out / f"run_{stamp}.jsonl"
            with open(target, "w", encoding="utf-8") as fh:
                for f in self.frames:
                    fh.write(json.dumps(f) + "\n")
            self.say(f"Recorded {len(self.frames)} frames to runs/{target.name}")
        except OSError as exc:
            self.say(f"ERROR  could not write run log: {exc}")

    def _build_network_declared(self):
        """The declared command structure, before a run: per network its
        authority / routing / coordinator / squads / leader-loss doctrine.
        The LIVE decider/reachable/measured-topology is filled by
        _refresh_network from telemetry."""
        tree = getattr(self, "tab_network", None)
        if tree is None:
            return
        tree.clear()
        nets = (self._view().get("networks") or {})
        dec = QTreeWidgetItem(tree, ["Declared"])
        for name, net in nets.items():
            net = net or {}
            n = QTreeWidgetItem(dec, [f"{name}"])
            n.setForeground(0, QBrush(QColor(network_colour(name))))
            auth = (net.get("authority") or net.get("architecture")
                    or net.get("topology") or "centralized")
            QTreeWidgetItem(n, [f"authority: {auth}"])
            QTreeWidgetItem(n, [f"routing: {net.get('routing', '(default)')}"])
            if net.get("coordinator"):
                QTreeWidgetItem(n, [f"coordinator: {net['coordinator']}"])
            if net.get("squads"):
                sq = QTreeWidgetItem(n, ["squads"])
                for sname, spec in (net.get("squads") or {}).items():
                    spec = spec or {}
                    QTreeWidgetItem(sq, [f"{sname}: {spec.get('leader')} leads "
                                         f"{', '.join(spec.get('members') or [])}"])
            # DOCTRINE - declarable, and the thing worth comparing: does a
            # squad fall back to the coordinator when its leader drops, or is
            # it stranded? See command_authority()'s leader_loss.
            if auth == "hierarchical":
                QTreeWidgetItem(n, [f"leader-loss doctrine: "
                                    f"{net.get('leader_loss', 'fallback')}"])
        dec.setExpanded(True)
        self._net_live = QTreeWidgetItem(tree, ["Live (run to populate)"])
        self._net_topo = QTreeWidgetItem(tree, ["Measured topology"])
        self._net_live.setExpanded(True)
        self._net_topo.setExpanded(True)

    def _refresh_network(self, frame):
        """Live half: each agent's decider/tier/reachable, and what the
        topology measures as versus its declaration."""
        if getattr(self, "tab_network", None) is None:
            return
        live = getattr(self, "_net_live", None)
        if live is not None:
            live.takeChildren()
            live.setText(0, "Live — who decides for whom")
            for a in frame.get("agents", []):
                au = a.get("authority") or {}
                dec = au.get("decider") or "—"
                tier = au.get("tier", "")
                reach = au.get("reachable", True)
                txt = (f"{a['id']}: decider {dec}  ({tier})  "
                       f"{'reachable' if reach else 'UNREACHABLE'}")
                row = QTreeWidgetItem(live, [txt])
                if not reach:
                    row.setForeground(0, QBrush(QColor(NETWORK_COLOURS["red"])))
        topo = getattr(self, "_net_topo", None)
        if topo is not None:
            topo.takeChildren()
            t = frame.get("topology") or {}
            shape = t.get("shape", "—")
            topo.setText(0, f"Measured topology: {shape}")
            if t.get("hub"):
                QTreeWidgetItem(topo, [f"hub: {t['hub']}"])
            if t.get("max_betweenness") is not None:
                QTreeWidgetItem(topo, [f"max betweenness: "
                                       f"{t['max_betweenness']:.2f}"])
            links = frame.get("links", [])
            active = sum(1 for l in links if l.get("active"))
            spare = sum(1 for l in links
                        if not l.get("active") and l.get("state") != "down")
            down = sum(1 for l in links if l.get("state") == "down")
            QTreeWidgetItem(topo, [f"links: {active} active, {spare} spare, "
                                   f"{down} down"])
            # The finding: does the measured shape match the declaration?
            declared = {(net or {}).get("routing")
                        for net in (self._view().get("networks") or {}).values()}
            if shape and declared and shape not in declared and \
                    None not in declared:
                note = QTreeWidgetItem(
                    topo, [f"⚠ measured '{shape}' ≠ declared "
                           f"{'/'.join(sorted(d for d in declared if d))}"])
                note.setForeground(0, QBrush(QColor(C_WARN)))

    def _build_contested_baseline(self):
        """The scene's declared contested BASELINE - the half of the
        Contested tree that is known before a run and does not move. The
        live half (emitters, per-agent experienced noise) is filled by
        _refresh_contested from telemetry."""
        tree = getattr(self, "tab_contested", None)
        if tree is None:
            return
        tree.clear()
        bg = (self._view().get("arena") or {})
        base = QTreeWidgetItem(tree, ["Scene baseline"])
        def q(node, key, unit):
            v = (node or {}).get(key)
            if isinstance(v, dict):
                v = v.get("value")
            return "not declared" if v is None else f"{v} {unit}".strip()
        prop = bg.get("propagation") or {}
        spec_ = bg.get("spectrum") or {}
        gnss = bg.get("gnss") or {}
        QTreeWidgetItem(base, [f"noise floor: {q(spec_, 'noise_floor', 'dBm')}"])
        QTreeWidgetItem(base, [f"path-loss exponent: "
                               f"{q(prop, 'path_loss_exponent', '')}"])
        QTreeWidgetItem(base, [f"GNSS: {gnss.get('availability', 'not declared')}"])
        base.setExpanded(True)
        # Placeholders the live refresh will fill; kept as headers so the
        # shape of the tree is stable whether or not a run is going.
        self._c_emitters = QTreeWidgetItem(tree, ["Emitters (none - run to see)"])
        self._c_spectrum = QTreeWidgetItem(tree, ["Experienced spectrum"])
        self._c_emitters.setExpanded(True)
        self._c_spectrum.setExpanded(True)

    def _refresh_contested(self, frame):
        """The live half: who is transmitting into the spectrum right now,
        and the noise floor each agent actually sees. Called every frame."""
        if getattr(self, "tab_contested", None) is None:
            return
        emitters = frame.get("attacks_active") or []
        em = getattr(self, "_c_emitters", None)
        if em is not None:
            em.takeChildren()
            em.setText(0, f"Emitters ({len(emitters)} active)"
                          if emitters else "Emitters (none active)")
            for e in emitters:
                row = QTreeWidgetItem(
                    em, [f"{e['id']} [{e['network']}]  "
                         f"{e['tx_dbm']:.0f} dBm @ {e['band_mhz']:.0f} MHz"
                         f"   (double-click to edit)"])
                row.setData(0, Qt.UserRole, ("jammer", e["id"]))
        sp = getattr(self, "_c_spectrum", None)
        if sp is not None:
            sp.takeChildren()
            for a in frame.get("agents", []):
                rf = a.get("rf") or {}
                if not rf:
                    continue
                floor = rf.get("noise_floor_dbm")
                base = rf.get("baseline_dbm")
                tag = "  JAMMED" if rf.get("jammed") else ""
                delta = ("" if floor is None or base is None
                         else f"  (+{floor - base:.0f} dB)"
                         if floor - base > 0.5 else "")
                row = QTreeWidgetItem(
                    sp, [f"{a['id']}: {floor:.0f} dBm{delta}{tag}"])
                if rf.get("jammed"):
                    row.setForeground(0, QBrush(QColor(NETWORK_COLOURS["red"])))

    def fill_comms(self, links=None):
        """Emitters from the scenario; link state from the run if there is one."""
        radios = self._view().get("radios") or {} if self.doc else {}
        rows = []
        for a in (self._view().get("agents") or []) if self.doc else []:
            for rname in a.get("radios") or []:
                r = radios.get(rname) or {}
                band = _num((r.get("band") or {}).get("value"))
                tx = (r.get("tx_power") or {}).get("value")
                rows.append((a.get("id"), f"{band:.0f} MHz" if band else "-",
                             f"{tx} dBm" if tx is not None else "unsourced",
                             r.get("link_type", "-")))
        self.emitters.setRowCount(len(rows))
        for i, row in enumerate(rows):
            for c, text in enumerate(row):
                it = QTableWidgetItem(str(text))
                it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                if text == "unsourced":
                    it.setForeground(QBrush(QColor(C_WARN)))
                self.emitters.setItem(i, c, it)
        self.emitters.resizeColumnsToContents()

        links = links if links is not None else []
        self.linktable.setRowCount(len(links))
        for i, l in enumerate(links):
            state = l.get("state", "-")
            # As a percentage, not a bare 0..1 fraction - "1.00" at 2dp
            # flattens a genuinely-computed 0.998 into looking perfect,
            # which is exactly the "comms look perfect and shouldn't" gap.
            # 2dp of a PERCENTAGE keeps the precision rf_link() already
            # computes (pdr is rounded to 3dp) visible instead of rounded
            # away.
            vals = [l.get("a"), l.get("b"), f"{l.get('distance_m', 0):.2f}",
                    f"{l.get('pdr', 0) * 100:.2f}%", state]
            for c, text in enumerate(vals):
                it = QTableWidgetItem(str(text))
                it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                if c == 4:
                    it.setForeground(QBrush(QColor(
                        {"up": C_OK, "degraded": C_WARN}.get(state, C_ERR))))
                self.linktable.setItem(i, c, it)
        self.linktable.resizeColumnsToContents()

    def refresh_sensor_view(self):
        frame = self.latest.get(self.selected_agent)
        scan = frame.get("scan") if frame else None
        self.sensor.set_scan(self.selected_agent, scan, frame)
        # Same scan, drawn on the map, so the panel can be checked against the
        # world instead of taken on trust.
        self.viewport.scan_overlay = (self.selected_agent, scan) if scan else None
        self.viewport.update()

    # -- running ------------------------------------------------------------

    def toggle_run(self):
        running = (self.proc and self.proc.state() != QProcess.NotRunning) or \
                  getattr(self, "ws", None) is not None
        if running:
            self.stop_run()
        elif self.source_combo.currentText().startswith("ROS"):
            self.start_ros_bridge()
        else:
            self.start_run()

    def restart_run(self):
        """Stop and clear, leaving the run ready to start again.

        Restart used to stop and immediately relaunch, which meant a click could
        wipe a run and begin another before you had looked at the first. Now it
        resets and waits for play - destructive things should need the second
        press.
        """
        self.stop_run()
        self.frames = []
        self.plots.frames = self.frames
        self.plots.cursor = None
        self.timeline.blockSignals(True)
        self.timeline.setValue(0)
        self.timeline.setMaximum(0)
        self.timeline.setEnabled(False)
        self.timeline.blockSignals(False)
        self.time_label.setText("no run")
        self.live_button.setChecked(True)
        self.show_static_scene()
        self.say("Reset. Press play to run again.")

    def wsl(self, command, name="wsl"):
        """Run a command in WSL as a tracked background process."""
        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.readyReadStandardOutput.connect(
            lambda p=proc, n=name: self.say(
                "\n".join(f"[{n}] {ln}" for ln in
                           bytes(p.readAllStandardOutput())
                           .decode(errors="replace").splitlines() if ln.strip())))
        full = (f"cd {REPO_WSL_PATH}/ros2 && "
                f"source /opt/ros/humble/setup.bash && "
                f"source install/setup.bash && {command}")
        proc.start("wsl.exe", ["-e", "bash", "-lc", full])
        return proc

    def start_ros_bridge(self):
        """Start the ROS 2 stack, record a bag, and connect to the bridge.

        Pressing play used to mean "attach to something you started yourself in
        a terminal", which left stale nodes holding port 8765 and made the next
        run fail in a way that looked like it had worked. Owning the processes
        means they are cleaned up when the run stops.
        """
        if not self.path:
            self.say("Nothing to run - use the Setup tab: choose a scene, "
                     "then a fleet.")
            return
        self._lock_setup(True)
        # Anything left from a previous run holds the port and wins the race.
        self.say("Clearing any previous ROS processes...")
        # [d]eadband is not a typo. pkill -f matches against the FULL command
        # line, and this shell's own command line contains the pattern, so a
        # plain `pkill -f deadband_ros` signals the shell that is running it.
        # The bracket makes the regex match "deadband_ros" while the literal
        # text "[d]eadband_ros" sitting in our own argv does not match it.
        cleanup = self.wsl("pkill -f '[d]eadband_ros'; "
                           "pkill -f 'ros2 bag [r]ecord'; true",
                           "cleanup")
        cleanup.waitForFinished(4000)

        from datetime import datetime
        self.bag_name = f"bag_{datetime.now():%Y%m%d_%H%M%S}"
        self.bag_saved = False

        # PASS THE SCENARIO THAT IS ACTUALLY OPEN. The launch file defaults to
        # three_car_fleet.yaml, so without this the Console would draw one
        # scenario while ROS simulated another - and the only symptom would be
        # that the picture did not match the file you had open, which is the
        # sort of thing that costs an afternoon.
        scn = _wsl_path(self.path) if getattr(self, "path", None) else ""
        self.say(f"Scenario: {scn or 'launch default'}")
        self.ros_proc = self.wsl(
            "ros2 launch deadband_ros fleet.launch.py"
            + (f" scenario:={scn}" if scn else ""), "ros")
        # MCAP where the storage plugin exists: PlotJuggler reads it with a
        # built-in loader and never opens the plugin-choice dialog that its
        # sqlite3 path goes through - which is where it has been aborting.
        # Launched in the BACKGROUND so its pid can be written down, then
        # waited on. Signalling a pid we recorded ourselves beats matching a
        # pattern against every process on the machine, which is how the last
        # version ended up signalling its own cleanup shell.
        self.bag_proc = self.wsl(
            # `set -m` IS LOAD-BEARING. POSIX says a non-interactive shell sets
            # SIGINT to SIG_IGN for any command it runs with `&`, and the child
            # inherits that - so without job control enabled, `kill -INT` on the
            # recorder does nothing at all and we fall through to SIGKILL every
            # time, which is the corruption we are here to fix. Verified both
            # ways before trusting it.
            f"set -m; mkdir -p {REPO_WSL_PATH}/runs && cd {REPO_WSL_PATH}/runs && "
            f"if ros2 pkg list 2>/dev/null | grep -q rosbag2_storage_mcap; then "
            f"  ros2 bag record -a -s mcap -o {self.bag_name} & "
            f"else "
            f"  echo 'mcap storage not installed (sudo apt install "
            f"ros-humble-rosbag2-storage-mcap) - using sqlite3'; "
            f"  ros2 bag record -a -o {self.bag_name} & "
            f"fi; "
            f"echo $! > {BAG_PIDFILE}; wait $!", "bag")
        self.say(f"Starting ROS 2 nodes, recording {self.bag_name}...")

        # The nodes need a moment before the socket exists, so retry rather than
        # failing on the first attempt.
        self._connect_tries = 0
        self.connect_bridge()
        try:
            from PySide6.QtWebSockets import QWebSocket
        except ImportError:
            self.say("ERROR  QtWebSockets missing. Install PySide6 Addons.")
            return
        from PySide6.QtCore import QUrl

        self.frames = []
        self._buf = ""
        self.run_button.setText("\u25a0")
        self.run_button.setToolTip("Stop")
        self.plots.live = True
        self.plots.refresh()

    def connect_bridge(self):
        try:
            from PySide6.QtWebSockets import QWebSocket
        except ImportError:
            self.say("ERROR  QtWebSockets missing. Install PySide6 Addons.")
            return
        from PySide6.QtCore import QTimer, QUrl

        self.frames = []
        self._buf = ""
        self.ws = QWebSocket()
        self.ws.textMessageReceived.connect(self.on_ws_frame)
        self.ws.connected.connect(
            lambda: self.say("Connected to the ROS 2 bridge."))
        self.ws.errorOccurred.connect(lambda _e: self.retry_bridge())
        self.ws.open(QUrl("ws://localhost:8765"))

    def retry_bridge(self):
        from PySide6.QtCore import QTimer
        self._connect_tries += 1
        if self._connect_tries > 12:
            self.say("ERROR  the bridge never came up. Check the Log above for "
                     "what the ROS nodes said.")
            return
        QTimer.singleShot(1000, self.connect_bridge)

    def on_ws_frame(self, text):
        try:
            frame = json.loads(text)
        except json.JSONDecodeError:
            return
        self.consume_frame(frame)

    def start_run(self):
        """Launch the telemetry source as a background process.

        Today that is the stub. When the real simulator exists this function is
        the only thing that changes — the Console never learns what is behind
        the pipe, which is the whole point of the boundary.
        """
        if not self.path:
            self.say("Nothing to run - use the Setup tab: choose a scene, "
                     "then a fleet.")
            return
        script = REPO_ROOT / "tools" / "stub_telemetry.py"
        if not script.exists():
            self.say(f"ERROR  cannot find {script}")
            return
        self._buf = ""
        self.frames = []
        # A live run is a time series again, whatever the last sweep left set.
        self.plots.xlabel = "s"
        self.plots.shared_scale = False
        self._lock_setup(True)
        self.proc = QProcess(self)
        self.proc.readyReadStandardOutput.connect(self.on_telemetry)
        self.proc.readyReadStandardError.connect(
            lambda: self.say(bytes(self.proc.readAllStandardError())
                             .decode(errors="replace").rstrip()))
        self.proc.finished.connect(lambda *_: self.on_stopped())
        # The stub reads the scenario the Console has open, so what you
        # configured is what runs. One flag; when the real simulator replaces
        # the stub it takes the same argument.
        argv = ["-u", str(script)]
        if self.path:
            argv += ["--scenario", str(self.path)]
        # The seed, when this run is an experiment cell being watched live. A
        # run is a pure function of (scenario, seed), so passing it is what
        # makes what you watch the SAME run the results table scored.
        argv += ["--seed", str(int(getattr(self, "_run_seed", 1)))]
        # A retask channel at a fixed, known path. The terminal's REOBJECTIVE
        # command writes here and the running sim picks it up next tick. Fixed
        # rather than passed around, so terminal and sim agree without wiring.
        self._retask_dir = REPO_ROOT / "runs" / "retask"
        try:
            self._retask_dir.mkdir(parents=True, exist_ok=True)
            # Start clean; no stale command fires. Clears the current
            # cmd_*.txt spool plus any leftover *.reading/*.tmp/legacy
            # "queue" file from an older run or an older format.
            for stale in self._retask_dir.iterdir():
                if stale.name == "queue" or stale.name.startswith("cmd_"):
                    stale.unlink()
        except OSError:
            pass
        argv += ["--retask", str(self._retask_dir)]
        self.proc.start(sys.executable, argv)
        self.run_button.setText("\u25a0")
        self.run_button.setToolTip("Stop")
        self.plots.live = True
        self.plots.refresh()
        self.say("Run started")

    def stop_run(self):
        t = getattr(self, "_play_timer", None)
        if t is not None:
            t.stop()
        ws = getattr(self, "ws", None)
        if ws is not None:
            ws.close()
            self.ws = None
            self.stop_ros_stack()
            self.on_stopped()
            return
        if self.proc and self.proc.state() != QProcess.NotRunning:
            self.proc.kill()
            self.proc.waitForFinished(2000)

    def stop_ros_stack(self):
        """Stop the nodes and the recorder, then ask what to do with the bag.

        THE RECORDER MUST BE ASKED TO STOP, NOT KILLED.

        rosbag2's MCAP writer holds the chunk index and the summary section in
        memory and writes them when the file is closed. Kill the process and
        that trailing section is never written, so the last record on disk is
        truncated - which is exactly the "record type 0x07 ... has length 1878
        but only 1784 bytes remaining" that PlotJuggler reports as a corrupt
        bag. The data is all there; the footer that says where it is, is not.

        pkill's default signal is SIGTERM, which rclpy does not handle, so the
        old code had the same problem even before QProcess.kill(). SIGINT is
        what Ctrl-C sends and is the one rclpy installs a handler for. So the
        sequence is: SIGINT, wait for the writer to flush, and only then
        escalate. The whole thing is one WSL call because each wsl.exe launch
        costs a fifth of a second and doing this in a Python poll loop would
        spend longer starting shells than waiting for the flush.
        """
        self.say("Closing the recorder cleanly (this is what keeps the bag "
                 "readable)...")
        self.wsl(
            # Signal the pid we wrote down at launch. The previous version
            # matched a pattern, and `pkill -f 'ros2 bag record'` matches the
            # command line of the very shell running it, because that pattern
            # is sitting in its own argv. So the wait loop could never see the
            # recorder go, always reported "did not exit on SIGINT", and then
            # SIGKILLed itself. A pid cannot be ambiguous.
            f"PID=$(cat {BAG_PIDFILE} 2>/dev/null); "
            f"if [ -n \"$PID\" ] && kill -0 $PID 2>/dev/null; then "
            f"  kill -INT $PID; "
            f"  for i in $(seq 1 40); do "          # up to 10 s to flush
            f"    kill -0 $PID 2>/dev/null || break; sleep 0.25; "
            f"  done; "
            f"  if kill -0 $PID 2>/dev/null; then "
            f"    echo 'recorder still running after 10 s - forcing; the bag "
            f"may be truncated'; kill -KILL $PID; "
            f"  else echo 'recorder closed cleanly'; fi; "
            f"else "
            # No pidfile: either it never started, or it has already gone.
            # Fall back to the pattern, bracketed so it cannot match us.
            f"  pkill -INT -f 'ros2 bag [r]ecord' 2>/dev/null "
            f"    && echo 'recorder stopped by pattern match' "
            f"    || echo 'no recorder was running'; "
            f"  sleep 1; "
            f"fi; "
            f"rm -f {BAG_PIDFILE}; "
            # Then the nodes. Same courtesy, shorter fuse - they have no file
            # to finish writing, they just have destructors worth running.
            f"pkill -INT -f '[d]eadband_ros' 2>/dev/null; "
            f"for i in $(seq 1 8); do "
            f"  pgrep -f '[d]eadband_ros' >/dev/null || break; sleep 0.25; "
            f"done; "
            f"pkill -KILL -f '[d]eadband_ros' 2>/dev/null; true",
            "cleanup").waitForFinished(20000)

        # Only now tear down the Windows-side shells. Doing this first would
        # take the WSL process tree with it and undo everything above.
        for attr in ("bag_proc", "ros_proc"):
            proc = getattr(self, attr, None)
            if proc is not None:
                proc.kill()
                proc.waitForFinished(2000)
                setattr(self, attr, None)

        name = getattr(self, "bag_name", None)
        if not name:
            return
        self.bag_name = None

        answer = QMessageBox.question(
            self, "Deadband Console",
            f"Keep the recording?\n\nruns/{name}\n\n"
            "A bag is every ROS message from this run, with timestamps. It can "
            "be replayed, and PlotJuggler opens it directly.",
            QMessageBox.Yes | QMessageBox.No)

        if answer == QMessageBox.No:
            self.wsl(f"rm -rf {REPO_WSL_PATH}/runs/{name}", "cleanup")
            self.say(f"Discarded {name}")
            return

        # Title the kept bag with everything the run was -
        # runs/<scene>_<fleet>_<mission>_<timestamp> - instead of an
        # anonymous bag_<timestamp>. Same scheme as the CSV export, one
        # _run_stem() rule for both. mv and ls in ONE shell so they cannot
        # race.
        stem = self._run_stem()
        titled = (f"{stem}_{name[4:]}" if name.startswith("bag_")
                  else f"{stem}_{name}")
        self.wsl(f"mv {REPO_WSL_PATH}/runs/{name} "
                 f"{REPO_WSL_PATH}/runs/{titled} && "
                 f"ls -la {REPO_WSL_PATH}/runs/{titled} | tail -n +2",
                 "bag")
        name = titled
        self.say(f"Kept runs/{name}")
        if QMessageBox.question(
                self, "Deadband Console", "Open it in PlotJuggler now?",
                QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            bag = f"{REPO_WSL_PATH}/runs/{name}"
            # Launch FIRST, then show the instructions. The dialog is modal, so
            # calling it first meant PlotJuggler only started once you had
            # dismissed the very instructions you needed while using it.
            self.wsl(
                "if command -v plotjuggler >/dev/null; then plotjuggler; "
                "elif ros2 pkg list 2>/dev/null | grep -qx plotjuggler; then "
                "  ros2 run plotjuggler plotjuggler; "
                "else echo 'PlotJuggler is not installed:'; "
                "  echo '  sudo apt install ros-humble-plotjuggler-ros'; fi",
                "plotjuggler")
            self.show_plotjuggler_help(bag)

    def show_plotjuggler_help(self, bagdir):
        """The path in a selectable field, not buried in a log line.

        Rewritten after watching the MCAP flow actually happen: the steps are
        different from the sqlite3 ones this used to describe, and the two
        warnings it now throws up look alarming and are not.
        """
        from PySide6.QtWidgets import QDialog, QDialogButtonBox

        name = bagdir.rstrip("/").rsplit("/", 1)[-1]
        # rosbag2 names its first split file <bag>_0.<ext>. Pasting that
        # straight in skips a navigation step; the folder still works if the
        # recorder fell back to sqlite3 because MCAP was not installed.
        mcap = f"{bagdir}/{name}_0.mcap"

        dlg = QDialog(self)
        dlg.setWindowTitle("Load the bag in PlotJuggler")
        lay = QVBoxLayout(dlg)

        text = QLabel(
            "PlotJuggler cannot take a bag on its command line, so load it\n"
            "from inside:\n\n"
            "  1.  Top left, under File, click the Data import icon\n"
            "       (the arrow pointing into a tray).\n"
            "  2.  Paste the path below into the file dialog, press Enter.\n"
            "  3.  If a plugin picker appears, PICK IT FROM THE LIST.\n"
            "       Typing a name there crashes PlotJuggler 3.17 - that is\n"
            "       the map::at abort, and it is their bug, not the bag.\n"
            "  4.  The MCAP Parser lists every channel it found. Ctrl-A\n"
            "       selects all of them; then OK.\n"
            "  5.  The topics appear in the tree on the left. Expand one and\n"
            "       drag a series onto a plot.\n\n"
            "If it says CORRUPTED MCAP FILE, or recovers only partially:\n"
            "answer yes, and the data loads anyway. It means the recorder was\n"
            "killed before it could write the index at the end of the file -\n"
            "every message is on disk, only the table of contents is missing.\n"
            "The Console now stops the recorder with Ctrl-C and waits for it,\n"
            "so bags recorded from here on should load clean. If one still\n"
            "warns, that is worth telling me about.")
        text.setTextFormat(Qt.PlainText)
        lay.addWidget(text)

        field = QLineEdit(mcap)
        field.setReadOnly(True)
        field.setFont(QFont("Consolas", 9))
        lay.addWidget(field)
        field.selectAll()

        note = QLabel(
            "Already copied to your clipboard. If that file is not there, the\n"
            f"recorder fell back to sqlite3 - open the folder instead:\n  {bagdir}")
        note.setTextFormat(Qt.PlainText)
        note.setStyleSheet(f"color: {C_DIM};")
        lay.addWidget(note)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok)
        buttons.accepted.connect(dlg.accept)
        lay.addWidget(buttons)

        QApplication.clipboard().setText(mcap)
        dlg.exec()

    def on_stopped(self):
        self._lock_setup(False)
        self.run_button.setText("\u25b6")
        self.run_button.setToolTip("Run")
        self.plots.live = False
        self.plots.refresh()
        # Objective history belongs to ONE run. Carrying it across runs would
        # show retasks from a previous run against the current one's clock,
        # which is worse than showing nothing. Rebuilding the tree resets the
        # history to each agent's initial objective from the file.
        self.populate_trees()
        self.say("Run stopped")
        self.write_run_log()
        self.update_status()

    def on_telemetry(self):
        """Each stdout line is one JSON frame. Render the newest."""
        self._buf += bytes(self.proc.readAllStandardOutput()).decode(errors="replace")
        *lines, self._buf = self._buf.split("\n")
        frame = None
        for line in lines:
            if line.strip():
                try:
                    frame = json.loads(line)
                except json.JSONDecodeError:
                    continue
        if not frame:
            return
        self.consume_frame(frame)

    def _refresh_live_objectives(self):
        """Update the Mission tree's objective rows from live telemetry.

        A retask changes what an agent is DOING; without this the tree keeps
        showing what the file SAID, which makes it actively misleading. The
        displayed text also carries the armed/assigned tag (see
        _objective_label), which can change on its own via LAUNCH/HALT with
        no change to the objective itself - that must repaint the row
        immediately, but must NOT write a new history entry, since a launch
        is not a new objective. The two are tracked separately: `text`
        (what's shown) vs `label` (what's logged).
        """
        rows = getattr(self, "_objective_rows", None)
        if not rows:
            return
        hist = getattr(self, "_history_rows", {})
        last = getattr(self, "_last_objective_label", {})
        now = 0.0
        if self.frames:
            now = _num(self.frames[-1].get("sim_time_s"))
        for aid, item in rows.items():
            live = (self.latest or {}).get(aid) or {}
            obj = live.get("objective")
            if not isinstance(obj, dict):
                continue
            label = _objective_label(obj)
            text = f"objective: {_objective_label(obj, armed=live.get('armed'))}"
            if item.text(0) != text:
                item.setText(0, text)
            if last.get(aid) != label:
                # The objective itself changed - log it. Only on change, so
                # a 20 Hz stream does not write 20 identical rows a second.
                last[aid] = label
                h = hist.get(aid)
                if h is not None:
                    row = QTreeWidgetItem(h, [f"t={now:.1f}  {label}"])
                    row.setDisabled(True)
                    h.setExpanded(True)
                # This is a genuine runtime assignment - the FIRST one,
                # since the header only flips once. Baked-in file
                # objectives never trigger this (nothing "changed" from
                # what the file already said on the first live frame).
                if self._mission_name is None:
                    self._mission_name = "assigned (live)"
                    root = getattr(self, "_mission_root_item", None)
                    if root is not None:
                        root.setText(0, f"Mission: {self._mission_name}")

    def consume_frame(self, frame):
        """One telemetry frame, from whichever source. The only place the
        Console turns numbers into what is on screen."""
        agents = frame.get("agents", [])
        self.latest = {a.get("id"): a for a in agents}
        # The run's mission name, set by SETMISSION, rides in every frame.
        # It titles the Mission tree, the Setup tab and any kept results.
        mname = frame.get("mission")
        if mname and mname != self._mission_name:
            self._mission_name = mname
            root = getattr(self, "_mission_root_item", None)
            if root is not None:
                root.setText(0, f"Mission: {mname}")
            if hasattr(self, "lbl_mission"):
                self.lbl_mission.setText(f"Mission: {mname}")
        self._refresh_live_objectives()
        # Keep the run for the Results tab. Capped so a forgotten overnight run
        # cannot quietly eat all the memory on the machine.
        if len(self.frames) < 36000:
            self.frames.append(frame)
        self.plots.frames = self.frames
        self.refresh_series_tree(frame)

        self.timeline.blockSignals(True)
        self.timeline.setEnabled(True)
        self.timeline.setMaximum(max(0, len(self.frames) - 1))
        if self.live_button.isChecked():
            self.timeline.setValue(len(self.frames) - 1)
            self.plots.cursor = len(self.frames) - 1
            self.time_label.setText(f"t = {frame.get('sim_time_s', 0):.1f} s   live")
        self.timeline.blockSignals(False)

        if self.live_button.isChecked():
            self.latest = {a.get("id"): a for a in agents}
            self.viewport.agents = agents
            self.viewport.links = frame.get("links", [])
            self.viewport.arena = frame.get("arena") or self.viewport.arena
            self.viewport.update()
            self.refresh_sensor_view()
        if self.stack.currentIndex() == 1:
            self.plots.refresh()
        self.fill_publications_from_telemetry(agents)
        self.fill_comms(frame.get("links", []))
        self._refresh_contested(frame)
        self._refresh_network(frame)
        self._refresh_band_strip(frame)

        down = sum(1 for l in self.viewport.links if l.get("state") != "up")
        source = "ROS 2" if frame.get("source") == "ros2" else "stub"
        topics = sum(len(a.get("publishes") or []) for a in agents)
        self.statusBar().showMessage(
            f"RUNNING [{source}]   t={frame.get('sim_time_s', 0):.1f}s   "
            f"agents {len(agents)}   topics {topics}   "
            f"links {len(self.viewport.links)} ({down} degraded)   "
            f"{self._prov()}")

    # -- misc ---------------------------------------------------------------

    def set_view(self, mode):
        self.viewport.mode = mode
        self.viewport.update()

    def _prov(self):
        if not self.report:
            return ""
        total = len(self.report.quantities)
        return f"provenance {total - len(self.report.unsourced)}/{total} sourced"

    def update_status(self):
        if not self.report:
            return
        state = "OK" if self.report.ok else f"{len(self.report.errors)} ERRORS"
        star = " *" if self.dirty else ""
        self.statusBar().showMessage(
            f"{self.report.path.name}{star}   {state}   {self._prov()}")

    def say(self, text):
        if text:
            self.log.appendPlainText(text)

    def closeEvent(self, ev):
        self.stop_run()
        if self.dirty:
            answer = QMessageBox.question(
                self, "Deadband Console",
                "The scenario has unsaved changes. Save before closing?",
                QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel)
            if answer == QMessageBox.Cancel:
                ev.ignore()
                return
            if answer == QMessageBox.Save:
                self.save_scenario()
        super().closeEvent(ev)


def _coerce(text):
    """Turn edited text back into a sensible YAML scalar."""
    if text == "":
        return None
    low = text.lower()
    if low in ("true", "false"):
        return low == "true"
    if low in ("null", "none", "~"):
        return None
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text


def _flatten(obj, label, path, rows):
    """Flatten nested scenario data into editable property rows.

    Each row carries the path back to the node that holds it, so an edit in the
    table can be written straight into the document.
    """
    if isinstance(obj, dict):
        if "value" in obj and "source" in obj:
            rows.append({"label": label, "value": obj.get("value"),
                         "unit": obj.get("unit", ""), "source": obj.get("source", ""),
                         "quantity": True, "path": path})
            return
        for key, value in obj.items():
            name = f"{label}.{key}" if label else str(key)
            if isinstance(value, (dict, list)):
                _flatten(value, name, path + [key], rows)
            else:
                rows.append({"label": name, "value": value, "unit": "", "source": "",
                             "quantity": False, "path": path + [key]})
    elif isinstance(obj, list):
        for i, item in enumerate(obj):
            tag = item.get("id") if isinstance(item, dict) and "id" in item else i
            _flatten(item, f"{label}[{tag}]", path + [i], rows)


# ---------------------------------------------------------------------------
# Crash reporting. Launched by double-click there is no console, so without
# this an unhandled exception makes the app vanish with no explanation.
# ---------------------------------------------------------------------------

ERROR_LOG = Path(__file__).resolve().parent / "last_error.log"


def _record(exc_type, exc, tb):
    import traceback
    text = "".join(traceback.format_exception(exc_type, exc, tb))
    header = (f"Deadband Console crash\npython  {sys.version}\n"
              f"exe     {sys.executable}\nrepo    {REPO_ROOT}\n{'-' * 60}\n")
    try:
        ERROR_LOG.write_text(header + text, encoding="utf-8")
    except OSError:
        pass
    sys.stderr.write(header + text)
    try:
        if QApplication.instance():
            QMessageBox.critical(None, "Deadband Console",
                                 f"{exc_type.__name__}: {exc}\n\nDetails: {ERROR_LOG}")
    except Exception:
        pass


def main():
    sys.excepthook = _record
    try:
        app = QApplication(sys.argv)
        app.setStyleSheet(STYLE)
        win = Console()
        win.show()
        sys.exit(app.exec())
    except SystemExit:
        raise
    except BaseException:
        _record(*sys.exc_info())
        sys.exit(1)


if __name__ == "__main__":
    main()
