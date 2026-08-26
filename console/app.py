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

import json
import math
import sys
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

from PySide6.QtCore import QMimeData, QPointF, QProcess, QRectF, Qt
from PySide6.QtGui import (
    QAction, QBrush, QColor, QDrag, QFont, QPainter, QPen, QPolygonF,
)
from PySide6.QtWidgets import (
    QApplication, QDockWidget, QFileDialog, QHBoxLayout, QLabel, QMainWindow,
    QMessageBox, QPlainTextEdit, QPushButton, QStackedWidget, QTabWidget,
    QTableWidget, QTableWidgetItem, QTreeWidget, QTreeWidgetItem,
    QComboBox, QLineEdit, QMenu, QSizePolicy, QSlider, QSplitter,
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
except Exception:
    _resolve_mission = None

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


def _objective_label(obj):
    """A one-line label for an objective, for the tree.

    e.g. {'type': 'shuttle', 'between': ['A','B']} -> "shuttle A-B"
    """
    if not isinstance(obj, dict):
        return "static"
    kind = obj.get("type", "static")
    if kind == "shuttle":
        bt = obj.get("between")
        if isinstance(bt, (list, tuple)) and len(bt) == 2:
            return f"shuttle {bt[0]}-{bt[1]}"
        return "shuttle"
    if kind == "pursuit":
        return f"pursue {obj.get('target', '?')}"
    if kind == "patrol":
        return "patrol"
    if kind == "orbit":
        return f"orbit r={obj.get('radius', '?')}"
    if kind == "script":
        f = str(obj.get("file", "")).split("/")[-1]
        return f"script {f}" if f else "script"
    return kind


def _objective_description(agent):
    """Plain-language description of what an agent is doing, for the popup.

    Resolves point names to coordinates where it can, so "shuttle between A and
    B" reads as the actual metres too. Kept to a couple of short sentences: this
    is a glance, not a manual.
    """
    obj = (agent or {}).get("mission") or {"type": "static"}
    kind = obj.get("type", "static")
    aid = agent.get("id", "this agent")

    if kind == "static":
        return f"{aid} holds position. It is not tasked to move."
    if kind == "shuttle":
        bt = obj.get("between")
        if isinstance(bt, (list, tuple)) and len(bt) == 2:
            return (f"{aid} drives back and forth between points {bt[0]} and "
                    f"{bt[1]}, repeating until retasked. The points are defined "
                    f"by the map, so the same objective moves with the map.")
        return f"{aid} shuttles between two points, repeating until retasked."
    if kind == "pursuit":
        return (f"{aid} chases {obj.get('target', 'another agent')}, holding a "
                f"standoff of {obj.get('standoff', 1.2)} m behind it.")
    if kind == "patrol":
        return f"{aid} loops a fixed circuit of waypoints until retasked."
    if kind == "orbit":
        return (f"{aid} circles the arena centre at radius "
                f"{obj.get('radius', 2.0)} m.")
    if kind == "script":
        return (f"{aid} runs the mission script {obj.get('file', '?')}. It "
                f"decides its own target each tick - open the file to see how.")
    return f"{aid}: {kind}."


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
        self.links = []
        self.selected = set()   # highlighted agent ids
        self.scan_overlay = None   # (agent_id, scan) drawn in world coordinates
        self.show_axes = False
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
        for link in self.links:
            a, b = by_id.get(link.get("a")), by_id.get(link.get("b"))
            if not a or not b:
                continue
            # No link at all beyond range: an absent line says "cannot hear
            # each other" far more clearly than a line drawn in a warning colour.
            if float(link.get("quality", 1.0)) <= 0.0:
                continue
            state = link.get("state", "up")
            pen = QPen(QColor(network_colour(link.get("network"))), 1.4)
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

        colour = QColor(agent.get("colour") or "#2E6FB0")
        selected = agent.get("id") in self.selected
        p.setBrush(QBrush(colour))
        p.setPen(QPen(QColor(C_TEXT) if selected else colour.lighter(150), 2 if selected else 1))

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

        # Legend. Style is state; colour is whichever network the link is on.
        nets = {l.get("network") for l in self.links} or {"blue"}
        base = network_colour(sorted(str(n) for n in nets)[0])
        y0 = 34
        for label, style in (("up", Qt.SolidLine), ("degraded", Qt.DashLine),
                             ("down", Qt.DashDotLine)):
            pen = QPen(QColor(base), 1.6)
            pen.setStyle(style)
            p.setPen(pen)
            p.drawLine(10, y0 - 3, 34, y0 - 3)
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(40, y0, label)
            y0 += 13
        p.setPen(QPen(QColor(C_DIM)))
        p.drawText(10, y0, "no line = out of range")
        s = self.scale()
        if s > 2:
            p.drawLine(10, self.height() - 16, 10 + int(s), self.height() - 16)
            p.drawText(14 + int(s), self.height() - 12, "1 m")


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
            if self.state.get("mission"):
                p.drawText(10, 60, f"mission {self.state.get('mission')}")
            top = 66
            p.setPen(QPen(QColor(C_LINE)))
            p.drawLine(8, top, self.width() - 8, top)

        if not self.scan or not self.scan.get("ranges"):
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(12, top + 20, "No lidar on this agent."
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
    ("Link reliability", ["pdr", "latency_ms", "quality"]),
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

    try:
        if parts[0] == "agents":
            agent = next(a for a in frame["agents"] if a.get("id") == parts[1])
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

        for idx, path in enumerate(self.series):
            vals = [series_value(frames, i, path) for i in range(len(frames))]
            pairs = [(tt, v) for tt, v in zip(t, vals) if isinstance(v, (int, float))]
            colour = QColor(PALETTE[idx % len(PALETTE)])
            if not pairs:
                p.setPen(QPen(colour))
                p.drawText(left + 6, top + 14 + idx * 14, f"{path}   no data")
                continue
            lo = min(v for _, v in pairs)
            hi = max(v for _, v in pairs)
            span = (hi - lo) or 1.0
            poly = QPolygonF()
            for tt, v in pairs:
                poly.append(QPointF(left + w * (tt - t0) / (t1 - t0),
                                    top + h - (v - lo) / span * (h - 8) - 4))
            p.setPen(QPen(colour, 1.4))
            p.setBrush(Qt.NoBrush)
            p.drawPolyline(poly)
            p.setPen(QPen(colour))
            p.drawText(left + 6, top + 14 + idx * 14,
                       f"{path}   {lo:.3g} .. {hi:.3g}")

        cursor = self.area.cursor
        if cursor is not None and 0 <= cursor < len(t):
            cx = left + w * (t[cursor] - t0) / (t1 - t0)
            p.setPen(QPen(QColor(C_WARN), 1))
            p.drawLine(QPointF(cx, top), QPointF(cx, top + h))

        p.setPen(QPen(QColor(C_DIM)))
        p.drawText(left, self.height() - 6, f"{t0:.1f} s")
        p.drawText(self.width() - right - 52, self.height() - 6, f"{t1:.1f} s")


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

    def __init__(self, repo_wsl_path, cwd=None):
        super().__init__()
        self.repo = repo_wsl_path
        self.cwd = cwd or repo_wsl_path
        self.procs = []

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setFont(QFont("Consolas", 9))
        self.inp = QLineEdit()
        self.inp.setFont(QFont("Consolas", 9))
        self.inp.setPlaceholderText(
            "Ubuntu command, then Enter.  Interactive tools (sudo prompts, vim) "
            "need a real terminal.")
        self.inp.returnPressed.connect(self.run)
        lay.addWidget(self.out, 1)
        lay.addWidget(self.inp)
        self.out.appendPlainText(f"# working directory: {self.cwd}")

    def run(self):
        cmd = self.inp.text().strip()
        if not cmd:
            return
        self.inp.clear()
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

        # REOBJECTIVE / REMISSION - retasking, handled here rather than sent to
        # the shell. Grammar:
        #     REOBJECTIVE <agent> <objective> <args...>
        #     e.g.  REOBJECTIVE car1 pursue car3
        #           REOBJECTIVE car3 shuttle A B
        #           REOBJECTIVE car2 stop
        # It writes one line to the run's retask queue, which the running sim
        # reads on its next tick. The queue line format is what parse_retask in
        # the stub already understands: "<agent>: <objective> <args>".
        head = cmd.split()
        if head and head[0].upper() in ("REOBJECTIVE", "REMISSION"):
            self._retask(head)
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

    def _retask(self, tokens):
        """Write a retask command to the running sim's queue.

        tokens[0] is REOBJECTIVE or REMISSION (already upper-checked).
            REOBJECTIVE <agent> <objective> <args...>
        becomes the queue line "<agent>: <objective> <args>", which the stub's
        parse_retask understands. REMISSION (retask a whole system at once) is
        recognised but not built yet - it says so rather than failing silently.
        """
        verb = tokens[0].upper()
        if verb == "REMISSION":
            self.out.appendPlainText(
                "[REMISSION - retasking a whole system - is not built yet. "
                "Use REOBJECTIVE per agent for now.]")
            return
        if len(tokens) < 3:
            self.out.appendPlainText(
                "[usage: REOBJECTIVE <agent> <objective> <args>   "
                "e.g. REOBJECTIVE car1 pursue car3]")
            return
        agent = tokens[1]
        rest = " ".join(tokens[2:])
        line = f"{agent}: {rest}\n"
        # runs/retask/queue - the fixed path start_run() launches the sim with.
        # Console and stub are both Windows-side Python sharing REPO_ROOT, so
        # writing here is the file the sim is watching.
        try:
            qdir = REPO_ROOT / "runs" / "retask"
            qdir.mkdir(parents=True, exist_ok=True)
            with open(qdir / "queue", "a", encoding="utf-8") as fh:
                fh.write(line)
            self.out.appendPlainText(f"[retask sent: {agent} -> {rest}]")
        except OSError as exc:
            self.out.appendPlainText(f"[retask failed: {exc}]")

    def stop_all(self):
        for proc in list(self.procs):
            proc.kill()
            proc.waitForFinished(1000)


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
        self.proc = None
        self.ws = None
        self.dirty = False
        self.selected_agent = None
        self.latest = {}         # newest telemetry frame, by agent id
        self.frames = []         # every frame of the current run, for Results
        self._known_series = []
        self._buf = ""

        self.viewport = Viewport()
        self.viewport.on_pick = self.select_agent_by_id
        self._build_centre()
        self._build_docks()
        self._build_menu()

        default = REPO_ROOT / "scenarios" / "three_car_fleet.yaml"
        if default.exists():
            self.load_scenario(default)
        else:
            self.statusBar().showMessage("No scenario loaded")

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
        self.speed_combo.addItems(["0.25x", "0.5x", "1x", "2x", "4x"])
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

        holder = QWidget()
        lay = QVBoxLayout(holder)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(bar)
        lay.addWidget(self.stack, 1)
        lay.addWidget(tl)
        self.setCentralWidget(holder)

    def _build_docks(self):
        # Left: the three-tab workflow. Left to right is the order you build a
        # study in: where it happens, what is in it, what goes wrong.
        self.tab_env = QTreeWidget()
        self.tab_env.setHeaderLabels(["Environment"])
        self.tab_scn = QTreeWidget()
        self.tab_scn.setHeaderLabels(["Overview"])
        # Two questions, two trees. OVERVIEW answers "what is each agent" -
        # system, network, agents, and each agent's equipment (sensors). MISSION
        # answers "what is each agent doing" - the same hierarchy down to agents,
        # then their objectives, and nothing about hardware.
        self.tab_msn = QTreeWidget()
        self.tab_msn.setHeaderLabels(["Mission"])
        for t in (self.tab_env, self.tab_scn, self.tab_msn):
            t.itemSelectionChanged.connect(self.on_select)
            # The default indent stacks five levels deep off the right edge of a
            # narrow panel. Networks > blue > Agents > car1 > lidar has to fit.
            t.setIndentation(12)
        # Double-click an agent or its objective row -> a short popup describing
        # what that agent is doing right now.
        self.tab_scn.itemDoubleClicked.connect(self.on_overview_double_click)
        self.tab_msn.itemDoubleClicked.connect(self.on_overview_double_click)

        cyber = QLabel(
            "Not built yet.\n\n"
            "This is where attacks live: pick a target (an agent, a link, a\n"
            "region or a whole network), pick a class, set its parameters,\n"
            "and fire it while the run is going.\n\n"
            "Deliberately last. An attack panel over a simulator that has no\n"
            "radio model yet would only be able to fake its own results."
        )
        cyber.setObjectName("hint")
        cyber.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        cyber.setWordWrap(True)

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
        self.linktable.setHorizontalHeaderLabels(["From", "To", "Dist m", "PDR", "State"])
        self.linktable.horizontalHeader().setStretchLastSection(True)
        self.linktable.verticalHeader().setVisible(False)
        clay.addWidget(self.linktable, 1)

        self.tabs = QTabWidget()
        # Order top-to-bottom down the left edge, as Will laid it out: the
        # analysis tabs first, then the two build trees, then tasking.
        self.tabs.addTab(results, "Results")
        self.tabs.addTab(cyber, "Cyber")
        self.tabs.addTab(comms, "Comms")
        self.tabs.addTab(self.tab_scn, "Overview")
        self.tabs.addTab(self.tab_env, "Environment")
        self.tabs.addTab(self.tab_msn, "Mission")
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
        d_props = QDockWidget("Properties")
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

        bottom = QTabWidget()
        bottom.addTab(outputs, "Output")
        bottom.addTab(self.terminals, "Terminals")
        self.bottom = bottom
        self.new_terminal()
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
        for label, fn in (("&Open scenario...", self.open_dialog),
                          ("&Save", self.save_scenario),
                          ("&Reload", self.reload_scenario)):
            a = QAction(label, self)
            a.triggered.connect(fn)
            m.addAction(a)
        m.addSeparator()
        a = QAction("E&xit", self)
        a.triggered.connect(self.close)
        m.addAction(a)

    # -- scenario -----------------------------------------------------------

    def open_dialog(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open scenario", str(REPO_ROOT / "scenarios"),
            "Scenario files (*.yaml *.yml)")
        if path:
            self.load_scenario(Path(path))

    def reload_scenario(self):
        if self.path:
            self.load_scenario(self.path)

    def load_scenario(self, path: Path):
        self.path = path
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

        scene = QTreeWidgetItem(self.tab_env, ["Scene"])
        room = QTreeWidgetItem(scene, [f"{arena.get('type', 'box')}  "
                                       f"({_num((arena.get('extent') or {}).get('x')):.0f}"
                                       f" x {_num((arena.get('extent') or {}).get('y')):.0f}"
                                       f" x {_num((arena.get('extent') or {}).get('z')):.0f} m)"])
        room.setData(0, Qt.UserRole, ("node", ["arena"]))
        for label in ("Map builder", "Import topography"):
            it = QTreeWidgetItem(scene, [f"{label}   (not built)"])
            it.setDisabled(True)

        background = QTreeWidgetItem(self.tab_env, ["Background"])
        for key in ("propagation", "spectrum", "gnss", "wind"):
            if key in arena:
                n = QTreeWidgetItem(background, [key])
                n.setData(0, Qt.UserRole, ("node", ["arena", key]))
        self.tab_env.expandAll()

        # OVERVIEW and MISSION share one hierarchy - system > network > agents -
        # and differ only in what hangs off each agent. Overview shows the
        # agent's EQUIPMENT (its sensors); Mission shows its OBJECTIVE. Building
        # both from one helper keeps them from drifting apart.
        self._build_agent_tree(self.tab_scn, leaf="equipment")
        self._build_agent_tree(self.tab_msn, leaf="objective")

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
        mission_name = view.get("name") or (self.path.stem if self.path else "")
        root = QTreeWidgetItem(tree, [f"Mission: {mission_name}"])
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
             "sensors": a.get("sensors", [])}
            for a in (view.get("agents") or [])
        ]
        self.viewport.links = []
        self.viewport.update()
        self.fill_publications_from_scenario()
        self.fill_comms()

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
        self.latest = {a.get("id"): a for a in f.get("agents", [])}
        self.viewport.agents = f.get("agents", [])
        self.viewport.links = f.get("links", [])
        self.viewport.arena = f.get("arena") or self.viewport.arena
        self.viewport.update()
        self.refresh_sensor_view()
        self.time_label.setText(
            f"t = {f.get('sim_time_s', 0):.1f} s   frame {index + 1}/{len(self.frames)}")

    def new_terminal(self, focus=False):
        """Open another shell tab. Each has its own directory and processes."""
        self._terminal_count = getattr(self, "_terminal_count", 0) + 1
        shell = ShellPanel(REPO_WSL_PATH)
        self.shells.append(shell)
        index = self.terminals.addTab(shell, f"Terminal {self._terminal_count}")
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
        path, _ = QFileDialog.getSaveFileName(
            self, "Export run", str(REPO_ROOT / "runs" / "run.csv"), "CSV (*.csv)")
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
            vals = [l.get("a"), l.get("b"), f"{l.get('distance_m', 0):.2f}",
                    f"{l.get('pdr', 0):.2f}", state]
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
        script = REPO_ROOT / "tools" / "stub_telemetry.py"
        if not script.exists():
            self.say(f"ERROR  cannot find {script}")
            return
        self._buf = ""
        self.frames = []
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
        # A retask channel at a fixed, known path. The terminal's REOBJECTIVE
        # command writes here and the running sim picks it up next tick. Fixed
        # rather than passed around, so terminal and sim agree without wiring.
        self._retask_dir = REPO_ROOT / "runs" / "retask"
        try:
            self._retask_dir.mkdir(parents=True, exist_ok=True)
            queue = self._retask_dir / "queue"
            if queue.exists():
                queue.unlink()          # start clean; no stale command fires
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

        self.say(f"Kept runs/{name}")
        # List it. "Did the recording actually save?" should be answerable from
        # the log rather than by going and looking.
        self.wsl(f"ls -la {REPO_WSL_PATH}/runs/{name} | tail -n +2", "bag")
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
        self.run_button.setText("\u25b6")
        self.run_button.setToolTip("Run")
        self.plots.live = False
        self.plots.refresh()
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

    def consume_frame(self, frame):
        """One telemetry frame, from whichever source. The only place the
        Console turns numbers into what is on screen."""
        agents = frame.get("agents", [])
        self.latest = {a.get("id"): a for a in agents}
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
