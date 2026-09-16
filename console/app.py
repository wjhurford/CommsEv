"""
CommsEv Console — the application.

This is the thing you double-click. Everything else runs behind it: the scenario
file is loaded and checked by commsev.spec, and the telemetry source is a
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
BAG_PIDFILE = "/tmp/commsev_bag.pid"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from PySide6.QtCore import (
    QMimeData, QPointF, QProcess, QRectF, Qt, QThread, Signal,
)
from PySide6.QtGui import (
    QAction, QBrush, QColor, QDrag, QFont, QImage, QPainter, QPen,
    QPixmap, QPolygonF,
)
from PySide6.QtWidgets import (
    QButtonGroup,
    QApplication, QDockWidget, QFileDialog, QHBoxLayout, QLabel, QMainWindow,
    QMessageBox, QPlainTextEdit, QPushButton, QStackedWidget, QTabWidget,
    QTableWidget, QTableWidgetItem, QTreeWidget, QTreeWidgetItem,
    QComboBox, QDialog, QInputDialog, QLineEdit, QMenu, QSizePolicy,
    QSlider, QSplitter, QProgressBar, QCheckBox, QSpinBox, QDoubleSpinBox,
    QScrollArea, QFrame, QAbstractSpinBox,
    QVBoxLayout, QWidget,
)

from commsev import spec

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
    # Formations are FUNCTIONS of (count, spacing), defined once in the model.
    # The Console imports them rather than owning a second copy, so what the
    # spawn dialog previews is exactly what a swept run will place.
    from stub_telemetry import FORMATIONS, formation_offsets
    from stub_telemetry import custom_formations, save_formation
    from stub_telemetry import clamp_to_arena as _clamp_to_arena
    from stub_telemetry import mission_goal_count as _mission_goal_count
    from stub_telemetry import mission_spec as _mission_spec
except Exception:
    _resolve_mission = None
    _jammer_range_m = None
    _tokenize_args = None
    FORMATIONS = ("line", "column", "abreast", "wedge", "echelon", "circle",
                  "diamond", "cube")
    formation_offsets = None
    custom_formations = lambda: []          # noqa: E731
    save_formation = None
    _clamp_to_arena = None
    _mission_goal_count = None
    _mission_spec = None

def snap(v, step=1.0):
    """The nearest whole metre.

    EVERYTHING PLACED BY HAND LANDS ON THE GRID. A drag produces whatever
    fraction of a metre the mouse happened to be on - 2.6371 - and those
    numbers then propagate into the composed run, the results CSV and every
    figure downstream, where they read as precision that was never measured.
    A formation you set up by eye is not accurate to a tenth of a millimetre
    and should not claim to be.

    The MODEL is untouched: formation_offsets still returns exact geometry, so
    a circle is still a circle. Only the coordinates a human put there are
    rounded, at the moment they are written.
    """
    try:
        return round(float(v) / step) * step
    except (TypeError, ValueError):
        return v


# The entry that means "leave every vehicle exactly where it was put". Used in
# the spawn dialog and as a value on the swept formation axis, so "no
# formation" is a choice you can compare against rather than an absence.
AS_SPAWNED = "(as spawned)"

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

# The sentinel an experiment writes for jam_rel_db when the jammer is SILENT.
# It has to match tools/sweep.py's JAM_OFF; it is repeated rather than
# imported because the Console must still draw a results CSV when the sweep
# module is not importable, and a wrong axis is worse than a missing plot.
JAM_OFF_DB = -999.0


# ---------------------------------------------------------------------------
# WHAT EVERY NUMBER MEANS - one glossary, used everywhere a metric is shown
# ---------------------------------------------------------------------------
# Reported: "can I have when I hover over bits in the results page a brief
# description such as the sinr_db". Right, and the fix is not a tooltip per
# widget - it is ONE definition per quantity, read by the results table, the
# metric chooser and anything added later. A metric that gains a tooltip in
# one place and not another is how a reader ends up with two ideas of what a
# column means.
#
# Each entry says what the number IS, and where it is easy to misread, what
# it is NOT. Anything without an entry falls back to its own name, which is
# a visible prompt to come and write one.
METRIC_HELP = {
    # --- what was configured -----------------------------------------------
    "authority": "WHO DECIDES. centralized: one coordinator tasks everyone. "
                 "decentralized: each vehicle decides for itself. "
                 "hierarchical: leaders decide for their squads. Independent "
                 "of routing - that is the framework's whole point.",
    "routing": "HOW THE MESSAGE TRAVELS. star: everything through the "
               "coordinator. mesh: any peer to any peer. tiered: up and down "
               "the command tree. A star can carry decentralized authority "
               "and a mesh can carry centralized.",
    "jam_rel_db": "THE JAMMER'S ADVANTAGE, P_j / P_t in dB - its transmit "
                  "power minus the fleet's. Dimensionless on purpose: the "
                  "absolute powers cancel, so the result is not about one "
                  "particular radio. 'off' is the silent-jammer control.",
    "formation": "The shape the fleet holds - a function of the fleet size "
                 "and spacing, not a list of coordinates.",
    "spacing": "Metres between adjacent slots in the formation. Held rigid "
               "through turns: the shape pivots, it does not squash.",
    "seed": "The random seed. Anything that differs between seeds is noise, "
            "not a finding.",
    # --- the outcome --------------------------------------------------------
    "mission_pass_frac": "Fraction of the fleet that COMPLETED the mission - "
                         "every waypoint, every lap. The outcome measure.",
    "mission_awaiting_orders": "Fraction of the fleet sitting still because "
                               "it could not reach its decider. Distinguishes "
                               "'beaten' from 'stuck': a fleet awaiting "
                               "orders was not out-driven, it was cut off.",
    "mission_drifted": "Fraction whose position belief drifted beyond "
                       "tolerance - they may be moving confidently to the "
                       "wrong place.",
    "penetration_m": "How far the fleet advanced along the axis of advance, "
                     "in metres. Measured along the axis, not as distance "
                     "travelled, so wandering sideways earns nothing.",
    "penetration_frac": "The same as a fraction of the full distance to the "
                        "objective. 1.0 is arrival.",
    "arrived": "How many vehicles reached the goal within their own arrival "
               "tolerance - which is the vehicle's own length, not a shared "
               "constant.",
    "commanded_fraction": "Fraction of ticks on which a vehicle could reach "
                          "whoever decides for it. Blue's side of the "
                          "interception coin: this is the link WORKING.",
    "ended_s": "Simulated seconds to the end of the run.",
    # --- the radio ----------------------------------------------------------
    "sinr_db": "SIGNAL TO INTERFERENCE PLUS NOISE, in dB. Wanted signal "
               "power divided by (every other same-band emitter + the noise "
               "floor). The single currency: distance, walls and jamming all "
               "reduce to this, and this becomes packet delivery, which "
               "becomes whether the link is up.",
    "worst_sinr_db": "The lowest SINR any link in the fleet saw. A fleet is "
                     "as commandable as its worst link, so the mean hides "
                     "exactly the thing that breaks it.",
    "pdr": "PACKET DELIVERY RATIO, 0 to 1 - the share of packets that "
           "arrive. Derived from SINR, not set directly.",
    "latency_ms": "One-way delay on the link, in milliseconds.",
    "quality": "Link quality, 0 to 1 - a display convenience derived from "
               "SINR. Never an input to anything.",
    "excess_db": "Extra path loss from WALLS, in dB, beyond free space: the "
                 "cheaper of going through the material or diffracting round "
                 "the end of it.",
    # --- interception -------------------------------------------------------
    "orders_sent": "How many orders blue transmitted. The denominator - "
                   "intercept_frac alone is misleading without it, because a "
                   "fleet that sends nothing is never overheard.",
    "orders_heard_any": "How many of those orders at least one red listener "
                        "decoded.",
    "intercept_frac": "Fraction of blue's orders that red heard. A LOWER "
                      "BOUND: the listener is omni, unaided and no better "
                      "than the intended receiver, where a real intercept "
                      "post would have gain and a quieter front end.",
    "first_intercept_s": "When red first learned anything, in simulated "
                         "seconds. Zero means it heard the opening order.",
    # --- position -----------------------------------------------------------
    "belief_error_m": "How far each vehicle's BELIEF about its own position "
                      "is from the truth, in metres. What GNSS denial costs, "
                      "and what missions actually steer on.",
    "station_err_m": "Formation SHAPE error: how far each vehicle is from "
                     "its rotated slot relative to the fleet's true centre. "
                     "Not transit error - a fleet moving as one rigid shape "
                     "scores zero however far it is from the waypoint.",
    "centre_err_m": "How wrong the fleet's idea of its own centre is. Built "
                    "from peer reports, so it degrades over the jammed link "
                    "even when every vehicle knows exactly where IT is.",
}


def metric_help(key):
    """One line saying what a metric is. Falls back to the bare name, which
    is a visible prompt that a definition is owed."""
    return METRIC_HELP.get(str(key), str(key))


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
        # NAMED POINTS the scene declares - HOME, FAR, A..F. These are what a
        # mission is written against ("advance to FAR"), so they are the other
        # half of setting up a run and they were invisible: you could see the
        # vehicles and not the thing they were being sent to.
        self.points = {}          # {name: {x, y, z}}
        # THE COMMAND LAYER: who answers to whom, and how much each link is
        # carrying. Drawn on top of the RF picture because they are different
        # questions - the links say what CAN carry traffic, this says what
        # actually is, and for whom.
        self.authority = {}       # {agent_id: {decider, tier, reachable}}
        # OFF UNTIL ASKED FOR. It is a diagnostic layer - arrows, rank marks
        # and per-link counts over the top of the world - and having it on at
        # startup means the first thing anyone sees is the busiest possible
        # picture. The button is one click away.
        self.show_c2 = False
        self.selected_points = set()
        # Indices into arena["walls"]. Walls are selected by index rather than
        # by identity because a wall has no id - it is geometry, not an actor.
        self.selected_walls = set()
        # THE SPECTRUM FIELD. Off by default and computed ON DEMAND - it costs
        # a third of a second for a 72x72 grid and there is no version of
        # "every frame" that is acceptable. Held as the raw grid plus a
        # rendered image, because the contours need the numbers and the fill
        # does not.
        self.spectrum = None
        self._spec_img = None
        self._spec_poses = None
        self.show_spectrum = False
        # THE OTHER QUESTION ON THE SAME PHYSICS. The spectrum field says how
        # much power is at a place; the wavefront says whose. Two toggles
        # rather than one view doing both, because colour cannot carry
        # magnitude and side at once without lying about one of them.
        self.wavefront = None
        self.advantage = None
        self._adv_img = None
        self.show_wavefront = False
        # THE THREE SECTION PLANES, and whether the scene is opened on them.
        # They belong to the SCENE, not to the camera: all three are live in
        # every view, which is what lets you cut into a building and then
        # walk round the cut in ISO.
        self.slice = {"x": 0.0, "y": 0.0, "z": 0.2}
        self.cut_away = False
        self._band = None         # rubber-band rectangle, screen coords
        self.scan_overlay = None   # (agent_id, scan) drawn in world coordinates
        self.scan_extras = []      # every OTHER ranging sensor on that agent
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

    def from_screen(self, pt, world):
        """Screen point -> world metres, in the two axes THIS view shows.

        The inverse of to_screen for the orthogonal views, which is what makes
        dragging a vehicle possible: TOP gives x and y, FRONT gives x and z,
        SIDE gives y and z. The third axis is whatever the vehicle already
        had, taken from `world` - dragging in plan must not silently zero an
        altitude you set in the side view.

        ISO is deliberately excluded. One screen point maps to a LINE in an
        isometric world, not a point, so a drag there would have to invent the
        depth - and inventing a coordinate the operator did not give is how a
        formation ends up subtly wrong with nothing on screen to show it.
        """
        s = self.scale()
        if s <= 0 or self.mode == self.ISO:
            return None
        u = (pt.x() - self.width() / 2 - self.pan.x()) / s
        v = -(pt.y() - self.height() / 2 - self.pan.y()) / s
        x, y, z = world
        if self.mode == self.TOP:
            return (u, v, z)
        if self.mode == self.FRONT:
            return (u, y, v)
        return (x, u, v)                      # SIDE

    def delta_world(self, d_screen):
        """A screen-pixel displacement as a world displacement, in the two
        axes THIS view shows.

        Separate from from_screen because a GROUP move is a translation, not a
        set of positions: every member has to move by the same vector, taken
        from the mouse, rather than each one being placed under the cursor in
        turn. Doing it in deltas also means no member accumulates rounding as
        it is dragged, which is what turns a formation into an almost-formation
        after a few moves.
        """
        s = self.scale()
        if s <= 0 or self.mode == self.ISO:
            return (0.0, 0.0, 0.0)
        du, dv = d_screen.x() / s, -d_screen.y() / s
        if self.mode == self.TOP:
            return (du, dv, 0.0)
        if self.mode == self.FRONT:
            return (du, 0.0, dv)
        return (0.0, du, dv)                  # SIDE

    def clamp(self, x, y, z):
        """Keep a dragged thing inside the arena.

        Not cosmetic, and deliberately THE MODEL'S OWN RULE rather than a
        second copy of it: objectives are validated against the arena when
        they are assigned, so a point dragged past the wall builds a run whose
        mission is refused - and then the fleet sits still and the reason is
        three panels away from the thing you did. Clamping against the very
        function that would reject it means the map cannot express a setup the
        model will not accept.
        """
        # SNAP FIRST, CLAMP SECOND. The other order lets a snap push a
        # position back out through the wall it was just pulled inside.
        x, y, z = snap(x), snap(y), snap(z)
        if _clamp_to_arena is None or not self.arena:
            return (x, y, z)
        cx, cy, cz = _clamp_to_arena(x, y, z, self.arena)
        # The clamp lands on the wall margin, which is not a whole metre;
        # step inward to the last integer that is still inside.
        return (math.copysign(math.floor(abs(cx)), cx) if cx != x else x,
                math.copysign(math.floor(abs(cy)), cy) if cy != y else y,
                math.floor(cz) if cz != z else z)

    def point_at(self, pt, radius_px=16.0):
        """The NAME of the scene point under this screen position."""
        best, bestd = None, radius_px
        for name, q in (self.points or {}).items():
            c = self.to_screen(_num(q.get("x")), _num(q.get("y")),
                               _num(q.get("z")))
            d = math.hypot(c.x() - pt.x(), c.y() - pt.y())
            if d <= bestd:
                best, bestd = name, d
        return best

    def bodies_in(self, rect, with_points=False):
        """(agent ids, point names) whose markers fall inside a screen rect.

        POINTS ARE EXCLUDED UNLESS ASKED FOR, and that is a safety rule rather
        than a preference. Penetration is measured to the goal point, so a box
        swept round a fleet that also quietly picked up FAR and dragged it
        along would move the ruler with the thing being measured - and the
        numbers would still look perfectly reasonable afterwards. Hold Shift
        to include points deliberately.
        """
        aids, names = set(), set()
        for a in self.agents:
            pose = a.get("pose", {})
            c = self.to_screen(_num(pose.get("x")), _num(pose.get("y")),
                               _num(pose.get("z")))
            if rect.contains(c):
                aids.add(a.get("id"))
        if with_points:
            for name, q in (self.points or {}).items():
                c = self.to_screen(_num(q.get("x")), _num(q.get("y")),
                                   _num(q.get("z")))
                if rect.contains(c):
                    names.add(name)
        return aids, names

    def wall_at(self, pt, radius_px=7.0):
        """The index of the wall under this screen position, or None.

        Distance to the SEGMENT, not to its ends, because a wall is a line and
        a click lands on the middle of one far more often than on a corner.
        """
        walls = (self.arena or {}).get("walls") or []
        best, bestd = None, radius_px
        for i, w in enumerate(walls):
            a = self.to_screen(_num(w.get("x1")), _num(w.get("y1")), 0.0)
            b = self.to_screen(_num(w.get("x2")), _num(w.get("y2")), 0.0)
            ex, ey = b.x() - a.x(), b.y() - a.y()
            L2 = ex * ex + ey * ey
            if L2 < 1e-9:
                d = math.hypot(pt.x() - a.x(), pt.y() - a.y())
            else:
                u = max(0.0, min(1.0, ((pt.x() - a.x()) * ex
                                       + (pt.y() - a.y()) * ey) / L2))
                d = math.hypot(pt.x() - (a.x() + u * ex),
                               pt.y() - (a.y() + u * ey))
            if d <= bestd:
                best, bestd = i, d
        return best

    def walls_in(self, rect):
        """Indices of every wall with any part inside a screen rect.

        Both ends OR the midpoint, so a long wall crossing the box is caught
        even when neither end is in it - which is the usual case when you
        sweep a corridor to re-material the lot.
        """
        out = set()
        for i, w in enumerate((self.arena or {}).get("walls") or []):
            a = self.to_screen(_num(w.get("x1")), _num(w.get("y1")), 0.0)
            b = self.to_screen(_num(w.get("x2")), _num(w.get("y2")), 0.0)
            mid = QPointF((a.x() + b.x()) / 2, (a.y() + b.y()) / 2)
            if rect.contains(a) or rect.contains(b) or rect.contains(mid):
                out.add(i)
        return out

    def agent_body_at(self, pt, radius_px=18.0):
        """The agent DICT under this point, for dragging.

        Deliberately not agent_at, which returns an id and is what selection
        has always used. A drag needs the body itself, because it writes the
        new pose straight into it - and having one function try to be both is
        how you end up calling .get("id") on a string.
        """
        best, bestd = None, radius_px
        for a in self.agents:
            pose = a.get("pose", {})
            c = self.to_screen(_num(pose.get("x")), _num(pose.get("y")),
                               _num(pose.get("z")))
            d = math.hypot(c.x() - pt.x(), c.y() - pt.y())
            if d <= bestd:
                best, bestd = a, d
        return best

    # -- interaction --------------------------------------------------------

    def wheelEvent(self, ev):
        self.zoom = max(0.15, min(12.0, self.zoom * 1.0015 ** ev.angleDelta().y()))
        self.update()

    # Set by the Console so a click on the map selects the agent everywhere.
    on_pick = None
    # Set by the Console to be told when a vehicle has been DRAGGED, so the
    # spawn table and the composed run follow the map rather than drifting
    # away from it.
    on_moved = None
    # Fired ONCE, when the vehicle is let go. Dragging fires on_moved every
    # mouse move, which is the right granularity for a readout and completely
    # the wrong one for recomposing the run - that writes a YAML and reloads
    # the world, and doing it sixty times a second while the mouse is down
    # would make the drag unusable.
    on_drop = None
    # Told when a scene POINT is dragged, and when the selection changes, so
    # the Console can write the new geometry into the composed run and say in
    # the status bar what is now under the mouse.
    on_point_moved = None
    on_selection = None
    # Called with the set of selected wall indices when it changes.
    on_wall_pick = None
    # True while Setup is placing a fleet: a press grabs a vehicle instead of
    # panning the view.
    placing = False

    def mousePressEvent(self, ev):
        # The key header is a control, not scenery: a click there toggles it
        # and does NOT start a pan or fall through to agent selection.
        if self._key_hit is not None and self._key_hit.contains(ev.position()):
            self.show_key = not self.show_key
            self._drag = None
            self._press_at = None
            self.update()
            return
        # MIDDLE DRAG ALWAYS PANS. While placing, the left button is the
        # placement tool - grab a vehicle, or sweep a selection box - so the
        # view needs its own button or there is no way to scroll a 200 m
        # corridor without moving something in it.
        if ev.button() == Qt.MiddleButton:
            self._drag = ev.position()
            self._press_at = None
            return
        # PLACING MODE: a press on a vehicle grabs it instead of panning, so
        # a formation can be built by dragging rather than typed as numbers.
        # Only before a run - moving an agent mid-run would be teleporting it,
        # which is not something the physics should have to explain.
        if getattr(self, "placing", False) and self.mode != self.ISO:
            # WALLS ARE PICKED BEFORE VEHICLES ONLY WHEN NOTHING ELSE IS
            # THERE. An agent standing against a wall is the common case in a
            # maze, and grabbing the wall instead of the car it is touching
            # would make the fleet unplaceable exactly where placement is
            # hardest.
            hit = self.agent_body_at(ev.position())
            if hit is None and self.point_at(ev.position()) is None:
                wi = self.wall_at(ev.position())
                if wi is not None:
                    add = bool(ev.modifiers()
                               & (Qt.ShiftModifier | Qt.ControlModifier))
                    sel = set(self.selected_walls) if add else set()
                    sel.symmetric_difference_update({wi}) if add else \
                        sel.update({wi})
                    self.selected_walls = sel
                    self.selected, self.selected_points = set(), set()
                    self._drag = None
                    self._press_at = None
                    if self.on_wall_pick:
                        self.on_wall_pick(self.selected_walls)
                    self.update()
                    return
            pname = None if hit is not None else self.point_at(ev.position())
            if hit is not None or pname is not None:
                # GRABBING A MEMBER OF THE SELECTION MOVES THE WHOLE
                # SELECTION. That is the difference between placing a fleet
                # and placing four vehicles that happen to be near each other:
                # once the shape is right, the shape is the thing you move,
                # and dragging it apart one vehicle at a time to reposition it
                # is how a formation stops being one.
                inset = (hit is not None and hit.get("id") in self.selected) \
                    or (pname is not None and pname in self.selected_points)
                if not inset:
                    self.selected = {hit.get("id")} if hit is not None else set()
                    self.selected_points = {pname} if pname is not None else set()
                    if hit is not None and self.on_pick:
                        self.on_pick(hit.get("id"))
                self._grab = self._grab_set(ev.position())
                self._drag = None
                self._press_at = ev.position()
                self.update()
                return
            # Empty space: sweep out a selection box.
            self._band = QRectF(ev.position(), ev.position())
            self._drag = None
            self._press_at = ev.position()
            return
        self._drag = ev.position()
        self._press_at = ev.position()

    def _grab_set(self, at):
        """Everything the drag will move, with the pose each started from.

        Recorded ONCE, at the press: the move is then original + delta rather
        than a chain of relative nudges, so nothing accumulates rounding and
        letting go in the same place leaves the formation exactly as it was.
        """
        items = []
        for a in self.agents:
            if a.get("id") in self.selected and not a.get("ghost"):
                pose = a.setdefault("pose", {})
                items.append(("agent", a.get("id"), pose,
                              (_num(pose.get("x")), _num(pose.get("y")),
                               _num(pose.get("z")))))
        for name in self.selected_points:
            q = (self.points or {}).get(name)
            if q is not None:
                items.append(("point", name, q,
                              (_num(q.get("x")), _num(q.get("y")),
                               _num(q.get("z")))))
        return {"at": at, "items": items}

    def mouseMoveEvent(self, ev):
        if self._band is not None and ev.buttons():
            self._band = QRectF(self._press_at, ev.position()).normalized()
            self.update()
            return
        grab = getattr(self, "_grab", None)
        if grab is not None and ev.buttons():
            dx, dy, dz = self.delta_world(ev.position() - grab["at"])
            for kind, ident, holder, (x0, y0, z0) in grab["items"]:
                holder["x"], holder["y"], holder["z"] = self.clamp(
                    x0 + dx, y0 + dy, z0 + dz)
                if kind == "agent" and self.on_moved:
                    self.on_moved(ident, holder)
                elif kind == "point" and self.on_point_moved:
                    self.on_point_moved(ident, holder)
            self.update()
            return
        if self._drag is not None and ev.buttons():
            self.pan += ev.position() - self._drag
            self._drag = ev.position()
            self.update()

    def mouseReleaseEvent(self, ev):
        # A SELECTION BOX. Everything whose marker fell inside is now one
        # thing to move - which is what "translate the formation" means:
        # sweep it, then drag any member.
        if self._band is not None:
            band, self._band = self._band, None
            if band.width() > 3 and band.height() > 3:
                self.selected, self.selected_points = self.bodies_in(
                    band, with_points=bool(ev.modifiers() & Qt.ShiftModifier))
                # A BAND THAT CAUGHT NO VEHICLES IS A BAND MEANT FOR WALLS.
                # Sweeping a corridor to re-material eight partitions at once
                # is the whole reason multi-select exists here, and making it
                # a separate mode would be one more thing to remember.
                walls = self.walls_in(band)
                if walls and not self.selected and not self.selected_points:
                    self.selected_walls = walls
                    if self.on_wall_pick:
                        self.on_wall_pick(self.selected_walls)
                elif self.selected or self.selected_points:
                    self.selected_walls = set()
                if self.on_selection:
                    self.on_selection(self.selected, self.selected_points)
            else:
                # A click on empty space clears the selection, so there is an
                # obvious way out of a group without picking members off.
                self.selected, self.selected_points = set(), set()
                self.selected_walls = set()
                if self.on_wall_pick:
                    self.on_wall_pick(self.selected_walls)
                if self.on_selection:
                    self.on_selection(self.selected, self.selected_points)
            self.update()
            return
        grab = getattr(self, "_grab", None)
        if grab is not None:
            self._grab = None
            moved = math.hypot(ev.position().x() - grab["at"].x(),
                               ev.position().y() - grab["at"].y())
            if self.on_drop and moved >= 2:
                self.on_drop([i[1] for i in grab["items"] if i[0] == "agent"],
                             [i[1] for i in grab["items"] if i[0] == "point"])
            return
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
                self._spectrum(p)
                self._contested(p)
                self._grid(p)
                self._bounds(p)
                self._walls(p)
                self._wavefront(p)
            self._scan_fan(p)
            self._range_rings(p)
            self._jammer_affect_lines(p)
            self._belief_ghosts(p)
            self._link_lines(p)
            self._points(p)
            if self.show_c2:
                self._authority_arrows(p)
            for a in self.agents:
                # GHOSTED, NOT DELETED. An agent on the far side of a cut is
                # drawn at a fifth strength rather than removed: you asked to
                # see INSIDE something, and a vehicle that vanishes entirely
                # reads as a bug or a loss, which is exactly the wrong signal
                # in an app whose subject is losing contact with vehicles.
                faint = self.cut_away and self._past_cut(a)
                if faint:
                    p.setOpacity(0.2)
                self._agent(p, a)
                if faint:
                    p.setOpacity(1.0)
            if self.show_c2:
                self._rank_glyphs(p)
                self._link_loads(p)
            if self.show_axes:
                self._axes(p)
            self._overlay(p)
            self._spectrum_key(p)
            self._wavefront_key(p)
            self._selection_box(p)
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

    # -- the command layer --------------------------------------------------

    C2_COL = "#C9A227"        # one colour, used by nothing else on the map

    def _authority_arrows(self, p):
        """An arrow from each vehicle to WHOEVER DECIDES FOR IT.

        This is the authority axis drawn as what it is - a tree. Centralized
        is a fan converging on the coordinator; hierarchical is two levels;
        decentralized draws nothing at all, which is the honest picture of
        every vehicle deciding for itself.

        Drawn UNDER the vehicles and in a colour nothing else uses, so it
        reads as a separate question from the RF links crossing the same
        space. Dashed when the decider cannot currently be reached: the
        vehicle still ANSWERS to them, it just cannot hear them, and those are
        different failures.
        """
        by = {a.get("id"): a for a in self.agents}
        for aid, auth in (self.authority or {}).items():
            dec = (auth or {}).get("decider")
            if not dec or dec == aid:
                continue                    # decides for itself: no arrow
            a, b = by.get(aid), by.get(dec)
            if not a or not b:
                continue
            pa = a.get("pose", {})
            pb = b.get("pose", {})
            s0 = self.to_screen(_num(pa.get("x")), _num(pa.get("y")),
                                _num(pa.get("z")))
            s1 = self.to_screen(_num(pb.get("x")), _num(pb.get("y")),
                                _num(pb.get("z")))
            col = QColor(self.C2_COL)
            if not auth.get("reachable", True):
                col = QColor("#E08A3C")
            pen = QPen(col, 2.2)
            pen.setStyle(Qt.SolidLine if auth.get("reachable", True)
                         else Qt.DashLine)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            # Stop short of the body so the arrowhead sits beside the vehicle
            # rather than under it.
            dx, dy = s1.x() - s0.x(), s1.y() - s0.y()
            d = math.hypot(dx, dy) or 1.0
            ux, uy = dx / d, dy / d
            gap = min(14.0, d * 0.25)
            e = QPointF(s1.x() - ux * gap, s1.y() - uy * gap)
            p.drawLine(QPointF(s0.x() + ux * gap, s0.y() + uy * gap), e)
            # A chevron pointing AT the decider. Direction is the whole
            # message: it says who answers to whom, not merely that they are
            # associated.
            ang = math.atan2(uy, ux)
            for sgn in (+1, -1):
                a2 = ang + math.pi + sgn * 0.42
                p.drawLine(e, QPointF(e.x() + 11 * math.cos(a2),
                                      e.y() + 11 * math.sin(a2)))

    def _rank_glyphs(self, p):
        """A mark above whoever holds rank. Coordinator, leader, or nothing.

        The cheapest possible answer to "who is senior here", and it makes a
        hierarchy over a star visibly absurd: a leader glyph with no arrows
        pointing at it is a rank nothing routes through.
        """
        rank = {}
        for aid, auth in (self.authority or {}).items():
            dec = (auth or {}).get("decider")
            tier = (auth or {}).get("tier")
            if not dec or dec == aid:
                continue
            # The decider's rank is named by how the SUBORDINATE reaches it.
            rank[dec] = ("coordinator" if tier == "coordinator"
                         else rank.get(dec, "leader"))
        if not rank:
            return
        p.setFont(QFont("Consolas", 11))
        for a in self.agents:
            r = rank.get(a.get("id"))
            if not r:
                continue
            pose = a.get("pose", {})
            c = self.to_screen(_num(pose.get("x")), _num(pose.get("y")),
                               _num(pose.get("z")))
            p.setPen(QPen(QColor(self.C2_COL)))
            p.setBrush(Qt.NoBrush)
            p.drawText(QPointF(c.x() - 5, c.y() - 16),
                       "\u2605" if r == "coordinator" else "\u25b2")

    def _link_loads(self, p):
        """HOW MANY VEHICLES' ORDERS CROSS THIS LINK, as a number on the line.

        A number rather than a thickness, deliberately. If everything reaches
        the coordinator through one vehicle, the link into that vehicle reads
        5 and the links out of it read 1 - and 5 is exact where a fatter line
        is a guess the reader has to calibrate against the other lines. It
        also names the stake: that vehicle is not merely ON the path, it is
        carrying four other people's command, and if it drops they go with it.

        Only carried links get a number. A spare link carries nothing by
        definition, and printing 0 along every unused pair would bury the
        ones that matter.
        """
        by = {a.get("id"): a for a in self.agents}
        p.setFont(QFont("Consolas", 9))
        for link in self.links:
            n = int(link.get("carries") or 0)
            if n <= 0 or not link.get("active", True):
                continue
            a, b = by.get(link.get("a")), by.get(link.get("b"))
            if not a or not b:
                continue
            pa, pb = a.get("pose", {}), b.get("pose", {})
            s0 = self.to_screen(_num(pa.get("x")), _num(pa.get("y")),
                                _num(pa.get("z")))
            s1 = self.to_screen(_num(pb.get("x")), _num(pb.get("y")),
                                _num(pb.get("z")))
            mid = QPointF((s0.x() + s1.x()) / 2, (s0.y() + s1.y()) / 2)
            r = 8.0 if n < 10 else 11.0
            # A disc behind it, so a number sitting on a line stays readable.
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(24, 28, 31, 235)))
            p.drawEllipse(mid, r, r)
            p.setPen(QPen(QColor(self.C2_COL), 1.2))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(mid, r, r)
            p.setPen(QPen(QColor(self.C2_COL)))
            p.drawText(QPointF(mid.x() - (4 if n < 10 else 7),
                               mid.y() + 4), str(n))

    def _points(self, p):
        """The scene's named points - what a mission is written against.

        Drawn as an open cross rather than a filled marker, so a point never
        reads as a vehicle. They were not drawn at all until now, which meant
        the destination in "advance to FAR" was the one part of a run you
        could not see: you set up the fleet by eye and the goal by faith.
        """
        if not self.points:
            return
        p.setFont(QFont("Consolas", 8))
        for name, q in sorted(self.points.items()):
            c = self.to_screen(_num(q.get("x")), _num(q.get("y")),
                               _num(q.get("z")))
            lit = name in self.selected_points
            col = QColor("#E08A3C") if lit else QColor("#7FA8B8")
            p.setPen(QPen(col, 2.0 if lit else 1.2))
            p.setBrush(Qt.NoBrush)
            r = 7.0 if lit else 5.0
            p.drawLine(QPointF(c.x() - r, c.y()), QPointF(c.x() + r, c.y()))
            p.drawLine(QPointF(c.x(), c.y() - r), QPointF(c.x(), c.y() + r))
            p.drawEllipse(c, r, r)
            p.drawText(QPointF(c.x() + r + 3, c.y() - r), name)

    def _selection_box(self, p):
        """The rubber band, while it is being swept."""
        if self._band is None:
            return
        p.setPen(QPen(QColor("#E08A3C"), 1.0, Qt.DashLine))
        p.setBrush(QBrush(QColor(224, 138, 60, 28)))
        p.drawRect(self._band)

    def _grid(self, p):
        """One line per metre across the arena footprint.

        SUPPRESSED WHILE THE SPECTRUM IS UP. Twenty grid lines over a
        continuous field is two lattices fighting, and the one that loses is
        the data - the contours are the structure worth reading there, and the
        metre grid is what the scale bar is for.
        """
        if getattr(self, "show_spectrum", False) and self.mode == self.TOP \
                and self.spectrum:
            return
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

    # MATERIAL, AS A LINE STYLE. Colour is the SIDE everywhere else on this
    # map and it stays that way; a wall is not on anybody's side. So the
    # material is carried by weight and dash instead, and the wall's own
    # greyish tone separates structure from agents without competing with them.
    WALL_STYLE = {
        # material:      (width, dash,          tone)
        "plasterboard": (3.0, Qt.SolidLine,   "#7E8A90"),
        "brick":        (4.5, Qt.SolidLine,   "#8A6F63"),
        "concrete":     (5.5, Qt.SolidLine,   "#6E7276"),
        "wood":         (3.5, Qt.SolidLine,   "#8A7A5E"),
        # Glass is drawn thin and dashed because that is what it IS to this
        # model: nearly transparent to radio and nearly invisible to a lidar.
        # A vehicle with no map will drive into it, and the drawing should
        # make that look plausible rather than like a mistake.
        "glass":        (1.6, Qt.DashLine,    "#5FA9C4"),
        # Metal gets the heaviest line: opaque to radio, and the only way past
        # it for a signal is round the end.
        "metal":        (5.0, Qt.SolidLine,   "#B9C2C8"),
    }

    def set_spectrum(self, field):
        """Hand the viewport a freshly computed field, or None to clear it."""
        self.spectrum = field
        self._spec_img = None
        # WHERE EVERYTHING WAS WHEN THIS WAS COMPUTED. The field costs a third
        # of a second, so it is a SNAPSHOT and not a live layer - and a
        # snapshot that silently goes on being drawn after the fleet has moved
        # (or after a replay has been rewound) is worse than no picture, since
        # it looks current. Remembering the poses lets the key say so.
        self._spec_poses = {a.get("id"): (round(_num((a.get("pose") or {}).get("x")), 2),
                                          round(_num((a.get("pose") or {}).get("y")), 2))
                            for a in (self.agents or [])}
        self.update()

    def spectrum_is_stale(self):
        """Has anything moved since the field was computed?"""
        if not self.spectrum or self._spec_poses is None:
            return False
        now = {a.get("id"): (round(_num((a.get("pose") or {}).get("x")), 2),
                             round(_num((a.get("pose") or {}).get("y")), 2))
               for a in (self.agents or [])}
        return now != self._spec_poses

    def _spectrum_image(self):
        """The field as a small QImage, scaled up at draw time.

        One pixel per grid cell and let Qt smooth it: a 72 x 72 image
        stretched over the arena is exactly the resolution the data has, and
        drawing six thousand rectangles to pretend otherwise would be slower
        AND a lie about how finely this was computed.
        """
        f = self.spectrum
        if not f:
            return None
        if self._spec_img is not None:
            return self._spec_img
        nx, ny = f["nx"], f["ny"]
        lo, hi = spectrum_range(f)
        self._spec_lo, self._spec_hi = lo, hi
        img = QImage(nx, ny, QImage.Format_ARGB32)
        img.fill(Qt.transparent)
        for j in range(ny):
            for i in range(nx):
                c = spectrum_colour(f["dbm"][j][i], lo, hi)
                if c is None:
                    continue
                c.setAlpha(210)
                img.setPixelColor(i, ny - 1 - j, c)   # world y up, image y down
        self._spec_img = img
        return img

    # Which view is allowed to draw which slice. A field of x and y drawn on
    # an elevation would be claiming a section this has not computed, so the
    # pairing is enforced rather than trusted: the toolbar recomputes the
    # field when the view changes, and until it has, nothing is drawn.
    SPECTRUM_VIEW = {"top": "TOP", "front": "FRONT", "side": "SIDE"}

    def _field_point(self, f, iu, iv):
        """Grid cell centre -> the world point it was sampled at.

        The one place the plane's geometry is decoded. Everything that draws
        the field - the image rectangle, every contour segment - goes through
        here, so adding a plane is a change in the model and one line here,
        not a new special case in each drawing routine.
        """
        u = f["u0"] + (f["u1"] - f["u0"]) * (iu + 0.5) / f["nx"]
        v = f["v0"] + (f["v1"] - f["v0"]) * (iv + 0.5) / f["ny"]
        return self._field_world(f, u, v)

    def _field_world(self, f, u, v):
        plane = f.get("plane", "top")
        at = _num(f.get("at"), 0.2)
        if plane == "front":
            return (u, at, v)
        if plane == "side":
            return (at, u, v)
        return (u, v, at)

    def _spectrum(self, p):
        """The field, then its contours."""
        f = self.spectrum
        if not self.show_spectrum or not f:
            return
        if self.mode != self.SPECTRUM_VIEW.get(f.get("plane", "top"), "TOP"):
            return
        img = self._spectrum_image()
        if img is None:
            return
        # The image spans the plane's own two axes, whatever they are: its
        # corners are the corners of the sampled rectangle, projected.
        tl = self._field_world(f, f["u0"], f["v1"])
        br = self._field_world(f, f["u1"], f["v0"])
        p.setRenderHint(QPainter.SmoothPixmapTransform, True)
        p.drawImage(QRectF(self.to_screen(*tl), self.to_screen(*br)), img)

        # CONTOURS AT 10 dB, with the sensitivity line picked out. The steps
        # are what make it a map rather than a smear; the sensitivity contour
        # is the one that MEANS something - inside it there is enough power to
        # decode what is out there, outside it there is not.
        grid = f["dbm"]
        vals = [v for row in grid for v in row if v is not None]
        if not vals:
            return
        lo, hi = min(vals), max(vals)
        levels = [SPECTRUM_SENSITIVITY_DBM]
        v = math.ceil(lo / 10.0) * 10.0
        while v <= hi:
            if abs(v - SPECTRUM_SENSITIVITY_DBM) > 1e-6:
                levels.append(v)
            v += 10.0
        for lev in levels:
            key = abs(lev - SPECTRUM_SENSITIVITY_DBM) < 1e-6
            pen = QPen(QColor(232, 238, 242, 200 if key else 85),
                       1.6 if key else 0.9)
            if not key:
                pen.setStyle(Qt.DotLine)
            p.setPen(pen)
            for (ax, ay), (bx, by) in contour_segments(grid, lev):
                p.drawLine(self.to_screen(*self._field_point(f, ax, ay)),
                           self.to_screen(*self._field_point(f, bx, by)))

    def _spectrum_key(self, p):
        """The ramp with numbers on it. A field with no scale is a picture."""
        f = self.spectrum
        if not self.show_spectrum or not f:
            return
        if self.mode != self.SPECTRUM_VIEW.get(f.get("plane", "top"), "TOP"):
            return
        self._spectrum_image()          # ensures the range is computed
        x, y0, w, h = 12, 58, 14, 150
        bot_v = getattr(self, "_spec_lo", SPECTRUM_RAMP[0][0])
        top_v = getattr(self, "_spec_hi", SPECTRUM_RAMP[-1][0])
        for k in range(h):
            u = k / float(h - 1)
            p.setPen(QPen(spectrum_colour(top_v + (bot_v - top_v) * u,
                                          bot_v, top_v)))
            p.drawLine(x, y0 + k, x + w, y0 + k)
        p.setPen(QPen(QColor(C_LINE)))
        p.setBrush(Qt.NoBrush)
        p.drawRect(x, y0, w, h)
        p.setFont(QFont("Consolas", 7))
        marks = [top_v, bot_v]
        if bot_v < SPECTRUM_SENSITIVITY_DBM < top_v:
            marks.insert(1, SPECTRUM_SENSITIVITY_DBM)
        for dbm in marks:
            u = (top_v - dbm) / max(top_v - bot_v, 1e-9)
            yy = y0 + u * (h - 1)
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(QPointF(x + w + 4, yy + 3), f"{dbm:.0f}")
            if abs(dbm - SPECTRUM_SENSITIVITY_DBM) < 1e-6:
                p.setPen(QPen(QColor(232, 238, 242, 200)))
                p.drawLine(QPointF(x, yy), QPointF(x + w, yy))
                p.drawText(QPointF(x + w + 28, yy + 3), "sensitivity")
        band = _num(f.get("band_mhz"))
        p.setPen(QPen(QColor(C_TEXT)))
        p.setFont(QFont("Consolas", 8))
        p.drawText(QPointF(x, y0 - 8),
                   f"{band/1000:.1f} GHz" if band >= 1000 else f"{band:.0f} MHz")
        p.setPen(QPen(QColor(C_DIM)))
        p.setFont(QFont("Consolas", 7))
        p.drawText(QPointF(x, y0 + h + 14), "dBm on the air (fixed scale)")
        # THE NUMBER THAT MOVES. dB are a ratio and hard to feel; the share of
        # the floor a receiver could actually work on is not, and it is what
        # changes when the fleet turns its power up or down.
        cov = spectrum_coverage(f)
        p.setPen(QPen(QColor(C_TEXT)))
        p.drawText(QPointF(x, y0 + h + 50),
                   f"{cov*100:.0f}% above sensitivity")
        p.setPen(QPen(QColor(C_DIM)))
        # WHERE THE SECTION WAS CUT. A contour map with no stated section
        # plane is the commonest way to mislead with one: the same room at
        # 0.2 m and at 2.5 m are different pictures, and only one of them is
        # about the vehicles on the floor.
        p.drawText(QPointF(x, y0 + h + 26),
                   f"{f.get('faxis','z')} = {_num(f.get('at')):.2f} m")
        # AND WHETHER IT STILL DESCRIBES THE SCENE IN FRONT OF YOU.
        if self.spectrum_is_stale():
            p.setPen(QPen(QColor(C_WARN)))
            p.drawText(QPointF(x, y0 + h + 38),
                       "STALE - click \u25a6 to refresh")

    def _past_cut(self, a):
        """Is this agent on the far side of any section plane?

        The kept half is the LOW side of each plane, which is the convention
        a cut-away drawing uses: you remove the material between you and the
        thing you want to look at. Z is the exception in spirit but not in
        form - an agent above the horizontal section is above the storey you
        are looking at, and goes faint for the same reason a wall below it
        disappears.
        """
        pose = (a.get("pose") or a.get("start") or {})
        s = self.slice
        return (_num(pose.get("x")) > s["x"] + 1e-6
                or _num(pose.get("y")) > s["y"] + 1e-6
                or _num(pose.get("z")) > s["z"] + 1e-6)

    def set_wavefront(self, rings, advantage=None):
        """Hand the viewport a fresh set of rings, or None to clear them."""
        self.wavefront = rings
        self.advantage = advantage
        self._adv_img = None
        self.update()

    def _contested(self, p):
        """The ground where the other side is the loudest thing on the band.

        Drawn UNDER the walls and the rings, as ground rather than as a mark,
        because that is what it is: an area, not an object. Only the negative
        half of the field is painted - where blue is winning there is nothing
        to say, and shading it too would turn a statement into wallpaper.
        """
        f = self.advantage
        if not self.show_wavefront or not f or self.mode != self.TOP:
            return
        if self._adv_img is None:
            nx, ny = f["nx"], f["ny"]
            img = QImage(nx, ny, QImage.Format_ARGB32)
            img.fill(Qt.transparent)
            for j in range(ny):
                for i in range(nx):
                    v = f["dbm"][j][i]
                    if v is None or v >= 0.0:
                        continue
                    # Full strength by 20 dB down: past that the point is made
                    # and more ink only hides the room.
                    k = min(1.0, -v / 20.0)
                    c = QColor(NETWORK_COLOURS["red"])
                    c.setAlpha(int(30 + 90 * k))
                    img.setPixelColor(i, ny - 1 - j, c)
            self._adv_img = img
        hx, hy = f["hx"], f["hy"]
        p.setRenderHint(QPainter.SmoothPixmapTransform, True)
        p.drawImage(QRectF(self.to_screen(-hx, hy, 0.0),
                           self.to_screen(hx, -hy, 0.0)), self._adv_img)

    def _wavefront(self, p):
        """Equal-power rings, one colour per side.

        PLAN VIEW ONLY: the rings are traced on a horizontal plane, and drawn
        on an elevation they would be claiming a section nobody cut.

        Each ring is an absolute dBm level, so a blue ring and a red ring of
        the same level mean the same thing and where they cross is a real
        statement about two signals. Inner rings are drawn more strongly than
        outer ones - the fade is the strength, and it is the same fade for
        both sides so neither looks louder for being drawn harder.
        """
        wf = self.wavefront
        if not self.show_wavefront or not wf or self.mode != self.TOP:
            return
        floor = _num(wf.get("floor_dbm"), -85.0)
        for e in wf.get("emitters") or []:
            col = QColor(network_colour(e.get("network") or "blue"))
            for ring in e.get("rings") or []:
                pts = ring.get("pts") or []
                if len(pts) < 8:
                    continue
                # How far above the sensitivity floor this ring is, 0 to 1.
                k = min(1.0, max(0.0, (_num(ring.get("dbm")) - floor) / 30.0))
                c = QColor(col)
                c.setAlpha(int(60 + 150 * k))
                pen = QPen(c, 1.0 + 1.6 * k)
                # A jammer's rings are dashed. It is the same physics and the
                # same scale - the dash says the intent is denial, not
                # traffic, which colour alone cannot carry once both sides
                # have transmitters.
                if e.get("jammer"):
                    pen.setStyle(Qt.DashLine)
                p.setPen(pen)
                p.setBrush(Qt.NoBrush)
                poly = QPolygonF([self.to_screen(x, y, 0.0) for x, y in pts])
                p.drawPolygon(poly)

    def _wavefront_key(self, p):
        """Who is who, and what one ring is worth."""
        wf = self.wavefront
        if not self.show_wavefront or not wf or self.mode != self.TOP:
            return
        x, y = 12, self.height() - 74
        p.setFont(QFont("Consolas", 8))
        p.setPen(QPen(QColor(C_TEXT)))
        band = _num(wf.get("band_mhz"))
        p.drawText(QPointF(x, y),
                   f"{band/1000:.1f} GHz" if band >= 1000 else f"{band:.0f} MHz")
        p.setFont(QFont("Consolas", 7))
        rows = [("blue", "friendly", Qt.SolidLine),
                ("red", "hostile", Qt.SolidLine),
                ("red", "jammer", Qt.DashLine)]
        for i, (side, label, style) in enumerate(rows):
            yy = y + 12 + i * 11
            pen = QPen(QColor(NETWORK_COLOURS[side]), 2.0)
            pen.setStyle(style)
            p.setPen(pen)
            p.drawLine(QPointF(x, yy - 3), QPointF(x + 18, yy - 3))
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(QPointF(x + 24, yy), label)
        p.setPen(QPen(QColor(C_DIM)))
        # The step is whatever the ROOM needed - a loud small room gets
        # coarser rungs so the picture does not become forty contours - so
        # the key reports the step that was used, never the one asked for.
        p.drawText(QPointF(x, y + 12 + len(rows) * 11),
                   f"one ring = {_num(wf.get('step_db'), 6):.0f} dB, "
                   f"{_num(wf.get('lo_dbm'), -85):.0f} to "
                   f"{_num(wf.get('hi_dbm'), 0):.0f} dBm in this room")
        if self.advantage:
            p.drawText(QPointF(x, y + 23 + len(rows) * 11),
                       "shaded: hostile signal is the louder one here")

    def _walls(self, p):
        """Interior walls, with their material legible at a glance.

        Drawn under everything else because a wall is the room, not an actor
        in it. A wall lower than the arena is drawn dotted as well as styled -
        a half-height partition is something a drone flies over and a car goes
        round, and that distinction is invisible from above unless the drawing
        says it.
        """
        walls = (self.arena or {}).get("walls") or []
        if not walls:
            return
        top = _num((self.arena.get("extent") or {}).get("z"), 3.0)
        if self.mode in (self.FRONT, self.SIDE):
            self._walls_in_section(p, walls, top)
            return
        for i, w in enumerate(walls):
            # CUT AWAY. A wall shorter than the Z plane is BELOW the cut - you
            # are looking down from above it, so it is not there. This is the
            # same rule the radio already obeys (a link passes over a wall it
            # is higher than), which is the point: what you see is what the
            # signal sees.
            if self.cut_away and _num(w.get("height"), top) < self.slice["z"]:
                continue
            width, dash, tone = self.WALL_STYLE.get(
                str(w.get("material", "")).lower(), (3.0, Qt.SolidLine, "#7E8A90"))
            h = _num(w.get("height"), top)
            low = h < top - 1e-6
            a = self.to_screen(_num(w.get("x1")), _num(w.get("y1")), 0.0)
            b = self.to_screen(_num(w.get("x2")), _num(w.get("y2")), 0.0)
            # SELECTED WALLS GET A HALO, not a colour change. The line's own
            # colour and weight are carrying the MATERIAL, which is the thing
            # you are about to edit - recolouring it to show selection would
            # hide the very property the dialog is about to ask you for.
            if i in (self.selected_walls or ()):
                halo = QPen(QColor(224, 138, 60, 150), width + 6)
                halo.setCapStyle(Qt.RoundCap)
                p.setPen(halo)
                p.drawLine(a, b)
            pen = QPen(QColor(tone), width)
            pen.setStyle(Qt.DotLine if low else dash)
            pen.setCapStyle(Qt.FlatCap)
            p.setPen(pen)
            p.drawLine(a, b)
            if low:
                # Say the height, because "you can fly over this" is the whole
                # information content of a low wall and a dotted line alone
                # does not carry a number.
                p.setFont(QFont("Consolas", 7))
                p.setPen(QPen(QColor(C_DIM)))
                p.drawText(QPointF((a.x() + b.x()) / 2 + 4,
                                   (a.y() + b.y()) / 2 - 3), f"{h:.1f} m")

    def _walls_in_section(self, p, walls, top):
        """Walls as a section drawing, in the elevations.

        In plan a wall is a line and drawing it as one is right. In an
        elevation it is not: projecting the same line puts every wall flat on
        the floor, which is worse than useless next to a vertical slice of the
        field, because the shadow has nothing casting it.

        So each wall the section plane CROSSES is drawn as what a section
        drawing would show - a bar standing from the floor to its own height,
        at the point the cut passes through it. A wall the plane misses is
        drawn as a faint tick at its own height instead: it is in the room,
        it is not in this cut, and pretending either way would mislead.
        """
        f = self.spectrum if self.show_spectrum else None
        cut = _num(f.get("at")) if f else None
        vertical = self.mode == self.FRONT     # front cuts at a fixed y
        for i, w in enumerate(walls):
            width, dash, tone = self.WALL_STYLE.get(
                str(w.get("material", "")).lower(),
                (3.0, Qt.SolidLine, "#7E8A90"))
            h = _num(w.get("height"), top)
            x1, y1 = _num(w.get("x1")), _num(w.get("y1"))
            x2, y2 = _num(w.get("x2")), _num(w.get("y2"))
            # Where the cut crosses it, in the axis this elevation shows
            # across the screen. FRONT shows x and cuts at a y; SIDE shows y
            # and cuts at an x - so the roles of the two coordinates swap.
            a0, a1 = (y1, y2) if vertical else (x1, x2)     # cut axis
            s0, s1 = (x1, x2) if vertical else (y1, y2)     # screen axis
            hit = cut is not None and min(a0, a1) - 1e-6 <= cut <= max(a0, a1) + 1e-6
            if hit and abs(a1 - a0) > 1e-9:
                s = s0 + (s1 - s0) * (cut - a0) / (a1 - a0)
            elif hit:
                s = s0
            else:
                s = (s0 + s1) / 2.0
            def _pt(sv, zv):
                return (self.to_screen(sv, cut if cut is not None else 0.0, zv)
                        if vertical else
                        self.to_screen(cut if cut is not None else 0.0, sv, zv))
            pen = QPen(QColor(tone), (width + 2) if hit else 1.0)
            pen.setStyle(Qt.SolidLine if hit else Qt.DotLine)
            pen.setCapStyle(Qt.FlatCap)
            if i in (self.selected_walls or ()):
                halo = QPen(QColor(224, 138, 60, 150), width + 8)
                p.setPen(halo)
                p.drawLine(_pt(s, 0.0), _pt(s, h))
            p.setPen(pen)
            p.drawLine(_pt(s, 0.0), _pt(s, h))
            if hit:
                p.setFont(QFont("Consolas", 7))
                p.setPen(QPen(QColor(C_DIM)))
                p.drawText(_pt(s, h) + QPointF(4, -3), f"{h:.1f} m")

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
        # EVERY RANGING SENSOR THIS AGENT CARRIES, not just the first. A car
        # with a lidar and a depth camera has two fields of view, and drawing
        # one of them makes the other invisible - which is the opposite of the
        # point of fitting both.
        for extra in (self.scan_extras or []):
            self._one_scan(p, agent, extra)
        self._one_scan(p, agent, scan)

    def _one_scan(self, p, agent, scan):
        """One sensor's return, drawn in world coordinates.

        The two sensors are drawn in DIFFERENT COLOURS and their reach rings
        are labelled with the sensor's own name, because the interesting thing
        about carrying both is where one stops and the other does not. A
        single colour would show two arcs and leave you working out which was
        which from their size.
        """
        if not scan or not scan.get("ranges"):
            return
        agent_id = agent.get("id")
        depth = bool(scan.get("slice_of_depth_image"))
        hull_col = QColor(120, 190, 130, 22) if depth else QColor(62, 154, 168, 20)
        dot_col = QColor(150, 224, 165) if depth else QColor(120, 214, 226)

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
            p.setBrush(QBrush(hull_col))
            p.setPen(Qt.NoPen)
            p.drawPolygon(hull)

        # THE EDGES OF THE FIELD OF VIEW, drawn out to the sensor's rated
        # range whether or not anything returned along them. For a 270-degree
        # lidar this is nearly the whole circle and adds little; for an
        # 87-degree camera it is the entire story - a narrow cone that stops
        # three metres out, next to a lidar sweeping the room. Without the
        # edges, a camera pointed at empty space draws NOTHING and reads as a
        # broken sensor rather than one whose window is empty.
        if depth:
            p.setBrush(Qt.NoBrush)
            edge = QPen(QColor(150, 224, 165, 130), 1.2)
            edge.setStyle(Qt.DashLine)
            p.setPen(edge)
            o = self.to_screen(x, y, z)
            for a in (yaw + a0, yaw + a1):
                p.drawLine(o, self.to_screen(x + rmax * math.cos(a),
                                             y + rmax * math.sin(a), z))
            # The far arc, so the cone is closed and its depth is legible.
            arc = [self.to_screen(x + rmax * math.cos(yaw + a0
                                                      + (a1 - a0) * k / 24.0),
                                  y + rmax * math.sin(yaw + a0
                                                      + (a1 - a0) * k / 24.0), z)
                   for k in range(25)]
            p.drawPolyline(QPolygonF(arc))

        p.setBrush(QBrush(dot_col))
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
        model = scan.get("model", "sensor")
        # A DEPTH CAMERA'S REACH IS NOT A RING. It sees through a window, so a
        # full circle at its range would claim coverage behind the vehicle
        # that it does not have. Its cone edges and far arc are drawn above
        # instead, and the ring is skipped.
        rings = () if depth else (
            (rmax, 70, Qt.DotLine, f"{model} rated {rmax:.0f} m"),
            (reff, 150, Qt.DashLine, f"reaches {reff:.1f} m"))
        for radius, alpha, style, label in rings:
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

    def _ring_polygon(self, cx, cy, cz, radius, n=72):
        """A horizontal circle of `radius` about (cx, cy), PROJECTED.

        Drawn as a polygon of world points rather than as a screen-space
        ellipse, so it lands correctly in every view instead of only on the
        plan. The rings used to be TOP-only for exactly that reason - a
        circle drawn with drawEllipse is a circle on screen, which is wrong
        the moment the camera is not looking straight down, so they were
        simply skipped and appeared to have vanished.
        """
        poly = QPolygonF()
        for i in range(n + 1):
            th = 2.0 * math.pi * i / n
            poly.append(self.to_screen(cx + radius * math.cos(th),
                                       cy + radius * math.sin(th), cz))
        return poly

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
        p.drawPolyline(self._ring_polygon(_num(pose.get("x")),
                                          _num(pose.get("y")),
                                          _num(pose.get("z")), r))
        p.setFont(QFont("Consolas", 7))
        p.setPen(QPen(col))
        p.drawText(c + QPointF(10, 12),
                   f"{agent.get('id')} reach ~{r:.0f} m @ {tx:.0f} dBm")

    def _range_rings(self, p):
        """Ring a SELECTED agent's reach: a jammer's influence, or anything
        else's comms range.

        Two rings for a jammer: the J/N=0 influence boundary (dashed) and the
        J/N=20 dB denial core (filled). A NOMINAL omni contour - real jammed
        areas are ragged (sensors 2024).

        Drawn in EVERY view now. They were plan-only because a screen-space
        ellipse is only correct looking straight down, so switching to ISO
        made them silently vanish - which reads as a bug rather than as a
        deliberate omission. Projecting the ring as a polygon of world points
        costs nothing and is right from any angle.
        """
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
            px, py, pz = (_num(pose.get("x")), _num(pose.get("y")),
                          _num(pose.get("z")))
            if core and core > 0:
                p.setPen(QPen(col, 1.4))
                p.setBrush(QBrush(QColor(col.red(), col.green(),
                                         col.blue(), 40)))
                p.drawPolygon(self._ring_polygon(px, py, pz, core))
            pen = QPen(col, 1.2, Qt.DashLine)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawPolyline(self._ring_polygon(px, py, pz, r0))
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

        section("POINTS   what a mission is written against")
        def d_point(yy):
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(QColor("#7FA8B8"), 1.2))
            cxp = x_sw + 12
            p.drawLine(QPointF(cxp - 5, yy), QPointF(cxp + 5, yy))
            p.drawLine(QPointF(cxp, yy - 5), QPointF(cxp, yy + 5))
            p.drawEllipse(QPointF(cxp, yy), 5, 5)
        row("named point - drag it to move the objective", d_point)

        if self.show_c2:
            section("COMMAND   who decides, and who carries it")
            def d_arrow(yy):
                p.setBrush(Qt.NoBrush)
                p.setPen(QPen(QColor(self.C2_COL), 2.0))
                p.drawLine(x_sw, yy, x_sw + 22, yy)
                p.drawLine(x_sw + 22, yy, x_sw + 16, yy - 4)
                p.drawLine(x_sw + 22, yy, x_sw + 16, yy + 4)
            row("answers to (dashed = cannot reach them)", d_arrow)
            p.setPen(QPen(QColor(self.C2_COL)))
            p.setFont(QFont("Consolas", 9))
            p.drawText(x_sw + 6, y, "\u2605")
            p.setPen(QPen(QColor(C_DIM)))
            p.setFont(QFont("Consolas", 8))
            p.drawText(x_tx, y, "coordinator")
            y += LH
            p.setPen(QPen(QColor(self.C2_COL)))
            p.setFont(QFont("Consolas", 9))
            p.drawText(x_sw + 6, y, "\u25b2")
            p.setPen(QPen(QColor(C_DIM)))
            p.setFont(QFont("Consolas", 8))
            p.drawText(x_tx, y, "squad leader")
            y += LH
            def d_count(yy):
                p.setPen(QPen(QColor(self.C2_COL), 1.2))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(QPointF(x_sw + 11, yy), 7, 7)
                p.setFont(QFont("Consolas", 8))
                p.drawText(QPointF(x_sw + 8, yy + 3), "3")
            row("vehicles whose orders cross this link", d_count)

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
        y += LH

        if self.placing:
            section("PLACING   Setup tab, before the run")
            p.setPen(QPen(QColor(C_DIM)))
            p.setFont(QFont("Consolas", 8))
            for line in ("drag a vehicle or a point to move it",
                         "sweep a box to select several",
                         "drag any selected one to move them all",
                         "middle-drag to pan"):
                p.drawText(x_sw, y, line)
                y += LH - 3

        s_ = self.scale()
        if s_ > 2:
            p.setPen(QPen(QColor(C_DIM)))
            p.drawLine(10, self.height() - 16, 10 + int(s_), self.height() - 16)
            p.drawText(14 + int(s_), self.height() - 12, "1 m")


# ---------------------------------------------------------------------------
# Sensor view
# ---------------------------------------------------------------------------

class ScanView(QWidget):
    """ONE sensor's returns, drawn in that sensor's own frame.

    Straight up is dead ahead, because that is how you read a scan when you
    are debugging a controller; rotating it into the world frame makes it
    prettier and much less useful.

    A lidar and a depth camera are drawn DIFFERENTLY on purpose. The lidar is
    a 270 degree plane and its natural picture is a polar fan with range
    rings. A D435i is an 87 x 58 degree pyramid, and the horizontal slice this
    model produces is one row out of a depth image - so it gets the wedge it
    actually sees, its vertical field stated rather than implied, and a DEPTH
    STRIP underneath: range against bearing, left to right, which is what the
    image itself gives you. Drawing a camera as a small lidar was the thing
    that made the two look interchangeable when they are not.
    """

    def __init__(self):
        super().__init__()
        self.scan = None
        self.setMinimumHeight(200)

    def set_scan(self, scan):
        self.scan = scan
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(C_BG))
        p.setFont(QFont("Consolas", 9))
        sc = self.scan
        if not sc or not sc.get("ranges"):
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(12, 20, "no returns")
            return

        ranges = sc["ranges"]
        rmax = float(sc.get("range_max", 10.0))
        a0 = float(sc.get("angle_min", -2.356))
        a1 = float(sc.get("angle_max", 2.356))
        cam = bool(sc.get("slice_of_depth_image"))

        # The header line says WHICH instrument this is, so two tabs of
        # coloured wedges cannot be confused for one another.
        p.setPen(QPen(QColor(C_TEXT)))
        p.drawText(10, 16, f"{sc.get('model', '?')}   "
                           f"{math.degrees(a1 - a0):.0f}\u00b0 h"
                           + (f" \u00d7 {sc['fov_v_deg']:.0f}\u00b0 v"
                              if cam and sc.get("fov_v_deg") else "")
                           + f"   {rmax:.1f} m")
        hits = [r for r in ranges if r is not None]
        misses = len(ranges) - len(hits)
        p.setPen(QPen(QColor(C_DIM)))
        p.drawText(10, 30, f"{len(ranges)} rays  "
                           + (f"near {min(hits):.2f} m" if hits else "no return")
                           + (f"  {misses} over range" if misses else ""))
        if cam:
            p.setPen(QPen(QColor(C_WARN)))
            p.drawText(10, 44, "depth-image slice")
        top = 52

        strip_h = 46 if cam else 0
        plot_h = self.height() - top - strip_h - 18
        cx = self.width() / 2
        # ANCHORED TO THE TOP, not to a fraction of whatever height the panel
        # happens to have. Placing the vehicle at 78% of the available height
        # was fine while the panel was short; in a tall dock it put the whole
        # plot at the bottom with several hundred pixels of nothing above it -
        # which is what was reported. The disc is as big as the panel allows
        # and then sits directly under the header, wherever that leaves the
        # bottom edge.
        radius = min(self.width() / 2, plot_h / 1.32) - 14
        cy = top + radius + 10
        if radius < 18:
            return
        scale = radius / rmax

        # Range rings. A 3 m camera and a 10 m lidar need different spacing or
        # the camera gets one ring and the lidar gets thirty.
        step = 0.5 if rmax <= 4.0 else 2.0
        p.setPen(QPen(QColor(C_GRID)))
        r = step
        while r <= rmax + 1e-9:
            p.drawEllipse(QPointF(cx, cy), r * scale, r * scale)
            p.drawText(QPointF(cx + 3, cy - r * scale - 2), f"{r:g}m")
            r += step

        # The field of view itself, as edges - for a narrow sensor this is
        # most of the information.
        p.setPen(QPen(QColor(C_LINE), 1.0, Qt.DashLine))
        for ang in (a0, a1):
            p.drawLine(QPointF(cx, cy),
                       QPointF(cx + rmax * scale * math.sin(ang),
                               cy - rmax * scale * math.cos(ang)))

        n = len(ranges)
        poly = QPolygonF([QPointF(cx, cy)])
        for i, rng in enumerate(ranges):
            ang = a0 + (a1 - a0) * i / max(n - 1, 1)
            d = (rmax if rng is None else min(float(rng), rmax)) * scale
            poly.append(QPointF(cx + d * math.sin(ang), cy - d * math.cos(ang)))
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
            ang = a0 + (a1 - a0) * i / max(n - 1, 1)
            d = min(float(rng), rmax) * scale
            run.append(QPointF(cx + d * math.sin(ang), cy - d * math.cos(ang)))
        if run.size() > 1:
            p.drawPolyline(run)

        if cam:
            # THE DEPTH STRIP. Bearing left-to-right, range as brightness:
            # the row of the depth image this slice came from, laid out the
            # way the image is.
            y0 = self.height() - strip_h - 2
            w = self.width() - 20
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(10, y0 - 2, "depth slice  L \u2192 R")
            for i, rng in enumerate(ranges):
                x = 10 + w * i / max(n - 1, 1)
                if rng is None:
                    p.setPen(QPen(QColor("#2A3338")))
                else:
                    f = max(0.0, min(1.0, 1.0 - float(rng) / rmax))
                    p.setPen(QPen(QColor(int(40 + 150 * f), int(90 + 120 * f),
                                         int(110 + 80 * f))))
                p.drawLine(QPointF(x, y0 + 2), QPointF(x, y0 + 28))
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(10, self.height() - 2, "near = bright")
        else:
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(10, self.height() - 6, "up = forward")


class SensorView(QWidget):
    """THE SELECTED AGENT'S SENSORS - the summary, then one tab per sensor.

    A single fixed lidar plot was fine while every car carried exactly one
    lidar. It stopped being fine the moment a car could carry a lidar AND a
    depth camera: the second sensor was invisible, and the panel implied the
    first one was the whole picture.

    So: the agent's state on top (it is true whatever is fitted), then a tab
    strip. `All` stacks every sensor one above the other - lidar first, camera
    under it - which is the view for asking "do these two agree?". The
    per-sensor tabs are for reading one instrument properly.
    """

    def __init__(self):
        super().__init__()
        self.agent_id = None
        self.state = None
        self._key = None
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self.summary = QWidget()
        self.summary.setFixedHeight(106)
        self.summary.paintEvent = self._paint_summary
        lay.addWidget(self.summary)
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        lay.addWidget(self.tabs, 1)
        self._views, self._stacked = {}, []

    @staticmethod
    def _scrolled(inner, min_h):
        """Put a plot in a scroll area with a real minimum height.

        Two sensors stacked in a dock that is 500 px tall gives each of them
        250 px, and a polar plot with a header and a depth strip does not fit
        in 250 px - it gets cut off at the bottom, which is exactly what was
        reported. Give each plot the height it actually needs and let the
        panel scroll instead of squeezing.
        """
        inner.setMinimumHeight(min_h)
        sa = QScrollArea()
        sa.setWidgetResizable(True)
        sa.setFrameShape(QFrame.NoFrame)
        sa.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sa.setWidget(inner)
        return sa

    # -- data ---------------------------------------------------------------

    def set_scan(self, agent_id, scan, state=None):
        self.agent_id, self.state = agent_id, state
        scans = [sc for sc in ([scan] + list((state or {}).get("scans") or []))
                 if sc and sc.get("ranges")]
        key = (agent_id, tuple(sc.get("frame") for sc in scans))
        if key != self._key:
            self._key = key
            self._rebuild(scans)
        by = {sc.get("frame"): sc for sc in scans}
        for frame_id, v in list(self._views.items()) + list(self._stacked):
            if by.get(frame_id) is not None:
                v.set_scan(by[frame_id])
        self.summary.update()

    def _rebuild(self, scans):
        while self.tabs.count():
            self.tabs.removeTab(0)
        self._views, self._stacked = {}, []
        if not scans:
            empty = QLabel("No ranging sensor on this agent."
                           if self.agent_id else "Select an agent.")
            empty.setObjectName("hint")
            empty.setAlignment(Qt.AlignCenter)
            self.tabs.addTab(empty, "Sensors")
            return
        if len(scans) > 1:
            page = QWidget()
            col = QVBoxLayout(page)
            col.setContentsMargins(0, 0, 0, 0)
            col.setSpacing(2)
            for sc in scans:
                v = ScanView()
                v.setMinimumHeight(300)
                col.addWidget(v, 1)
                self._stacked.append((sc.get("frame"), v))
            self.tabs.addTab(self._scrolled(page, 300 * len(scans)), "All")
        for sc in scans:
            v = ScanView()
            self._views[sc.get("frame")] = v
            name = (sc.get("frame") or "?").split("/")[-1]
            self.tabs.addTab(self._scrolled(v, 340), name)

    # -- the agent's own state, above the tabs ------------------------------

    def _paint_summary(self, _):
        p = QPainter(self.summary)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.summary.rect(), QColor(C_BG))
        p.setFont(QFont("Consolas", 9))
        if not self.state:
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(10, 20, "Select an agent.")
            return
        st = self.state
        pose = st.get("pose") or {}
        p.setPen(QPen(QColor(C_TEXT)))
        p.drawText(10, 16, str(self.agent_id or ""))
        p.setPen(QPen(QColor(C_DIM)))
        p.drawText(10, 32, f"x {_num(pose.get('x')):7.2f}  "
                           f"y {_num(pose.get('y')):7.2f}  "
                           f"z {_num(pose.get('z')):7.2f}  m")
        p.drawText(10, 46, f"yaw {math.degrees(_num(pose.get('yaw'))):6.1f} deg"
                           f"   speed {_num(pose.get('speed')):5.2f} m/s")
        obj = st.get("objective")
        if isinstance(obj, dict):
            p.drawText(10, 60, "objective  "
                       + _objective_label(obj, armed=st.get("armed")))
        elif st.get("mission"):
            p.drawText(10, 60, f"objective  {st.get('mission')}")
        err = _num(st.get("position_error_m"))
        if st.get("gnss_denied"):
            p.setPen(QPen(QColor("#E08A3C")))
            p.drawText(10, 74, "GNSS DENIED - dead reckoning, "
                               f"est. error {err:5.2f} m")
        else:
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(10, 74, "GNSS ok"
                       + (f"  (est. error {err:.2f} m)" if err > 0.05 else ""))
        sensors = st.get("sensors") or []
        names = ", ".join(sen.get("type", "?") for sen in sensors) or "none"
        p.setPen(QPen(QColor(C_DIM)))
        p.drawText(10, 90, f"fitted: {names}")
        p.setPen(QPen(QColor(C_LINE)))
        p.drawLine(8, 100, self.summary.width() - 8, 100)


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
        self._key = []          # (number, colour, path, lo, hi) per series
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
        a = menu.addAction("Legend...")
        a.triggered.connect(self.show_legend)
        menu.addSeparator()
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

    def show_legend(self):
        """The names behind the numbers, in a window with room for them."""
        if not self._key:
            QMessageBox.information(self, "Legend",
                                    "Nothing plotted here yet.")
            return
        dlg = QDialog(self)
        dlg.setWindowTitle("Legend")
        lay = QVBoxLayout(dlg)
        tbl = QTableWidget(len(self._key), 4)
        tbl.setHorizontalHeaderLabels(["#", "Series", "Min", "Max"])
        tbl.verticalHeader().setVisible(False)
        tbl.horizontalHeader().setStretchLastSection(True)
        for r, (num, col, path, lo, hi) in enumerate(self._key):
            cells = [str(num), self.series_label(path),
                     f"{lo:.4g}", f"{hi:.4g}"]
            for c, text in enumerate(cells):
                it = QTableWidgetItem(text)
                it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                if c == 0:
                    it.setForeground(QBrush(QColor(col)))
                it.setToolTip(path)
                tbl.setItem(r, c, it)
        tbl.resizeColumnsToContents()
        tbl.setMinimumWidth(460)
        lay.addWidget(tbl, 1)
        close = QPushButton("Close")
        close.clicked.connect(dlg.accept)
        lay.addWidget(close)
        dlg.resize(520, min(120 + 24 * len(self._key), 640))
        dlg.exec()

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

    AUTH_COLOUR = {"centralized": "#D2694F",
                   "decentralized": "#4FA3D1",
                   "hierarchical": "#6FAE7E"}
    ROUTE_DASH = {"star": Qt.SolidLine, "mesh": Qt.DashLine,
                  "tiered": Qt.DotLine}

    @classmethod
    def series_style(cls, path, idx):
        """(colour, dash) for a series - by CONFIGURATION where there is one.

        Nine curves in nine arbitrary colours needs a nine-row legend, and a
        nine-row legend is a wall of text lying on the data. Reported twice,
        and the second time with the answer: "the rest of them are the colour
        only of their authority, otherwise the legend gets way way too long,
        this way the legend is 4 things - the 3 authorities and selected."

        Right, and it makes the chart say something rather than merely
        distinguish: colour IS the authority, dash IS the routing, so the
        three warm curves are the centralized family whatever else varies.
        A series whose name is not a configuration (a live run, where it is a
        vehicle and a metric) falls back to the ordinary palette.
        """
        label = path[7:] if path.startswith("agents.") else path
        bits = label.split(".", 1)[0].split("_")
        auth = next((b for b in bits if b in cls.AUTH_COLOUR), None)
        route = next((b for b in bits if b in cls.ROUTE_DASH), None)
        if auth is None:
            return QColor(PALETTE[idx % len(PALETTE)]), Qt.SolidLine
        return (QColor(cls.AUTH_COLOUR[auth]),
                cls.ROUTE_DASH.get(route, Qt.SolidLine))

    @staticmethod
    def series_label(path):
        """The readable name of a series - what the legend calls it.

        `agents.` is how a series is ADDRESSED, not what it is called, and on
        a swept chart the id is a configuration whose parts are joined with
        underscores. Both get unpicked; the metric is kept, because on a live
        run it is the half that distinguishes two series of the same car.
        """
        label = path[7:] if path.startswith("agents.") else path
        who, _, metric = label.partition(".")
        who = who.replace("_", " / ")
        return f"{who}  {metric}" if metric else who

    def _draw(self, p):
        self._key = []
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
        # THE SELECTED SERIES IS DRAWN LAST, so it is on top of its family
        # rather than buried under it.
        chosen = getattr(self.area, "selected", None)
        order = sorted(range(len(self.series)),
                       key=lambda i: self.series[i] == chosen)
        for idx in order:
            path = self.series[idx]
            pairs = allpairs[path]
            lit = (path == chosen)
            colour, dash = self.series_style(path, idx)
            if lit:
                colour = QColor(C_WARN)
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
            pen = QPen(colour, 2.4 if lit else 1.2)
            pen.setStyle(Qt.SolidLine if lit else dash)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawPolyline(poly)
            # A COMPACT LEGEND. The full dotted path is how the series is
            # addressed, not what it is called: on a swept chart every entry
            # began "agents." and ended with the same metric, so the nine
            # names differed only in the middle and the legend was a wall.
            self._key.append((idx + 1, colour.name(), path, own_lo, own_hi))

        # A FOUR-ROW KEY, whatever the number of series: the three
        # authorities, and whichever one is selected. Click a row in the
        # series list to light one up.
        fams = [(a, c) for a, c in self.AUTH_COLOUR.items()
                if any(self.series_style(q, 0)[0].name().lower() == c.lower()
                       for q in self.series)]
        rows = [(c, a) for a, c in fams]
        if chosen in self.series:
            rows.append((C_WARN, self.series_label(chosen)))
        elif fams:
            rows.append((C_DIM, "click a series to light it up"))
        p.setFont(QFont("Consolas", 8))
        # Bottom left, where a decaying curve leaves the page empty - the top
        # left is exactly where the data starts.
        for r, (col, text) in enumerate(rows):
            y = top + h - 6 - (len(rows) - 1 - r) * 12
            p.fillRect(QRectF(left + 6, y - 6, 8, 8), QBrush(QColor(col)))
            p.setPen(QPen(QColor(C_TEXT if col == C_WARN else C_DIM)))
            p.drawText(QPointF(left + 19, y + 1), text)
        p.setFont(QFont("Consolas", 9))

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
        # The axis NAME sits at the right-hand end, and any tick label that
        # would run into it is dropped rather than overprinted - "30" and
        # "dB P_j/P_t" on top of each other was neither.
        unit_x = self.width() - right - 8 - len(unit) * 5
        for tt in sorted(set(t)):
            cx = left + w * (tt - t0) / (t1 - t0)
            p.drawLine(QPointF(cx, top + h), QPointF(cx, top + h + 3))
            if cx + 14 < unit_x:
                p.drawText(QPointF(cx - 12, self.height() - 6), f"{tt:g}")
        p.drawText(QPointF(unit_x, self.height() - 6), unit)
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
        # WHICH SERIES IS LIT. One at a time, chosen by clicking its row in
        # the series list - see Console._select_series.
        self.selected = None
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

    def first_plot(self):
        """The plot the Legend button speaks for: the first with anything on
        it, or simply the first."""
        panes = self.panes()
        if not panes:
            return None
        return next((q for q in panes if q.series), panes[0])

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
        if head and head[0].upper() == "SETPLAN":
            if self._cell_allows("SETMISSION"):
                self._retask(head)
            return
        if head and head[0].upper() == "JAM":
            scope = head[1] if len(head) > 1 else None
            if self._cell_allows("JAM", scope=scope):
                self._send_queue_line(cmd + "\n", cmd)
            return
        # TXPOWER is JAM's friendly twin and is routed identically - it was
        # added to the model and NOT to the terminal, so typing it dropped
        # through to bash and did nothing but print "command not found".
        # Scoped like launch/halt: a cell may only change its own side's
        # transmitters, which is what makes it a command rather than a knob.
        if head and head[0].upper() == "TXPOWER":
            scope = head[1] if len(head) > 1 else None
            if self._cell_allows("TXPOWER", scope=scope):
                self._send_queue_line(cmd + "\n", cmd)
                # SAY WHAT IT WILL AND WILL NOT DO. Reported: "TXPOWER gcs 60
                # - why did that not over power a JAM jam1 power 8?" Because
                # a link is scored on its WEAKER direction, and turning up one
                # end leaves the other end exactly where it was. The physics
                # was right; the application said nothing, and an operator who
                # turns a knob to its stop and sees no change has been let
                # down by the tool, not by the model.
                # DEFERRED BY ONE FRAME, deliberately. Reading the last
                # frame right now describes the power BEFORE this command,
                # because the sim has not seen it yet - which made every
                # report describe the previous command. The Console prints it
                # on the next frame instead.
                if self.console is not None:
                    if getattr(self.console, "running", None) is None \
                            and not (self.console.frames or []):
                        for line in self.console.limiting_links(scope):
                            self.out.appendPlainText("  " + line)
                    else:
                        self.console._power_report_for = (scope, self.out)
            return
        if head and head[0].upper() == "LISTEN":
            if self._cell_allows("LISTEN"):
                self._listen(head[1:])
            return
        # CUT is the one command that does NOT reach the model. It moves the
        # section planes the viewport draws on, and that is all: no agent, no
        # wall and no scene file is touched by it, which is why it is not
        # written to the retask spool like JAM and TXPOWER are. Every cell may
        # use it, because looking at something is not an action against a side.
        if head and head[0].upper() == "CUT":
            self._cut(head[1:])
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

    def _cut(self, args):
        """CUT x|y|z <metres> | CUT on | CUT off | CUT

        Move the section planes the viewport draws on, from the terminal -
        the same grammar as JAM, so there is one way to talk to this
        application rather than two.

        IT IS A CAMERA COMMAND, NOT A WORLD ONE. Nothing it does reaches the
        model: no agent moves, no wall changes, no file is written, and a run
        in progress does not notice. That is why it is handled here instead
        of being written to the retask spool - a command that only changes
        what you can see should not travel down the same pipe as one that
        changes what is true.
        """
        con = self.console
        if con is None or not hasattr(con, "slice_sliders"):
            self.out.appendPlainText("[CUT needs the main Console window]")
            return
        if not args:
            on = getattr(con.viewport, "cut_away", False)
            self.out.appendPlainText(
                "cut " + ("ON" if on else "off") + "   "
                + "  ".join(f"{a}={con.slice_at(a):.2f} m" for a in "xyz")
                + "\n  CUT x|y|z <metres>   move a section plane"
                  "\n  CUT on | CUT off     open or close the scene on them")
            return
        what = str(args[0]).lower()
        if what in ("on", "off"):
            con.toggle_cut(what == "on")
            if getattr(con, "cut_button", None) is not None:
                con.cut_button.setChecked(what == "on")
            self.out.appendPlainText(f"cut {what}")
            return
        if what not in ("x", "y", "z") or len(args) < 2:
            self.out.appendPlainText(
                "CUT: usage CUT x|y|z <metres>, or CUT on / CUT off")
            return
        try:
            v = float(args[1])
        except ValueError:
            self.out.appendPlainText(f"CUT: '{args[1]}' is not a number")
            return
        sl = con.slice_sliders[what]
        lo, hi = sl.minimum() / 100.0, sl.maximum() / 100.0
        if not (lo <= v <= hi):
            self.out.appendPlainText(
                f"CUT: {what} = {v:g} m is outside the scene "
                f"({lo:.2f} to {hi:.2f} m)")
            return
        sl.setValue(int(round(v * 100)))
        self.out.appendPlainText(f"cut {what} = {con.slice_at(what):.2f} m")

    def _listen(self, args):
        """LISTEN <MHz> | LISTEN band <MHz> | LISTEN off | LISTEN

        Tune this cell's receiver to a band and print every transmission on it
        that one of your OWN agents could actually hear. Whether a given order
        reaches you is ordinary RF - your listener's position, the
        coordinator's transmit power, the path loss, and whatever you are
        jamming with - so eavesdropping on a channel you are also attacking
        gets harder the harder you attack it. That trade is the interesting
        part, and it comes out of the link budget rather than a rule.

        This is the first half of deception. You cannot spoof an order you
        have never heard.
        """
        c = self.console
        if c is None:
            return
        toks = [t for t in args if t]
        if toks and toks[0].lower() == "band":
            toks = toks[1:]
        if not toks:
            cur = getattr(c, "_listen_band", None)
            self.out.appendPlainText(
                f"# listening on {cur:g} MHz" if cur else
                "# not listening. LISTEN <MHz>, e.g. LISTEN 2400 (the "
                "command band) or LISTEN 1575.42 (GPS L1).")
            return
        if toks[0].lower() in ("off", "stop", "none"):
            c._listen_band = None
            c._listen_shell = None
            self.out.appendPlainText("# receiver off")
            return
        try:
            band = float(toks[0])
        except ValueError:
            self.out.appendPlainText(
                "[LISTEN: expected a frequency in MHz, e.g. LISTEN 2400]")
            return
        c._listen_band = band
        c._listen_shell = self
        self.out.appendPlainText(
            f"# receiver on {band:g} MHz. Anything your side can hear on this "
            f"band prints here as it is transmitted.\n"
            f"# Nothing will appear until blue actually sends an order - the "
            f"coordinator only transmits when a vehicle reports reaching a "
            f"waypoint and needs the next one.")

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
        if verb == "LISTEN":
            # RECEIVING IS RED'S. Blue listening to its own command net is not
            # an intercept, it is just blue reading its own traffic, and
            # letting the blue cell do it here would blur the one distinction
            # the three cells exist to keep: a cell sees what its OWN side
            # sees. Blue already knows what it ordered.
            if remit != "red":
                self.out.appendPlainText(
                    "[blocked: LISTEN is a RED-cell action - blue already "
                    "knows what it ordered]")
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


# ---------------------------------------------------------------------------
# The spectrum field: dB as a place
# ---------------------------------------------------------------------------
# ONE HUE, LIGHT TO DARK. Sequential magnitude gets a single hue and never a
# rainbow - and in THIS application there is a second reason for the choice
# that is stronger than the general rule. Colour here means SIDE: blue is
# friendly, red is hostile, orange is broken. A spectrum field painted in
# saturated blues and reds would read as "these are the blue bits" and it
# means nothing of the kind. So the ramp is low-chroma slate-to-pale-cyan:
# unmistakably a magnitude, unmistakably not a team, and recessive enough to
# sit underneath the agents without competing with them.
SPECTRUM_RAMP = [
    (-115.0, (16, 21, 25)),
    (-100.0, (24, 36, 45)),
    (-85.0,  (39, 64, 78)),      # receiver sensitivity - the interesting line
    (-70.0,  (58, 98, 116)),
    (-55.0,  (87, 144, 159)),
    (-40.0,  (134, 192, 201)),
    (-20.0,  (195, 228, 231)),
]
SPECTRUM_SENSITIVITY_DBM = -85.0


def spectrum_colour(dbm, lo=None, hi=None):
    """The ramp, stretched across the range this field actually occupies.

    A FIXED ABSOLUTE SCALE LOOKED RIGHT AND WAS USELESS. Anchoring the ramp to
    -115 .. -20 dBm is defensible on paper and produces a flat wash in
    practice, because a 20 m room with 20 dBm radios in it is loud EVERYWHERE:
    the whole field sat in the top two stops and every contour vanished. The
    physics was correct and the picture said nothing.

    So the ramp is stretched to the data and the KEY CARRIES THE NUMBERS, which
    is what makes two runs comparable - not a shared colour, a shared axis you
    can read. Clamped to the 2nd and 98th percentile so that one cell a
    quarter-metre from a transmitter cannot compress everything else into one
    stop.
    """
    if dbm is None:
        return None
    if lo is None or hi is None or hi - lo < 1e-6:
        lo, hi = SPECTRUM_RAMP[0][0], SPECTRUM_RAMP[-1][0]
    u = max(0.0, min(1.0, (dbm - lo) / (hi - lo)))
    span = SPECTRUM_RAMP[-1][0] - SPECTRUM_RAMP[0][0]
    v = SPECTRUM_RAMP[0][0] + u * span
    for (a_v, a_c), (b_v, b_c) in zip(SPECTRUM_RAMP, SPECTRUM_RAMP[1:]):
        if v <= b_v:
            t = (v - a_v) / max(b_v - a_v, 1e-9)
            return QColor(*[int(a_c[i] + (b_c[i] - a_c[i]) * t)
                            for i in range(3)])
    return QColor(*SPECTRUM_RAMP[-1][1])


# The colour ramp's fixed span, in dBm. ABSOLUTE, and that is the whole
# point: a magnitude scale that renormalises to its own data cannot show a
# change in magnitude. Anchored below the -85 dBm sensitivity line so the
# dead ground is always the dark end, and open at the top so standing next
# to a transmitter is always the bright end.
SPECTRUM_SCALE_DBM = (-100.0, 10.0)


def spectrum_range(field, absolute=True):
    """(lo, hi) dBm for the ramp.

    FIXED BY DEFAULT. It used to stretch to the 2nd and 98th percentile of
    whatever was in the field, which is right for showing STRUCTURE and
    exactly wrong for showing LEVEL: turn the fleet's power down by 20 dB and
    every value drops by 20 dB, the ramp re-stretches, and the picture comes
    out looking the same. Reported, and reasonably, as "the heatmap is not
    affected by the power of the agent" - the numbers were changing and the
    colours were not allowed to.

    On a fixed scale, quieter looks quieter. The stretch is kept for the case
    it was right for - reading the shape of a field whose absolute level does
    not matter - but it is no longer what you get by default.
    """
    if absolute:
        return SPECTRUM_SCALE_DBM
    vals = sorted(v for row in field["dbm"] for v in row if v is not None)
    if not vals:
        return (-115.0, -20.0)
    lo = vals[int(0.02 * (len(vals) - 1))]
    hi = vals[int(0.98 * (len(vals) - 1))]
    return (lo, hi) if hi - lo > 1.0 else (lo - 5.0, lo + 5.0)


def spectrum_coverage(field, sensitivity=None):
    """Fraction of the room where a receiver could actually hear something.

    THE NUMBER THAT ACTUALLY MOVES WHEN YOU CHANGE POWER. dB are a ratio and
    hard to feel; "62% of the floor is above sensitivity" is not. It is also
    the operational question - a wall's attenuation in dB does not change when
    you turn the power down, but whether the far side of it is still usable
    very much does.
    """
    lim = SPECTRUM_SENSITIVITY_DBM if sensitivity is None else sensitivity
    vals = [v for row in field["dbm"] for v in row if v is not None]
    if not vals:
        return 0.0
    return sum(1 for v in vals if v >= lim) / float(len(vals))


def contour_segments(grid, level):
    """Marching squares: the line segments where the field crosses `level`.

    Returned in GRID coordinates (fractional cell indices), so the caller can
    project them however it likes. Cells with a missing corner are skipped -
    a contour through a hole in the data would be an invention.
    """
    segs = []
    ny = len(grid)
    nx = len(grid[0]) if ny else 0
    for j in range(ny - 1):
        for i in range(nx - 1):
            c = [grid[j][i], grid[j][i + 1], grid[j + 1][i + 1], grid[j + 1][i]]
            if any(v is None for v in c):
                continue
            corners = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
            cross = []
            for k in range(4):
                v1, v2 = c[k], c[(k + 1) % 4]
                if (v1 < level) == (v2 < level):
                    continue
                u = (level - v1) / (v2 - v1)
                (x1, y1), (x2, y2) = corners[k], corners[(k + 1) % 4]
                cross.append((x1 + (x2 - x1) * u, y1 + (y2 - y1) * u))
            # Two crossings is the ordinary case. Four is the saddle, where
            # the connection is genuinely ambiguous; joining them in order is
            # the standard resolution and any error is one cell wide.
            for k in range(0, len(cross) - 1, 2):
                segs.append((cross[k], cross[k + 1]))
    return segs


class WallDialog(QDialog):
    """What are these walls made of?

    Opens on a click or a swept box, and applies to EVERY selected wall - so
    re-materialling a corridor is one action rather than eight. A material
    sets two things at once, and they are not correlated: what the radio pays
    to get through it, and how well a lidar sees it. Glass is nearly
    transparent AND nearly invisible; metal is opaque to radio and bright to a
    lidar. That is the whole reason this dialog exists rather than a single
    "hardness" slider.

    THE FILE STILL DECIDES, and this does not overwrite it. A scene ships its
    materials; this changes the world THIS RUN uses, the way the fleet builder
    changes a fleet without editing agents/. Keep it with Save scene as...
    """

    # Kept in step with MATERIALS in tools/stub_telemetry.py, which carries the
    # provenance. Shown here as a summary so the choice is informed - the dB is
    # measured (Wilson 2002 E10589 Table 3), the reflectance is a declared free
    # parameter, and the dialog says which is which rather than presenting two
    # numbers of equal standing.
    BLURB = {
        "plasterboard": "0.49 dB   ·  reflectance 0.70   —  the ordinary "
                        "partition. Barely there for radio, easy for a lidar",
        "glass":        "0.50 dB   ·  reflectance 0.08   —  nearly transparent "
                        "AND nearly invisible. A lidar car with no map drives "
                        "into it",
        "wood":         "2.79 dB   ·  reflectance 0.45   —  a door, a stud "
                        "partition, furniture",
        "brick":        "4.44 dB   ·  reflectance 0.35   —  a real internal "
                        "wall",
        "concrete":     "6.71 dB   ·  reflectance 0.30   —  structural. Two of "
                        "these and a link is in trouble",
        "metal":        "opaque    ·  reflectance 0.60   —  no way through at "
                        "all. The signal must go round the end (ITU-R P.526 "
                        "diffraction), which costs about 6 dB at a graze",
    }

    def __init__(self, walls, indices, parent=None):
        super().__init__(parent)
        self.walls, self.indices = walls, sorted(indices)
        n = len(self.indices)
        self.setWindowTitle(f"Material — {n} wall{'s' if n != 1 else ''}")
        lay = QVBoxLayout(self)

        head = QLabel(self._describe())
        head.setObjectName("hint")
        head.setWordWrap(True)
        lay.addWidget(head)

        row = QHBoxLayout()
        row.addWidget(QLabel("Material"))
        self.mat = QComboBox()
        for m in self.BLURB:
            self.mat.addItem(m)
        cur = str((self.walls[self.indices[0]] or {}).get("material", ""))
        i = self.mat.findText(cur.lower())
        if i >= 0:
            self.mat.setCurrentIndex(i)
        self.mat.currentTextChanged.connect(self._blurb)
        row.addWidget(self.mat, 1)
        lay.addLayout(row)

        self.note = QLabel("")
        self.note.setObjectName("hint")
        self.note.setWordWrap(True)
        self.note.setMinimumHeight(48)
        lay.addWidget(self.note)
        self._blurb(self.mat.currentText())

        # HEIGHT IS HERE because it is the ground/air distinction and nothing
        # else expresses it: a 1.2 m partition stops a car and a link between
        # two cars, and a drone two metres up does not know it is there.
        hrow = QHBoxLayout()
        hrow.addWidget(QLabel("Height"))
        self.height = QDoubleSpinBox()
        self.height.setRange(0.0, 50.0)
        self.height.setDecimals(2)
        self.height.setSuffix(" m")
        self.height.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.height.setValue(_num((self.walls[self.indices[0]] or {})
                                  .get("height"), 3.0))
        self.height.setToolTip("A wall shorter than an agent's altitude does "
                               "not block it - not its radio, not its lidar, "
                               "not its wheels.")
        hrow.addWidget(self.height)
        hrow.addWidget(QLabel("Thickness"))
        self.thick = QDoubleSpinBox()
        self.thick.setRange(0.01, 2.0)
        self.thick.setDecimals(3)
        self.thick.setSuffix(" m")
        self.thick.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.thick.setValue(_num((self.walls[self.indices[0]] or {})
                                 .get("thickness"), 0.1))
        self.thick.setToolTip("Collision only. The radio model treats a wall "
                              "as one traversal at the material's loss, not "
                              "as a slab.")
        hrow.addWidget(self.thick)
        hrow.addStretch(1)
        lay.addLayout(hrow)

        # THE OVERRIDE, and it is here because the honest figure often is not
        # the material's. Wilson measured single thin SAMPLES; a built stud
        # partition is two boards, a cavity, studs and cabling and runs several
        # dB where one board runs half of one. Typing the real number is
        # better than pretending a material name covers it.
        orow = QHBoxLayout()
        self.over = QCheckBox("Override loss")
        orow.addWidget(self.over)
        self.rf = QDoubleSpinBox()
        self.rf.setRange(0.0, 300.0)
        self.rf.setDecimals(2)
        self.rf.setSuffix(" dB")
        self.rf.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.rf.setEnabled(False)
        self.over.toggled.connect(self.rf.setEnabled)
        w0 = self.walls[self.indices[0]] or {}
        if w0.get("rf_db") is not None:
            self.over.setChecked(True)
            self.rf.setValue(_num(w0.get("rf_db")))
        orow.addWidget(self.rf)
        orow.addStretch(1)
        lay.addLayout(orow)
        hint = QLabel("A material is a SAMPLE measurement, not a built wall. "
                      "A real stud partition is several dB where a single "
                      "board is half of one — override it and say so in the "
                      "scene.")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        lay.addWidget(hint)

        btns = QHBoxLayout()
        btns.addStretch(1)
        cancel = QPushButton("Cancel")
        cancel.clicked.connect(self.reject)
        ok = QPushButton(f"Apply to {n}")
        ok.setDefault(True)
        ok.clicked.connect(self.accept)
        btns.addWidget(cancel)
        btns.addWidget(ok)
        lay.addLayout(btns)
        self.resize(520, 340)

    def _describe(self):
        mats = {str((self.walls[i] or {}).get("material", "?"))
                for i in self.indices}
        total = sum(math.dist(((self.walls[i] or {}).get("x1", 0),
                               (self.walls[i] or {}).get("y1", 0)),
                              ((self.walls[i] or {}).get("x2", 0),
                               (self.walls[i] or {}).get("y2", 0)))
                    for i in self.indices)
        return (f"{len(self.indices)} selected, {total:.1f} m of wall, "
                f"currently {', '.join(sorted(mats))}.")

    def _blurb(self, name):
        self.note.setText(self.BLURB.get(name, ""))

    def apply_to(self):
        """Write the choice into every selected wall."""
        for i in self.indices:
            w = self.walls[i]
            w["material"] = self.mat.currentText()
            w["height"] = round(self.height.value(), 3)
            w["thickness"] = round(self.thick.value(), 4)
            w["rf_db"] = round(self.rf.value(), 3) if self.over.isChecked() \
                else None
            # The reflectance follows the material unless a scene set one
            # deliberately; clearing it here is what makes the dropdown
            # actually take effect for the lidar as well as the radio.
            w["reflectance"] = None
        return len(self.indices)


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
        # THE NAME AND THE SAVE ARE ONE CONTROL. The file is called whatever
        # the box says, so the button belongs against the box - a fleet named
        # here and saved under a different name in a file dialog two steps
        # later is a trap. Reported: "when we give the fleet a fleet_name
        # should this not be the same as the save as? maybe we make the save
        # button next to the fleet name input as a small icon".
        save = QPushButton("\u2913")
        save.setFixedWidth(28)
        save.setToolTip("Save this fleet as fleets/<name>.yaml, using the "
                        "name in the box.\nNothing is written unless you "
                        "press this.")
        save.clicked.connect(self.save_as)
        top.addWidget(save)
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

        # A CLEAN SLATE, deliberately empty. It used to seed a ground station
        # and three cars, which is not "build a custom fleet" - it is "edit
        # the standard fleet", and it meant deleting four rows before you
        # could start. The one thing worth saying is what a fleet needs.
        self._empty = QLabel(
            "Empty. Add the agents you want.\n"
            "A blue fleet normally needs a ground station if anything is to "
            "be commanded centrally; a decentralized one does not."
            if side == "blue" else
            "Empty. Add the jammers or listeners you want.")
        self._empty.setObjectName("hint")
        self._empty.setWordWrap(True)
        lay.insertWidget(lay.indexOf(self.table) + 1, self._empty)

    def save_as(self):
        """Write this fleet straight to fleets/<name>.yaml."""
        _name, doc = self.fleet_doc()
        if not doc.get("agents"):
            QMessageBox.information(self, "Save fleet",
                                    "Add at least one agent first.")
            return
        import yaml as _yaml
        stem = (self.name.text() or f"custom_{self.side}").strip()
        stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", stem).strip("_") or self.side
        path = REPO_ROOT / "fleets" / f"{stem}.yaml"
        if path.exists() and QMessageBox.question(
                self, "Save fleet",
                f"fleets/{path.name} already exists. Overwrite it?"
                ) != QMessageBox.Yes:
            return
        path = str(path)
        doc = dict(doc)
        doc["name"] = stem
        try:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            Path(path).write_text(
                "# Saved from the Console's fleet builder.\n"
                "# A COMPOSITION of agents/ hardware: which vehicles, their\n"
                "# ids and where they start. It declares no authority, no\n"
                "# routing and no doctrine - those are picked in Setup, every\n"
                "# run, and are never baked in here.\n"
                + _yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
        except OSError as exc:
            QMessageBox.warning(self, "Save fleet", f"cannot write: {exc}")
            return
        par = self.parent()
        if hasattr(par, "_refresh_setup_lists"):
            par._refresh_setup_lists()
        if hasattr(par, "say"):
            par.say(f"Saved {Path(path).name} "
                    f"({len(doc['agents'])} agents)")

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
        self._sync_empty()

    def remove_row(self):
        r = self.table.currentRow()
        if r >= 0:
            self.table.removeRow(r)
        self._sync_empty()

    def _sync_empty(self):
        if hasattr(self, "_empty"):
            self._empty.setVisible(self.table.rowCount() == 0)

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
                 # COLOUR IS THE SIDE, NOT THE HARDWARE. Every agent file used
                 # to carry its own colour, so a fleet of one blue car, one
                 # lidar car and one camera car came out blue, green and
                 # purple - three colours that say nothing about who is on
                 # whose side, which is the only thing colour is for on this
                 # map. Sensor and radio fit are read from the sensor panel
                 # and the glyphs; the side is read from the colour.
                 "colour": network_colour(self.side),
                 "pose": {"x": num(2), "y": num(3), "z": num(4),
                          "yaw": num(5)}}
            # EVERY HARDWARE KEY THE AGENT FILE CARRIES. `radio` and `radios`
            # were missing, so a fleet built in the Console had no radios at
            # all and the Comms tab correctly reported "no radio - cannot be
            # commanded" for every vehicle in it. The list is now the whole
            # set that appears in agents/*.yaml, so adding a hardware key to
            # an agent file cannot silently fail to reach a built fleet.
            for key in ("dimensions", "performance", "sensors", "ghost",
                        "radio", "radios"):
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

class AxisRange(QWidget):
    """A swept axis as min / max / steps, rather than a row of tickboxes.

    Tickboxes could only ever offer the values somebody had thought to put
    there, which is fine for a demo and useless for a curve: four points do
    not show you where a cliff is. Min, max and a step count say what the
    axis IS, so the resolution of the answer is a dial rather than a
    code change.

    `steps = 1` means "one value, the minimum" - which is how an axis is
    pinned without needing a separate control for pinning it.
    """

    def __init__(self, lo, hi, steps, unit="", decimals=1, lo_min=-200.0,
                 hi_max=200.0, tip=""):
        super().__init__()
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(4)
        self.lo = QDoubleSpinBox()
        self.hi = QDoubleSpinBox()
        for b, v in ((self.lo, lo), (self.hi, hi)):
            b.setDecimals(decimals)
            b.setRange(lo_min, hi_max)
            b.setValue(v)
            b.setSuffix(f" {unit}" if unit else "")
            # NO STEPPER ARROWS. They take about a third of the width of a
            # short box and, with a unit suffix on top, the VALUE was what got
            # clipped - "30.0 d" for 30.0 dB. Nobody dials a sweep range one
            # click at a time anyway; it is typed. Reported: "we dont need
            # these arrows when setting the experiment numbers, it just means
            # the actual number gets cut off".
            b.setButtonSymbols(QAbstractSpinBox.NoButtons)
            b.setFixedWidth(84)
        self.steps = QSpinBox()
        self.steps.setRange(1, 200)
        self.steps.setValue(steps)
        self.steps.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.steps.setFixedWidth(56)
        self.steps.setToolTip("How many values between min and max, "
                              "inclusive. 1 pins the axis to the minimum.")
        row.addWidget(QLabel("min"))
        row.addWidget(self.lo)
        row.addWidget(QLabel("max"))
        row.addWidget(self.hi)
        row.addWidget(QLabel("steps"))
        row.addWidget(self.steps)
        row.addStretch(1)
        if tip:
            self.setToolTip(tip)

    def changed(self, fn):
        for b in (self.lo, self.hi, self.steps):
            b.valueChanged.connect(lambda *_: fn())

    def values(self):
        """The swept values, inclusive of both ends.

        Rounded to three decimals on purpose: these become part of the cell
        key that identifies a run, and a key carrying 12.700000000000001 is a
        key you cannot type back in to replay it.
        """
        n = int(self.steps.value())
        lo, hi = float(self.lo.value()), float(self.hi.value())
        if n <= 1:
            return [round(lo, 3)]
        return [round(lo + (hi - lo) * i / (n - 1), 3) for i in range(n)]


class PointDialog(QDialog):
    """Where a new point goes. Three numbers, typed, on whole metres.

    A point is created by COORDINATE rather than by clicking the map, because
    the first thing you usually know is the number - "the far end is at x=95"
    - and dragging to find 95 exactly is worse than typing it. Once it exists
    it can be dragged like anything else.
    """

    def __init__(self, arena=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add point")
        lay = QVBoxLayout(self)
        lab = QLabel("Where is it? Metres, from the centre of the arena. "
                     "You can drag it afterwards.")
        lab.setWordWrap(True)
        lay.addWidget(lab)
        e = ((arena or {}).get("extent") or {})
        hx, hy = _num(e.get("x"), 20) / 2, _num(e.get("y"), 20) / 2
        hz = _num(e.get("z"), 5)
        row = QHBoxLayout()
        self.boxes = []
        for axis, lo, hi in (("x", -hx, hx), ("y", -hy, hy), ("z", 0.0, hz)):
            b = QDoubleSpinBox()
            b.setDecimals(0)                 # whole metres, like everything
            b.setRange(lo, hi)
            b.setValue(0.0)
            b.setPrefix(f"{axis} ")
            b.setFixedWidth(92)
            row.addWidget(b)
            self.boxes.append(b)
        row.addStretch(1)
        lay.addLayout(row)
        hint = QLabel(f"This arena is {2 * hx:.0f} x {2 * hy:.0f} m, so x runs "
                      f"{-hx:.0f} to {hx:.0f} and y runs {-hy:.0f} to {hy:.0f}.")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        lay.addWidget(hint)
        btns = QHBoxLayout()
        btns.addStretch(1)
        cancel = QPushButton("Cancel")
        cancel.clicked.connect(self.reject)
        ok = QPushButton("Add point")
        ok.setDefault(True)
        ok.clicked.connect(self.accept)
        btns.addWidget(cancel)
        btns.addWidget(ok)
        lay.addLayout(btns)

    def point(self):
        return {"x": snap(self.boxes[0].value()),
                "y": snap(self.boxes[1].value()),
                "z": snap(self.boxes[2].value())}


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
                     "own. x/y in metres, yaw in radians. Pick a formation to "
                     "place the MOBILE vehicles as a shape; the ground "
                     "station and any jammer keep the position you gave them.")
        lab.setWordWrap(True)
        lay.addWidget(lab)

        # ---- FORMATION ------------------------------------------------------
        # A formation is a FUNCTION of (count, spacing), not a list of
        # coordinates, so it works for any fleet size: a line adds one more on
        # the end, a wedge alternates flanks so it stays symmetric, a circle
        # re-spaces every vehicle. The spacing dial is proportional - doubling
        # it doubles every gap in the shape, whatever the shape is - which is
        # why "how big is the formation" can be one number.
        #
        # The GCS IS NEVER IN THE FORMATION. Every link in the run is measured
        # against where the bench is, so shuffling it because the fleet
        # changed shape would move the ruler along with the thing being
        # measured.
        # ---- SPAWN POINT ----------------------------------------------------
        # ONE COORDINATE PLACES THE FLEET. A ground station is a real place -
        # a bench, a vehicle, a mast - and everything else deploys FROM it, so
        # typing four sets of coordinates to express "we set up here and they
        # went that way" was three sets too many. Move this and the whole
        # fleet moves with it, formation intact.
        srow = QHBoxLayout()
        srow.addWidget(QLabel("Spawn point"))
        self.sp_boxes = []
        for axis in ("x", "y", "z"):
            b = QDoubleSpinBox()
            b.setDecimals(1)
            b.setRange(-10000.0, 10000.0)
            b.setFixedWidth(78)
            b.setPrefix(f"{axis} ")
            b.valueChanged.connect(lambda _v: self._reform())
            srow.addWidget(b)
            self.sp_boxes.append(b)
        srow.addStretch(1)
        lay.addLayout(srow)
        sphint = QLabel(
            "Where this side sets up. The ground station goes here and the "
            "vehicles form up ahead of it, one spacing clear of the bench. "
            "Move it and everything moves together.")
        sphint.setObjectName("hint")
        sphint.setWordWrap(True)
        lay.addWidget(sphint)

        frow = QHBoxLayout()
        frow.addWidget(QLabel("Formation"))
        self.form_combo = QComboBox()
        self.form_combo.addItem(AS_SPAWNED)
        self.form_combo.addItems(list(FORMATIONS))
        # Shapes drawn by hand and saved. A generated shape is a function of
        # the fleet size; a drawn one is a drawing, so it places as many
        # vehicles as it was drawn with and leaves any extras where they are.
        self.form_combo.addItems(list(custom_formations()))
        self.form_combo.setToolTip(
            "line/column  nose to tail; one more goes on the end.\n"
            "abreast      shoulder to shoulder along y.\n"
            "echelon      a diagonal rank.\n"
            "wedge        apex forward, flanks filled ALTERNATELY so four\n"
            "             vehicles make an arrowhead and not a limp.\n"
            "circle       evenly spaced, re-spaced whenever the count\n"
            "             changes - the spacing is the chord between\n"
            "             neighbours.\n"
            "diamond      front, both flanks, rear, then outward rings.\n"
            "cube         a 3-D lattice. The only shape that uses z, and the\n"
            "             one that matters the moment these are aircraft.")
        self.form_combo.activated.connect(lambda _i: self._reform())
        frow.addWidget(self.form_combo, 1)
        frow.addWidget(QLabel("Spacing"))
        self.space_slider = QSlider(Qt.Horizontal)
        # Tenths of a metre, 0.5 m to 20 m. Proportional: the shape is
        # generated at this spacing, so the dial scales the whole formation
        # rather than nudging one gap.
        self.space_slider.setRange(5, 200)
        self.space_slider.setValue(30)
        self.space_slider.setFixedWidth(180)
        self.space_slider.setToolTip(
            "Distance between neighbouring vehicles, in metres. The whole "
            "formation scales with it - double this and every gap doubles.")
        self.space_slider.valueChanged.connect(lambda _v: self._reform())
        frow.addWidget(self.space_slider)
        self.space_lbl = QLabel("3.0 m")
        self.space_lbl.setFixedWidth(64)
        frow.addWidget(self.space_lbl)
        lay.addLayout(frow)
        self.table = QTableWidget(len(agents), 6)
        self.table.setHorizontalHeaderLabels(
            ["Agent", "x", "y", "z", "yaw", "on link loss"])
        self.table.verticalHeader().setVisible(False)
        self._ids = []
        self._doctrine = {}
        # The poses the dialog opened with, kept so (as spawned) can put them
        # back: a formation preview must be reversible or it silently destroys
        # coordinates the operator typed.
        self._original = {}
        # Who a formation may move. Not the ground station, not a jammer, not
        # a ghost - see the comment on the formation row above.
        self._movable = set()
        for r, a in enumerate(agents):
            pose = a.get("pose") or {}
            aid = str(a.get("id", f"agent{r}"))
            self._ids.append(aid)
            self._original[aid] = {k: _num(pose.get(k)) for k in "xyz"}
            if not (a.get("ghost") or a.get("jammer")
                    or a.get("platform") == "ground_station"):
                self._movable.add(aid)
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

        # THE ANCHOR: the ground station if this side has one, otherwise the
        # middle of whatever moves. Everything the spawn point does is
        # expressed as a shift of this, so "spawn here" means the same thing
        # for a blue fleet with a bench and for a red one with none.
        self._anchor = next(
            (str(a.get("id")) for a in agents
             if a.get("platform") == "ground_station"), None)
        self._has_gcs = self._anchor is not None
        if self._anchor is None and self._movable:
            self._anchor = None          # centroid of the movable, computed live
        a0 = self._anchor_pose()
        for b, v in zip(self.sp_boxes, a0):
            b.blockSignals(True)
            b.setValue(v)
            b.blockSignals(False)

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

    # -- formation ---------------------------------------------------------

    def formation(self):
        """The chosen shape, or AS_SPAWNED."""
        return self.form_combo.currentText()

    def spacing(self):
        """Metres between neighbouring vehicles."""
        return self.space_slider.value() / 10.0

    def _anchor_pose(self):
        """Where this side currently sets up, as originally spawned.

        The ground station if there is one - a bench is a real place and the
        obvious thing to point at. Otherwise the middle of whatever moves,
        which is the only defensible anchor for a side that is all vehicles
        (the red fleet, typically).
        """
        if self._anchor and self._anchor in self._original:
            q = self._original[self._anchor]
            return (q["x"], q["y"], q["z"])
        rows = [aid for aid in self._ids if aid in self._movable] or self._ids
        if not rows:
            return (0.0, 0.0, 0.0)
        return tuple(sum(self._original[a][k] for a in rows) / len(rows)
                     for k in "xyz")

    def spawn_point(self):
        return tuple(b.value() for b in self.sp_boxes)

    def _reform(self):
        """Rewrite the table's x/y/z from the spawn point, shape and spacing.

        Everything is computed from the ORIGINAL poses rather than from
        whatever is currently in the table, so dragging the spawn point back
        and forth cannot accumulate drift, and choosing (as spawned) restores
        the arrangement the dialog opened with rather than a slightly-mangled
        version of it.

        Three rules, in order:
          the anchor lands on the spawn point;
          everything that is not a vehicle moves with it, rigidly;
          the vehicles take their shape, placed one spacing ahead of the
          bench so a formation never spawns on top of the thing every link in
          the run is measured against.
        """
        self.space_lbl.setText(f"{self.spacing():.1f} m")
        a0 = self._anchor_pose()
        sp = self.spawn_point()
        shift = (sp[0] - a0[0], sp[1] - a0[1], sp[2] - a0[2])
        shape = self.formation()
        rows = [r for r, aid in enumerate(self._ids) if aid in self._movable]

        def put(r, x, y, z):
            # WHOLE METRES. The formation function itself is exact - a circle
            # is still a circle - but the coordinates a fleet is actually
            # placed at are rounded here, so the composed run and every result
            # downstream carry numbers that read as what they are: a setup
            # somebody chose, not a measurement.
            self.table.item(r, 1).setText(f"{snap(x):.0f}")
            self.table.item(r, 2).setText(f"{snap(y):.0f}")
            self.table.item(r, 3).setText(f"{snap(z):.0f}")

        # Rigid translation for everything the formation does not govern -
        # the ground station, a jammer, a ghost.
        for r, aid in enumerate(self._ids):
            if r in rows:
                continue
            q = self._original[aid]
            put(r, q["x"] + shift[0], q["y"] + shift[1], q["z"] + shift[2])

        if not rows:
            return
        if shape == AS_SPAWNED or formation_offsets is None:
            for r in rows:
                q = self._original[self._ids[r]]
                put(r, q["x"] + shift[0], q["y"] + shift[1], q["z"] + shift[2])
            return

        offs = formation_offsets(shape, len(rows), self.spacing())
        if self._has_gcs and offs:
            # ONE SPACING CLEAR OF THE BENCH, along +x. The formation's own
            # depth decides how far its centre has to be for its REAR rank to
            # sit that clear, so a deep column starts further out than a flat
            # rank does and neither ends up inside the ground station.
            depth = max(q[0] for q in offs) - min(q[0] for q in offs)
            cx = sp[0] + depth / 2.0 + self.spacing()
            cy, cz = sp[1], sp[2]
        else:
            # No bench: hold the shape where the vehicles already were.
            cx = sum(self._original[self._ids[r]]["x"] for r in rows) \
                / len(rows) + shift[0]
            cy = sum(self._original[self._ids[r]]["y"] for r in rows) \
                / len(rows) + shift[1]
            cz = sum(self._original[self._ids[r]]["z"] for r in rows) \
                / len(rows) + shift[2]
        for r, (dx, dy, dz) in zip(rows, offs):
            put(r, cx + dx, cy + dy, cz + dz)
        # A SAVED SHAPE HAS A FIXED SIZE. Whoever it ran out of room for keeps
        # the pose they had, and is TOLD SO - a fleet quietly half in
        # formation is a run nobody can explain afterwards.
        short = len(rows) - len(offs)
        if short > 0:
            for r in rows[len(offs):]:
                q = self._original[self._ids[r]]
                put(r, q["x"] + shift[0], q["y"] + shift[1], q["z"] + shift[2])
            self.space_lbl.setText(
                f"{self.spacing():.1f} m  ({short} not placed)")


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


_SWEEP_MOD = None


def _sweep_module():
    """tools/sweep.py, imported as a module.

    The sweep runs IN-PROCESS. It used to be a QThread driving a subprocess,
    which is machinery for a problem that does not exist: 81 runs take two
    seconds. What it bought instead was a whole class of failure - a worker
    thread, a pipe, cross-thread signals and a second Python interpreter - and
    it crashed the Console on Windows. A loop with processEvents() is simpler,
    cannot crash that way, and gives finer progress.
    """
    global _SWEEP_MOD
    if _SWEEP_MOD is not None:
        return _SWEEP_MOD
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "commsev_sweep", str(REPO_ROOT / "tools" / "sweep.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    _SWEEP_MOD = mod
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
        """{(authority, routing): [(x, mean y), ...]} - seeds averaged.

        THE SILENT-JAMMER CONTROL IS NOT A COORDINATE. Experiments write
        jam_rel_db = -999 to mean "the jammer is switched off" - a control
        condition, not a power a thousand decibels below the fleet. Plotted
        as a number it sets the axis minimum to -999 and squashes every real
        cell into the last two percent of the chart, which is exactly what
        the intercept plot looked like. It is split out here and drawn as a
        labelled control point off the left end of the axis instead.
        """
        acc, ctrl = {}, {}
        for r in self.rows:
            try:
                x = float(r.get("jam_rel_db"))
                y = float(r.get(self.metric))
            except (TypeError, ValueError):
                continue
            key = (r.get("authority"), r.get("routing"))
            if x <= JAM_OFF_DB / 2:
                ctrl.setdefault(key, []).append(y)
                continue
            acc.setdefault(key, {}).setdefault(x, []).append(y)
        self.control = {k: sum(v) / len(v) for k, v in ctrl.items()}
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
        # THE CONTROL, off the end of the axis and visibly detached from it.
        # A dotted rule separates it, because "jammer off" is a different
        # condition and not a point on the power scale - joining it to the
        # curve would draw a line across a discontinuity that does not exist.
        ctrl = getattr(self, "control", None)
        if ctrl:
            cxp = L - 26
            p.setPen(QPen(QColor("#3A4247"), 1.0, Qt.DotLine))
            p.drawLine(int(cxp + 12), int(T), int(cxp + 12), int(T + h))
            p.setPen(QPen(QColor(C_DIM)))
            p.drawText(int(cxp) - 12, int(T + h + 18), "off")
            for (auth, route), v in sorted(ctrl.items()):
                c = QColor("#E08A3C") if self.highlight == (auth, route) \
                    else QColor(self.AUTH_COLOUR.get(auth, "#AAAAAA"))
                p.setPen(QPen(c, 2.0))
                p.setBrush(QBrush(c))
                p.drawEllipse(QPointF(cxp, sy(v)), 3.2, 3.2)
            p.setBrush(Qt.NoBrush)
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
        # Subprocess state for the grid - see _start_subprocess. Held here so
        # a dialog that never ran one still answers for them.
        self._sweep_proc = None
        self._sweep_tail = ""
        self._sweep_outdir = None
        lay.addLayout(top)

        mrow = QHBoxLayout()
        mrow.addWidget(QLabel("Show"))
        self.metric_combo = QComboBox()
        # THE METRICS WORTH A HEADLINE, in the order the argument runs:
        # how far did it get, was it commanded, did it stop, did it know where
        # it was, and what was the radio doing. Everything else is in Results.
        for key, lab in (
                ("mission_pass_frac", "mission passed (fraction of the fleet)"),
                ("mission_awaiting_orders",
                 "stalled - arrived, awaiting an order that cannot reach"),
                ("mission_drifted", "failed on drift (reported a false arrival)"),
                ("penetration_m", "penetration (m advanced)"),
                ("penetration_frac", "penetration (fraction of the corridor)"),
                ("commanded_fraction", "commanded fraction"),
                ("held_fraction", "held fraction (frozen by doctrine)"),
                ("belief_err_m", "position error (belief vs truth, m)"),
                ("track_err_m", "tracking error (off the ordered path, m)"),
                ("worst_sinr_db", "worst link SINR (dB)"),
                ("worst_pdr", "worst link packet delivery"),
                ("arrived", "vehicles that arrived"),
                ("ended_s", "run length (s)"),
                # WHAT RED ACTUALLY HEARD. The intercept experiment produced
                # these and nothing plotted them, so the answer to the
                # question it was built to ask was only ever in the CSV.
                # `orders_sent` is here as a metric in its own right because
                # it is the DENOMINATOR: a low intercepted fraction means
                # "could not hear it" or "there was nothing to hear", and
                # only the count separates the two.
                # IS THE FLEET STILL A FLEET? Neither of these shows up in
                # a pass rate, and both are what a formation is FOR.
                ("station_err_m",
                 "formation - mean distance off station (m)"),
                ("centre_err_m",
                 "formation - how wrong the fleet's idea of its own centre is (m)"),
                ("intercept_frac",
                 "intercepted - fraction of blue's orders red heard"),
                ("orders_sent",
                 "orders transmitted by blue (the denominator)"),
                ("orders_heard_any",
                 "orders heard by at least one listener")):
            self.metric_combo.addItem(lab, key)
            # The same definition the Results table shows, on the item that
            # chooses it - so what a metric means is legible at the moment
            # you pick it, not only after it has been plotted.
            self.metric_combo.setItemData(self.metric_combo.count() - 1,
                                          metric_help(key), Qt.ToolTipRole)
        self.metric_combo.setToolTip(
            "Which metric the headline chart draws against jammer advantage. "
            "Every one of them, per architecture, is also in the Results tab "
            "where panes can be split and series overlaid.\n"
            "Hover an entry for what it measures.")
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
        # ONE CLICK LIGHTS THE CURVE, two runs it. The table and the chart are
        # the same nine configurations twice over, and until they were linked
        # you had to find your row's line by eye among nine.
        self.table.cellClicked.connect(self._light_row)
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

        # OUT OF THE GUI THREAD, ON EVERY CORE.
        #
        # This used to run the whole grid in-process, on the GUI thread, with
        # a processEvents() every three cells. The comment justifying it said
        # "81 runs take two seconds", and that was true of three cars on a
        # short mission. It stopped being true: eight cars over a 200 m
        # corridor is about fourteen seconds a cell, so the window froze for
        # forty-two seconds at a time and Windows painted it Not Responding -
        # reported, and correctly, as the experiment not working.
        #
        # tools/sweep.py already has a CLI with a worker pool, so the fix is
        # to run the tool rather than to re-implement it here: one QProcess,
        # every core, progress parsed from its own output, and a window that
        # stays alive and can be cancelled. QProcess is what the Console
        # already uses for the run and the terminals, so it is the mechanism
        # this application is known to survive on Windows.
        if exp_path is not None and self._start_subprocess(exp_path, n):
            return
        # FALLBACK: no interpreter, or the process would not start. Same work,
        # in this thread, pumping events EVERY cell rather than every third -
        # slow and single-cored, but it finishes and it repaints.
        rows = []
        for i, cell in enumerate(cells, 1):
            rows.append(sweep.run_one((cfg, cell, False)))
            self.bar.setValue(i)
            self.bar.setFormat("%v / %m runs (single core - close nothing)")
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

    # -- running the grid out of the GUI thread -----------------------------
    def _start_subprocess(self, exp_path, n):
        """Run tools/sweep.py as a child process. True if it started.

        Returns immediately; _sweep_done() picks the results up when the
        child exits. Everything the dialog needs afterwards is on disk, which
        is the point: the child writes the same results.csv the in-process
        path writes, so nothing downstream knows the difference.
        """
        import shutil
        exe = sys.executable or shutil.which("python3") or shutil.which("python")
        if not exe:
            return False
        script = REPO_ROOT / "tools" / "sweep.py"
        if not script.exists():
            return False
        # Cleared, or a second run that fails would load the FIRST run's
        # results and look as though it had succeeded.
        self._sweep_outdir = None
        self._sweep_tail = ""
        self._sweep_rows_expected = n
        self.bar.setRange(0, n)
        self.bar.setValue(0)
        self.bar.setFormat("starting %m runs on every core...")
        self._sweep_proc = QProcess(self)
        self._sweep_proc.setProcessChannelMode(QProcess.MergedChannels)
        self._sweep_proc.readyReadStandardOutput.connect(self._sweep_output)
        self._sweep_proc.finished.connect(self._sweep_done)
        self._sweep_tail = ""
        self._sweep_proc.setWorkingDirectory(str(REPO_ROOT))
        self._sweep_proc.start(exe, ["-u", str(script), str(exp_path)])
        if not self._sweep_proc.waitForStarted(4000):
            self._sweep_proc = None
            return False
        # CANCELLABLE, because a grid you cannot stop is a grid you cannot
        # afford to start. The Run button becomes Stop for the duration.
        self.run_btn.setEnabled(True)
        self.run_btn.setText("Stop")
        try:
            self.run_btn.clicked.disconnect()
        except (TypeError, RuntimeError):
            pass
        self.run_btn.clicked.connect(self._sweep_cancel)
        return True

    def _sweep_output(self):
        """Parse the tool's own progress lines: '  12/36  (123.4s)'."""
        if self._sweep_proc is None:
            return
        text = bytes(self._sweep_proc.readAllStandardOutput()).decode(
            errors="replace")
        self._sweep_tail += text
        for line in text.splitlines():
            line = line.strip()
            m = re.match(r"^(\d+)\s*/\s*(\d+)\b", line)
            if m:
                done, total = int(m.group(1)), int(m.group(2))
                self.bar.setRange(0, total)
                self.bar.setValue(done)
                self.bar.setFormat(f"%v / %m runs   {line[m.end():].strip()}")
            elif line.startswith("->"):
                self._sweep_outdir = line[2:].strip()

    def _sweep_cancel(self):
        if self._sweep_proc is not None:
            self.bar.setFormat("stopping...")
            self._sweep_proc.kill()

    def _sweep_done(self, code=0, _status=None):
        proc, self._sweep_proc = self._sweep_proc, None
        self.run_btn.setText("Run")
        try:
            self.run_btn.clicked.disconnect()
        except (TypeError, RuntimeError):
            pass
        self.run_btn.clicked.connect(self.start)
        self.run_btn.setEnabled(True)
        out = getattr(self, "_sweep_outdir", None)
        if code != 0 and not out:
            self.bar.setFormat(f"stopped (exit {code})")
            if self.console is not None and self._sweep_tail.strip():
                self.console.say("sweep output:\n" + self._sweep_tail[-2000:])
            return
        path = Path(out) if out else None
        if path is not None and path.is_dir():
            path = path / "results.csv"
        if path is None or not path.exists():
            self.bar.setFormat("finished, but no results file was written")
            if self.console is not None:
                self.console.say("sweep output:\n" + self._sweep_tail[-2000:])
            return
        self.outdir = path.parent
        self._csv_path = path
        if self.console is not None:
            self.console.say(f"experiment finished -> {path}")
        self.load_csv(path)

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

    FILTER_COLS = ("authority", "routing", "jam_rel_db", "formation",
                   "spacing")

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
            # A filter with one option filters nothing. It is a button that
            # cannot do anything, sitting next to the ones that can.
            if len(vals) < 2:
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
                            "formation", "spacing",
                            # THE OUTCOME FIRST. Penetration says how far they
                            # got; the pass rate says whether the job was done,
                            # and "awaiting orders" says whether the fleet is
                            # stuck rather than beaten.
                            "mission_pass_frac", "mission_awaiting_orders",
                            "mission_drifted",
                            "penetration_m", "penetration_frac", "arrived",
                            "commanded_fraction", "ended_s")
                if rows and c in rows[0]]
        self.table.setColumnCount(len(cols))
        self.table.setHorizontalHeaderLabels(cols)
        self.table.setRowCount(len(rows))
        # EVERY COLUMN SAYS WHAT IT IS. A results table whose headings are
        # short identifiers is only readable by whoever wrote them, and the
        # whole point of this repo is that somebody else picks it up. The
        # glossary lives in one place (METRIC_HELP) and is used by the header
        # tooltips here and by the metric chooser above.
        for c, key in enumerate(cols):
            head = self.table.horizontalHeaderItem(c)
            if head is not None:
                head.setToolTip(metric_help(key))
        for r, row in enumerate(rows):
            for c, key in enumerate(cols):
                it = QTableWidgetItem(str(row.get(key, "")))
                it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                it.setToolTip(metric_help(key))
                if key == "authority":
                    it.setForeground(QBrush(QColor(
                        ResultPlot.AUTH_COLOUR.get(row.get(key), "#AAAAAA"))))
                # The silent-jammer control reads as a nonsense power unless
                # the cell says what it is.
                if key == "jam_rel_db":
                    try:
                        if float(row.get(key)) <= JAM_OFF_DB / 2:
                            it.setText("off")
                            it.setToolTip("The CONTROL condition: the jammer "
                                          "is silent. Not a power - it is "
                                          "what this cell measures with "
                                          "nothing attacking it.")
                    except (TypeError, ValueError):
                        pass
                self.table.setItem(r, c, it)
        self.table.resizeColumnsToContents()


    def _draw_summary(self, rows):
        """Load penetration into this window's own plot, one series per
        architecture, already selected - so the answer is on screen without
        anyone having to build a chart first."""
        if not rows or self.console is None:
            return
        try:
            self._offer_run_metrics(rows)
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

    def _offer_run_metrics(self, rows):
        """Add whatever THIS run produced that the fixed list cannot name.

        Per-listener interception columns are named after the listener
        (`intercept_frac_ear_near`), so they do not exist until an experiment
        with listeners in it has been run. Discovered from the results rather
        than hard-coded, which also means a red fleet with six ears offers six
        curves without anyone editing this file.
        """
        fixed = {self.metric_combo.itemData(i)
                 for i in range(self.metric_combo.count())}
        pretty = {"intercept_frac": "intercepted",
                  "intercept_sinr": "how well heard (SINR dB)",
                  "first_intercept_s": "first heard at (s)",
                  "heard": "orders heard"}
        found = []
        for r in rows:
            for k in r:
                if k in fixed or k in found:
                    continue
                for pre, lab in pretty.items():
                    if k.startswith(pre + "_"):
                        found.append(k)
                        who = k[len(pre) + 1:]
                        self.metric_combo.addItem(f"{lab} - {who}", k)
                        break
        return found

    def _light_row(self, row, _col=0):
        """Light this row's configuration on the summary chart."""
        shown = getattr(self, "_table_rows", None) or self.rows
        if row < 0 or row >= len(shown) or self.console is None:
            return
        varying = getattr(self.console, "_sweep_varying", [])
        metric = (self.metric_combo.currentData()
                  if hasattr(self, "metric_combo") else "penetration_m")
        path = (f"agents."
                f"{self.console._series_id(shown[row], varying)}.{metric}")
        self.summary.selected = (None if self.summary.selected == path
                                 else path)
        self.summary.refresh()

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
                "commsev_plot", str(REPO_ROOT / "tools" / "plot_results.py"))
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
    @staticmethod
    def _compose_from_file(cfg):
        """The composition a saved experiment describes: scene, fleets,
        points, spawns and doctrine, in the shape the Console runs.

        Deliberately the same fields sweep.build() reads, so a replay cannot
        drift away from the cell it is replaying.
        """
        if not cfg.get("scene"):
            return None
        fleets = [cfg["blue_fleet"]] if cfg.get("blue_fleet") else []
        if cfg.get("red_fleet"):
            fleets.append(cfg["red_fleet"])
        compose = {"scene": cfg["scene"], "fleets": fleets}
        if cfg.get("points"):
            compose["points"] = copy.deepcopy(cfg["points"])
        agents = []
        for aid, xy in (cfg.get("spawns") or {}).items():
            pose = {k: float(v) for k, v in (xy or {}).items()}
            pose.setdefault("z", 0.0)
            pose.setdefault("yaw", 0.0)
            agents.append({"id": aid, "pose": pose})
        # DOCTRINE IS PER VEHICLE and the file states it for all of them.
        # Written per agent rather than as a network default because that is
        # where the model reads it, and because a later experiment will want
        # to vary it per squad.
        doct = cfg.get("doctrine")
        if doct:
            by = {a["id"]: a for a in agents}
            for aid in (cfg.get("spawns") or {}):
                by[aid]["on_link_loss"] = doct
        if agents:
            compose["agents"] = agents
        return compose

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
                # A FILE EXPERIMENT COMPOSES LIKE ANY OTHER. It used to refuse
                # here - "run it from the Setup tab" - which made a saved
                # experiment a table of numbers with nothing behind it:
                # reported as "if I run an experiment from a set file I want
                # it to operate the same as if I had run it at a setup level;
                # right now I can't select a series or watch a run, so the
                # results almost mean nothing".
                #
                # There is nothing special about a file. It names a scene, a
                # blue fleet, a red fleet, its own points and its own spawns,
                # which is exactly the composition the Setup tab produces -
                # and it is exactly what sweep.build() assembles to run the
                # cell in the first place. Built the same way here, so the
                # replay is the same run by the same path.
                compose = self._compose_from_file(cfg)
                if compose is None:
                    self.bar.setFormat("this experiment names no scene to run")
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

            # THE FORMATION, TOO. Every aspect of a cell has to survive into
            # the replay or the run you watch is not the run you clicked -
            # which has already happened once here, with the jammer's power,
            # and produced a replay that looked almost unjammed. The shape is
            # applied to the RESOLVED agents (which know which of them is the
            # ground station) and written back as pose overrides.
            shape = r.get("formation")
            if shape and shape != AS_SPAWNED:
                spacing = float(r.get("spacing") or 3.0)
                moved = _st.apply_formation(_agents, shape, spacing=spacing)
                placed = {x["id"]: (x.get("start") or x.get("pose") or {})
                          for x in _agents if x["id"] in set(moved)}
                by_pose = {x.get("id"): x
                           for x in compose.setdefault("agents", [])}
                for aid, pose in placed.items():
                    entry = by_pose.get(aid)
                    if entry is None:
                        entry = {"id": aid}
                        compose["agents"].append(entry)
                        by_pose[aid] = entry
                    entry["pose"] = {k: float(pose.get(k, 0.0))
                                     for k in ("x", "y", "z")}
                    entry["pose"]["yaw"] = float(pose.get("yaw", 0.0))
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
                jam_dbm=dbm,
                goal=(list(cfg.get("goals") or [])
                      or ([cfg["goal"]] if cfg.get("goal") else [])))
        self.bar.setFormat(f"running {r.get('cell')} live in the Console")



# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class Console(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CommsEv Console")
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
        # DRAG TO PLACE. Typing x, y, z for six vehicles to make a shape is
        # data entry, not design - you cannot see whether it is a wedge until
        # you press Spawn. Dragging in the top-down view sets x and y, in the
        # side view x and z, and the numbers follow the picture instead of the
        # other way round. Only in Setup and only before a run: moving a
        # vehicle mid-run is teleporting it, which is not something the
        # physics should have to explain.
        self.viewport.on_moved = self._agent_dragged
        self.viewport.on_drop = self._agent_dropped
        self.viewport.on_point_moved = self._point_dragged
        self.viewport.on_selection = self._selection_changed
        self.viewport.on_wall_pick = self._walls_picked
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
        self._formation = {}           # {side: shape} chosen in the spawn dialog
        self._points_override = {}     # {name: {x,y,z}} points moved by hand
        self._spawn_point = {}         # {side: (x,y,z)} where it set up
        self._spacing = {}             # {side: metres}
        self._blue_ids = set()
        self._red_ids = set()
        self._played_once = False
        self._focus_setup()             # nothing but Setup until Play
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
        c2 = tool("\u2605", "Command layer: an arrow from each vehicle to "
                             "whoever decides for it, a mark over whoever "
                             "holds rank, and the number of vehicles whose "
                             "orders cross each link.", None)
        c2.setCheckable(True)
        c2.setChecked(False)
        c2.clicked.connect(
            lambda on: (setattr(self.viewport, "show_c2", on),
                        self.viewport.update()))
        spec = tool("\u25a6", "SPECTRUM: what a receiver would measure at every "
                             "point on one frequency, as dBm.\n"
                             "Contours every 10 dB, with the receiver "
                             "sensitivity line picked out - inside it there is "
                             "enough power to decode what is out there.\n"
                             "Computed on demand, not per frame: press again "
                             "to refresh after anything moves. The band comes "
                             "from the Comms band strip.", None)
        spec.setCheckable(True)
        spec.clicked.connect(self.toggle_spectrum)
        wave = tool("⧖", "WAVEFRONT: each transmitter's reach as equal-"
                              "power rings, blue for friendly and red for "
                              "hostile.\n"
                              "One ring every 6 dB out to receiver "
                              "sensitivity, so a blue ring and a red ring at "
                              "the same level mean the same thing. A jammer's "
                              "rings are dashed.\n"
                              "Shaded ground is where the hostile signal is "
                              "the louder one - the contested area.\n"
                              "Computed on demand like the spectrum: press "
                              "again to refresh.", None)
        wave.setCheckable(True)
        wave.clicked.connect(self.toggle_wavefront)

        # THE SECTION PLANES, as a drawing office cuts them - three of them,
        # one per axis, all live at once.
        #
        # One slider that changed meaning with the view was the wrong object.
        # A section is a property of the SCENE, not of the camera looking at
        # it: X, Y and Z are three independent cuts, and you want to set all
        # three and then walk round the result. Tying the single slider to the
        # active view also meant it did nothing in ISO, which is the one view
        # where cutting into a building is most of the point.
        #
        # So: three sliders, always live, in every view. The spectrum reads
        # whichever one matches the plane it is cutting; the cut toggle beside
        # them uses all three to open the scene up.
        cut = tool("✂", "CUT AWAY on the section planes.\n"
                             "Hides the walls below the Z plane and ghosts "
                             "anything past the X and Y planes, so you can "
                             "see inside a building in any view - including "
                             "ISO.\n"
                             "The sliders set where the cuts are; this says "
                             "whether the scene is opened on them.", None)
        cut.setCheckable(True)
        cut.clicked.connect(self.toggle_cut)
        # Held so the CUT command can keep the button in step with itself -
        # a toolbar that disagrees with the terminal is worse than either.
        self.cut_button = cut
        self.slice_sliders, self.slice_reads = {}, {}
        for axis, lo, hi, start in (("x", -10.0, 10.0, 0.0),
                                    ("y", -10.0, 10.0, 0.0),
                                    ("z", 0.0, 3.0, 0.2)):
            sl = QSlider(Qt.Horizontal)
            sl.setFixedWidth(74)
            sl.setRange(int(lo * 100), int(hi * 100))
            sl.setValue(int(start * 100))
            sl.setToolTip(
                {"x": "The X section plane - a cut across the room, seen "
                      "edge-on in Side view.",
                 "y": "The Y section plane - a cut along the room, seen "
                      "edge-on in Front view.",
                 "z": "The Z section plane - the HEIGHT of the horizontal "
                      "slice, floor to ceiling. A slice above a half-height "
                      "wall does not see it at all."}[axis]
                + "\nThe spectrum recomputes on this plane when you let go "
                  "of the handle; Cut away opens the scene on it.")
            sl.valueChanged.connect(self._slice_moved)
            sl.sliderReleased.connect(self._slice_released)
            rd = QLabel(f"{axis} {start:5.2f}")
            rd.setObjectName("hint")
            rd.setFont(QFont("Consolas", 8))
            self.slice_sliders[axis] = sl
            self.slice_reads[axis] = rd
            row.addWidget(sl)
            row.addWidget(rd)

        axes_btn = tool("\u22a2", "Show measured axes with metre ticks", None)
        axes_btn.setCheckable(True)
        axes_btn.clicked.connect(
            lambda on: (setattr(self.viewport, "show_axes", on),
                        self.viewport.update()))
        row.addSpacing(16)
        self.run_button = tool(
            "\u25b6", "Run. With the ROS 2 source this also starts the nodes "
            "and records a bag.", self.toggle_run)
        tool("\u21bb", "RESTART the run. Clears the recorded frames and waits "
                        "for Play.\n"
                        "The Console itself is untouched - the setup, the "
                        "points and the mission all stay exactly as they "
                        "are. This is 'run that again', or 'run it with one "
                        "thing changed'.",
             self.restart_run)
        row.addSpacing(16)
        tool("\u2913", "Save the scenario file, keeping its comments",
             self.save_scenario)
        tool("\u21ba", "REFRESH the Console: close it, reopen it on the "
                        "current code, and start from a CLEAN SLATE.\n"
                        "Nothing is carried over - no scene, no fleet, no "
                        "points, no mission. This is the button for after a "
                        "code change, when what you want is the new Console "
                        "and none of the old state.\n"
                        "To re-run what is already set up, use Restart "
                        "instead - it does not close anything.",
             self.reload_console)

        # THE TWO OPERATORS, ONE BUTTON EACH. White stays below as the tab it
        # has always been - it is the umpire's working terminal. Blue and red
        # are two people looking at two different pictures, which is hard to
        # arrange when only one of them can be on screen at a time.
        row.addSpacing(20)
        for _cell, _lab, _tip in (
                ("blue", "BLUE", "Open the blue operator's terminal in its "
                                 "own window.\n"
                                 "SETPLAN, SETMISSION, REOBJECTIVE, "
                                 "launch / halt."),
                ("red", "RED", "Open the red operator's terminal in its own "
                               "window.\n"
                               "JAM to attack a band, LISTEN to hear what "
                               "blue is ordering on one.")):
            b = QPushButton(_lab)
            b.setToolTip(_tip)
            b.setFixedHeight(26)
            b.setObjectName("tool")
            b.setStyleSheet(
                f"color:{self._CELL_COLOUR[_cell]}; font-weight:600;")
            b.clicked.connect(lambda _c=False, k=_cell: self.open_cell(k))
            row.addWidget(b)

        row.addSpacing(24)
        row.addWidget(QLabel("Source"))
        self.source_combo = QComboBox()
        self.source_combo.addItems(["Stub (no ROS)", "ROS 2 bridge"])
        self.source_combo.setMinimumWidth(130)
        self.source_combo.setToolTip(
            "Stub: a local process, nothing to install.\n"
            "ROS 2 bridge: connect to commsev_ros running in WSL.")
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

        # THE APP IS THREE APPLICATIONS THAT SHARE A SCHEMA, not three
        # applications. Simulation is everything the Console has always been.
        # Scene and Agent are the editors - build a world, build a vehicle -
        # and they are stubs today, deliberately visible ones.
        #
        # Separate executables would mean three readers of the same YAML, and
        # they WILL drift: one gains a field, a scene saved in the builder
        # fails to load in the Console, and the bug looks like bad data when
        # it is duplicated code. One window, one schema, one place a new field
        # has to be added.
        self.app_stack = QStackedWidget()
        self.app_stack.addWidget(holder)
        for name, why in (
            ("Scene",
             "Draw walls, set what they are made of, place the points a "
             "mission is sent to, and save a real scenes/*.yaml.\n\n"
             "Most of the machinery already exists in Simulation: walls are "
             "selectable, the material dialog works, and the viewport knows "
             "how to snap to whole metres. What is missing is DRAWING - "
             "click-drag to create, delete, and Save scene as..."),
            ("Agent",
             "Build a vehicle: dimensions, performance, which sensors and "
             "which radios, and save a real agents/*.yaml.\n\n"
             "The fleet builder in Simulation already COMPOSES agents into a "
             "fleet. This is the layer below it - the hardware itself, which "
             "today is written by hand in YAML and should not be."),
        ):
            page = QWidget()
            # A bare QWidget promoted into the central stack paints the
            # platform's default window colour, not the app's. Every other
            # surface in the Console is dark because it sits inside something
            # the global stylesheet names; this one does not, so it says so
            # itself. Without this the Scene page comes up white.
            page.setObjectName("stubpage")
            page.setAutoFillBackground(True)
            page.setStyleSheet(
                f"QWidget#stubpage {{ background: {C_BG}; }} "
                f"QWidget#stubpage QLabel {{ background: transparent; }}")
            pl = QVBoxLayout(page)
            pl.addStretch(1)
            t = QLabel(f"{name} builder")
            t.setAlignment(Qt.AlignCenter)
            t.setStyleSheet(f"font-size: 22px; color: {C_DIM};")
            pl.addWidget(t)
            n = QLabel("NOT BUILT YET")
            n.setAlignment(Qt.AlignCenter)
            n.setStyleSheet(f"font-size: 13px; letter-spacing: 3px; "
                            f"color: {C_WARN};")
            pl.addWidget(n)
            d = QLabel(why)
            d.setAlignment(Qt.AlignCenter)
            d.setWordWrap(True)
            d.setObjectName("hint")
            d.setMaximumWidth(560)
            row = QHBoxLayout()
            row.addStretch(1)
            row.addWidget(d)
            row.addStretch(1)
            pl.addLayout(row)
            pl.addStretch(2)
            self.app_stack.addWidget(page)
        self.setCentralWidget(self.app_stack)

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
        # A SPLITTER, WITH THE CONTROLS IN A SCROLL AREA. The tab has grown
        # from three dropdowns to a full run configuration - run type, scene,
        # two fleets, authority, routing, mission, goal, three swept axes -
        # and a fixed column silently CLIPPED whatever did not fit, which is
        # how the run-count line ended up half a line tall and unreadable. A
        # scroll area cannot clip, and the splitter means the scene tree below
        # can be dragged out of the way when the controls need the room.
        souter = QVBoxLayout(setup)
        souter.setContentsMargins(0, 0, 0, 0)
        souter.setSpacing(0)
        setup_split = QSplitter(Qt.Vertical)
        controls = QWidget()
        slay = QVBoxLayout(controls)
        slay.setContentsMargins(6, 6, 6, 6)
        slay.setSpacing(4)
        setup_scroll = QScrollArea()
        setup_scroll.setWidget(controls)
        setup_scroll.setWidgetResizable(True)
        setup_scroll.setFrameShape(QFrame.NoFrame)
        # Never a HORIZONTAL bar: the panel is narrow and every control in it
        # is meant to fit its width. Wrapping labels then wrap instead of
        # pushing a scrollbar nobody wants under them.
        setup_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

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
        # Saving a fleet belongs to the fleet builder that made it - see
        # CustomFleetDialog.save_as. It used to be two buttons here, offering
        # to save a red fleet you had never built.
        savef = QPushButton("Save formation as...")
        savef.setToolTip(
            "Keep the arrangement now on the map as a named, reusable shape.\n"
            "Drag the vehicles into the shape you want, then save it - it "
            "becomes a value on the experiment's formation axis, so two "
            "hand-drawn shapes can be swept against each other.\n"
            "The ground station is never part of it.")
        savef.clicked.connect(self.save_formation_as)
        clay.addWidget(savef)
        slay.addWidget(self.compose_box)
        self.compose_box.setVisible(False)

        # ---- MISSION: both run types ---------------------------------------
        # ABOVE the architecture, and shared. It used to be an experiment-only
        # control sitting at the bottom of the tab, which said two wrong
        # things: that a sandbox run has no mission to choose (it does - it was
        # just typed at a terminal instead), and that the mission matters less
        # than the routing. What the fleet is being ASKED TO DO comes first;
        # who decides and how the packets travel are answers to that question.
        self.mission_box = QWidget()
        mlay = QVBoxLayout(self.mission_box)
        mlay.setContentsMargins(0, 6, 0, 0)
        mlay.setSpacing(4)
        mlay.addWidget(QLabel("Mission"))
        self.exp_mission = QComboBox()
        # A different mission needs a different number of goals, so the
        # pickers below are rebuilt the moment it changes.
        self.exp_mission.activated.connect(lambda _i: self._refresh_goals())
        self.exp_mission.setToolTip(
            "The order the fleet is given. In an experiment it is issued to "
            "every vehicle when each run starts, because there is nobody to "
            "type at a headless run. In a sandbox it is issued when you press "
            "'Issue mission', or by hand at the terminal.")
        mlay.addWidget(self.exp_mission)

        # GOAL. A mission names a point; a scene defines the points. Pinning
        # the goal to FAR in the mission file welded every experiment to the
        # corridor - the only scene that has a point by that name - so the
        # goal is chosen here, from the points the CHOSEN SCENE actually
        # declares, and the mission is re-pointed at it.
        # THE GOALS THIS MISSION NEEDS, one picker each.
        #
        # A mission declares HOW MANY points it needs and not where they are -
        # `advance` wants one, `shuttle` wants two, `forward` wants none. So
        # this row is built from the chosen mission rather than being a fixed
        # single dropdown, and choosing a two-goal mission grows a second
        # picker in front of you. That is the whole correction: a scene is the
        # world, a mission is the shape of the task, and WHERE is a decision
        # about this run that belongs to whoever is running it.
        self.lbl_goals = QLabel("Goals")
        mlay.addWidget(self.lbl_goals)
        self.goal_box = QWidget()
        self.goal_lay = QVBoxLayout(self.goal_box)
        self.goal_lay.setContentsMargins(0, 0, 0, 0)
        self.goal_lay.setSpacing(3)
        self.goal_combos = []
        mlay.addWidget(self.goal_box)
        grow = QHBoxLayout()
        addg = QPushButton("+ goal")
        addg.setFixedWidth(72)
        addg.setToolTip(
            "Another leg. The fleet visits the goals in order, so three goals "
            "is a route and one is the penetration command - the same mission "
            "file either way.")
        addg.clicked.connect(lambda: self._change_goals(+1))
        delg = QPushButton("- goal")
        delg.setFixedWidth(72)
        delg.clicked.connect(lambda: self._change_goals(-1))
        grow.addWidget(addg)
        grow.addWidget(delg)
        grow.addSpacing(12)
        self.lbl_lapcap = QLabel("Laps")
        grow.addWidget(self.lbl_lapcap)
        self.mission_laps = QSpinBox()
        self.mission_laps.setRange(1, 999)
        self.mission_laps.setValue(2)
        self.mission_laps.setFixedWidth(64)
        self.mission_laps.valueChanged.connect(
            lambda _v: self._compose_mission())
        self.mission_laps.setToolTip(
            "How many times round the circuit before the mission is PASSED.\n"
            "Only patrol has this: advance visits each point once, by\n"
            "definition, and a patrol of two points is a shuttle.")
        grow.addWidget(self.mission_laps)
        grow.addStretch(1)
        mlay.addLayout(grow)
        prow2 = QHBoxLayout()
        addpt = QPushButton("Add point...")
        addpt.setToolTip(
            "Put a point on the map at coordinates you type, then drag it to "
            "adjust. Points are named P1, P2, ... and are what a mission's "
            "goals are chosen from.\n"
            "Scenes no longer carry objectives of their own: a scene is the "
            "world, and where you send a fleet inside it is yours to decide.")
        addpt.clicked.connect(self.add_point)
        prow2.addWidget(addpt)
        self.lbl_points = QLabel("")
        self.lbl_points.setObjectName("hint")
        prow2.addWidget(self.lbl_points, 1)
        mlay.addLayout(prow2)
        slay.addWidget(self.mission_box)
        self.mission_box.setVisible(False)

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
        # WHO IS THE COORDINATOR. Centralized and hierarchical both need one;
        # decentralized has none by definition, so the picker disappears
        # rather than sitting there greyed and implying otherwise.
        self.lbl_coord = QLabel("Coordinator (the one who decides)")
        alay.addWidget(self.lbl_coord)
        self.coord_combo = QComboBox()
        self.coord_combo.setToolTip(
            "The agent everyone ultimately answers to. Normally the ground "
            "station - it has the mains power, the big antenna and the "
            "operator - but it can be a vehicle, which is what a fleet with "
            "no bench looks like.")
        self.coord_combo.activated.connect(lambda _i: self._on_arch_chosen())
        alay.addWidget(self.coord_combo)

        # WHO REPORTS TO WHOM. Shown when the AUTHORITY is hierarchical (a
        # command tree needs one) or the ROUTING is tiered (squads are what
        # tiered routing partitions on). One row per vehicle, so a squad is
        # built by saying who each vehicle answers to rather than by naming
        # groups - the tree is the thing that matters and this is it, written
        # out.
        self.lbl_tiers = QLabel("Reports to")
        alay.addWidget(self.lbl_tiers)
        self.tier_box = QWidget()
        self.tier_lay = QVBoxLayout(self.tier_box)
        self.tier_lay.setContentsMargins(0, 0, 0, 0)
        self.tier_lay.setSpacing(3)
        self.tier_combos = {}
        alay.addWidget(self.tier_box)

        self.lbl_arch = QLabel("")
        self.lbl_arch.setObjectName("hint")
        self.lbl_arch.setWordWrap(True)
        self.lbl_arch.setMinimumHeight(
            4 * self.lbl_arch.fontMetrics().height() + 4)
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
        issue = QPushButton("Issue mission")
        issue.setToolTip(
            "Give the fleet the mission above.\n"
            "Before Play it is written into the composed run, so the trees "
            "and the map show what the fleet has been told without a sim "
            "having to be running to be told it.\n"
            "During a run it goes down the command channel as SETMISSION, "
            "gated by command authority - a vehicle its commander cannot "
            "reach is skipped, and the skip is printed. That refusal is a "
            "RESULT, not an error.\n\n"
            "ARMING IS SEPARATE, and stays at the terminal: type "
            "`blue launch`. Tasking a fleet and setting it going are two "
            "decisions, and being able to inspect what it intends to do "
            "between them is the point of the split.")
        issue.clicked.connect(self.issue_sandbox_mission)
        sblay.addWidget(issue)

        hint = QLabel(
            "Then, in the BLUE terminal:   blue launch\n\n"
            "Drag a vehicle to move it. Sweep a box to select several and "
            "drag any one of them to move the formation as one. Drag a named "
            "point to move the objective itself. Middle-drag pans.")
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
        # THE BUTTON FIRST. It was at the bottom of a panel long enough to
        # scroll, so the one control everything else exists to feed was the
        # one you had to go looking for. The run COUNT stays down there with
        # the axes, because that is arithmetic about the axes and it belongs
        # beside them.
        expb = QPushButton("Run an experiment...")
        expb.setToolTip("Sweep, then double-click any result to watch that "
                        "exact run play out.")
        expb.clicked.connect(self.open_experiment)
        elay.addWidget(expb)
        elay.addWidget(QLabel("Jammer advantage P_j/P_t (dB)"))
        self.exp_power_axis = AxisRange(
            0.0, 30.0, 7, unit="dB", decimals=1, lo_min=-60.0, hi_max=120.0,
            tip="The jammer's power RELATIVE to the fleet's own radios. A "
                "ratio, so the absolute powers cancel out of the physics and "
                "the result holds for any radio at any scale. Sweep it "
                "finely: the interesting part of this curve is the cliff, "
                "and four points will not find it.")
        self.exp_power_axis.changed(self._refresh_run_count)
        elay.addWidget(self.exp_power_axis)

        # FORMATION as a swept axis. The shape is a decision like any other,
        # so it belongs on the grid rather than in a fleet file - and it only
        # interacts with routing that has peer links, which is itself a result
        # worth measuring rather than assuming.
        elay.addWidget(QLabel("Formations to sweep"))
        frow2 = QHBoxLayout()
        self.exp_form_btn = QPushButton("formations \u25be")
        self.exp_form_btn.setToolTip(
            "Every ticked shape is run against every architecture and every "
            "jammer power. Two shapes doubles the grid.\n"
            "(as spawned) uses the poses on the map, formation or not - the "
            "control the other shapes are compared against.")
        menu = QMenu(self.exp_form_btn)
        self.exp_forms = {}
        for shape in (AS_SPAWNED,) + tuple(FORMATIONS) \
                + tuple(custom_formations()):
            act = QAction(shape, menu)
            act.setCheckable(True)
            act.setChecked(shape == AS_SPAWNED)
            act.toggled.connect(lambda *_: self._refresh_run_count())
            menu.addAction(act)
            self.exp_forms[shape] = act
        self.exp_form_btn.setMenu(menu)
        frow2.addWidget(self.exp_form_btn)
        frow2.addStretch(1)
        elay.addLayout(frow2)

        elay.addWidget(QLabel("Formation spacing (m)"))
        self.exp_space_axis = AxisRange(
            3.0, 12.0, 1, unit="m", decimals=1, lo_min=0.5, hi_max=200.0,
            tip="How far apart neighbouring vehicles sit. Sweeping it asks "
                "the question a formation exists to answer: a tighter shape "
                "holds shorter peer links and relays better, a looser one "
                "spreads the fleet so a single emitter cannot cover it. "
                "Ignored while the only ticked formation is (as spawned), "
                "because there is no shape to scale.")
        self.exp_space_axis.changed(self._refresh_run_count)
        elay.addWidget(self.exp_space_axis)
        self.lbl_runs = QLabel("")
        self.lbl_runs.setObjectName("hint")
        self.lbl_runs.setWordWrap(True)
        # RESERVED HEIGHT. A word-wrapped QLabel reports the height for ONE
        # line until it has been laid out at its final width, so in a narrow
        # column the run count came out as half a line of clipped text. Three
        # lines is what the longest form of this sentence needs.
        self.lbl_runs.setMinimumHeight(
            3 * self.lbl_runs.fontMetrics().height() + 4)
        elay.addWidget(self.lbl_runs)
        # THE EXPERIMENT PANEL GOES FIRST, above the scene and the fleet.
        # In experiment mode the run button is the thing you came for and the
        # composition below it is the detail; putting the panel last meant
        # scrolling past four collapsed boxes to reach it. `insertWidget` at
        # the row after the mode picker rather than `addWidget` at the end.
        slay.insertWidget(slay.indexOf(self.mode_combo) + 1, self.exp_box)
        self.exp_box.setVisible(False)

        # The scene tree (arena + background conditions) lives under the
        # pickers - it describes what Setup composed.
        slay.addStretch(1)
        setup_split.addWidget(setup_scroll)
        setup_split.addWidget(self.tab_env)
        setup_split.setStretchFactor(0, 3)
        setup_split.setStretchFactor(1, 2)
        souter.addWidget(setup_split)
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
        self.series_tree.itemClicked.connect(
            lambda it, _c: self._select_series(it.data(0, Qt.UserRole)))
        rlay.addWidget(self.series_tree, 1)
        hint = QLabel("Drag a series onto a plot, or double-click it.\n"
                      "Right-click a plot to split it.")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        rlay.addWidget(hint)

        hint2 = QLabel("Colour is the authority, dash is the routing. "
                       "Click a series to light it up.")
        hint2.setObjectName("hint")
        hint2.setWordWrap(True)
        rlay.addWidget(hint2)

        findings = QPushButton("Copy findings")
        findings.setToolTip(
            "A plain-text summary of what this run or sweep actually "
            "measured - configuration, mission outcome, and every plotted "
            "series with its range and where it crosses.\n"
            "Copied to the clipboard AND written to the log, so it can be "
            "pasted straight into a write-up.")
        findings.clicked.connect(self.copy_findings)
        rlay.addWidget(findings)

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
        self.emitters.setHorizontalHeaderLabels(["Agent", "Band", "Tx", "Role"])
        self.emitters.setToolTip(
            "Everything that transmits, both sides. The ground station's "
            "30 dBm is why a star reaches as far as it does; a jammer's power "
            "is the attack. Both belong in the same table.")
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
        # (compose the run: scene, fleet, spawns), System (what each side
        # are), Mission (what they are doing), then the analysis tabs
        # Comms, Contested, Results.
        self.tabs.addTab(setup, "Setup")
        self.tabs.addTab(self.tab_scn, "System")
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
        # THE SENSOR PANEL RUNS TO THE BOTTOM OF THE WINDOW. Properties is a
        # short table of fixed length; the sensor view now holds a tab per
        # sensor and needs the height far more than the table does.
        self.resizeDocks([d_props, d_sensor], [180, 900], Qt.Vertical)

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

        # EXPERIMENT HAS NO MENU. It is opened from Setup, where you have just
        # finished deciding what the experiment IS - a menu at the top of the
        # window offered it from anywhere, including from three tabs where
        # nothing is composed yet and pressing it can only produce a refusal.
        # The shortcut stays, as an action on the window with no menu to
        # appear in, because Ctrl+E costs nothing and skips the trip.
        # THE THREE APPLICATIONS, as menus rather than tabs, because they are
        # not three views of one run - they are three different jobs, and a
        # tab strip would put them beside Setup and Results as though they
        # were peers of a tab. Clicking one swaps the whole window.
        self._app_menus = {}
        for i, name in enumerate(("&Simulation", "Sc&ene", "&Agent")):
            menu = self.menuBar().addMenu(name)
            plain = name.replace("&", "")
            act = QAction(f"Open {plain}", self)
            act.triggered.connect(lambda _c=False, k=i: self.show_app(k))
            menu.addAction(act)
            self._app_menus[plain] = menu
            # The menu title is itself the button: clicking it opens a
            # one-item menu, which is a click too many. Make the menu's own
            # press switch the app.
            menu.aboutToShow.connect(lambda k=i: self.show_app(k))

        a = QAction("Run an experiment...", self)
        a.setShortcut("Ctrl+E")
        a.setShortcutContext(Qt.ApplicationShortcut)
        a.triggered.connect(self.open_experiment)
        self.addAction(a)

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

    # -- drag to place ------------------------------------------------------

    def _agent_dragged(self, aid, pose):
        """Live, while the mouse is down: update the spawn and say where it is.

        The readout is the point. A formation built by eye needs a number to
        argue with afterwards, so the gap to the nearest other vehicle is
        printed as it changes - which is also how you discover that the shape
        you drew is 2.6 m across when you thought it was 3.
        """
        if not aid or aid not in self._spawns:
            return
        keep = self._spawns[aid]
        keep.update({"x": round(float(pose.get("x", 0.0)), 3),
                     "y": round(float(pose.get("y", 0.0)), 3),
                     "z": round(float(pose.get("z", 0.0)), 3)})
        n = len(self.viewport.selected) + len(self.viewport.selected_points)
        if n > 1:
            self.statusBar().showMessage(
                f"moving {n} together - {aid} at "
                f"({keep['x']:.2f}, {keep['y']:.2f}, {keep['z']:.2f}) m")
            return
        near = None
        for oid, o in self._spawns.items():
            if oid == aid:
                continue
            d = math.dist((keep["x"], keep["y"], keep["z"]),
                          (o.get("x", 0.0), o.get("y", 0.0), o.get("z", 0.0)))
            near = d if near is None else min(near, d)
        self.statusBar().showMessage(
            f"{aid}  ({keep['x']:.2f}, {keep['y']:.2f}, {keep['z']:.2f}) m"
            + (f"   nearest vehicle {near:.2f} m" if near is not None else ""))

    def _point_dragged(self, name, q):
        """A scene point, moved by hand.

        Held as an OVERRIDE rather than written back to scenes/: the scene is
        the world as it is, and where you decide to send a fleet inside it is
        a decision about this run. _compose_setup writes the override into the
        composed doc, where the model's own one-level points merge puts it
        over the scene's own value.
        """
        if not name:
            return
        self._points_override[name] = {
            "x": round(float(q.get("x", 0.0)), 3),
            "y": round(float(q.get("y", 0.0)), 3),
            "z": round(float(q.get("z", 0.0)), 3)}
        v = self._points_override[name]
        self.statusBar().showMessage(
            f"point {name}  ({v['x']:.2f}, {v['y']:.2f}, {v['z']:.2f}) m")

    def _agent_dropped(self, aids, pnames=()):
        """Let go: recompose once, so the run, the trees and the map agree.

        Recomposing writes the composed YAML and reloads the world, which is
        far too expensive to do on every mouse move - hence a separate drop
        hook rather than doing it in the move handlers.
        """
        aids = [a for a in (aids or []) if a in self._spawns]
        if not aids and not pnames:
            return
        # A NAMED SHAPE SURVIVES A TRANSLATION AND NOTHING ELSE. Every member
        # of a group drag moves by the same vector, so if EVERY mobile vehicle
        # on a side moved, its shape is untouched and calling it a wedge is
        # still true. Move some of them and it is not a wedge any more, so
        # the experiment tab must stop saying it is - claiming a shape the map
        # does not show is exactly the kind of quiet wrongness that makes a
        # result impossible to explain later.
        moved = set(aids)
        for side in ("blue", "red"):
            mine = self._mobile_ids(side)
            if not mine or not (moved & mine):
                continue
            if not mine <= moved:
                self._formation[side] = AS_SPAWNED
        if self._setup_scene:
            self._compose_setup()

    def _mobile_ids(self, side):
        """The spawned agents on this side a formation governs.

        Not the ground station: it is never in a formation, so whether it came
        along on a drag says nothing about whether the shape survived.
        """
        ids = (self._red_ids if side == "red" else self._blue_ids) or set()
        view = getattr(self, "resolved", None) or self.doc or {}
        bodies = {a.get("id"): a for a in (view.get("agents") or [])}
        out = set()
        for aid in ids:
            b = bodies.get(aid) or {}
            if b.get("ghost") or b.get("jammer") \
                    or b.get("platform") == "ground_station":
                continue
            out.add(aid)
        return out

    def _selection_changed(self, aids, pnames):
        n = len(aids) + len(pnames)
        if n == 0:
            self.statusBar().showMessage(
                "nothing selected - sweep a box round the fleet to move it "
                "as one, or drag a single vehicle")
        else:
            self.statusBar().showMessage(
                f"{n} selected ({', '.join(sorted(aids) + sorted(pnames))})"
                f" - drag any one of them to move them all together")

    def _update_placing(self):
        """Dragging is allowed only where it means something: on the Setup
        tab, before a run, in a view that has an inverse projection.

        Not ISO - one screen point maps to a LINE in an isometric world, so a
        drag there would have to invent the third coordinate, and an invented
        coordinate is exactly the kind of quiet wrongness that makes a
        formation subtly off with nothing on screen to show it.
        """
        on_setup = (self.tabs.tabText(self.tabs.currentIndex()) == "Setup"
                    if hasattr(self, "tabs") else False)
        self.viewport.placing = bool(
            on_setup and self._spawns
            and not getattr(self, "_setup_locked", False))

    # -- interception -------------------------------------------------------

    def _print_intercepts(self, f):
        """Print, into the listening cell, every order it could actually hear.

        The RF decision is the model's - each transmission arrives already
        carrying who could hear it, scored with the same rf_link as everything
        else. All this does is choose which of them to show: the ones on the
        band the red cell tuned to, heard by an agent on the listening cell's
        own side.
        """
        band = getattr(self, "_listen_band", None)
        shell = getattr(self, "_listen_shell", None)
        if band is None or shell is None:
            return
        side = ShellPanel.CELL_SIDE.get(getattr(shell, "cell", "red"), "red")
        for tx in (f.get("transmissions") or []):
            if abs(_num(tx.get("band_mhz")) - band) > 0.5:
                continue
            mine = [h for h in (tx.get("heard_by") or [])
                    if h.get("network") == side]
            if not mine:
                continue
            best = max(mine, key=lambda h: _num(h.get("sinr_db")))
            shell.out.appendPlainText(
                f"[{_num(tx.get('t')):7.1f}s  {band:g} MHz  "
                f"SINR {_num(best.get('sinr_db')):+5.1f} dB  "
                f"via {best.get('id')}]  "
                f"{tx.get('from')} -> {tx.get('to')}:  {tx.get('text')}")

    def _show_mission_score(self, f):
        """The mission outcome in the status bar, live.

        Three numbers, and the middle one is the interesting one: passed,
        AWAITING ORDERS, failed. A fleet that is awaiting orders has not
        failed and cannot proceed - it arrived somewhere and the order telling
        it where to go next never came.
        """
        sc = f.get("mission_score") or {}
        if not sc.get("tasked"):
            self._mission_line = ""
            return
        bits = [f"mission {sc.get('pass_frac', 0) or 0:.0%}"
                f" ({sc.get('complete', 0)}/{sc.get('tasked', 0)})"]
        if sc.get("awaiting_orders"):
            bits.append(f"{sc['awaiting_orders']} AWAITING ORDERS")
        if sc.get("failed"):
            bits.append(f"{sc['failed']} failed"
                        + (f" ({sc['drifted']} drifted)"
                           if sc.get("drifted") else ""))
        self._mission_line = "   ".join(bits)
        if hasattr(self, "lbl_mission") and self._mission_name:
            self.lbl_mission.setText(
                f"Mission: {self._mission_name}   {self._mission_line}")

        # AND SAY IT WHERE THE OPERATOR IS LOOKING. The score was only ever
        # written to a label at the top of the window, which is exactly where
        # nobody is looking while a run is up - the whole point of the blue
        # cell is that it is the blue commander's console. Every change to the
        # tally is announced once, in the log and in the cell that owns the
        # fleet, so "mission 100% (3/3)" is a line you can scroll back to
        # rather than a number that was on screen for a moment.
        key = (sc.get("complete"), sc.get("failed"), sc.get("tasked"),
               sc.get("awaiting_orders"), sc.get("drifted"))
        if key != getattr(self, "_mission_score_key", None):
            self._mission_score_key = key
            name = self._mission_name or "mission"
            line = f"{name}: {self._mission_line}"
            if sc.get("complete") == sc.get("tasked"):
                line += "  - MISSION COMPLETE"
            self.say(line)
            self.cell_say("blue", line)

    # -- reload -------------------------------------------------------------

    # Where a Console session USED to be parked across a reload. Refresh is
    # now a clean slate by design (see reload_console), so nothing writes or
    # reads this - the name is kept only so an old file is recognisable.
    SESSION_FILE = "runs/console_session.json"

    def reload_console(self):
        """REFRESH: close the Console, reopen it on the current code, empty.

        Two buttons that were doing nearly the same thing now do two clearly
        different things, which is what was asked for:

          Refresh (this)  close and reopen, and FORGET the setup. Pressed
                          after a change to the Console's own code, when what
                          you want is the new code and a clean slate. It used
                          to write the setup out and put it back, which made
                          it indistinguishable from Restart and meant a stale
                          half-composed run followed you across the reload.
          Restart         re-run what is already set up, without closing
                          anything. The Console is working; you just want the
                          run again, or the run with one thing changed.

        A fresh PROCESS rather than a hot reload, deliberately: re-importing a
        running Qt application leaves half the old widgets alive and connected
        to functions that no longer exist, and those failures are far worse
        than the seconds it saves.

        The SIMULATOR needs none of this - it is re-imported every time you
        press Play, so a change to the model is picked up with no restart at
        all.
        """
        import subprocess
        if (self.proc and self.proc.state() != QProcess.NotRunning) \
                or getattr(self, "ws", None) is not None:
            self.say("Stop the run first - reloading under a live simulator "
                     "would leave it talking to a Console that no longer "
                     "exists.")
            return
        # CLEAN SLATE MEANS CLEAN SLATE. Drop any session an older build may
        # have left behind, so the new process cannot restore one.
        path = REPO_ROOT / self.SESSION_FILE
        try:
            path.unlink()
        except OSError:
            pass
        try:
            subprocess.Popen([sys.executable] + sys.argv,
                             cwd=str(REPO_ROOT), close_fds=True)
        except OSError as exc:
            self.say(f"could not restart: {exc}")
            return
        self._reloading = True
        QApplication.quit()

    def _focus_setup(self):
        """SETUP IS A MODAL STEP, so make the window say so.

        Reported: "every other tab needs to be greyed out when in setup view
        to stop confusion". It is the right instinct - while a run is being
        composed, every other tab is showing the LAST world, or nothing at
        all, and reading a network diagram that does not correspond to the
        fleet you are currently choosing is worse than having no diagram.

        So: on Setup with no run up, the rest of the window is greyed. Press
        Play and it inverts - Setup locks (see _lock_setup) and everything
        else comes alive against a world that actually exists.
        """
        if not hasattr(self, "tabs"):
            return
        # UNTIL PLAY, NOTHING BUT SETUP. Not "while the Setup tab happens to
        # be showing" - that let a click on another tab escape into a world
        # that does not exist yet. Reported: "until we press play we shouldn't
        # be able to click on anything other than setup".
        #
        # It unlocks on the first Play of a composition and stays unlocked
        # after Stop, so Results is readable; choosing a different scene or
        # fleet is a new composition and locks it again.
        composing = not (getattr(self, "_played_once", False)
                         or getattr(self, "_setup_locked", False))
        for i in range(self.tabs.count()):
            if self.tabs.tabText(i) == "Setup":
                continue
            self.tabs.setTabEnabled(i, not composing)
            self.tabs.setTabToolTip(
                i, "Compose the run and press Play - this tab has no world "
                   "to show yet" if composing else "")

    def _lock_setup(self, locked):
        """The Setup tab is how a run is COMPOSED; while one is actually
        running, recomposing under it is a crash waiting to happen (the sim
        holds the old world, the Console loads a new one). Lock the tab for
        the duration; everything else stays live."""
        self._setup_locked = bool(locked)
        for i in range(self.tabs.count()):
            if self.tabs.tabText(i) == "Setup":
                if locked and self.tabs.currentIndex() == i:
                    self.tabs.setCurrentIndex(i + 1)   # step off it first
                self.tabs.setTabEnabled(i, not locked)
                self.tabs.setTabToolTip(
                    i, "Locked while a run is up - Stop to recompose"
                       if locked else "")
                break
        self._focus_setup()
        self._update_placing()

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
        # A NEW COMPOSITION. Whatever was on the other tabs described the last
        # world; lock them again until this one has been run - see
        # _focus_setup.
        self._played_once = False
        self._setup_scene = self.scene_combo.currentText()
        self._formation, self._spacing, self._spawn_point = {}, {}, {}
        # Points are coordinates in the OLD world. Carrying them would put
        # FAR somewhere the new scene never put it.
        self._points_override = {}
        # A new scene invalidates any placed fleet - spawns are coordinates
        # in the OLD world. Ask again rather than silently carrying them.
        self._setup_fleet = None
        self._setup_red = None
        self._spawns = {}
        self.fleet_combo.setEnabled(True)
        self.fleet_combo.setCurrentIndex(0)
        self.red_combo.setEnabled(True)
        self.red_combo.setCurrentIndex(0)
        # COMPOSE FIRST, then list the points: the composition is what
        # actually resolves the scene (and any points added by hand on top of
        # it), so listing before it would offer the previous world's points.
        self._compose_setup()
        self._refresh_goals()
        self.statusBar().showMessage(
            f"Scene {self._setup_scene} - now choose a blue fleet")

    def on_fleet_chosen(self, index, side):
        combo = self.red_combo if side == "red" else self.fleet_combo
        if not self._setup_scene:
            return
        self._played_once = False           # a new composition; see _focus_setup
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
        # Recorded, not just applied. The spawn dialog has already baked the
        # shape into the coordinates, so the run does not need this - but the
        # EXPERIMENT tab does, to offer the shape you actually placed as the
        # default value on the formation axis, and the results need it to say
        # what was flown.
        self._formation[side] = dlg.formation()
        self._spacing[side] = dlg.spacing()
        self._spawn_point[side] = dlg.spawn_point()
        if side == "red":
            self._setup_red = fleet
            self._red_ids = set(new_spawns)
        else:
            self._setup_fleet = fleet
            self._blue_ids = set(new_spawns)
        self._spawns.update(new_spawns)
        self._refresh_command_structure()
        self._compose_setup()
        self._update_placing()
        both = " + ".join(x for x in (self._setup_fleet, self._setup_red) if x)
        self.statusBar().showMessage(
            f"{self._setup_scene} + {both} - drag a vehicle on the map to "
            f"move it, then Play, SETMISSION <name>, blue launch")

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

    def save_formation_as(self):
        """Write the arrangement on the map as a named formation.

        The offsets are taken from the MOBILE vehicles only and centred on
        their own middle, so the saved shape is a shape and not a position -
        it can be placed anywhere afterwards. The ground station is excluded
        for the same reason it is excluded everywhere else: every link in the
        run is measured against where the bench is.
        """
        if save_formation is None:
            self.say("formations are unavailable - the model did not import.")
            return
        movable = []
        view = getattr(self, "resolved", None) or self.doc or {}
        bodies = {a.get("id"): a for a in (view.get("agents") or [])}
        for aid, pose in (self._spawns or {}).items():
            b = bodies.get(aid) or {}
            if b.get("ghost") or b.get("jammer") \
                    or b.get("platform") == "ground_station" \
                    or aid in (self._red_ids or set()):
                continue
            movable.append((aid, pose))
        if len(movable) < 2:
            self.say("A formation needs at least two mobile vehicles on the "
                     "map. Choose a scene and a blue fleet first.")
            return
        name, ok = QInputDialog.getText(
            self, "Save formation", "Name this shape:")
        name = (name or "").strip().replace(" ", "_")
        if not ok or not name:
            return
        try:
            path = save_formation(
                name, [(p.get("x", 0.0), p.get("y", 0.0), p.get("z", 0.0))
                       for _aid, p in movable])
        except (OSError, ValueError) as exc:
            self.say(f"cannot save that formation: {exc}")
            return
        self._refresh_formation_axis()
        self.say(f"Saved formation '{name}' ({len(movable)} vehicles) -> "
                 f"{path.name}. It is now a value on the experiment's "
                 f"formation axis.")

    def _refresh_formation_axis(self):
        """(Re)list the saved shapes into the experiment's formation menu,
        keeping whatever is already ticked."""
        btn = getattr(self, "exp_form_btn", None)
        if btn is None:
            return
        menu = btn.menu()
        ticked = {k for k, a in self.exp_forms.items() if a.isChecked()}
        for name in custom_formations():
            if name in self.exp_forms:
                continue
            act = QAction(name, menu)
            act.setCheckable(True)
            act.setChecked(name in ticked)
            act.toggled.connect(lambda *_: self._refresh_run_count())
            menu.addAction(act)
            self.exp_forms[name] = act
        self._refresh_run_count()

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
        self._formation.pop(side, None)
        self._spacing.pop(side, None)
        self._spawn_point.pop(side, None)


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
    SWEEP_METRICS = ("mission_pass_frac", "mission_complete", "mission_failed",
                     "mission_awaiting_orders", "mission_drifted",
                     "penetration_m", "penetration_frac", "commanded_fraction",
                     "held_fraction", "belief_err_m", "track_err_m",
                     "worst_sinr_db", "worst_pdr", "arrived", "distance_m",
                     "ended_s")

    # The columns that describe HOW a run was configured, as opposed to what
    # came out of it. Filtering or naming a series by an output is how you
    # fool yourself, so outputs are deliberately absent.
    SWEEP_CONFIG_COLS = ("authority", "routing", "formation", "spacing",
                         "jam_rel_db", "seed")

    # Bookkeeping, not measurement. Kept out of the plottable set so a chart
    # cannot offer you the wall-clock time it took to compute a result.
    NOT_A_METRIC = ("cell", "error", "wall_s", "jam_dbm", "runs_agents")

    @staticmethod
    def _series_id(row, varying):
        """A legend name from the configuration, short enough to read."""
        bits = [str(row.get(c)) for c in ("authority", "routing")
                if row.get(c) not in (None, "")]
        bits += [str(row.get(c)) for c in varying
                 if c not in ("authority", "routing")
                 and row.get(c) not in (None, "")]
        return "_".join(bits) or "run"

    def sweep_frames(self, rows, xkey="jam_rel_db"):
        """Turn swept results into frames the existing plotting can draw.

        A sweep has the same SHAPE as a run - a value per step - so each
        swept parameter value becomes a frame whose sim_time_s IS that value,
        and each architecture becomes an agent whose fields are its metrics.
        That one substitution is what makes every plot in this application
        work on a sweep without a line of new drawing code.
        """
        # The silent-jammer control is dropped from the swept axis for the
        # same reason it is off the chart's x scale: it is a condition, not a
        # power, and -999 on an axis of decibels destroys the axis. It is
        # still in the Results table, where it belongs and reads correctly.
        xs = sorted({float(r[xkey]) for r in rows
                     if r.get(xkey) not in (None, "")
                     and not (xkey == "jam_rel_db"
                              and float(r[xkey]) <= JAM_OFF_DB / 2)})
        # ONE SERIES PER CONFIGURATION THAT ACTUALLY VARIES. The series used
        # to be named authority_routing, which was right until formation and
        # spacing became axes: two shapes then collapsed into one line and
        # silently averaged with each other, which is the worst possible way
        # for a chart to be wrong. Any configured column with more than one
        # value in these rows goes into the name; any column with a single
        # value stays out of it, so the legend does not carry a constant.
        varying = [c for c in self.SWEEP_CONFIG_COLS
                   if c != xkey
                   and len({r.get(c) for r in rows if r.get(c) not in
                            (None, "")}) > 1]
        # Kept, so a row in the results table can work out which series on the
        # chart is its own - see ExperimentDialog._light_row.
        self._sweep_varying = list(varying)
        frames = []
        for x in xs:
            agents = []
            for r in rows:
                try:
                    if float(r.get(xkey)) != x:
                        continue
                except (TypeError, ValueError):
                    continue
                aid = self._series_id(r, varying)
                body = {"id": aid, "platform": "result", "network": "blue",
                        "colour": ResultPlot.AUTH_COLOUR.get(
                            r.get("authority"), "#AAAAAA")}
                # EVERY NUMERIC OUTPUT THIS RUN PRODUCED, not a fixed list.
                # SWEEP_METRICS could only ever carry metrics somebody had
                # thought to add to it, so the interception columns - which
                # are named after the listener and cannot be listed in
                # advance - existed in the CSV and could not be plotted. The
                # configuration columns are excluded by name: plotting a
                # series against how it was configured is how you fool
                # yourself.
                for k, v in r.items():
                    if (k in self.SWEEP_CONFIG_COLS
                            or k in self.NOT_A_METRIC or v in (None, "")):
                        continue
                    try:
                        body[k] = float(v)
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
                        warmup_s=0.0, jam_dbm=None, goal=None):
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
            # THE GOAL TRAVELS WITH THE MISSION. Replaying a cell that ran on
            # a scene whose points are not the corridor's would otherwise
            # issue `advance to FAR` into a world with no FAR in it, and the
            # fleet would sit still while the table said it had advanced.
            gs = goal if isinstance(goal, (list, tuple)) else (
                [goal] if goal else [])
            self._send_setup_orders(
                f"{mission} to {' '.join(gs)}" if gs else mission, red=False)
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
        self._send_lines(lines)

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
        self.mission_box.setVisible(mode > 0)
        self.arch_box.setVisible(mode > 0)
        self.sandbox_box.setVisible(mode == 1)
        self.exp_box.setVisible(mode == 2)
        if mode == 0:
            return
        experiment = mode == 2
        self._refresh_missions()
        if experiment:
            self._refresh_formation_axis()
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


    # THE RECOMMENDED OBJECTIVES, with what each is FOR. A verb list is not
    # much use on its own - `orbit 2.0` tells you nothing about why you would
    # want it - so each carries the situation it was written for. These are
    # the verbs _parse_objective_verb already understands; nothing here is a
    # new capability, it is the existing grammar made findable.
    # THE OBJECTIVE PALETTE IS GONE, and its absence is the design.
    #
    # It offered eight verbs on the Setup tab beside the mission, which put
    # two ways of tasking a fleet next to each other and made it look as
    # though you had to choose. You do not. A MISSION is the task - go to
    # these points, in this order, this many times - and the OBJECTIVE each
    # vehicle holds is generated from it one leg at a time by the
    # coordinator. It is always just "go there", and the formation is
    # preserved, so there was never anything for an operator to pick.
    #
    # The verbs still exist for what they are for: retasking ONE vehicle onto
    # something the fleet is not doing - a scout, a pursuer, a deliberate
    # relay left behind. That is a per-agent exception, it is typed at the
    # terminal where exceptions belong, and it is documented in COMMANDS.md:
    #
    #     car3: pursue car1
    #     car3: wall_follow right
    #     car3: stop
    #
    # A vehicle retasked that way loses its plan, because `pursue` has no end
    # and putting a task that can never finish into the pass/fail column would
    # leave it failing forever.

    def mission_plan(self):
        """The plan the pickers describe, or None.

        {who, waypoints, laps} - the whole mission as one dict, which is what
        goes into the composed run and what SETMISSION reconstructs.
        """
        name = (self.exp_mission.currentText()
                if hasattr(self, "exp_mission") else "")
        goals = self._goal_names()
        if not name or not goals:
            return None
        return {"who": "all", "waypoints": goals,
                "laps": self.mission_laps_value(), "mission": name}

    def _compose_mission(self):
        """Write the chosen mission into the composed run.

        THE MISSION IS SET BEFORE PLAY. It used to be a command and nothing
        else, which meant that until the sim was running the fleet had no
        mission to look at: the trees said UNASSIGNED, the map showed nothing,
        and you pressed Play partly to find out what you had set up. A run is
        COMPOSED here - scene, fleet, spawns, doctrine, architecture - and the
        mission is one more thing composed with it.

        Nothing about the command gate is lost. At t=0 nothing has been jammed
        yet, so gating the initial assignment would be theatre; every order
        issued after the run starts still travels the command channel and is
        still refused when it cannot get through.
        """
        if not getattr(self, "_setup_scene", None):
            return
        plan = self.mission_plan()
        if plan == getattr(self, "_plan_composed", None):
            return
        self._plan_composed = plan
        self._compose_setup()
        if plan:
            self._mission_name = plan["mission"]
            laps = plan["laps"]
            self.lbl_mission.setText(
                f"Mission: {plan['mission']}   "
                + " -> ".join(plan["waypoints"])
                + (f"   x{laps}" if laps > 1 else ""))
        else:
            self._mission_name = None
            self.lbl_mission.setText("Mission: UNASSIGNED")

    def issue_sandbox_mission(self):
        """Give the fleet its mission - composed before Play, commanded after.

        ARMING IS NOT HERE. `blue launch` stays at the terminal because
        tasking a fleet and setting it going are two decisions, and being able
        to inspect what it intends to do in between is the whole reason they
        were split.
        """
        plan = self.mission_plan()
        if plan is None:
            self.say("Choose a mission and give it at least one goal first. "
                     "Add point, then pick it in the goal slot.")
            return
        running = (self.proc and self.proc.state() != QProcess.NotRunning) \
            or getattr(self, "ws", None) is not None
        self._compose_mission()
        laps = plan["laps"]
        line = (f"{plan['mission']} to {' '.join(plan['waypoints'])}"
                + (f" laps {laps}" if laps > 1 else ""))
        if running:
            self._send_lines([f"SETMISSION {line}"])
            self.say(f"SETMISSION {line}\n"
                     f"  Gated by command authority: any vehicle its "
                     f"commander cannot reach is skipped and the skip "
                     f"printed. That is a result.\n"
                     f"  Now arm them - in the BLUE terminal: blue launch")
        else:
            self.say(f"Mission set: {line}\n"
                     f"  Written into the composed run, so it is already "
                     f"assigned when the world starts. Press Play, then in "
                     f"the BLUE terminal: blue launch")

    def add_point(self):
        """Put a point on the map at coordinates you type, named P1, P2, ...

        THIS IS WHERE OBJECTIVES COME FROM NOW. A scene used to ship named
        points - HOME, FAR, APEX - and every one of them was an objective in
        disguise, decided by whoever wrote the scene rather than by whoever is
        running it. A scene is the world; where you send a fleet inside it is
        yours.

        Numbered rather than named because naming six points is friction in
        the way of the thing you actually wanted to do, and because a mission
        no longer cares what they are called - it asks for N goals and you
        point each slot at one of these.
        """
        if not self._setup_scene:
            self.say("Choose a scene first - a point has to be somewhere.")
            return
        dlg = PointDialog(self.viewport.arena, self)
        if dlg.exec() != QDialog.Accepted:
            return
        used = set(self.viewport.points or {}) | set(self._points_override)
        n = 1
        while f"P{n}" in used:
            n += 1
        name = f"P{n}"
        self._points_override[name] = dlg.point()
        self._compose_setup()
        self._refresh_goals()
        self.viewport.selected_points = {name}
        self.viewport.update()
        q = self._points_override[name]
        self.say(f"Added {name} at ({q['x']:.0f}, {q['y']:.0f}, {q['z']:.0f}) "
                 f"- drag it on the map to adjust, then pick it as a goal.")

    def _on_goal_chosen(self, _i):
        self._goal_touched = True
        self._compose_mission()

    def _send_lines(self, lines):
        """Write command lines to the retask spool - one file per command."""
        d = getattr(self, "_retask_dir", None) or (REPO_ROOT / "runs" / "retask")
        try:
            d.mkdir(parents=True, exist_ok=True)
            for i, ln in enumerate(lines):
                (d / f"cmd_{time.time_ns()}_{i}.txt").write_text(
                    ln + "\n", encoding="utf-8")
        except OSError as exc:
            self.say(f"could not send that order: {exc}")

    def _refresh_missions(self):
        cur = self.exp_mission.currentText()
        self.exp_mission.blockSignals(True)
        self.exp_mission.clear()
        for f in sorted((REPO_ROOT / "missions").glob("*.yaml")):
            self.exp_mission.addItem(f.stem)
        i = self.exp_mission.findText(cur or "advance")
        self.exp_mission.setCurrentIndex(max(i, 0))
        self.exp_mission.blockSignals(False)
        self._refresh_goals()

    def _change_goals(self, delta):
        """Add or drop a goal slot. A mission that fixes its own count says
        so rather than silently ignoring the button."""
        spec = self._mission_limits()
        if spec["goals"] != "any":
            self.say(f"'{self.exp_mission.currentText()}' fixes its own goals "
                     f"({spec['goals']}) - there is nothing to add.")
            return
        self._want_goals = max(1, int(getattr(self, "_want_goals", 1)) + delta)
        self._refresh_goals()

    def _mission_limits(self):
        """{"goals": n|"any"|0, "laps": n|"any"} for the chosen mission."""
        name = (self.exp_mission.currentText()
                if hasattr(self, "exp_mission") else "")
        if not name or _mission_spec is None:
            return {"goals": 0, "laps": 1}
        try:
            return _mission_spec(REPO_ROOT / "missions" / f"{name}.yaml")
        except Exception:                                  # noqa: BLE001
            return {"goals": 0, "laps": 1}

    def _refresh_goals(self):
        """Build the goal pickers this mission needs and fill them.

        HOW MANY IS ALSO A DECISION. A mission declares the SHAPE of a task -
        visit points in order, or loop them - and nothing about geometry, not
        even how much of it there is. `advance` with one goal is the
        penetration command; with three it is a route; it is the same file.
        So the count comes from the operator here, not from the mission, and
        the + / - buttons are how it is said.

        Laps appear only for a mission that comes back. Advance visits each
        point once by definition, and offering a lap count for it would be
        offering to turn it into a patrol under another name.
        """
        if not hasattr(self, "goal_lay"):
            return
        pts = dict((self._view() or {}).get("points") or {})
        spec = self._mission_limits()
        name = self.exp_mission.currentText() if hasattr(self, "exp_mission") \
            else ""
        if spec["goals"] == "any":
            want = max(1, int(getattr(self, "_want_goals", 1) or 1))
        else:
            # A mission that fixes its own count must NOT overwrite what the
            # operator built for one that does not - stepping through
            # `forward` (which needs none) on the way back to `advance` used
            # to silently reset a three-goal route to one.
            want = int(spec["goals"])

        keep = [c.currentData() for c in self.goal_combos]
        while len(self.goal_combos) > want:
            self.goal_combos.pop()._row.setParent(None)
        while len(self.goal_combos) < want:
            row = QWidget()
            rl = QHBoxLayout(row)
            rl.setContentsMargins(0, 0, 0, 0)
            rl.setSpacing(4)
            lab = QLabel(f"Goal {len(self.goal_combos) + 1}")
            lab.setFixedWidth(52)
            rl.addWidget(lab)
            c = QComboBox()
            c.activated.connect(self._on_goal_chosen)
            c.setToolTip(
                "One of the points on the map. The fleet visits the goals in "
                "order, in formation.\n"
                "Drag the point itself on the map to move it.")
            rl.addWidget(c, 1)
            self.goal_lay.addWidget(row)
            c._row = row
            self.goal_combos.append(c)

        names = sorted(pts, key=lambda k: (len(k), k))
        for i, c in enumerate(self.goal_combos):
            c.blockSignals(True)
            c.clear()
            for n in names:
                q = pts[n] or {}
                c.addItem(f"{n}   ({_num(q.get('x')):.0f}, "
                          f"{_num(q.get('y')):.0f}, {_num(q.get('z')):.0f})")
                c.setItemData(c.count() - 1, n)
            # KEEP WHAT WAS CHOSEN, then fall back to a DIFFERENT point per
            # slot: a two-goal mission whose ends default to the same point is
            # a vehicle standing still, and it would look like the model
            # failing rather than the defaults being lazy.
            j = c.findData(keep[i]) if i < len(keep) else -1
            if j < 0:
                j = min(i, max(len(names) - 1, 0))
            c.setCurrentIndex(max(j, 0))
            c.blockSignals(False)

        has = want > 0
        self.goal_box.setVisible(has and bool(names))
        self.lbl_goals.setVisible(has)
        laps_open = spec["laps"] == "any"
        self.lbl_lapcap.setVisible(laps_open)
        self.mission_laps.setVisible(laps_open)
        if not has:
            self.lbl_goals.setText(f"'{name}' needs no goals")
        elif not names:
            self.lbl_goals.setText(
                "Goals - add a point first (Add point, below)")
        else:
            self.lbl_goals.setText(
                f"Goals for '{name}' - visited in order"
                + (", then repeated" if laps_open else ", once each"))
        self.lbl_points.setText(
            f"{len(names)} point{'s' if len(names) != 1 else ''} on the map"
            if names else "no points yet")
        self._compose_mission()

    def mission_laps_value(self):
        """The lap count, or 1 for a mission that does not repeat."""
        return (int(self.mission_laps.value())
                if self._mission_limits()["laps"] == "any" else 1)

    def _goal_names(self):
        """Every chosen goal, in slot order."""
        return [c.currentData() for c in getattr(self, "goal_combos", [])
                if c.currentData()]

    def _goal_name(self):
        """The FIRST goal's name, or None. Penetration is measured to it."""
        names = self._goal_names()
        return names[0] if names else None

    def _sweep_axes(self):
        """Every swept axis and its values, in one place.

        Both the run counter and the experiment config read this, so the
        number on the label and the number of runs cannot disagree - which
        they could when each built its own list.
        """
        shapes = [k for k, act in getattr(self, "exp_forms", {}).items()
                  if act.isChecked()] or [AS_SPAWNED]
        axes = {
            "authority": (["centralized", "decentralized", "hierarchical"]
                          if self.auth_combo.currentText() == self.SWEEP_ALL
                          else [self.auth_combo.currentText()]),
            "routing": (["star", "mesh", "tiered"]
                        if self.route_combo.currentText() == self.SWEEP_ALL
                        else [self.route_combo.currentText()]),
            "jam_rel_db": self.exp_power_axis.values(),
            "formation": shapes,
        }
        # SPACING ONLY WHEN THERE IS A SHAPE TO SCALE. Sweeping it against
        # (as spawned) alone would multiply the grid by runs that are
        # byte-identical to each other, which is not a result, it is a wait.
        if shapes != [AS_SPAWNED]:
            axes["spacing"] = self.exp_space_axis.values()
        return axes

    def _refresh_run_count(self):
        """The grid size, live, as you tick. Not a time estimate - an estimate
        is a guess you then have to defend; this is arithmetic."""
        if not hasattr(self, "lbl_runs") or self.mode_combo.currentIndex() != 2:
            return
        axes = self._sweep_axes()
        parts = " x ".join(f"{len(v)} {k}" for k, v in axes.items())
        # THE SWEEP'S OWN GRID FUNCTION, not a second copy of the arithmetic.
        # cells_of drops the cells that are duplicates by construction (a
        # spacing swept against (as spawned) scales nothing), so counting the
        # product here would promise runs that never happen.
        try:
            cells, _names = _sweep_module().cells_of(
                {"axes": axes, "seeds": [1]})
            n = len(cells)
        except Exception:                                  # noqa: BLE001
            n = 1
            for v in axes.values():
                n *= max(len(v), 0)
        product = 1
        for v in axes.values():
            product *= max(len(v), 0)
        dropped = product - n
        self.lbl_runs.setText(
            (f"{n} runs  =  {parts}"
             + (f"   minus {dropped} that spacing cannot change - "
                f"(as spawned) has no shape to scale" if dropped > 0 else ""))
            if n else "An axis has no values - give every axis at least one.")
        if hasattr(self, "exp_form_btn"):
            ticked = axes["formation"]
            self.exp_form_btn.setText(
                f"{len(ticked)} formation"
                f"{'s' if len(ticked) != 1 else ''}  \u25be")


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
        axes = self._sweep_axes()
        if not axes["jam_rel_db"]:
            return None, "The jammer axis has no values - set steps to 1 or more."
        if self._mission_limits()["goals"] and not self._goal_names():
            return None, ("This mission needs at least one goal. Add a point "
                          "(Setup -> Add point), then pick it in the goal "
                          "slot.")
        # Squads for the hierarchical and tiered cells. The first blue vehicle
        # leads - stated here rather than assumed silently, and replaced by a
        # squad column in the fleet table when that lands.
        # THE SAME TREE THE SANDBOX WOULD HAVE RUN, drawn on the Setup tab.
        squads = self._squads()
        name = "_".join(x for x in (self._setup_scene,
                                    self.exp_mission.currentText(),
                                    self._goal_name()) if x)
        return {
            "name": name or "experiment",
            "compose": copy.deepcopy(composed),
            "mission": self.exp_mission.currentText() or "advance",
            # None means "leave the mission's own destination alone". The
            # sweep re-points every `advance` objective at this, so the same
            # one-line mission file runs on any scene.
            "goal": self._goal_name(),
            # EVERY goal, in slot order - a two-point shuttle needs both. The
            # single `goal` above stays because penetration is measured along
            # the line to the FIRST one, and the sweep reads it for that.
            "goals": self._goal_names(),
            # Laps, where the mission leaves them open. The experiment sweeps
            # the same mission the sandbox would have run, goals and all.
            "laps": self.mission_laps_value(),
            "duration_s": 400.0, "warmup_s": 5.0, "rate_hz": 10.0,
            "stop_when_stalled": True, "stall_grace_s": 5.0,
            "squads": squads,
            "coordinator": self._coordinator() or "gcs",
            "leader_loss": "fallback",
            # THE SAME ARITHMETIC THE LABEL SHOWED. Built by _sweep_axes so
            # the run counter on the Setup tab and the grid that actually runs
            # cannot disagree - which they could when each assembled its own
            # list of values.
            "axes": axes,
            # One seed: measured, not assumed. Nothing is stochastic unless a
            # GNSS-band jammer or a lidar fleet is in play, and then this
            # should grow. See experiments/penetration.yaml.
            "seeds": [1],
        }, None

    # WHAT EACH COMBINATION ACTUALLY DOES. Nine of them, because authority
    # and routing are independent axes and every pairing is a real
    # configuration - which is the framework's whole claim. Written as what
    # happens, not as whether it is a good idea: the operator is choosing an
    # architecture, and the useful thing to know is how command will flow and
    # what breaks it.
    ARCH_NOTES = {
        ("centralized", "star"):
            "Every vehicle talks only to {coord}, and {coord} decides for all "
            "of them. One hop, one decision-maker: the quickest to command, "
            "and each vehicle goes silent the moment its own hop to {coord} "
            "breaks. Nothing relays.",
        ("centralized", "mesh"):
            "{coord} decides for everyone, but the fleet relays for each "
            "other. A vehicle out of {coord}'s reach can still be commanded "
            "through a peer - so the NETWORK survives distance that the "
            "command structure alone would not. This is where a stalled "
            "vehicle becomes a relay for the ones still moving.",
        ("centralized", "tiered"):
            "{coord} decides for everyone, but only leaders talk to it. A "
            "member reaches {coord} through its leader, so losing one leader "
            "silences its whole squad even though {coord} is still there and "
            "still deciding.",
        ("decentralized", "star"):
            "Every vehicle decides for itself, and a star carries nothing "
            "between them. Effectively independent robots sharing a channel: "
            "jamming cannot strip an authority nobody is exercising, which is "
            "why this is the CONTROL condition and not a result.",
        ("decentralized", "mesh"):
            "Every vehicle decides for itself and can hear every other. The "
            "most survivable arrangement here - and today the least "
            "coordinated, because they do not yet negotiate. Read it as a "
            "control, not as a claim.",
        ("decentralized", "tiered"):
            "Every vehicle decides for itself; the network still partitions "
            "into squads. Routing structure with no command structure over "
            "it - useful for isolating what the topology alone is worth.",
        ("hierarchical", "star"):
            "Members answer to their leader and leaders to {coord} - but a "
            "star carries no peer links, so a member reaches its leader VIA "
            "{coord}, the node it was supposed to be less dependent on. Every "
            "path runs through {coord}, so this behaves exactly as "
            "centralized. A legitimate thing to configure and a real finding "
            "that it collapses.",
        ("hierarchical", "mesh"):
            "Members answer to their leader, leaders to {coord}, and every "
            "link in the fleet is available to carry either. Losing the link "
            "to {coord} leaves each squad still commanded by its own leader - "
            "degraded, not decapitated.",
        ("hierarchical", "tiered"):
            "Command and routing agree: each squad meshes internally, only "
            "the leader talks upward to {coord}. The structure the network "
            "carries is the structure that decides, which is the one case "
            "where the two axes are not independent.",
    }

    def _on_arch_chosen(self):
        """An architecture override, applied and reported.

        Recomposes the run immediately so the map redraws with the new
        topology - a routing change you cannot see is a routing change you
        cannot trust - and says, in a sentence, what this particular
        combination DOES.
        """
        auth = self.auth_combo.currentText()
        route = self.route_combo.currentText()
        if self.mode_combo.currentIndex() == 2:
            self._refresh_run_count()
        self._refresh_command_structure()

        sweep = self.SWEEP_ALL
        if auth == sweep or route == sweep:
            self.lbl_arch.setText(
                "Sweeping every combination of "
                + ("authority" if auth == sweep else auth)
                + " x "
                + ("routing" if route == sweep else route)
                + ". Each cell is a different answer to 'who decides' and "
                  "'how do the packets get there', and they are independent - "
                  "which is the thing the experiment exists to show.")
        else:
            note = self.ARCH_NOTES.get((auth, route), "")
            self.lbl_arch.setText(
                note.format(coord=self._coordinator() or "the coordinator"))
        if self._setup_scene:
            self._compose_setup()

    def _declared_authority(self):
        """{agent: {decider, tier, reachable}} from the Setup pickers.

        Assumed reachable, because nothing has been jammed yet - this is the
        structure as DECLARED, and the live frame replaces it with the
        structure as it actually stands the moment a run starts.
        """
        auth = (self.auth_combo.currentText()
                if hasattr(self, "auth_combo") else "")
        if auth == "decentralized":
            return {}
        coord = self._coordinator()
        out = {}
        for aid in sorted(self._blue_ids or []):
            if aid == coord:
                continue
            if auth == "hierarchical":
                boss = (self.tier_combos[aid].currentData()
                        if aid in (self.tier_combos or {}) else coord)
            else:
                boss = coord
            if not boss or boss == aid:
                continue
            out[aid] = {"decider": boss, "reachable": True,
                        "tier": "coordinator" if boss == coord else "leader"}
        return out

    def _coordinator(self):
        return (self.coord_combo.currentData()
                if hasattr(self, "coord_combo") else None)

    def _blue_mobile(self):
        """Blue vehicles a command tree can be built out of - the bench is
        what they answer TO, never a rung on the ladder."""
        view = getattr(self, "resolved", None) or self.doc or {}
        bodies = {a.get("id"): a for a in (view.get("agents") or [])}
        out = []
        for aid in sorted(self._blue_ids or []):
            b = bodies.get(aid) or {}
            if b.get("ghost") or b.get("jammer") \
                    or b.get("platform") == "ground_station":
                continue
            out.append(aid)
        return out

    def _refresh_command_structure(self):
        """Show the coordinator picker and the reports-to rows this
        architecture actually uses, and fill them from what is composed."""
        if not hasattr(self, "coord_combo"):
            return
        auth = self.auth_combo.currentText()
        route = self.route_combo.currentText()
        sweep = self.SWEEP_ALL
        # Sweeping an axis means every value of it will be run, so the
        # structure has to exist for the ones that need it.
        needs_coord = auth in ("centralized", "hierarchical", sweep)
        needs_tree = auth in ("hierarchical", sweep) or route in ("tiered",
                                                                 sweep)

        ids = sorted(self._blue_ids or [])
        keep = self.coord_combo.currentData()
        self.coord_combo.blockSignals(True)
        self.coord_combo.clear()
        for aid in ids:
            self.coord_combo.addItem(aid)
            self.coord_combo.setItemData(self.coord_combo.count() - 1, aid)
        i = self.coord_combo.findData(keep)
        if i < 0:
            # The ground station by default: mains power, a bigger amplifier
            # and a proper antenna are exactly why a bench makes a good hub.
            i = max(self.coord_combo.findData("gcs"), 0)
        self.coord_combo.setCurrentIndex(max(i, 0))
        self.coord_combo.blockSignals(False)
        self.lbl_coord.setVisible(needs_coord and bool(ids))
        self.coord_combo.setVisible(needs_coord and bool(ids))

        mobile = self._blue_mobile()
        coord = self._coordinator()
        for aid in list(self.tier_combos):
            if aid not in mobile:
                self.tier_combos.pop(aid)._row.setParent(None)
        for n, aid in enumerate(mobile):
            c = self.tier_combos.get(aid)
            if c is None:
                row = QWidget()
                rl = QHBoxLayout(row)
                rl.setContentsMargins(0, 0, 0, 0)
                rl.setSpacing(4)
                lab = QLabel(aid)
                lab.setFixedWidth(52)
                rl.addWidget(lab)
                c = QComboBox()
                c.setToolTip(
                    "Who this vehicle answers to. Point several at one "
                    "vehicle and that vehicle is their squad leader; point "
                    "them at the coordinator and they are commanded "
                    "directly.")
                c.activated.connect(lambda _i: self._on_arch_chosen())
                rl.addWidget(c, 1)
                self.tier_lay.addWidget(row)
                c._row = row
                self.tier_combos[aid] = c
            prev = c.currentData()
            c.blockSignals(True)
            c.clear()
            for opt in ([coord] if coord else []) + [m for m in mobile
                                                     if m != aid]:
                if opt is None:
                    continue
                c.addItem(f"{opt}  (coordinator)" if opt == coord else opt)
                c.setItemData(c.count() - 1, opt)
            j = c.findData(prev)
            if j < 0:
                # THE FIRST VEHICLE LEADS, and everyone else reports to it.
                # Stated rather than assumed silently - it is the arrangement
                # the corridor experiment has always used, and now it is
                # visible and changeable instead of buried.
                want = coord if n == 0 else mobile[0]
                j = max(c.findData(want), 0)
            c.setCurrentIndex(max(j, 0))
            c.blockSignals(False)
        self.lbl_tiers.setVisible(needs_tree and bool(mobile))
        self.tier_box.setVisible(needs_tree and bool(mobile))

    def _squads(self):
        """The command tree the rows describe, as {name: {leader, members}}.

        Derived from "who reports to whom" rather than typed as groups,
        because the tree is the thing that matters and a squad is just a
        leader with followers. Vehicles reporting straight to the coordinator
        form no squad - they are commanded directly, which is what
        centralized means.
        """
        coord = self._coordinator()
        follow = {}
        for aid, c in (self.tier_combos or {}).items():
            boss = c.currentData()
            if boss and boss != coord and boss != aid:
                follow.setdefault(boss, []).append(aid)
        names = ("alpha", "bravo", "charlie", "delta", "echo", "foxtrot")
        return {names[i] if i < len(names) else f"squad{i + 1}":
                {"leader": leader, "members": sorted(members)}
                for i, (leader, members) in enumerate(sorted(follow.items()))}

    def _arch_override(self):
        """{"blue": {...}} of whatever the operator has overridden, or {}."""
        over = {}
        auth_w = getattr(self, "auth_combo", None)
        route_w = getattr(self, "route_combo", None)
        auth = auth_w.currentText() if auth_w is not None else ""
        route = route_w.currentText() if route_w is not None else ""
        # ALWAYS written, never "whatever the file said". A fleet declares no
        # command policy at all now, so if these were not written the model
        # would fall back to an invisible default - and an invisible default
        # is the thing that made the routing axis silently do nothing for a
        # whole sweep. What the dropdown shows is what runs.
        # `all` is a SWEEP instruction, never a value: it must never reach
        # the model, which would not know what to do with it.
        if auth and auth != self.SWEEP_ALL:
            # `topology` is the legacy alias command_authority() falls back on.
            over["authority"] = over["topology"] = auth
        if route and route != self.SWEEP_ALL:
            over["routing"] = route
        # THE COORDINATOR, chosen rather than assumed. Everything downstream
        # reads it - command_authority walks toward it, the reassignment
        # transmissions come FROM it, and the whole star-vs-mesh comparison
        # is measured against where it is standing.
        coord = self._coordinator()
        if coord:
            over["coordinator"] = coord
        # THE COMMAND TREE, from the reports-to rows. A hierarchy needs squads
        # to find a leader in and tiered routing needs squads to partition on;
        # both now come from what the operator actually drew instead of "the
        # first vehicle leads", which was a guess that happened to match the
        # corridor experiment and would have quietly mismatched anything else.
        # ONLY WHERE THEY MEAN SOMETHING. A squads block written into a
        # centralized/star run is a structure nothing reads, and a stale
        # structure nothing reads is the kind of thing that gets believed six
        # months later when somebody opens the composed YAML.
        if auth in ("hierarchical", self.SWEEP_ALL) \
                or route in ("tiered", self.SWEEP_ALL):
            squads = self._squads()
            if squads:
                over["squads"] = squads
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
        # THE MISSION, COMPOSED. Written with the run rather than issued at
        # it, so what the fleet has been told is visible before Play.
        plan = getattr(self, "_plan_composed", None)
        if plan:
            doc["plan"] = {"who": plan["who"],
                           "waypoints": list(plan["waypoints"]),
                           "laps": int(plan["laps"])}
        if self._points_override:
            # POINTS MOVED BY HAND. A scene declares where FAR is; where you
            # decide to send a fleet is a decision about this run, so it is an
            # override on the composition and not an edit to scenes/. The
            # model merges `points` one level deep, so naming one leaves the
            # rest of the scene's points alone.
            doc["points"] = {k: dict(v)
                             for k, v in self._points_override.items()}
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
        # Placing is only meaningful once there is something on the map to
        # place; a recompose is where that becomes true.
        self._update_placing()
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
            "CommsEv files (*.yaml *.yml)")
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

        # SYSTEM and MISSION share one hierarchy - system > network > agents -
        # and differ only in what hangs off each agent. System shows what each
        # side IS MADE OF (sensors, radios, and a slot for algorithms); Mission
        # shows what it is DOING. Building both from one helper keeps them from
        # drifting apart.
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

        leaf = 'equipment' hangs each agent's sensors, radios and algorithms
        under it (the System answer: what the agent IS). leaf = 'objective'
        hangs the agent's current objective under it (the Mission answer: what
        it is DOING). Nothing about hardware appears in the objective tree, and
        no objective appears in the equipment tree."""
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
            # THE SYSTEM VIEW, and it has no root above the systems. It was
            # headed "Scenario: <name>", which put a run-level word at the top
            # of a hardware tree and pushed everything down a level for
            # nothing. What this tree is FOR is: here are the sides, here is
            # what each of them is made of. So the systems are the top level.
            root = tree
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
                    else:  # equipment - WHAT THIS AGENT IS MADE OF
                        sens = agent.get("sensors") or []
                        sfold = QTreeWidgetItem(
                            a, [f"sensors ({len(sens)})" if sens
                                else "sensors   (none fitted)"])
                        if not sens:
                            sfold.setDisabled(True)
                        for j, sen in enumerate(sens):
                            it = QTreeWidgetItem(
                                sfold, [f"{sen.get('id')}  ({sen.get('type')})"])
                            it.setData(0, Qt.UserRole,
                                       ("node", ["agents", i, "sensors", j]))
                        # RADIOS BY NAME, on the agent that carries them.
                        # They were listed once at the bottom of the tree,
                        # detached from whoever was fitted with them, which
                        # is how a fleet with no radios at all went unnoticed
                        # until the Comms tab said "no radio" four times.
                        rfold = QTreeWidgetItem(a, ["radios"])
                        rad = agent.get("radio")
                        names = list(agent.get("radios") or [])
                        if rad:
                            tx = (rad.get("tx_power") or {})
                            txv = tx.get("value") if isinstance(tx, dict) \
                                else tx
                            bd = (rad.get("band") or {})
                            bdv = bd.get("value") if isinstance(bd, dict) \
                                else bd
                            lab = "radio"
                            if bdv:
                                lab += f"  {_num(bdv):.0f} MHz"
                            if txv is not None:
                                lab += f"  {_num(txv):.0f} dBm"
                            ri = QTreeWidgetItem(rfold, [lab])
                            ri.setData(0, Qt.UserRole,
                                       ("node", ["agents", i, "radio"]))
                        for rn in names:
                            QTreeWidgetItem(rfold, [str(rn)])
                        if agent.get("jammer"):
                            j_ = agent["jammer"]
                            jb = (j_.get("band") or {})
                            jbv = jb.get("value") if isinstance(jb, dict) \
                                else jb
                            QTreeWidgetItem(
                                rfold, [f"emitter  {_num(jbv):.0f} MHz"])
                        if not (rad or names or agent.get("jammer")):
                            rfold.setText(0, "radios   (NONE - cannot be "
                                             "commanded)")
                            rfold.setDisabled(True)
                        # ALGORITHMS. Empty on purpose and named on purpose:
                        # what a vehicle RUNS is the third thing it is made
                        # of, alongside what it can sense and what it can
                        # say, and leaving the slot visible is how it gets
                        # filled rather than forgotten. Nothing is invented
                        # here - when a planner, a SLAM front end or a
                        # consensus rule exists in the model it hangs here.
                        alg = QTreeWidgetItem(a, ["algorithms   (none yet)"])
                        alg.setDisabled(True)

        loose = [i for i in range(len(agents)) if i not in placed]
        if loose:
            orphan = QTreeWidgetItem(root, ["Agents (no network)"])
            for i in loose:
                a = QTreeWidgetItem(orphan, [agents[i].get("id", "?")])
                a.setData(0, Qt.UserRole, ("agent", ["agents", i]))

        # A scene-level radios block, if one is declared. Per-agent radios now
        # hang off the agent that carries them, which is where they belong;
        # this is only the shared catalogue some older scenes define.
        if leaf == "equipment" and (view.get("radios") or {}):
            radios = QTreeWidgetItem(tree, ["Radio types declared by the scene"])
            for name in (view.get("radios") or {}):
                r = QTreeWidgetItem(radios, [name])
                r.setData(0, Qt.UserRole, ("node", ["radios", name]))

        tree.expandAll()
        if leaf == "equipment":
            # Opened to the systems and their networks; the agents' own
            # equipment is one click away rather than thirty rows of it.
            for i_ in range(tree.topLevelItemCount()):
                top_ = tree.topLevelItem(i_)
                for j_ in range(top_.childCount()):
                    for k_ in range(top_.child(j_).childCount()):
                        top_.child(j_).child(k_).setExpanded(False)

    def show_static_scene(self):
        """The scene as the file defines it, before any run.

        Drawn from the RESOLVED view (map + mission), so a split mission - whose
        agents live in its map - still shows its cars rather than an empty box.
        """
        view = getattr(self, "resolved", None) or self.doc
        if not view:
            return
        self.viewport.arena = view.get("arena")
        # The scene's named points, so the DESTINATION of a mission is on the
        # map beside the fleet rather than being a word in a dropdown.
        self.viewport.points = dict(view.get("points") or {})
        self.viewport.agents = [
            {"id": a.get("id"), "colour": a.get("colour"),
             "dimensions": a.get("dimensions", {}), "pose": a.get("pose", {}),
             "sensors": a.get("sensors", []),
             "platform": a.get("platform"), "jammer": a.get("jammer")}
            for a in (view.get("agents") or [])
        ]
        self._push_scene_rf()
        # THE COMMAND LAYER BEFORE PLAY. Setup has just declared who
        # coordinates and who reports to whom, so the arrows can be drawn from
        # the declaration - you should be able to SEE the structure you chose
        # without starting a run to find out what you picked.
        self.viewport.authority = self._declared_authority()
        self.viewport.links = []
        self.viewport.update()
        self.fill_publications_from_scenario()
        self.fill_comms()

    def on_contested_edit(self, item, _col):
        """Double-click any emitter to tune its transmit power live.

        BOTH SIDES HAVE A VOLUME KNOB. Red's power has been editable here
        since the Contested tab existed; blue's was whatever the fleet file
        said, which quietly made "turn it up" an adversary-only move. It is
        not - EMCON is a friendly decision, and the whole power-control
        question (shout over the jammer, or go quiet and stay unheard) cannot
        be asked while only one side can change.

        Both write a command into the same retask spool the terminal uses, so
        the run is edited from the Console and not from a file.
        """
        data = item.data(0, Qt.UserRole)
        if not (isinstance(data, tuple) and data[0] in ("jammer", "radio")):
            return
        if not getattr(self, "_retask_dir", None):
            self.say("Start the run first, then double-click to tune a "
                     "transmitter.")
            return
        kind, aid = data[0], data[1]
        live = (self.latest or {}).get(aid) or {}
        if kind == "jammer":
            cur = _num((live.get("jammer") or {}).get("tx_dbm"), 20.0)
            title, what = "Tune jammer", "jamming power"
        else:
            cur = _num((live.get("radio") or {}).get("tx_dbm"), 20.0)
            title, what = "Tune radio", "transmit power"
        val, ok = QInputDialog.getDouble(
            self, title, f"{aid} {what} (dBm):", cur, -30.0, 60.0, 1)
        if not ok:
            return
        line = (f"JAM {aid} power {val}" if kind == "jammer"
                else f"TXPOWER {aid} {val}")
        self._send_queue_line_console(line + "\n", line)

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

    def limiting_links(self, scope):
        """Which direction is holding back each of this agent's links.

        A LINK HAS TWO DIRECTIONS AND IS ONLY AS GOOD AS THE WEAKER ONE. That
        is in the model and it is right - a ground station may reach a car
        easily while the car cannot be heard replying - but it is invisible,
        and invisible is how "I turned the power up and nothing happened"
        happens. Reads the last frame, so it reports what IS rather than what
        the command is about to change.
        """
        out = []
        for l in (self.viewport.links or []):
            if scope not in (None, "all") and scope not in (l["a"], l["b"]) \
                    and str(scope).lower() not in (str(l.get("network") or "").lower(),):
                continue
            lim = l.get("limited_by")
            if not lim:
                continue
            ab, ba = _num(l.get("sinr_ab_db")), _num(l.get("sinr_ba_db"))
            out.append(f"{l['a']}<->{l['b']}  {l['a']}->{l['b']} {ab:+.1f} dB, "
                       f"{l['b']}->{l['a']} {ba:+.1f} dB  "
                       f"-> limited by {lim} ({l.get('state')})")
        if out:
            out.append("a link is only as good as its WEAKER direction - "
                       "raising one end alone does not raise the link")
        return out[:8]

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
            rows.append((aid, f"/{aid}/state", "commsev/AgentState", "-"))
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
        # A DEFERRED POWER REPORT LANDS HERE, on the first frame that can
        # actually have seen the command. Reported as a one-command lag: the
        # report was written the instant TXPOWER was typed, but it reads the
        # LAST FRAME, which predates the command reaching the sim - so every
        # line described the previous power. Verified against Will's own log:
        # "TXPOWER blue 1" printed +44 dB, which is the answer for blue 3.
        pend = getattr(self, "_power_report_for", None)
        if pend is not None:
            self._power_report_for = None
            for line in self.limiting_links(pend[0]):
                if pend[1] is not None:
                    pend[1].appendPlainText("  " + line)
        # Points ride in the frame's arena during a live run, so the goal
        # stays drawn while the fleet advances on it.
        self.viewport.points = ((f.get("arena") or {}).get("points")
                                or self.viewport.points)
        self.viewport.authority = {
            a.get("id"): (a.get("authority") or {}) for a in f.get("agents", [])}
        self._print_intercepts(f)
        self._show_mission_score(f)
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
        """WHITE STAYS IN THE WINDOW. Blue and red open as their own.

        The umpire's terminal is where you work - shell commands, git, the
        test suite - so it belongs in the main window where it has always
        been. Blue and red are different: they are two OPERATORS, and having
        them as tabs behind the umpire's meant only one side was ever visible,
        which is a strange way to run an exercise whose whole point is that
        the two sides see different things. As separate windows they can sit
        side by side on the desk, or on two screens, or in front of two
        people. See docs/cells-and-network.md.
        """
        shell = ShellPanel(REPO_WSL_PATH, cell="white", console=self)
        self.shells.append(shell)
        idx = self.terminals.addTab(shell, "White cell")
        self.terminals.tabBar().setTabTextColor(
            idx, QColor(self._CELL_COLOUR["white"]))
        self._cell_windows = {}
        self._cells_built = True

    def open_cell(self, cell):
        """Open (or raise) the blue or red operator's terminal as a window."""
        win = (getattr(self, "_cell_windows", None) or {}).get(cell)
        if win is None:
            shell = ShellPanel(REPO_WSL_PATH, cell=cell, console=self)
            self.shells.append(shell)
            win = QDialog(self)
            win.setWindowTitle(
                {"blue": "BLUE cell - friendly command",
                 "red": "RED cell - adversary"}.get(cell, cell))
            # NOT MODAL. The whole point is to type into it while the run goes
            # on in the window behind, and to have both cells open at once.
            win.setModal(False)
            lay = QVBoxLayout(win)
            lay.setContentsMargins(6, 6, 6, 6)
            hdr = QLabel(
                {"blue": "Blue commands the BLUE network.   SETPLAN, "
                         "SETMISSION, REOBJECTIVE, launch / halt.",
                 "red": "Red commands the RED network.   JAM, LISTEN, "
                        "red launch / halt."}.get(cell, ""))
            hdr.setWordWrap(True)
            hdr.setStyleSheet(f"color:{self._CELL_COLOUR.get(cell, '#B8C0C6')};")
            lay.addWidget(hdr)
            lay.addWidget(shell, 1)
            win.resize(820, 460)
            self._cell_windows[cell] = win
            win.finished.connect(lambda *_c, k=cell: self._cell_closed(k))
        win.show()
        win.raise_()
        win.activateWindow()
        for sh in self.shells:
            if sh.cell == cell:
                sh.inp.setFocus()
                break
        return win

    def _cell_closed(self, cell):
        """Closing the window hides it; the shell and its history survive, so
        reopening a cell mid-exercise does not lose what it has been told."""
        return

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
        self._focus_setup()
        self._update_placing()
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

    def show_app(self, index):
        """Swap the whole window between Simulation, Scene and Agent."""
        if not hasattr(self, "app_stack"):
            return
        self.app_stack.setCurrentIndex(index)
        name = ("Simulation", "Scene builder", "Agent builder")[index]
        self.statusBar().showMessage(
            name if index == 0 else f"{name} - not built yet")
        # The docks belong to Simulation. Leaving a Properties panel and a
        # sensor view floating beside a page that says NOT BUILT YET would be
        # worse than blank - it would look broken rather than absent.
        for d in self.findChildren(QDockWidget):
            d.setVisible(index == 0)

    def toggle_spectrum(self, on):
        """Compute the field and hand it to the viewport, or clear it.

        ON DEMAND, and the button is also the refresh. A 72 x 72 grid over
        five emitters and eight walls is about a third of a second - fine to
        wait for, impossible to do at 10 Hz - so it is computed when asked
        and then left alone. Press it again after anything has moved.
        """
        # A STALE PICTURE MAKES THE FIRST CLICK A REFRESH, NOT A HIDE.
        #
        # The key prints "STALE - press the button" the moment anything moves
        # or a power command lands. But the button is a TOGGLE and it is
        # already checked, so pressing it turned the field OFF - and the
        # honest conclusion from the outside was "the heatmap is not affected
        # by TXPOWER", when the model was right all along and the advice on
        # screen could not be followed. Verified in the model: 20 -> 3 -> 40
        # dBm moves the field maximum 6.1 -> -20.2 -> 16.8 dBm.
        #
        # So: while what is drawn is out of date, a click REFRESHES it. The
        # second click, on a picture that is current, hides it as before.
        if (not on and self.viewport.show_spectrum
                and self.viewport.spectrum_is_stale()):
            self.viewport.show_spectrum = True
            sender = self.sender()
            if sender is not None and hasattr(sender, "setChecked"):
                sender.setChecked(True)
            self.refresh_spectrum()
            return
        self.viewport.show_spectrum = bool(on)
        if not on:
            self.viewport.set_spectrum(None)
            return
        self.refresh_spectrum()

    def refresh_spectrum(self):
        """Recut the section and redraw it. Called by the button, the slider
        and the view buttons - three ways of asking the same question."""
        if not getattr(self.viewport, "show_spectrum", False):
            return
        plane, axis = self.SLICE_FOR.get(self.viewport.mode, (None, None))
        if plane is None:                      # ISO: no honest section exists
            self.viewport.set_spectrum(None)
            self.say("The spectrum section needs a flat view - "
                     "choose Top, Front or Side.")
            return
        # THE SECTION IS THE PLANE THIS VIEW CUTS, at the position ITS OWN
        # slider is on - so switching view swaps which of the three is being
        # read and never loses where the other two were left.
        at = self.slice_at(axis)
        world = self._live_world()
        if world is None:
            self.viewport.show_spectrum = False
            return
        st, arena, agents, poses, band = world
        try:
            field = st.spectrum_field(arena, agents, poses, band,
                                      plane=plane, at=at)
        except Exception as exc:                       # noqa: BLE001
            self.viewport.show_spectrum = False
            self.say(f"spectrum: {exc}")
            return
        if not field.get("emitters"):
            # AN EMPTY BAND IS A RESULT, and drawing a stretched picture of
            # nothing would be a lie about it. Tuning to GNSS with no GNSS
            # jammer in the scene is exactly this case.
            self.viewport.set_spectrum(None)
            self.say(f"Nothing is transmitting on {band:.0f} MHz in this "
                     f"scene, so there is no field to draw.")
            return
        self.viewport.set_spectrum(field)
        self.say(f"Spectrum at {band:.0f} MHz from {field['emitters']} "
                 f"emitters, {field['faxis']} = {field['at']:.2f} m. "
                 f"Press the button again to refresh it after anything moves.")

    def _live_world(self):
        """The composed scene as the MODEL sees it, at the poses on screen.

        Shared by every on-demand view, so they can never disagree about
        where anything is or which band is being looked at. Returns
        (module, arena, agents, poses, band_mhz), or None with a message
        already said.
        """
        view = getattr(self, "resolved", None) or self.doc
        if not view:
            self.say("Nothing composed yet - choose a scene and a fleet first.")
            return None
        # WHICH FREQUENCY. The Comms band strip is already the frequency
        # selector for the whole window, so it is the one here too rather
        # than a second control that can disagree with it.
        bf = getattr(self.viewport, "band_filter", None)
        band = None
        if isinstance(bf, (int, float)):
            band = float(bf)
        elif bf == "gnss":
            band = 1575.42
        if band is None:
            # NO BAND CHOSEN MEANS NO FIELD. Both of these views are about ONE
            # frequency - the spectrum sums the power on it, the wavefront
            # draws reach on it - and "ALL" is not a frequency. Summing power
            # across bands that do not interfere would draw a picture of a
            # radio environment nobody is in, so the honest answer to ALL is
            # to draw nothing and say which band to pick.
            self.say("Pick a band on the Comms strip first - the spectrum and "
                     "the wavefront are drawn for ONE frequency, and ALL is "
                     "not one.")
            return None

        try:
            import stub_telemetry as st
            arena, agents, _links = st.load_scenario(copy.deepcopy(view))
            poses = {a["id"]: dict(a["start"], speed=0.0) for a in agents}
            live = self.latest or {}
            for a in agents:
                p_ = (live.get(a["id"]) or {}).get("pose")
                if p_:
                    poses[a["id"]].update({k: _num(p_.get(k))
                                           for k in ("x", "y", "z")})
                # THE POWERS HAVE TO COME FROM THE RUN, NOT FROM THE FILE.
                # Reported: "I don't think the heatmap is affected by the
                # power of the agent". It was not, and this is why - the
                # world was rebuilt from the composed DOCUMENT every time and
                # only pose and armed were patched from the frame, so every
                # live JAM and TXPOWER was drawn at whatever the YAML said.
                # A spectrum view that ignores the spectrum commands is worse
                # than no spectrum view.
                jam = (live.get(a["id"]) or {}).get("jammer")
                if jam is not None:
                    a["armed"] = bool(jam.get("on", a.get("armed")))
                    if jam.get("tx_dbm") is not None:
                        a.setdefault("jammer", {})["tx_power"] = {
                            "value": _num(jam.get("tx_dbm")), "unit": "dBm",
                            "source": "live"}
                    if jam.get("band_mhz") is not None:
                        a.setdefault("jammer", {})["band"] = {
                            "value": _num(jam.get("band_mhz")), "unit": "MHz",
                            "source": "live"}
                rad = (live.get(a["id"]) or {}).get("radio")
                if rad and rad.get("tx_dbm") is not None:
                    a.setdefault("radio", {})["tx_power"] = {
                        "value": _num(rad.get("tx_dbm")), "unit": "dBm",
                        "source": "live"}
        except Exception as exc:                       # noqa: BLE001
            self.say(f"cannot read the world: {exc}")
            return None
        return st, arena, agents, poses, band

    def toggle_wavefront(self, on):
        """Rings by side, and the ground the other side owns.

        The companion to the spectrum toggle and deliberately a SEPARATE
        view. The field answers "how much power is here" and has to be one
        hue to stay readable as a magnitude; this answers "whose", which
        needs the side colours. Trying to do both at once would mean a
        rainbow, and a rainbow field is unreadable as either.
        """
        self.viewport.show_wavefront = bool(on)
        if not on:
            self.viewport.set_wavefront(None, None)
            return
        if self.viewport.mode != self.viewport.TOP:
            self.viewport.show_wavefront = False
            self.say("The wavefront view is drawn in plan - choose Top.")
            return
        world = self._live_world()
        if world is None:
            self.viewport.show_wavefront = False
            return
        st, arena, agents, poses, band = world
        # The rings are traced on the Z plane - the same one the plan section
        # uses - so the two views always agree about what height they mean.
        z = self.slice_at("z")
        try:
            rings = st.wavefront_rings(arena, agents, poses, band, z=z)
            sides = {str(a.get("network")).lower() for a in agents}
            adv = (st.advantage_field(arena, agents, poses, band, at=z)
                   if {"blue", "red"} <= sides else None)
        except Exception as exc:                       # noqa: BLE001
            self.viewport.show_wavefront = False
            self.say(f"wavefront: {exc}")
            return
        n = len(rings.get("emitters") or [])
        if not n:
            self.viewport.set_wavefront(None, None)
            self.say(f"Nothing is transmitting on {band:.0f} MHz in this "
                     f"scene, so there are no wavefronts to draw.")
            return
        self.viewport.set_wavefront(rings, adv)
        if adv is None:
            self.say(f"Wavefront at {band:.0f} MHz: {n} emitters, "
                     f"z = {z:.2f} m. Only one side is present, so there is "
                     f"no contested ground to shade.")
            return
        vals = [v for row in adv["dbm"] for v in row if v is not None]
        lost = sum(1 for v in vals if v < 0) / max(len(vals), 1)
        self.say(f"Wavefront at {band:.0f} MHz: {n} emitters, z = {z:.2f} m. "
                 f"The hostile signal is the louder one over "
                 f"{lost*100:.0f}% of the floor.")

    def _walls_picked(self, indices):
        """A wall was clicked, or a box swept over several: ask what they are.

        Opened straight away rather than behind a menu, because there is
        exactly one thing anybody wants to do with a selected wall and making
        them find it twice is not a feature. Cancel leaves the selection up.
        """
        if not indices:
            return
        walls = (self.viewport.arena or {}).get("walls") or []
        if not walls:
            return
        dlg = WallDialog(walls, indices, self)
        if dlg.exec() != QDialog.Accepted:
            return
        n = dlg.apply_to()
        mat = dlg.mat.currentText()
        # THE LIVE WORLD AND THE COMPOSED DOCUMENT ARE TWO OBJECTS, and both
        # have to change or the edit survives until the next recompose and
        # then silently reverts.
        for holder in (self.doc, getattr(self, "resolved", None)):
            hw = ((holder or {}).get("arena") or {}).get("walls")
            if hw is not None and len(hw) == len(walls):
                for i in dlg.indices:
                    hw[i].update({k: walls[i][k] for k in
                                  ("material", "height", "thickness",
                                   "rf_db", "reflectance")})
        self.viewport.selected_walls = set()
        self.viewport.update()
        self.say(f"{n} wall{'s' if n != 1 else ''} set to {mat}. "
                 f"This run only - use File > Save scene as... to keep it.")
        self.dirty = True

    def _select_series(self, path):
        """Light one series on every plot; click it again to unlight it.

        With colour carrying the AUTHORITY, the individual curve you are
        chasing needs some other way to stand out - so it is picked, not named
        in a legend. The chart then says two things at once: which family a
        curve belongs to, and which single curve you are reading.
        """
        if not isinstance(path, str):
            return
        self.plots.selected = None if self.plots.selected == path else path
        self.plots.refresh()

    def findings_text(self):
        """WHAT THIS RUN ACTUALLY MEASURED, as text you can paste.

        A chart is how you see a result; it is not how you report one. Reading
        numbers back off a plot by eye is where write-ups acquire their
        mistakes, and a screenshot of nine overlaid curves cannot be quoted at
        all. This states the configuration, the outcome, and every plotted
        series with its endpoints, its range and - for a sweep - where it
        crosses zero, which for penetration is the point at which the fleet
        stops making ground.

        Deliberately just the measurements. It draws no conclusions, because
        the conclusions are the researcher's job and a tool that writes them
        is a tool that can be wrong in prose.
        """
        frames = self.frames or []
        if not frames:
            return "No run recorded. Press Play, then Stop."
        sweep = getattr(self.plots, "xlabel", "s") != "s"
        xlabel = getattr(self.plots, "xlabel", "s")
        first, last = frames[0], frames[-1]
        nets = ((self.viewport.arena or {}).get("networks")
                or (self.doc or {}).get("networks") or {})
        blue = nets.get("blue") or {}
        movers = [a for a in (last.get("agents") or [])
                  if a.get("platform") not in ("ground_station", "result")
                  and not a.get("jammer")]
        out = []
        out.append("CommsEv " + ("SWEEP" if sweep else "RUN")
                   + f" - {time.strftime('%Y-%m-%d %H:%M')}")
        out.append("")
        out.append("CONFIGURATION")
        out.append(f"  scene           {self._setup_scene or '?'}")
        out.append(f"  blue fleet      {self._setup_fleet or '?'}"
                   + (f"   red fleet {self._setup_red}"
                      if getattr(self, "_setup_red", None) else ""))
        out.append(f"  authority       "
                   f"{blue.get('authority') or blue.get('topology') or '(not declared)'}")
        out.append(f"  routing         {blue.get('routing') or '(not declared)'}")
        out.append(f"  coordinator     {blue.get('coordinator') or '(none)'}")
        if blue.get("squads"):
            for sq, spec in (blue["squads"] or {}).items():
                out.append(f"  squad {sq:<9} leader {(spec or {}).get('leader')}"
                           f"  members {', '.join((spec or {}).get('members') or [])}")
        out.append(f"  vehicles        {len(movers)}")
        if sweep:
            out.append(f"  swept axis      {xlabel}   "
                       f"{_num(first.get('sim_time_s')):g} .. "
                       f"{_num(last.get('sim_time_s')):g}"
                       f"   ({len(frames)} values)")
        else:
            out.append(f"  duration        {_num(last.get('sim_time_s')):.1f} s"
                       f"   ({len(frames)} frames)")
            out.append(f"  mission         {self._mission_name or '(none set)'}"
                       + (f"   {self._mission_line}"
                          if getattr(self, "_mission_line", "") else ""))
        out.append("")

        plotted = []
        for pane in self.plots.panes():
            for path in pane.series:
                if path not in plotted:
                    plotted.append(path)
        if not plotted:
            out.append("NOTHING PLOTTED - drag a series onto a plot and press "
                       "this again to have its numbers written out.")
            return "\n".join(out)

        xs = [_num(f.get("sim_time_s")) for f in frames]
        out.append("MEASUREMENTS" + (f"   (x = {xlabel})" if sweep else ""))
        for path in plotted:
            vals = [series_value(frames, i, path) for i in range(len(frames))]
            pairs = [(x, v) for x, v in zip(xs, vals)
                     if isinstance(v, (int, float))]
            name = PlotPane.series_label(path)
            if not pairs:
                out.append(f"  {name:<34} no data")
                continue
            ys = [v for _x, v in pairs]
            mean = sum(ys) / len(ys)
            out.append(f"  {name}")
            out.append(f"      start {pairs[0][1]:.4g}   end {pairs[-1][1]:.4g}"
                       f"   min {min(ys):.4g}   max {max(ys):.4g}"
                       f"   mean {mean:.4g}")
            # WHERE IT CROSSES ZERO. On a swept penetration this is the
            # jammer advantage at which the fleet stops making ground, which
            # is the number a write-up actually quotes.
            for (x0, v0), (x1, v1) in (zip(pairs, pairs[1:]) if sweep else ()):
                if (v0 > 0) != (v1 > 0) and v0 != v1:
                    xc = x0 + (x1 - x0) * (v0 / (v0 - v1))
                    out.append(f"      crosses zero at {xlabel} = {xc:.3g}")
                    break
        out.append("")
        out.append("Numbers only - the interpretation is yours.")
        return "\n".join(out)

    def copy_findings(self):
        text = self.findings_text()
        QApplication.clipboard().setText(text)
        self.say("")
        self.say(text)
        self.say("")
        self.say("(the above is on the clipboard)")

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
        self._c_radios = QTreeWidgetItem(tree, ["Radios (none - run to see)"])
        self._c_spectrum = QTreeWidgetItem(tree, ["Experienced spectrum"])
        self._c_emitters.setExpanded(True)
        self._c_radios.setExpanded(True)
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
        # THE FRIENDLY TRANSMITTERS, in the same list and editable the same
        # way. Listing only jammers under "Emitters" was a small untruth with
        # a real consequence: every radio in the scene is an emitter, they are
        # what a listener hears and what the fleet interferes with itself
        # over, and leaving them out made blue look like it was not on the
        # air at all.
        rd = getattr(self, "_c_radios", None)
        if rd is not None:
            rd.takeChildren()
            radios = [a for a in frame.get("agents", [])
                      if (a.get("radio") or {}).get("tx_dbm") is not None]
            rd.setText(0, f"Radios ({len(radios)})" if radios
                       else "Radios (none - run to see)")
            for a in radios:
                r = a["radio"]
                row = QTreeWidgetItem(
                    rd, [f"{a['id']} [{a.get('network','?')}]  "
                         f"{_num(r.get('tx_dbm')):.0f} dBm"
                         + (f" @ {_num(r.get('band_mhz')):.0f} MHz"
                            if r.get("band_mhz") else "")
                         + "   (double-click to edit)"])
                row.setData(0, Qt.UserRole, ("radio", a["id"]))
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
        """EVERY TRANSMITTER IN THE RUN, blue and red.

        This listed only agents carrying a legacy `radios: [name]` reference
        into the scene's radio block - which, once hardware moved onto the
        agent as its own `radio:`, meant it listed almost nothing. The ground
        station was missing from the emitter table while transmitting at
        30 dBm and being the reason a star reaches as far as it does, and the
        jammers were missing while being the entire adversary.

        A thing that transmits is an emitter. The band comes from the agent's
        own radio if it declares one and otherwise from the network it is on,
        because that is where a fleet's channel is actually decided.
        """
        view = self._view() if self.doc else {}
        radios = view.get("radios") or {}
        nets = view.get("networks") or {}
        rows = []
        for a in (view.get("agents") or []):
            aid = a.get("id")
            net = nets.get(a.get("network")) or {}
            net_band = _num((net.get("band") or {}).get("value")
                            if isinstance(net.get("band"), dict)
                            else net.get("band"))
            # A JAMMER emits on its own terms - its band and its power are the
            # attack, not a property of any network.
            j = a.get("jammer")
            if j:
                jb = j.get("band")
                jp = j.get("tx_power")
                rows.append((
                    aid,
                    f"{_num(jb.get('value') if isinstance(jb, dict) else jb):.0f} MHz",
                    (f"{_num(jp.get('value') if isinstance(jp, dict) else jp):.0f} dBm"),
                    "jammer"))
                continue
            # The agent's OWN radio - hardware, which is where a transmit
            # power belongs and where the ground station's 30 dBm lives.
            r = a.get("radio") or {}
            if r:
                tx = r.get("tx_power")
                txv = tx.get("value") if isinstance(tx, dict) else tx
                rb = r.get("band")
                band = _num(rb.get("value") if isinstance(rb, dict) else rb) \
                    or net_band
                role = ("ground station" if a.get("platform") == "ground_station"
                        else f"{a.get('network', '-')} net")
                rows.append((aid, f"{band:.0f} MHz" if band else "-",
                             f"{_num(txv):.0f} dBm" if txv is not None
                             else "unsourced", role))
            # Legacy: a named radio from the scene's radios block.
            for rname in a.get("radios") or []:
                rr = radios.get(rname) or {}
                band = _num((rr.get("band") or {}).get("value")) or net_band
                tx = (rr.get("tx_power") or {}).get("value")
                rows.append((aid, f"{band:.0f} MHz" if band else "-",
                             f"{tx} dBm" if tx is not None else "unsourced",
                             rr.get("link_type", "-")))
            if not r and not (a.get("radios") or []):
                # SAID, NOT OMITTED. An agent with no radio at all cannot be
                # commanded, and a blank row is how that goes unnoticed.
                rows.append((aid, "-", "no radio", "cannot be commanded"))
        self.emitters.setRowCount(len(rows))
        for i, row in enumerate(rows):
            for c, text in enumerate(row):
                it = QTableWidgetItem(str(text))
                it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                if text in ("unsourced", "no radio", "cannot be commanded"):
                    it.setForeground(QBrush(QColor(C_WARN)))
                if text == "jammer":
                    it.setForeground(QBrush(QColor("#C4685A")))
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
        self.viewport.scan_extras = (frame.get("scans") or []) if frame else []
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
        self._played_once = True            # the rest of the window is live now
        self._lock_setup(True)
        # Anything left from a previous run holds the port and wins the race.
        self.say("Clearing any previous ROS processes...")
        # [c]ommsev is not a typo. pkill -f matches against the FULL command
        # line, and this shell's own command line contains the pattern, so a
        # plain `pkill -f commsev_ros` signals the shell that is running it.
        # The bracket makes the regex match "commsev_ros" while the literal
        # text "[c]ommsev_ros" sitting in our own argv does not match it.
        cleanup = self.wsl("pkill -f '[c]ommsev_ros'; "
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
            "ros2 launch commsev_ros fleet.launch.py"
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

        This is also the moment the rest of the window becomes meaningful -
        there is now a world for the other tabs to describe. See _focus_setup.

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
        self._played_once = True            # the rest of the window is live now
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
            self, "CommsEv Console",
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
                self, "CommsEv Console", "Open it in PlotJuggler now?",
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
            self.viewport.points = ((frame.get("arena") or {}).get("points")
                                    or self.viewport.points)
            self.viewport.authority = {
                a.get("id"): (a.get("authority") or {})
                for a in frame.get("agents", [])}
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
        # THE SECTION FOLLOWS THE VIEW. Top cuts a horizontal slice, front and
        # side cut vertical ones; the field belongs to exactly one of them, so
        # changing the view has to recut it or the picture silently vanishes
        # and looks like a bug.
        if getattr(self.viewport, "show_spectrum", False):
            self.refresh_spectrum()
        self.viewport.update()

    # Which section plane each view is allowed to show, and what the slider
    # means there: (plane, axis it moves along).
    SLICE_FOR = {"TOP": ("top", "z"), "FRONT": ("front", "y"),
                 "SIDE": ("side", "x")}

    def _sync_slice_range(self):
        """Give each slider the extent of the room along its own axis.

        Called when a scene is composed, not when the view changes: the
        planes belong to the scene, so a new room re-ranges them and turning
        the camera does not.
        """
        if not getattr(self, "slice_sliders", None):
            return
        ext = ((self.viewport.arena or {}).get("extent") or {})
        for axis, sl in self.slice_sliders.items():
            if axis == "z":
                lo, hi, default = 0.0, _num(ext.get("z"), 3.0), 0.2
            else:
                half = _num(ext.get(axis), 8.0) / 2.0
                lo, hi, default = -half, half, 0.0
            sl.blockSignals(True)
            sl.setRange(int(round(lo * 100)), int(round(hi * 100)))
            # Keep where it was if that is still inside the room - loading a
            # scene of the same size should not throw away a chosen section.
            if not (lo <= sl.value() / 100.0 <= hi):
                sl.setValue(int(round(default * 100)))
            sl.blockSignals(False)
        self._slice_label()

    def slice_at(self, axis):
        sl = (getattr(self, "slice_sliders", None) or {}).get(axis)
        return (sl.value() / 100.0) if sl is not None else (0.2 if axis == "z"
                                                            else 0.0)

    def _slice_label(self):
        for axis, rd in (getattr(self, "slice_reads", None) or {}).items():
            rd.setText(f"{axis} {self.slice_at(axis):5.2f}")
        # The viewport needs the numbers whether or not anything recomputes -
        # the cut-away drawing follows the handle live, because hiding a wall
        # is free and waiting for it would feel broken.
        self.viewport.slice = {a: self.slice_at(a) for a in ("x", "y", "z")}
        self.viewport.update()

    def _slice_moved(self):
        """Live while dragging, recompute on release.

        The cut-away follows the handle immediately. The FIELD does not: a
        third of a second per section is fine to ask for and impossible to do
        sixty times a second, so the picture catches up when you let go.
        """
        self._slice_label()
        if not any(sl.isSliderDown()
                   for sl in self.slice_sliders.values()):
            self._slice_released()

    def _slice_released(self):
        if getattr(self.viewport, "show_spectrum", False):
            self.refresh_spectrum()
        if getattr(self.viewport, "show_wavefront", False):
            self.toggle_wavefront(True)

    def toggle_cut(self, on):
        """Open the scene on the three section planes, or close it again."""
        self.viewport.cut_away = bool(on)
        self._slice_label()
        self.say("Cut away ON - walls below the Z plane are hidden and "
                 "anything past the X or Y plane is ghosted. The three "
                 "sliders move the planes."
                 if on else "Cut away off - the whole scene is drawn.")

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

    def cell_say(self, side, text):
        """Write a line into a command cell's own window.

        The cells are where the run is actually driven from, so anything the
        operator has to react to belongs in them and not only in the log."""
        if not text or not hasattr(self, "terminals"):
            return
        for i in range(self.terminals.count()):
            w = self.terminals.widget(i)
            if getattr(w, "cell", None) == side and hasattr(w, "out"):
                w.out.appendPlainText(text)

    def closeEvent(self, ev):
        self.stop_run()
        if self.dirty:
            answer = QMessageBox.question(
                self, "CommsEv Console",
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
    header = (f"CommsEv Console crash\npython  {sys.version}\n"
              f"exe     {sys.executable}\nrepo    {REPO_ROOT}\n{'-' * 60}\n")
    try:
        ERROR_LOG.write_text(header + text, encoding="utf-8")
    except OSError:
        pass
    sys.stderr.write(header + text)
    try:
        if QApplication.instance():
            QMessageBox.critical(None, "CommsEv Console",
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
        # AFTER show(), so the window is up before anything is put back into
        # it - a restore that runs first leaves you looking at a blank frame
        # for however long the compose takes, which reads as a failed reload.
        sys.exit(app.exec())
    except SystemExit:
        raise
    except BaseException:
        _record(*sys.exc_info())
        sys.exit(1)


if __name__ == "__main__":
    main()
