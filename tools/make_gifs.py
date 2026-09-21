#!/usr/bin/env python3
"""
CommsEv — record the feature-tour GIFs from the real Console, headless.

    python3 tools/make_gifs.py                 # all of them -> docs/gifs/
    python3 tools/make_gifs.py jamming drift   # just these

Drives console/app.py under Qt's offscreen platform exactly as the tests do:
composes a run through the same methods the Setup tab calls, presses Play,
types the same commands into the same terminals, and grabs the window every
few hundred milliseconds. So the GIFs are evidence rather than illustration,
and they can be regenerated after any UI change with one command.

No new dependencies beyond the Console's own: Pillow is used if present for
smaller files, otherwise frames are written with Qt.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "docs" / "gifs"
sys.path.insert(0, str(REPO / "console"))
sys.path.insert(0, str(REPO))

from PySide6.QtCore import QCoreApplication          # noqa: E402
from PySide6.QtWidgets import QApplication, QDialog   # noqa: E402

import app as gui                                     # noqa: E402

WIDTH, HEIGHT = 1600, 900       # recorded size; scaled down on save
SCALE = 0.8                     # README-friendly: 1280 px wide
FRAME_MS = 250                  # capture cadence


class Recorder:
    def __init__(self, console, name):
        self.console, self.name, self.frames = console, name, []
        self._last = 0.0

    def pump(self, seconds, capture=True, hold_first=0):
        """Run the event loop for `seconds`, grabbing frames as it goes."""
        end = time.time() + seconds
        while time.time() < end:
            QCoreApplication.processEvents()
            now = time.time()
            if capture and now - self._last >= FRAME_MS / 1000:
                self.snap(); self._last = now
            time.sleep(0.02)
        for _ in range(hold_first):
            self.snap()

    def snap(self, widget=None, repeat=1):
        pix = (widget or self.console).grab()
        img = pix.toImage().scaled(int(pix.width() * SCALE), int(pix.height() * SCALE))
        for _ in range(repeat):
            self.frames.append(img)

    def save(self):
        OUT.mkdir(parents=True, exist_ok=True)
        path = OUT / f"{self.name}.gif"
        try:
            from PIL import Image
            pil = []
            for im in self.frames:
                im = im.convertToFormat(im.Format.Format_RGB888)
                ptr = im.constBits()
                pil.append(Image.frombytes("RGB", (im.width(), im.height()),
                                           bytes(ptr), "raw", "RGB",
                                           im.bytesPerLine()).quantize(192, method=Image.Quantize.MEDIANCUT))
            pil[0].save(path, save_all=True, append_images=pil[1:],
                        duration=FRAME_MS, loop=0, optimize=True)
        except ImportError:
            for i, im in enumerate(self.frames):
                im.save(str(OUT / f"{self.name}_{i:03d}.png"))
            path = OUT / f"{self.name}_*.png"
        print(f"  {path}  ({len(self.frames)} frames)")
        return path


# ---------------------------------------------------------------------------
# Driving the Console the way a person does, minus the dialogs.
# ---------------------------------------------------------------------------

# Where each side sets up for the next spawn dialog: (x, y, formation,
# spacing_m). A fleet file's own poses belong to the scene it was written
# for, so - exactly as a person does in the dialog - the spawn point is
# chosen looking at the scene in hand.
SPAWN = {"blue": (-7.0, -7.0, "wedge", 1.5), "red": (5.0, 0.0, None, 1.5)}


def _accept(self):
    """SpawnDialog.exec, scripted: place the side, then press Spawn here."""
    side = "red" if "red" in self.windowTitle().lower() else "blue"
    x, y, shape, spacing = SPAWN[side]
    self.space_slider.setValue(int(spacing * 10))
    if shape:
        i = self.form_combo.findText(shape)
        if i >= 0:
            self.form_combo.setCurrentIndex(i)
    self.sp_boxes[0].setValue(x)
    self.sp_boxes[1].setValue(y)
    self._reform()
    return QDialog.Accepted


def new_console():
    gui.SpawnDialog.exec = _accept
    win = gui.Console()
    win.resize(WIDTH, HEIGHT)
    win.show()
    QCoreApplication.processEvents()
    # A wider sidebar so the tab you are on is readable, and the map filled
    # rather than floating in the middle of the viewport.
    for d in win.findChildren(gui.QDockWidget):
        if d.windowTitle() == "Build":
            win.resizeDocks([d], [300], gui.Qt.Horizontal)
    win.viewport.zoom = 1.6
    QCoreApplication.processEvents()
    return win


def choose(combo, text, handler, *args):
    idx = combo.findText(text)
    if idx < 0:
        raise SystemExit(f"'{text}' is not in the dropdown: "
                         f"{[combo.itemText(i) for i in range(combo.count())]}")
    combo.setCurrentIndex(idx)
    handler(idx, *args)
    QCoreApplication.processEvents()


def compose(win, scene, blue, red=None, points=()):
    choose(win.scene_combo, scene, win.on_scene_chosen)
    choose(win.fleet_combo, blue, win.on_fleet_chosen, "blue")
    if red:
        choose(win.red_combo, red, win.on_fleet_chosen, "red")
    for i, (x, y) in enumerate(points, 1):
        win._points_override[f"P{i}"] = {"x": float(x), "y": float(y), "z": 0.0}
    if points:
        win._compose_setup()
        win._refresh_goals()
    QCoreApplication.processEvents()


def shell_for(win, cell):
    if cell != "white":
        win.open_cell(cell)
    for sh in win.shells:
        if sh.cell == cell:
            return sh
    raise RuntimeError(cell)


def say(win, cell, cmd):
    sh = shell_for(win, cell)
    sh.inp.setText(cmd)
    sh.run()
    QCoreApplication.processEvents()
    return sh


def tab(win, name):
    for i in range(win.tabs.count()):
        if win.tabs.tabText(i) == name:
            win.tabs.setCurrentIndex(i)
            QCoreApplication.processEvents()
            return
    raise RuntimeError(name)


def stop(win):
    try:
        if getattr(win, "proc", None) is not None:
            win.proc.kill()
            win.proc.waitForFinished(2000)
    except Exception:
        pass
    for w in list((getattr(win, "_cell_windows", None) or {}).values()):
        w.close()
    win.close()
    QCoreApplication.processEvents()


# ---------------------------------------------------------------------------
# The tour. One GIF per claim in the README.
# ---------------------------------------------------------------------------

def gif_setup():
    """Compose a run: scene, blue fleet, red fleet, points, Play."""
    win = new_console(); rec = Recorder(win, "01-setup")
    rec.snap(repeat=4)
    choose(win.scene_combo, "maze", win.on_scene_chosen);            rec.pump(1.0)
    choose(win.fleet_combo, "8_roboracer_no_lidar", win.on_fleet_chosen, "blue"); rec.pump(1.2)
    choose(win.red_combo, "custom_red", win.on_fleet_chosen, "red"); rec.pump(1.2)
    for i, (x, y) in enumerate([(6, 6), (-6, 6), (0, -7)], 1):
        win._points_override[f"P{i}"] = {"x": x, "y": y, "z": 0.0}
        win._compose_setup(); win._refresh_goals();                  rec.pump(0.7)
    win.start_run();                                                  rec.pump(2.0)
    rec.snap(repeat=6)
    stop(win); return rec.save()


def gif_command():
    """SETMISSION + launch: the fleet moves; the wrong cell is refused."""
    win = new_console(); rec = Recorder(win, "02-command")
    compose(win, "maze", "8_roboracer_no_lidar", "custom_red",
            points=[(6, 6), (-6, 6), (0, -7)])
    win.start_run(); rec.pump(1.5)
    blue = say(win, "blue", "SETMISSION advance to P1 P2 P3");         rec.pump(1.0)
    say(win, "blue", "blue launch");                                   rec.pump(6.0)
    say(win, "blue", "red launch")            # refused - wrong cell
    rec.pump(0.5)
    for _ in range(8):
        rec.snap(widget=win._cell_windows["blue"])
    rec.pump(3.0)
    stop(win); return rec.save()


def gif_jamming():
    """A jammer comes up; links degrade; command authority shrinks."""
    win = new_console(); rec = Recorder(win, "03-jamming")
    compose(win, "maze", "8_roboracer_no_lidar", "custom_red",
            points=[(6, 6), (-6, 6), (0, -7)])
    win.start_run(); rec.pump(1.0)
    say(win, "blue", "SETMISSION advance to P1 P2 P3")
    say(win, "blue", "blue launch")
    tab(win, "Network");                                               rec.pump(4.0)
    say(win, "red", "JAM jam1 band 2400 power 30")
    say(win, "red", "red launch");                                     rec.pump(8.0)
    tab(win, "Contested");                                             rec.pump(3.0)
    say(win, "red", "red halt");                                       rec.pump(4.0)
    stop(win); return rec.save()


def gif_drift():
    """GNSS jammed: belief and truth part company."""
    win = new_console(); rec = Recorder(win, "04-drift")
    SPAWN["blue"] = (-7.0, -7.0, "wedge", 1.5); SPAWN["red"] = (6.0, 0.0, None, 1.5)
    compose(win, "open_field", "8_roboracer_no_lidar", "custom_red",
            points=[(7, 7), (-7, 7), (0, -8)])
    win.start_run(); rec.pump(1.0)
    say(win, "blue", "SETMISSION advance to P1 P2 P3")
    say(win, "blue", "blue launch");                                   rec.pump(3.0)
    say(win, "red", "JAM jam1 band 1575.42 power 30")
    say(win, "red", "red launch");                                     rec.pump(14.0)
    stop(win); return rec.save()


def gif_wavefront():
    """Whose signal dominates where: the wavefront overlay and the spectrum."""
    win = new_console(); rec = Recorder(win, "05-wavefront")
    compose(win, "maze", "8_roboracer_no_lidar", "custom_red",
            points=[(6, 6), (-6, 6), (0, -7)])
    win.start_run(); rec.pump(1.0)
    say(win, "blue", "SETMISSION advance to P1 P2 P3")
    say(win, "blue", "blue launch")
    say(win, "red", "JAM jam1 band 2400 power 30")
    say(win, "red", "red launch");                                     rec.pump(2.0)
    win.viewport.show_wavefront = True; win.viewport.update();         rec.pump(5.0)
    win.viewport.show_spectrum = True; win.viewport.update();          rec.pump(5.0)
    stop(win); return rec.save()


def gif_results():
    """A sweep's results: any output against any axis, click a row to replay.

    Uses the newest runs/sweep_*/results.csv; run one first, e.g.
        python3 tools/sweep.py experiments/penetration.yaml
    """
    win = new_console(); rec = Recorder(win, "06-results")
    SPAWN["blue"] = (-95.0, 0.0, "wedge", 4.0); SPAWN["red"] = (95.0, 10.0, None, 4.0)
    compose(win, "corridor_200m", "8_roboracer_no_lidar", "custom_red")
    sweeps = sorted((REPO / "runs").glob("sweep_*/results.csv"),
                    key=lambda p: p.stat().st_mtime)
    if not sweeps:
        raise SystemExit("no runs/sweep_*/results.csv - run tools/sweep.py first")
    tab(win, "Results")
    # The experiment window is what the GIF is about; every frame is of it,
    # so they all share one size.
    exp = gui.ExperimentWindow(console=win)
    exp.resize(1400, 820); exp.show()
    QCoreApplication.processEvents()
    for _ in range(4):
        rec.snap(widget=exp)
    exp.load_csv(sweeps[-1])
    QCoreApplication.processEvents()
    for _ in range(10):
        rec.snap(widget=exp)
    # Step through the plottable outputs if the window offers a chooser.
    for combo_name in ("y_combo", "metric_combo", "output_combo"):
        combo = getattr(exp, combo_name, None)
        if combo is not None:
            for i in range(min(combo.count(), 4)):
                combo.setCurrentIndex(i); QCoreApplication.processEvents()
                for _ in range(6):
                    rec.snap(widget=exp)
            break
    exp.close()
    stop(win); return rec.save()


TOUR = {
    "setup": gif_setup, "command": gif_command, "jamming": gif_jamming,
    "drift": gif_drift, "wavefront": gif_wavefront, "results": gif_results,
}


def main(argv):
    names = argv or list(TOUR)
    qa = QApplication.instance() or QApplication([])
    qa.setStyleSheet(gui.STYLE)
    for n in names:
        print(f"{n}:")
        TOUR[n]()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
