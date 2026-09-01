"""
Shared plumbing for the Deadband ROS 2 nodes.

The simulation core — missions, motion, collision, the lidar raycast — lives in
tools/stub_telemetry.py and is imported here rather than reimplemented. One
physics implementation means the no-ROS stub and the ROS nodes can never quietly
disagree about where a car is.

TODO: when the interfaces settle, move that core into the `deadband` package
proper and have both import it from there. Reaching into tools/ is honest but
temporary.
"""

import importlib.util
import os
import sys
from pathlib import Path

# Finding the repo from inside an installed package is not a matter of counting
# parent directories: colcon copies these files into install/.../site-packages/
# and any fixed count is then wrong. Look for the repo's own landmarks instead,
# and let an environment variable override for an unusual layout.
MARKERS = ("scenarios", "tools", "console")


def find_repo_root():
    override = os.environ.get("DEADBAND_ROOT")
    if override:
        return Path(override).expanduser().resolve()

    candidates = [Path(__file__).resolve(), Path.cwd().resolve()]
    for start in candidates:
        for parent in [start] + list(start.parents):
            if all((parent / m).is_dir() for m in MARKERS):
                return parent
    raise FileNotFoundError(
        "Cannot find the Deadband repository. Set DEADBAND_ROOT, e.g.\n"
        "  export DEADBAND_ROOT=/mnt/c/Users/will-/Documents/repos/deadband")


REPO_ROOT = None


def repo_root():
    global REPO_ROOT
    if REPO_ROOT is None:
        REPO_ROOT = find_repo_root()
    return REPO_ROOT


def load_sim_core():
    """Import the shared simulation core from tools/stub_telemetry.py."""
    path = repo_root() / "tools" / "stub_telemetry.py"
    if not path.exists():
        raise FileNotFoundError(f"simulation core not found at {path}")
    spec = importlib.util.spec_from_file_location("deadband_sim_core", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["deadband_sim_core"] = mod
    spec.loader.exec_module(mod)
    return mod


def default_scenario():
    return str(repo_root() / "default_run.yaml")
