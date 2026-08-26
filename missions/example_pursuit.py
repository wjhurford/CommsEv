"""
Example mission script.

Point a scenario at this file and the agent follows it:

    mission:
      type: script
      file: missions/example_pursuit.py
      target: car1          # anything else you put here reaches you in `agent`

Write one function. Return where the agent should be heading right now, in
metres, in world coordinates. Speed limits and collision still apply, so you
cannot drive through a wall by returning a point behind it.
"""

import math


def target(agent, t, poses, arena):
    """Sit 1.2 m behind whichever agent this one is chasing.

    agent  - this agent's own config: id, dimensions, speed, mission, sensors
    t      - seconds since the run started
    poses  - every agent's current pose, keyed by id: {'car1': {x, y, z, yaw}}
    arena  - extent, boundaries, propagation, spectrum
    """
    chase = agent["mission"].get("target", "car1")
    lead = poses.get(chase)
    if lead is None:
        return (agent["start"]["x"], agent["start"]["y"])

    standoff = 1.2
    bearing = lead["yaw"] + math.pi
    return (lead["x"] + standoff * math.cos(bearing),
            lead["y"] + standoff * math.sin(bearing))
