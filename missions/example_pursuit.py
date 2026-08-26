"""
Pursuit - the smallest useful mission.

Point a scenario at this file and the agent follows it:

    mission:
      type: script
      file: missions/example_pursuit.py
      target: car1          # anything else you put here reaches you in `agent`

ONE FUNCTION. Return where the agent should be heading right now, in metres,
in world coordinates. Speed limits and collision still apply, so you cannot
drive through a wall by returning a point behind it.

Nothing in this file imports the framework, and nothing imports rclpy. That is
the point: the identical file runs in this simulator and on a real car.
"""

import math


def target(agent, world):
    """Sit `standoff` metres behind whichever agent this one is chasing.

    agent  - this agent's own config: id, dimensions, speed, mission, sensors
    world  - what this agent knows. See docs/writing-a-mission.md.
             world.t, world.dt, world.arena
             world.pose(id), world.distance_to(a, b), world.bearing_to(a, b)
             world.scan(id), world.nearest_return(id, lo, hi)
             world.link(a, b)
    """
    chase = agent["mission"].get("target", "car1")
    lead = world.pose(chase)
    if lead is None:
        return (agent["start"]["x"], agent["start"]["y"])

    standoff = float(agent["mission"].get("standoff", 1.2))

    # Behind the leader means behind ITS nose, not behind ours. Adding pi to the
    # leader's yaw gives the point trailing it whichever way it is facing.
    bearing = lead["yaw"] + math.pi
    return (lead["x"] + standoff * math.cos(bearing),
            lead["y"] + standoff * math.sin(bearing))
