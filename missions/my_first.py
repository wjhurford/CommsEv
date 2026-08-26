"""
my_first.py - the mission you write to prove the interface works.

    mission:
      type: script
      file: missions/my_first.py
      follow: car1          # whose flank to hold
      offset: 2.0           # metres out to the side
      side: left            # left | right of the lead's nose

WHAT IT DOES. Keep station a fixed distance off the side of another car, and
stay there while that car drives around. Nothing more. It is a path follower:
it reads where the lead is and returns a point beside it. That is the smallest
mission that actually does something you can watch.

WHY THIS ONE FIRST. It only needs one thing from the world - another agent's
pose - and localisation is assumed solved for you. The next mission up,
wall_follow.py, has to close a loop through a noisy lidar; the one after that,
return_on_link_loss.py, reacts to the network. Start here, where the only thing
that can go wrong is the geometry.

THE ONE IDEA. "Left of car1" means left of ITS nose, not left on the map. If
car1 is driving north, its left is west; if it turns around and drives south,
its left is now east. So the offset direction is computed from the lead's yaw
every tick, not fixed to a compass point. Get this wrong and the car sits on
the correct side only while the lead happens to be pointing one way.

TRY THIS, once it is running:
  * change offset 2.0 -> 3.5 and watch the gap open
  * change side: left -> right and watch it swap flanks
  * change follow: car1 -> car2 and watch it pick a different lead
Each is one number, one Play, one visible difference. That loop - edit, run,
see it change - is the whole point of writing a mission by hand before there
is a dropdown that does it for you.
"""

import math


def target(agent, world):
    # agent - this car's own config (id, speed, start, mission block, ...)
    # world - everything it is allowed to know. See docs/writing-a-mission.md.

    # Read our parameters out of the mission block. Anything you put under
    # `mission:` in the scenario file arrives here, no plumbing to write.
    m = agent["mission"]
    lead_id = m.get("follow", "car1")
    offset = float(m.get("offset", 2.0))
    side = m.get("side", "left")

    # Where is the car we are following?
    lead = world.pose(lead_id)
    if lead is None:
        # Not heard from yet (it has not published a pose). Hold our spawn
        # point rather than lurching toward the origin. Returning our own
        # start is the mission's way of saying "stay put, I have no target".
        return (agent["start"]["x"], agent["start"]["y"])

    # "To the side of the lead's nose." The lead faces along its yaw; its left
    # is a quarter-turn anticlockwise (+pi/2), its right a quarter-turn
    # clockwise (-pi/2). This is the one line that has to be right.
    quarter = math.pi / 2 if side == "left" else -math.pi / 2
    bearing = lead["yaw"] + quarter

    return (lead["x"] + offset * math.cos(bearing),
            lead["y"] + offset * math.sin(bearing))
