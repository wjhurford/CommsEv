"""
Wall following - a mission that uses the LIDAR.

    mission:
      type: script
      file: missions/wall_follow.py
      side: right           # right | left
      standoff: 0.8         # metres to hold off the wall
      lookahead: 1.2        # metres ahead to aim

This is the first mission worth reading, because it closes a loop through a
sensor. Pursuit only needs to know where another agent is, which in real life
means somebody already solved localisation for you. This one needs a range
reading, and range readings are noisy, finite, and sometimes absent.

THE THINGS THAT BITE, in the order they will bite you:

  1.  SIGN. Getting too far from the wall must turn you TOWARD it, and which
      way that is depends on which side you are following. Get this backwards
      and the car drives away from the wall, finds the opposite one, and
      spirals - it looks like a tuning problem and it is not.
  2.  A ray that returned NOTHING is float('inf'), not range_max. A UST-10LX
      sees 10 m off white card and about 4 m off a dark matt wall - past that
      it reports no return at all. Treat inf as "wall at 10 m" and the car
      will confidently turn into an open doorway.
  3.  Angles are in the SENSOR's frame: 0 is dead ahead, positive to the left
      (right-handed, z up). The wall on your right is near -pi/2.
  4.  Corners need their own case. A pure side-distance controller has no idea
      anything is in front of it and drives into the corner it was following
      toward, at full speed, every time.
  5.  You return a POINT, not a steering angle. The controller does pure
      pursuit onto that point, so "steer left a bit" is expressed as "aim
      somewhere a bit to the left of straight ahead".
"""

import math


def target(agent, world):
    me = world.pose(agent["id"])
    if me is None:
        return (agent["start"]["x"], agent["start"]["y"])

    m = agent["mission"]
    side = m.get("side", "right")
    standoff = float(m.get("standoff", 0.8))
    lookahead = float(m.get("lookahead", 1.2))

    # Turning by +turn is anticlockwise. Following the RIGHT wall, "toward the
    # wall" is clockwise, so the correction is negated. This one line is the
    # bug in every first draft of a wall follower ever written.
    toward_wall = -1.0 if side == "right" else 1.0

    # Look at a 60-degree wedge centred abeam on the chosen side. Narrow enough
    # that the wall ahead does not contaminate it, wide enough to survive a bend.
    abeam = -math.pi / 2 if side == "right" else math.pi / 2
    dist, _ = world.nearest_return(agent["id"], abeam - 0.5, abeam + 0.5)

    if not (dist < float("inf")):
        # NO RETURN on that side. Not "the wall is far away" - we genuinely
        # cannot see it, which is what happens at a doorway or off a dark
        # surface. Hold heading and let the wall come back, rather than turning
        # toward something we have not measured.
        correction = 0.0
    else:
        # Positive error = too far from the wall, so turn toward it.
        # The 1.2 rad/m gain is a TUNING CHOICE, not a physical quantity:
        # too high and it weaves, too low and it cuts the corners. Set by hand
        # in an 8 m box at 1.5 m/s; retune for a different arena or speed.
        correction = toward_wall * max(-0.7, min(0.7, 1.2 * (dist - standoff)))

    # Corner: something is close ahead, so turn AWAY from the wall hard, and
    # do it in preference to the side correction rather than on top of it.
    ahead, _ = world.nearest_return(agent["id"], -0.4, 0.4)
    reach = lookahead
    if ahead < standoff * 1.2:
        correction = -toward_wall * 1.2
        reach = max(0.6, lookahead * 0.6)     # keep moving; do not crawl

    heading = me["yaw"] + correction
    return (me["x"] + reach * math.cos(heading),
            me["y"] + reach * math.sin(heading))
