"""
Wall following - a mission that uses the LIDAR.

    mission:
      type: script
      file: missions/wall_follow.py
      side: right           # right | left
      standoff: 0.8         # metres to hold off the wall
      lookahead: 1.2        # metres ahead to aim

TWO BEAMS, NOT ONE. This is the standard formulation (it is the F1TENTH wall-
follow lab, and the same geometry appears in every textbook treatment), and the
reason to use it rather than the obvious thing is worth understanding.

The obvious wall follower measures one distance to the side and steers on the
error. It oscillates, always, because distance alone tells you WHERE you are
and nothing about WHERE YOU ARE GOING - so the car only discovers it is closing
on the wall once it is already too close, over-corrects, and bounces off an
invisible line at the set distance. Adding a derivative term over time helps
and adds lag.

Two beams give you the wall's ANGLE for free, in the same instant:

    b = range abeam (90 degrees off the nose)
    a = range at theta degrees forward of that

    alpha  = atan2(a*cos(theta) - b, a*sin(theta))    <- angle to the wall
    D      = b * cos(alpha)                           <- true perpendicular distance
    D_next = D + L * sin(alpha)                       <- where you WILL be in L metres

Steering on D_next instead of on b is what removes the oscillation: the
correction is already looking ahead, so the car settles onto the standoff
instead of hunting around it. It costs one extra array lookup.

THE OTHER THINGS THAT BITE:

  1.  A ray that returned NOTHING is float('inf'), not range_max. A UST-10LX
      reaches 10 m off white card and about 4 m off a dark matt wall; past
      that there is no return at all. Treat inf as "wall at 10 m" and the car
      will drive confidently into an open doorway.
  2.  SIGN. Too far from the wall must turn you TOWARD it, and which way that
      is depends on the side. Get it backwards and the car spirals away.
  3.  Corners need a SMOOTH term, not a switch. A hard "something ahead, slam
      the wheel" branch is a bang-bang controller and it is visibly janky -
      the car alternates between gentle tracking and a full-lock turn.
  4.  You return a POINT, not a steering angle. "Steer left a bit" is
      expressed as "aim somewhere a bit to the left of straight ahead".
"""

import math

# Angle between the two beams. Anything from about 40 to 70 degrees works; too
# small and the two ranges are nearly equal so alpha is dominated by sensor
# noise, too large and the forward beam stops looking at the same wall.
THETA = math.radians(50.0)


def target(agent, world):
    me = world.pose(agent["id"])
    if me is None:
        return (agent["start"]["x"], agent["start"]["y"])

    m = agent["mission"]
    side = m.get("side", "right")
    standoff = float(m.get("standoff", 0.8))
    lookahead = float(m.get("lookahead", 1.2))
    gain = float(m.get("gain", 0.9))          # rad per metre of error

    # +1 turns anticlockwise. Following the RIGHT wall, "toward the wall" is
    # clockwise, so the correction is negated.
    toward_wall = -1.0 if side == "right" else 1.0
    abeam = -math.pi / 2 if side == "right" else math.pi / 2

    b = world.ray(agent["id"], abeam)                       # straight out
    a = world.ray(agent["id"], abeam - toward_wall * THETA)  # angled forward

    if not (b < float("inf")) or not (a < float("inf")):
        # One of the two beams saw nothing. Not "the wall is far away" - we
        # cannot measure it, which is what a doorway or a dark surface does.
        # Hold heading and let the wall come back rather than turning toward
        # something we have not measured.
        correction = 0.0
    else:
        # The wall's angle relative to our nose. NEGATIVE means we are closing
        # on it. No mirroring for the left wall: because the second beam is
        # taken on the same side, the two cases are reflections of each other
        # and the formula already comes out with the right sign for both.
        # (Checked by hand: a car turned 0.2 rad toward the wall gives
        # alpha = -0.199 whichever side the wall is on. An earlier version
        # negated this for the left wall, which made "closing on it" read as
        # "opening away from it" and the car drove straight into it.)
        alpha = math.atan2(a * math.cos(THETA) - b, a * math.sin(THETA))

        perp = b * math.cos(alpha)                   # true perpendicular range
        ahead_of_us = perp + lookahead * math.sin(alpha)   # where we will be

        error = ahead_of_us - standoff               # positive = too far out
        correction = toward_wall * max(-0.8, min(0.8, gain * error))

    # Corners, smoothly. Urgency runs 0 (nothing near) to 1 (nose on the wall)
    # and is squared, so this does nothing at all until it needs to do
    # something and then rises continuously. No switch, no jank.
    front, _ = world.nearest_return(agent["id"], -0.4, 0.4)
    trigger = max(standoff * 2.5, lookahead)
    if front < trigger:
        urgency = min(1.0, max(0.0, (trigger - front) / trigger))
        correction += -toward_wall * 1.6 * urgency * urgency

    correction = max(-1.2, min(1.2, correction))
    heading = me["yaw"] + correction
    return (me["x"] + lookahead * math.cos(heading),
            me["y"] + lookahead * math.sin(heading))
