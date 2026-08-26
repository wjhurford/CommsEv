"""
Patrol, but come home when the link degrades.

    mission:
      type: script
      file: missions/return_on_link_loss.py
      coordinator: gcs      # who we need to stay in contact with
      degraded_below: 0.45  # link quality that counts as "losing it"
      recovered_above: 0.65 # ...and the quality that counts as "got it back"

THIS IS THE ONE THAT IS ACTUALLY THE RESEARCH. Everything else in missions/ is
a path follower and could be written against any simulator. This one reads the
state of the NETWORK and changes what the vehicle does because of it - which is
the entire reason this framework models the network as an object rather than
assuming messages arrive.

Run it twice: once with attacks: [] and once with a jammer scheduled against
the coordinator's link, and the difference between the two runs is a result.

THE HYSTERESIS IS NOT DECORATION. Two thresholds, not one. With a single
threshold an agent sitting exactly at the boundary flips between patrolling and
returning every tick, which looks like a bug in the simulator and is in fact a
bug in the controller. Real fallback logic has this and so should yours.
"""

import math


def target(agent, world):
    me = world.pose(agent["id"])
    if me is None:
        return (agent["start"]["x"], agent["start"]["y"])

    m = agent["mission"]
    coordinator = m.get("coordinator", "gcs")
    low = float(m.get("degraded_below", 0.45))
    high = float(m.get("recovered_above", 0.65))

    link = world.link(agent["id"], coordinator)

    # State lives on the mission dict, not in a module global. Two agents can
    # run this same file at once, and a global would have them share one brain.
    state = m.setdefault("_state", "patrol")
    if link is None:
        state = "return"                     # no coordinator = assume the worst
    elif state == "patrol" and link["quality"] < low:
        state = "return"
    elif state == "return" and link["quality"] > high:
        state = "patrol"
    m["_state"] = state

    if state == "return":
        home = world.pose(coordinator)
        if home is None:
            return (agent["start"]["x"], agent["start"]["y"])
        # Stop 1.5 m short. Driving onto the ground station is not "returning".
        d = math.hypot(home["x"] - me["x"], home["y"] - me["y"])
        if d < 1.5:
            return (me["x"], me["y"])        # hold station
        u = (1.5 / d)
        return (home["x"] + (me["x"] - home["x"]) * u,
                home["y"] + (me["y"] - home["y"]) * u)

    # --- normal patrol: a rectangle inset from the arena walls ---------------
    hx = world.arena["extent"]["x"] / 2 - 0.8
    hy = world.arena["extent"]["y"] / 2 - 0.8
    corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    speed = max(float(agent["speed"]), 0.05)
    perim = sum(math.dist(corners[i], corners[(i + 1) % 4]) for i in range(4))
    # Where along the loop we should be by now. Distance, not index, so the
    # agent moves at a constant speed rather than jumping between corners.
    travelled = (world.t * speed) % perim
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        leg = math.dist(a, b)
        if travelled <= leg:
            u = travelled / leg
            return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u)
        travelled -= leg
    return corners[0]
