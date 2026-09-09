"""
The bridge: real ROS 2 topics in, Console telemetry out.

The Console knows nothing about ROS. It reads one stream of JSON frames from a
socket. This node subscribes to every agent's topics, packs them into that same
frame shape, and serves them — so pointing the Console at this instead of the
stub is a one-line change and nothing in the UI has to know the difference.

Which is the point of having had a boundary there from the start: the stub was
never throwaway, it was the contract.
"""

import json
import math
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from .common import default_scenario, load_sim_core

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=5)

# The Console draws a few hundred pixels wide; 1081 points per agent per frame
# is a lot of JSON for that. Downsampling here is a display decision and does
# not touch what the world node actually published.
DISPLAY_RAYS = 271


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BridgeNode(Node):
    def __init__(self):
        super().__init__("deadband_bridge")
        self.declare_parameter("scenario", default_scenario())
        self.declare_parameter("port", 8765)
        self.declare_parameter("rate_hz", 10.0)

        # An unsupplied launch argument arrives as "", not as absent, and ""
        # would otherwise beat the default we just declared.
        scenario = self.get_parameter("scenario").value or default_scenario()
        self.port = int(self.get_parameter("port").value)

        sim = load_sim_core()
        self.arena, self.agents, self.links = sim.load_scenario(scenario)
        self.sim = sim
        self.state = {a["id"]: {"pose": {"x": a["start"]["x"], "y": a["start"]["y"],
                                         "z": a["start"]["z"],
                                         "yaw": a["start"]["yaw"], "speed": 0.0},
                                "scan": None}
                      for a in self.agents}
        self.lock = threading.Lock()
        self.seq = 0
        self.t0 = None

        for a in self.agents:
            aid = a["id"]
            self.create_subscription(Odometry, f"/{aid}/odom",
                                     lambda m, i=aid: self.on_odom(i, m), 10)
            # Only subscribe where a lidar actually exists. A subscription alone
            # is enough to make a topic show up in `ros2 topic list`, so
            # subscribing blindly invents a /gcs/scan for a ground station that
            # has no lidar - which is a confusing thing to leave in a graph
            # other people will read.
            # NAMESPACED PER SENSOR, matching what world_node publishes and
            # what publications_for() advertises. A car with a lidar and a
            # depth camera has two of these; both are subscribed, so the
            # bridge carries whatever the vehicle actually senses rather than
            # the first sensor that happened to match a hardcoded type.
            for sen in a["sensors"]:
                if self.sim.sensor_spec(sen["type"]) is None:
                    continue
                self.create_subscription(
                    LaserScan, f"/{aid}/{sen['id']}/scan",
                    lambda m, i=aid: self.on_scan(i, m), SENSOR_QOS)

        self.frames = []
        threading.Thread(target=self.serve, daemon=True).start()
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)
        self.get_logger().info(f"bridge on ws://localhost:{self.port}")

    # -- subscriptions ------------------------------------------------------

    def on_odom(self, aid, msg):
        with self.lock:
            self.state[aid]["pose"] = {
                "x": round(msg.pose.pose.position.x, 4),
                "y": round(msg.pose.pose.position.y, 4),
                "z": round(msg.pose.pose.position.z, 4),
                "yaw": round(yaw_from_quaternion(msg.pose.pose.orientation), 4),
                "speed": round(msg.twist.twist.linear.x, 4),
            }

    def on_scan(self, aid, msg):
        step = max(1, len(msg.ranges) // DISPLAY_RAYS)
        ranges = [None if not math.isfinite(r) else round(float(r), 3)
                  for r in msg.ranges[::step]]
        with self.lock:
            self.state[aid]["scan"] = {
                "frame": msg.header.frame_id,
                "model": "UST-10LX",
                "plane_z": None,
                "angle_min": float(msg.angle_min),
                "angle_max": float(msg.angle_max),
                "range_min": float(msg.range_min),
                "range_max": float(msg.range_max),
                "range_effective": self.sim.effective_range(
                    self.arena.get("surface_reflectivity")),
                "surface_reflectivity": self.arena.get("surface_reflectivity"),
                "ranges": ranges,
            }

    # -- frame assembly -----------------------------------------------------

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        with self.lock:
            poses = {i: dict(v["pose"]) for i, v in self.state.items()}
            agents_out = []
            for a in self.agents:
                st = self.state[a["id"]]
                agents_out.append({
                    "id": a["id"], "platform": a["platform"],
                    "network": a["network"], "colour": a["colour"],
                    "mission": a["mission"].get("type", "static"),
                    "dimensions": a["dimensions"], "pose": dict(st["pose"]),
                    "scan": st["scan"],
                    "publishes": self.sim.publications_for(a),
                    "sensors": [{"id": s["id"], "type": s["type"],
                                 "offset": s["offset"], "ok": True,
                                 "summary": {}} for s in a["sensors"]],
                    "health": {"ok": True, "warnings": []},
                })
        links_out = [{**l, **self.sim.link_state(poses[l["a"]], poses[l["b"]])}
                     for l in self.links]
        frame = {
            "seq": self.seq, "sim_time_s": round(now - self.t0, 3),
            "wall_time": now, "run_state": "running", "source": "ros2",
            "arena": self.arena, "agents": agents_out, "links": links_out,
            "attacks_active": [], "contacts": [],
        }
        self.seq += 1
        self.frames.append(json.dumps(frame))
        del self.frames[:-2]        # only the newest matters to a viewer

    # -- websocket ----------------------------------------------------------

    def serve(self):
        import asyncio
        try:
            import websockets
        except ImportError:
            self.get_logger().error(
                "websockets not installed. pip3 install websockets")
            return

        async def handler(ws):
            last = -1
            try:
                while True:
                    if self.frames and self.seq != last:
                        last = self.seq
                        await ws.send(self.frames[-1])
                    await asyncio.sleep(0.02)
            except Exception:
                pass

        async def run():
            async with websockets.serve(handler, "0.0.0.0", self.port):
                await asyncio.Future()

        asyncio.run(run())


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
