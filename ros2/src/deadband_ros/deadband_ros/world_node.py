"""
The world: physics and sensors for every agent, as real ROS 2 topics.

This is the simulator half. It owns where things are and what the sensors see,
and it publishes exactly the message types a real RoboRacer publishes:

    /<agent>/<sensor>/scan          sensor_msgs/LaserScan
                                    one per RANGING sensor - a lidar sweep or
                                    a depth camera's horizontal slice
    /<agent>/<sensor>/camera_info   sensor_msgs/CameraInfo
                                    a depth camera's intrinsics, from the
                                    datasheet's field of view
    /<agent>/odom   nav_msgs/Odometry           pose and velocity
    /<agent>/speed  std_msgs/Float32            convenience, as f1tenth does

and subscribes to the one a real car listens on:

    /<agent>/drive  ackermann_msgs/AckermannDriveStamped

Matching those types is the whole sim-to-real argument. A controller written
against this node subscribes to sensor_msgs/LaserScan; on the hardware it
subscribes to sensor_msgs/LaserScan from urg_node. Same code, same topic shape,
different publisher.

A drive command overrides the agent's scripted mission for as long as it keeps
arriving, so a researcher's controller takes the wheel simply by publishing.
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
try:
    from sensor_msgs.msg import CameraInfo
    HAVE_CAMERA_INFO = True
except ImportError:                       # pragma: no cover - ROS not present
    CameraInfo = None
    HAVE_CAMERA_INFO = False
from std_msgs.msg import Float32

from .common import default_scenario, load_sim_core

try:
    from ackermann_msgs.msg import AckermannDriveStamped
    HAVE_ACKERMANN = True
except ImportError:          # ackermann_msgs is not in a bare desktop install
    HAVE_ACKERMANN = False

# Sensor data is best-effort by convention: a scan that arrives late is worth
# less than the next one. Odometry is reliable because a controller integrating
# it cannot afford silent gaps.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST, depth=5,
    durability=QoSDurabilityPolicy.VOLATILE)


# Odometry covariance.
#
# A Kalman filter decides how much to trust a measurement by its covariance.
# Publishing zeros tells it the measurement is perfect, so it throws away its own
# prediction and follows the sensor blindly - a filter debugged against that
# falls apart the moment the data stops being perfect.
#
# These are the variances of the odometry estimate a real RoboRacer would
# produce from wheel encoders and IMU integration. THEY NEED A SOURCE: measure
# the drift over a known straight run and a known turn, and replace them.
# Until then they are declared, not hidden, and flagged here.
ODOM_VARIANCE = {
    "xy": 0.0025,      # m^2   -> 5 cm standard deviation.  UNSOURCED
    "z": 1e-6,         # m^2   -> the cars stay on the floor
    "rollpitch": 1e-6, # rad^2 -> ditto
    "yaw": 0.0025,     # rad^2 -> ~2.9 degrees.  UNSOURCED
    "vx": 0.01,        # (m/s)^2 -> 10 cm/s.  UNSOURCED
}


def odom_covariance():
    """A 6x6 row-major covariance for [x, y, z, roll, pitch, yaw]."""
    diag = [ODOM_VARIANCE["xy"], ODOM_VARIANCE["xy"], ODOM_VARIANCE["z"],
            ODOM_VARIANCE["rollpitch"], ODOM_VARIANCE["rollpitch"],
            ODOM_VARIANCE["yaw"]]
    cov = [0.0] * 36
    for i, v in enumerate(diag):
        cov[i * 6 + i] = v
    return cov


def twist_covariance():
    diag = [ODOM_VARIANCE["vx"], ODOM_VARIANCE["vx"], 1e-6,
            1e-6, 1e-6, ODOM_VARIANCE["yaw"]]
    cov = [0.0] * 36
    for i, v in enumerate(diag):
        cov[i * 6 + i] = v
    return cov


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WorldNode(Node):
    def __init__(self):
        super().__init__("deadband_world")
        self.declare_parameter("scenario", default_scenario())
        self.declare_parameter("rate_hz", 20.0)

        # An unsupplied launch argument arrives as "", not as absent, and ""
        # would otherwise beat the default we just declared.
        scenario = self.get_parameter("scenario").value or default_scenario()
        rate = float(self.get_parameter("rate_hz").value)

        self.sim = load_sim_core()
        self.arena, self.agents, self.links = self.sim.load_scenario(scenario)
        self.get_logger().info(
            f"scenario {scenario}: {len(self.agents)} agents, {len(self.links)} links")

        import random
        self.rng = random.Random(1)
        self.poses = {a["id"]: {"x": a["start"]["x"], "y": a["start"]["y"],
                                "z": a["start"]["z"], "yaw": a["start"]["yaw"],
                                "speed": 0.0}
                      for a in self.agents}
        self.commands = {}          # agent id -> (stamp, speed, steering)

        self.pubs = {}
        for a in self.agents:
            aid = a["id"]
            self.pubs[aid] = {
                "odom": self.create_publisher(Odometry, f"/{aid}/odom", 10),
                "speed": self.create_publisher(Float32, f"/{aid}/speed", 10),
            }
            # ONE PUBLISHER PER RANGING SENSOR, on the topic
            # publications_for() advertises. A car carrying a lidar AND a
            # RealSense has two fields of view and two scans; a single flat
            # /<agent>/scan would put two publishers on one topic, and a
            # subscriber would receive an interleaved mixture of a 270-degree
            # 10 m sweep and an 87-degree 3 m cone - which looks exactly like
            # a sensor going mad and is very hard to diagnose.
            for sen in a["sensors"]:
                spec = self.sim.sensor_spec(sen["type"])
                if spec is None:
                    continue
                self.pubs[aid].setdefault("scans", {})[sen["id"]] = \
                    self.create_publisher(
                        LaserScan, f"/{aid}/{sen['id']}/scan", SENSOR_QOS)
                # A DEPTH CAMERA ALSO PUBLISHES ITS INTRINSICS. Without a
                # CameraInfo nothing downstream can turn a depth frame into
                # metres, so a camera that published only ranges would be
                # advertising a capability the graph cannot actually use.
                if HAVE_CAMERA_INFO and spec is self.sim.DEPTHCAM:
                    self.pubs[aid].setdefault("info", {})[sen["id"]] = \
                        self.create_publisher(
                            CameraInfo, f"/{aid}/{sen['id']}/camera_info", 10)
            # Static furniture does not take drive commands, so it should not
            # advertise a /drive topic either.
            if HAVE_ACKERMANN and a["platform"] != "ground_station":
                self.create_subscription(
                    AckermannDriveStamped, f"/{aid}/drive",
                    lambda msg, i=aid: self.on_drive(i, msg), 10)

        if not HAVE_ACKERMANN:
            self.get_logger().warn(
                "ackermann_msgs not installed - agents will follow their scripted "
                "missions and ignore /drive. Install with: "
                "sudo apt install ros-humble-ackermann-msgs")

        self.dt = 1.0 / rate
        self.t = 0.0
        self.create_timer(self.dt, self.tick)

    def on_drive(self, agent_id, msg):
        """A controller has taken the wheel for this agent."""
        self.commands[agent_id] = (self.t, float(msg.drive.speed),
                                   float(msg.drive.steering_angle))

    def tick(self):
        self.t += self.dt
        self.apply_commands()
        self.sim.step(self.agents, self.poses, self.t, self.dt, self.arena)
        stamp = self.get_clock().now().to_msg()
        for a in self.agents:
            self.publish_agent(a, stamp)

    def apply_commands(self):
        """Turn recent /drive commands into motion, superseding the mission.

        A command is honoured for 0.5 s. If the controller stops publishing —
        crashed, or its link was cut — the agent falls back to its mission
        rather than coasting on a stale command forever. That fallback is a
        research variable in its own right: it is on_link_loss made real.
        """
        for a in self.agents:
            cmd = self.commands.get(a["id"])
            if not cmd or self.t - cmd[0] > 0.5:
                continue
            _, speed, steering = cmd
            p = self.poses[a["id"]]
            speed = max(-a["speed"], min(a["speed"], speed))
            # Bicycle kinematics. Wheelbase is a real vehicle parameter and
            # NEEDS A SOURCE; 0.32 m is a placeholder for a 1:10 chassis.
            wheelbase = 0.32
            p["yaw"] = self.sim.wrap_pi(
                p["yaw"] + speed / wheelbase * math.tan(steering) * self.dt)
            nx = p["x"] + speed * math.cos(p["yaw"]) * self.dt
            ny = p["y"] + speed * math.sin(p["yaw"]) * self.dt
            if self.sim.blocked(a, nx, ny, self.poses, self.agents, self.arena) is None:
                p["x"], p["y"] = nx, ny
            p["speed"] = abs(speed)
            # Consumed: stop step() from also moving this agent this tick.
            a["mission"] = {"type": "static"}

    def publish_agent(self, agent, stamp):
        aid = agent["id"]
        p = self.poses[aid]
        pubs = self.pubs[aid]

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = f"{aid}/base_link"
        odom.pose.pose.position.x = float(p["x"])
        odom.pose.pose.position.y = float(p["y"])
        odom.pose.pose.position.z = float(p["z"])
        odom.pose.pose.orientation = yaw_to_quaternion(p["yaw"])
        odom.twist.twist.linear.x = float(p.get("speed", 0.0))
        odom.pose.covariance = odom_covariance()
        odom.twist.covariance = twist_covariance()
        pubs["odom"].publish(odom)

        msg = Float32()
        msg.data = float(p.get("speed", 0.0))
        pubs["speed"].publish(msg)

        first = True
        for sen in agent["sensors"]:
            pub = (pubs.get("scans") or {}).get(sen["id"])
            if pub is None:
                continue
            spec = self.sim.sensor_spec(sen["type"])
            scan = self.sim.scan_for(agent, sen, self.poses, self.agents,
                                     self.arena, self.rng)
            # Hand the PRIMARY scan to the mission layer for the NEXT tick.
            # Without this a scenario mission of `type: script` sees
            # world.scan() -> None in the ROS path but real data in the
            # headless path, so a lidar-driven mission works standalone and
            # drives into a wall under ROS. Same one-tick lag a real
            # subscriber has.
            if first:
                self.sim._LAST_SCANS[aid] = scan
                first = False
            out = LaserScan()
            out.header.stamp = stamp
            out.header.frame_id = f"{aid}/{sen['id']}"
            out.angle_min = float(scan["angle_min"])
            out.angle_max = float(scan["angle_max"])
            n = len(scan["ranges"])
            out.angle_increment = (out.angle_max - out.angle_min) / max(n - 1, 1)
            out.range_min = float(scan["range_min"])
            out.range_max = float(scan["range_max"])
            # A no-return is +inf, which is what the LaserScan message
            # specifies for a beam that found nothing. Writing range_max
            # instead would tell every downstream node there is a wall
            # exactly at the sensor's limit.
            out.ranges = [float("inf") if r is None else float(r)
                          for r in scan["ranges"]]
            # LaserScan carries no covariance, so a filter has to take range
            # uncertainty from the datasheet. The lidar is +/-40 mm absolute
            # (2-sigma, so sigma = 20 mm); the depth camera is <2% of range,
            # which is why its noise is applied proportionally in scan_for
            # rather than as one number here.
            pub.publish(out)

            info_pub = (pubs.get("info") or {}).get(sen["id"])
            if info_pub is not None:
                info = CameraInfo()
                info.header.stamp = stamp
                info.header.frame_id = f"{aid}/{sen['id']}"
                info.width, info.height = spec["res"]
                # A PINHOLE MODEL FROM THE DATASHEET'S FIELD OF VIEW. No
                # distortion coefficients, because the sim has no lens - and
                # publishing zeros as though they were a calibration would
                # invite somebody to trust an intrinsic matrix that was never
                # measured. The frame says what it is.
                fx = (info.width / 2.0) / math.tan(
                    math.radians(spec["fov_deg"]) / 2.0)
                fy = (info.height / 2.0) / math.tan(
                    math.radians(spec.get("fov_v_deg", spec["fov_deg"])) / 2.0)
                cx, cy = info.width / 2.0, info.height / 2.0
                info.distortion_model = "plumb_bob"
                info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
                info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0,
                          0.0, 0.0, 1.0, 0.0]
                info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = WorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
