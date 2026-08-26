"""
A controller: one agent's behaviour, as a ROS 2 node.

THIS IS WHERE A RESEARCHER'S CODE GOES. The node is a wrapper and nothing more:
it subscribes to this agent's sensors, calls a plain Python function, and
publishes a drive command. The function knows nothing about ROS.

    ros2 run deadband_ros controller --ros-args \
        -p agent:=car3 -p mission:=missions/example_pursuit.py

Run it against this simulator today; run the identical mission file on a real
RoboRacer tomorrow by pointing this node at the car's own topics. That is the
whole sim-to-real argument, and it only works because target() never imports
rclpy. Keep it that way.
"""

import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node

from .common import default_scenario, load_sim_core

try:
    from ackermann_msgs.msg import AckermannDriveStamped
except ImportError:
    AckermannDriveStamped = None


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ControllerNode(Node):
    def __init__(self):
        super().__init__("deadband_controller")
        self.declare_parameter("agent", "car3")
        self.declare_parameter("scenario", default_scenario())
        self.declare_parameter("mission", "missions/example_pursuit.py")

        self.me = self.get_parameter("agent").value
        self.sim = load_sim_core()
        self.arena, agents, _ = self.sim.load_scenario(
            self.get_parameter("scenario").value or default_scenario())
        self.agent = next(a for a in agents if a["id"] == self.me)
        self.others = agents

        # Mission paths in a scenario are relative to the repo, and a ROS node
        # is launched from wherever the researcher happens to be standing.
        from .common import repo_root
        mission_path = Path(self.get_parameter("mission").value
                            or "missions/example_pursuit.py")
        if not mission_path.is_absolute():
            mission_path = repo_root() / mission_path
        self.mission = self.sim.load_mission_script(str(mission_path))
        self.get_logger().info(
            f"{self.me} running {self.get_parameter('mission').value}")

        if AckermannDriveStamped is None:
            self.get_logger().error(
                "ackermann_msgs not installed: "
                "sudo apt install ros-humble-ackermann-msgs")
            raise SystemExit(1)

        self.poses = {}
        self.scans = {}
        for a in agents:
            self.create_subscription(Odometry, f"/{a['id']}/odom",
                                     lambda m, i=a["id"]: self.on_odom(i, m), 10)
        # Our OWN lidar only. A mission is entitled to what this agent senses;
        # subscribing to everyone's scans would let it cheat in a way no real
        # vehicle can, and cheating quietly is worse than not working.
        if any(sn["type"] == "ust10lx" for sn in self.agent["sensors"]):
            self.create_subscription(LaserScan, f"/{self.me}/scan",
                                     self.on_scan, 10)
        self.pub = self.create_publisher(
            AckermannDriveStamped, f"/{self.me}/drive", 10)
        self.t = 0.0
        self.dt = 0.05
        self.create_timer(self.dt, self.tick)

    def on_scan(self, msg):
        """Translate LaserScan into the same dict a mission sees in the sim.

        One conversion, in one place. The mission never learns whether its
        ranges came from a ROS topic or from the simulator's raycaster, which
        is exactly the property that lets the same file run on a real car.
        """
        self.scans[self.me] = {
            "angle_min": float(msg.angle_min),
            "angle_max": float(msg.angle_max),
            "angle_increment": float(msg.angle_increment),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            # inf stays inf. ROS uses it for "no return" and so do we; mapping
            # it to range_max here would silently invent a wall.
            "ranges": [float(r) for r in msg.ranges],
        }

    def on_odom(self, aid, msg):
        self.poses[aid] = {
            "x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "yaw": yaw_from_quaternion(msg.pose.pose.orientation),
            "speed": msg.twist.twist.linear.x,
        }

    def tick(self):
        self.t += self.dt
        if self.me not in self.poses:
            return                      # nothing heard yet
        world = self.sim.World(self.t, self.dt, self.poses, self.arena,
                               self.others, scans=self.scans)
        try:
            tx, ty = self.sim._call_mission(self.mission, self.agent, world)
        except Exception as exc:
            self.get_logger().error(f"mission failed: {exc}")
            return

        # Pure pursuit, the standard steering law: aim at the target point and
        # steer by the angle between where you are pointing and where it is.
        me = self.poses[self.me]
        dx, dy = tx - me["x"], ty - me["y"]
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        err = self.sim.wrap_pi(bearing - me["yaw"])

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if dist < 0.1:
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
        else:
            # Slow down when the target is off to the side or close by: driving
            # flat out at something 90 degrees away just describes a wide circle.
            msg.drive.speed = float(self.agent["speed"] *
                                    max(0.2, math.cos(err)) *
                                    min(1.0, dist / 0.5))
            msg.drive.steering_angle = float(max(-0.4, min(0.4, err)))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ControllerNode()
    except SystemExit:
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
