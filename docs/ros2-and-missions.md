# Getting to ROS 2, and writing missions

This document answers two questions: how ROS 2 fits into the framework, and how
a researcher writes a behaviour that runs in simulation and on a real car
without being rewritten.

---

## Does ROS 2 replace everything?

No, and the distinction matters for how this framework is built.

**ROS 2 is middleware.** It is the plumbing between processes: topics, services,
discovery, QoS. It is not a language, a control library, or a place where
algorithms live. A ROS 2 node is an ordinary Python or C++ program that happens
to publish and subscribe.

A real RoboRacer is therefore three layers, and only the middle one is ROS:

| Layer | What it is | Is it ROS 2? |
| --- | --- | --- |
| Firmware / drivers | VESC firmware, the lidar's own scan engine, motor commutation | **No.** Vendor firmware with a ROS wrapper on top |
| Integration | `/scan`, `/odom`, `/drive`, discovery, QoS | **Yes.** This is what ROS 2 is for |
| Algorithms | Kalman filter, controller, detector, ML model | **No.** Plain Python or C++ inside a node |

The algorithm layer does not know ROS exists. That is what makes the same
controller file runnable in the simulator and on hardware: only the wrapper
changes. An algorithm that imports `rclpy` has mixed two layers and cannot be
tested without a robot.

**Design rule for this project: never let ROS types into an algorithm.** A
mission or detector takes plain numbers and returns plain numbers.

---

## Standing ROS 2 up

Ubuntu 22.04 is required — AP_DDS supports Humble only.

### 1. ROS 2 Humble

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

Check the installation: `ros2 topic list` should print `/parameter_events` and
`/rosout`.

### 2. Verify the installation before adding anything

Two terminals:

```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

Every later step depends on this one; resolve any failure here first.

### 3. The f1tenth simulator

```bash
mkdir -p ~/sim_ws/src && cd ~/sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros
cd ~/sim_ws && rosdep install -i --from-path src --rosdistro humble -y
colcon build && source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

This is the reference to compare against. It publishes `/scan`,
`/ego_racecar/odom` and takes `/drive`.

### 4. ArduPilot, for the air layer

Follow ArduPilot's own ROS 2 + Gazebo guide. Two points each cost a day if
missed: set `export GZ_VERSION=harmonic` before building, and never install
`ros-humble-ros-gz*` alongside `ros-humble-ros-gzharmonic` — they conflict.

---

## How the Console connects

The Console is a Windows Qt application. ROS 2 runs in WSL. They meet at one
place only: a stream of telemetry frames over a socket on localhost, which
crosses the WSL boundary without special setup.

```
  WSL (Ubuntu)                                    Windows
  ------------                                    -------
  Gazebo / f1tenth_gym / real cars
        |  ROS 2 topics
        v
  commsev_bridge  ---- JSON frames over ws ---->  CommsEv Console
        ^                                          (draws, records, plots)
        |  ROS 2 topics
  controller nodes
```

`commsev_bridge` is one node: it subscribes to every agent's topics, packs them
into the frame shape `tools/stub_telemetry.py` already emits, and writes them to
a WebSocket. Nothing in the Console changes — `start_run()` points at the bridge
instead of the stub.

The stub remains. It is the zero-dependency mode: the Console can be developed,
demonstrated and handed over with no ROS installed at all.

---

## Writing a mission

A mission answers one question: **where should this agent be heading right now?**

> **The full walkthrough is [`history/writing-a-mission.md`](history/writing-a-mission.md)** —
> the contract, common pitfalls, one mission written from scratch, and how to
> run it. What follows is the summary.

### The simulation version

`missions/example_pursuit.py`:

```python
def target(agent, world):
    """Return (x, y) in metres, world frame."""
    lead = world.pose("car1")
    return (lead["x"] - 1.2 * math.cos(lead["yaw"]),
            lead["y"] - 1.2 * math.sin(lead["yaw"]))
```

The function receives:

- `agent` — this agent's own config: id, dimensions, speed, sensors, mission
- `world` — everything the agent is allowed to know:
  - `world.t`, `world.dt`, `world.arena`
  - `world.pose(id)`, `world.distance_to(a, b)`, `world.bearing_to(a, b)`
  - `world.scan(id)`, `world.nearest_return(id, lo, hi)` — the lidar
  - `world.link(a, b)` — quality, state, latency, packet-delivery ratio (PDR)

Speed limits and collision still apply, so physics cannot be bypassed by
returning a point behind a wall.

The older `target(agent, t, poses, arena)` still runs with a deprecation
warning. It cannot see the lidar or the link.

Point a scenario at it:

```yaml
mission:
  type: script
  file: missions/my_mission.py
  target: car1        # anything extra reaches the script inside `agent["mission"]`
```

### The same behaviour on a real car

The function above has no ROS in it, so the node is a thin wrapper:

```python
class MissionNode(Node):
    def __init__(self):
        super().__init__("mission")
        self.pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_timer(0.05, self.tick)

    def tick(self):
        tx, ty = target(self.agent, self.clock(), self.poses, self.arena)
        msg = steer_toward(tx, ty, self.poses[self.me])   # plain maths
        self.pub.publish(msg)
```

`target()` is imported unchanged from the same file the simulator ran. This is
the sim-to-real argument: test the behaviour against three simulated cars under
a jammed link, then run the identical file on hardware.

### Starting point

1. Copy `missions/example_pursuit.py` to `missions/my_mission.py`
2. Change the returned point — make it circle, or hold formation
3. Point `car3`'s mission at the new file and press Run
4. If it throws, the error prints to the Log tab. It is never silently ignored

---

## Still to build

- `commsev_bridge` — the ROS 2 node described above
- A VESC model, so a command becomes motion through the same limits the real
  controller imposes (current, ERPM, servo travel) and reports odometry with
  the same quantisation
- Per-agent compute, so an onboard processor is a modelled, limited resource
  rather than free
