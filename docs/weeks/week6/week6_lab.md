# Week 6 — LD19 Driver, LaserScan & Obstacle Detection

---

**Objectives:**

1. Understand how a 2D time-of-flight lidar produces a `LaserScan` message — the relationship between range, angle, and scan rate.
2. Verify the Mini Pupper 2's lidar driver configuration against your actual hardware.
3. Read and interpret the `sensor_msgs/LaserScan` message structure directly from `/scan`.
4. Write a ROS2 node that subscribes to `/scan` and detects the closest obstacle within a configurable field of view.
5. Combine obstacle detection with teleop to implement a basic stop-before-collision safety behavior.

---

**Reference Material:**

- [ldlidar_stl_ros2 driver (GitHub)](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2)
- [sensor_msgs/LaserScan message definition](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html)
- [mini_pupper_ros (GitHub)](https://github.com/mangdangroboticsclub/mini_pupper_ros)
- Week 3 Lab — Teleop, RViz2 & TF Tree (you'll reuse the TF skills and `teleop_twist_keyboard` from there)

---

## Background

A 2D lidar like the Mini Pupper 2's works on time-of-flight (ToF): it fires a laser pulse, measures how long the reflection takes to come back, and converts that into a distance. The whole unit spins, taking a burst of these measurements at a fixed rate — for the LD19, about 4,500 distance samples per second, completing a full 360° rotation roughly 10 times per second. Divide it out and that's around 450 points per revolution, each about 0.8° apart.

Every full rotation gets packaged into one ROS2 message: `sensor_msgs/LaserScan`. The fields that matter most:

| Field | Meaning |
|---|---|
| `angle_min`, `angle_max` | The start and end angle of the scan, in radians |
| `angle_increment` | The angle between consecutive points in `ranges[]` |
| `ranges[]` | The actual distance measurements, in meters, one per angle step |
| `range_min`, `range_max` | Valid distance bounds — anything outside this is unreliable |

If a particular laser pulse doesn't get a valid reflection (nothing in range, or a surface that doesn't reflect well), that slot in `ranges[]` shows up as `inf` rather than a real number. Any code that processes `ranges[]` has to handle that case explicitly, or it'll crash or silently produce garbage.

!!! note "Driver note: LD06 vs LD19"
    Looking at `mini_pupper_driver/launch/lidar_ld06.launch.py`, the lidar node is configured with `product_name: 'LDLiDAR_LD06'`, even though current Mini Pupper 2 units ship with the LD19. The LD19 is mechanically an upgraded LD06 — better accuracy (±10mm vs ±15mm), same communication protocol and baud rate — so `/scan` still works fine with the LD06 setting. But it's worth knowing this mismatch exists; if you ever see odd behavior from the lidar, this config is one of the first things to check. The driver package (`ldlidar_stl_ros2`) does support `LDLiDAR_LD19` as a distinct, correct option if you want to fix it.

---

## Setup

### Step 1 — Confirm the Lidar Is Live

With bringup running, check that `/scan` is publishing:

```bash
ros2 topic list
```

Check the actual publish rate:

```bash
ros2 topic hz /scan
```

**Task 1:** Paste your `ros2 topic hz /scan` output. The LD19's spec sheet says it scans at 10 Hz — how close does your measured rate come to that?

---

## Investigation

### Step 2 — Inspect the LaserScan Message

Look at the message definition directly:

```bash
ros2 interface show sensor_msgs/msg/LaserScan
```

Then capture one real scan:

```bash
ros2 topic echo /scan --once
```

**Task 2:** Answer both of the following:

- In your own words, explain what `angle_increment` represents and why `ranges[]` is a flat array rather than, say, a list of (angle, distance) pairs.
- Using `angle_min`, `angle_max`, and `angle_increment` from your captured scan, calculate how many points you'd expect in `ranges[]`. Then count the actual length of `ranges[]` from the echoed message. Do they match?

!!! note "Which index is 'front'?"
    Don't assume `ranges[0]` corresponds to straight ahead of the robot — that depends on how the lidar is mounted and which direction it scans (`laser_scan_dir` in the launch file).

---

## Building the Obstacle Detector

### Step 3 — Write the Node

Write a node that watches `/scan` and reports whether something is too close. Create the file:

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/obstacle_detector.py
```

Fill in the starter code below.

```python
#!/usr/bin/env python3
"""
obstacle_detector.py

Subscribes to /scan and reports whether an obstacle is closer than
safety_distance, within a forward-facing field of view.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleDetectorNode(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        self.safety_distance = 0.3 # meters
        self.fov_deg = 60 # total field of view, centered on "front"

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info(
            f'ObstacleDetectorNode started! '
            f'safety_distance={self.safety_distance}m, fov={self.fov_deg}deg'
        )

    def scan_callback(self, msg: LaserScan):

        # Task: Figure out how many indices on either side of "front" your
        # field of view covers. You know the total angular width you want
        # (self.fov_deg, converted to radians) and the angle between each
        # point (msg.angle_increment). How many array slots does that span?
        # Hint: half the angular width, divided by angle_increment, gives
        # you how many indices to look at on EACH side of center.

        half_fov_indices = # Your code

        # Task: Until you've determined which index is "front,"
        # use index 0 as a placeholder center.
        # You'll come back and fix this once you've tested it against a
        # real object placed directly in front of the robot.

        center_index = 0

        start_index = max(0, center_index - half_fov_indices)
        end_index = min(len(msg.ranges) - 1, center_index + half_fov_indices)

        # Task: Pull out the slice of msg.ranges from start_index to
        # end_index (inclusive), but filter out any values that are inf,
        # -inf, or nan before looking for the minimum. A value isn't a
        # valid distance reading if math.isfinite() returns False for it.

        valid_ranges = # Your code

        if not valid_ranges:
            self.get_logger().info('No valid readings in field of view.')
            return

        # Task: Find the smallest distance in valid_ranges.
        closest_distance = # Your code 

        obstacle_detected = closest_distance < self.safety_distance

        self.get_logger().info(
            f'Closest obstacle: {closest_distance:.3f}m  |  '
            f'Obstacle detected: {obstacle_detected}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3.1 — Register the Node

Open `setup.py` in the `mini_pupper_labs` package and add to `entry_points`:

```
'obstacle_detector = mini_pupper_labs.obstacle_detector:main',
```

### 3.2 — Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_labs --symlink-install
source install/setup.bash
ros2 run mini_pupper_labs obstacle_detector
```

### 3.3 — Find "Front" Empirically

With the node running, place an object directly in front of the robot — close enough to be well within `range_max`, far enough that it's clearly intentional (not touching the sensor). Watch the logged `closest_distance` value.

Right now, `center_index = 0` is just a placeholder, so the node is checking around whatever `ranges[0]` happens to point at — which may or may not be the front of the robot. Move the test object around the robot (front, side, back) while watching the log, and figure out which index range actually corresponds to "directly in front."

**Task (back in the code):** Once you've identified the correct front-facing index, replace `center_index = 0` with the right value.

**Task 4:** Describe how you determined which index corresponds to "front." What did you observe as you moved the test object around the robot?

**Task 5:** With `center_index` corrected, place an object at roughly 20cm directly in front of the robot (inside your 30cm safety distance) and screenshot the log output showing `Obstacle detected: True`. Then move it to 50cm and screenshot `Obstacle detected: False`.

---

## Integration

### Step 4 — Safety Stop While Driving

Right now the obstacle detector only logs. It doesn't actually do anything to stop the robot. Let's fix that, using the same teleop setup from Week 3.

The idea: instead of `teleop_twist_keyboard` publishing straight to `/cmd_vel`, we'll have it publish to a renamed topic, and let the obstacle detector sit in the middle — passing velocity commands through when it's safe, and zeroing them out when something's too close.

Launch teleop remapped to a different topic name:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_unsafe
```

Now extend `obstacle_detector.py` to subscribe to `/cmd_vel_unsafe` and republish to `/cmd_vel`:

```python
from geometry_msgs.msg import Twist

# Add to __init__:
self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
self.cmd_vel_sub = self.create_subscription(
    Twist, '/cmd_vel_unsafe', self.cmd_vel_callback, 10
)
self.obstacle_detected = False  # updated by scan_callback

# Task: In scan_callback, instead of just logging obstacle_detected,
# store it on self so cmd_vel_callback can check it:
# self.obstacle_detected = obstacle_detected

def cmd_vel_callback(self, msg: Twist):
    # Task: If self.obstacle_detected is True AND the robot is trying to
    # move FORWARD (msg.linear.x > 0), publish a zeroed-out Twist instead
    # of passing the command through. Turning in place or backing away
    # should still be allowed even with an obstacle ahead — only forward
    # motion into the obstacle needs to be blocked.
    # Hint: you'll need to construct a new Twist() with all fields at 0
    # for the "blocked" case, and just republish msg unchanged otherwise.
    
    pass  # Your code
```

Rebuild, run the obstacle detector, and drive with teleop toward an object. The robot should refuse to move forward once it's within your safety distance, but should still let you back up or turn.

**Task 6: Video of driving toward an object with teleop and show the safety stop engaging. Then show that backing away and turning still work while the obstacle is detected.

---

### Step 5 - 360° View

Your current field of view only looks at a forward-facing cone. Extend the node to scan the *entire* 360° range and report the closest obstacle anywhere around the robot, along with which general direction it's in (front/back/left/right). This is a step toward the kind of full-surroundings awareness Nav2's costmaps will use starting in Week 8.

**Task 7:** Demonstrate your 360° proximity detector and describe how you determined direction (front/back/left/right) from the scan data.

---

## Tasks

1. `ros2 topic hz /scan` output and comparison to the LD19's 10 Hz spec (Step 1).
2. LaserScan field explanation + point-count calculation vs. actual `len(ranges)` (Step 2).
3. Completed `obstacle_detector.py` with working FOV/filtering/minimum-distance logic (Step 3).
4. Description of how you empirically determined the "front" index (Step 3.3).
5. Screenshots showing `Obstacle detected: True` at 20cm and `False` at 50cm (Step 3.3).
6. Safety-stop demo showing forward motion blocked, but backing up/turning still allowed (Step 4).
7. 360° proximity detection stretch goal (Step 5)

---

## Troubleshooting

??? question "`ros2 topic hz /scan` shows nothing"
    Confirm bringup is actually running and the lidar is connected. Check `ros2 node list` for a node named `LD06` (yes, still named that even on LD19 hardware — see the Background note above).

??? question "ranges[] contains `inf` values and my min() call crashes"
    That's expected — `inf` means no valid reflection at that angle. Make sure you're filtering with `math.isfinite()` *before* taking the minimum, not after.

??? question "closest_distance never changes no matter where I put the object"
    Double check `center_index` and your FOV index math — it's easy to end up looking at the wrong slice of the array. Try printing `msg.ranges[0]`, `msg.ranges[len(msg.ranges)//4]`, etc. individually while moving an object around, to map out the array by hand before trusting your FOV slice.

??? question "Forward motion isn't actually being blocked in Step 4"
    Check that `teleop_twist_keyboard` is really publishing to `/cmd_vel_unsafe` and not `/cmd_vel` — use `ros2 topic list` to confirm both topics exist, and `ros2 topic echo /cmd_vel_unsafe` to confirm teleop's output is landing there.

??? question "Robot won't move at all, even with no obstacle"
    Check that `cmd_vel_callback` actually republishes the original `msg` unchanged in the "no obstacle" case — it's easy to accidentally fall through without publishing anything.

---
