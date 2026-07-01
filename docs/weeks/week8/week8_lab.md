# Week 8 — IMU, Orientation Estimation & Tilt Safety

---

**Objectives:**

1. Understand what the Mini Pupper 2's IMU actually publishes — and what it deliberately leaves out.
2. Derive roll and pitch from raw accelerometer data and explain where that derivation breaks down.
3. Implement a complementary filter that fuses gyroscope and accelerometer data into a stable orientation estimate.
4. Publish filtered orientation as a valid `sensor_msgs/Imu` message with a real quaternion.
5. Compare your filter against `imu_filter_madgwick` from ROS2's `imu_tools` package.
6. Use filtered orientation to build a tilt-based safety node that cuts `/cmd_vel` if the robot tips past a threshold.

---

**Reference Material:**

- [sensor_msgs/Imu message definition](https://docs.ros2.org/humble/api/sensor_msgs/msg/Imu.html)
- [ROS REP-145 — Conventions for IMU Sensor Drivers](https://www.ros.org/reps/rep-0145.html)
- [imu_tools (GitHub)](https://github.com/CCNYRoboticsLab/imu_tools)
- [imu_filter_madgwick ROS2 documentation](https://github.com/CCNYRoboticsLab/imu_tools/tree/ros2/imu_filter_madgwick)
- Week 2 Lab — the `read_imu` subscriber you wrote there is the starting point for this week
- Week 7 Lab — Cartographer's `use_imu_data = false` setting is directly relevant here

---

## Background

### What the IMU actually publishes

In Week 2 you wrote a subscriber that printed raw values from `/imu/data`. Take another look at the message definition:

```bash
ros2 interface show sensor_msgs/msg/Imu
```

There are three data fields: `orientation`, `angular_velocity`, and `linear_acceleration`. The Mini Pupper 2's IMU driver publishes all three — but if you look at `imu_interface.py` in `mini_pupper_driver`, you'll find this:

```python
msg.orientation_covariance[0] = -1
msg.orientation.w = 1.0   # identity quaternion — not real data
```

Per [REP-145](https://www.ros.org/reps/rep-0145.html), setting `orientation_covariance[0] = -1` is the standard way of saying "this orientation field is not valid — ignore it." The driver sets a hardcoded identity quaternion and flags it as unknown. That means `/imu/data` is giving you raw gyroscope rates and raw accelerometer measurements, but no computed orientation. You have to fuse those two streams yourself to get a usable attitude estimate.

This is also exactly why `slam.lua` has `use_imu_data = false` — Cartographer could use IMU orientation to improve scan matching, but not if the orientation field is flagged as unavailable. Fixing this properly would require publishing a real fused quaternion. This is what you will be doing this lab.

### Two noisy sensors, one useful estimate

The gyroscope measures angular velocity (rad/s). Integrate it over time and you get an angle. The problem: any small constant offset in the gyro measurement (bias) accumulates without bound. A bias of 0.01 rad/s looks harmless, but after 60 seconds of integration it's added 0.6 rad of error. Aka the gyros drift.

The accelerometer measures specific force (m/s²), which at rest is just gravity pointing down. You can compute tilt directly from which direction gravity pulls: `roll = atan2(ay, az)`, `pitch = atan2(-ax, sqrt(ay² + az²))`. No drift. But the accelerometer can't tell the difference between gravity and any other acceleration — when the robot is walking and its body is bouncing, the accelerometer is noisy.

A complementary filter exploits the fact that these two failure modes are frequency-separated. Gyros are accurate at high frequency (fast motions) but drift at low frequency (over time). Accelerometers are accurate at low frequency (average gravity direction) but noisy at high frequency (vibration). The filter combines them:

```
angle = alpha × (angle + gyro_rate × dt) + (1 - alpha) × accel_angle
```

where `alpha` is close to 1. Most of the time you trust the gyro integration; every update you pull the estimate back a little toward what the accelerometer says gravity is pointing. This kills the drift without inheriting the noise.

!!! note "Why not just use the Madgwick filter directly?"
    You will, in Step 5. But implementing the simpler complementary filter first gives you the intuition for what any attitude filter is doing by deciding how much to trust each sensor at each moment. The Madgwick filter is a more sophisticated version of the same idea, just in quaternion space rather than Euler angles.

---

## Setup

### Step 1 — Confirm the IMU Is Live and Inspect Its Data

With bringup running:

```bash
ros2 topic hz /imu/data
```

The IMU publishes at 100 Hz — confirm you're seeing close to that.

Then capture one message and inspect the orientation covariance:

```bash
ros2 topic echo /imu/data --once
```

**Task 1:** Paste the `orientation_covariance` array from your output. What does the first value tell you, per REP-145? Are the `linear_acceleration` and `angular_velocity` fields populated with non-zero real values?

---

## Understanding Raw IMU Data

### Step 2 — Visualize Raw Accel and Gyro

Create a script that subscribes to `/imu/data` and prints roll and pitch computed directly from the accelerometer only:

```bash
touch ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/imu_raw_angles.py
```

```python
#!/usr/bin/env python3
"""
imu_raw_angles.py

Computes roll and pitch from raw accelerometer data only (no filtering)
and prints them so you can observe the noise.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuRawAnglesNode(Node):

    def __init__(self):
        super().__init__('imu_raw_angles')
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.get_logger().info('ImuRawAnglesNode started')

    def imu_callback(self, msg: Imu):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Task: Compute roll and pitch in degrees from raw accelerometer data.
        # Use the standard tilt-from-gravity formulas:
        #   roll  = atan2(ay, az)
        #   pitch = atan2(-ax, sqrt(ay*ay + az*az))
        # Convert to degrees with math.degrees().

        roll_deg  = # Your code
        pitch_deg = # Your code

        self.get_logger().info(
            f'roll={roll_deg:+7.2f} deg   pitch={pitch_deg:+7.2f} deg   '
            f'gz={math.degrees(msg.angular_velocity.z):+7.2f} deg/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuRawAnglesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register in `setup.py`:

```
'imu_raw_angles = mini_pupper_labs.imu_raw_angles:main',
```

Build and run it, then try two things: hold the robot still and observe the noise, then (slighty shake the robot or the table to see how the vibrations could corrupt the readings.

**Task 2:** With the robot standing still, what is the peak-to-peak spread (max minus min) of your roll estimate over a 10-second window? Now run teleop and drive the robot in a small circle — describe qualitatively how the accelerometer-only estimate behaves while the robot is walking vs. while it's standing still.

---

## Implementing the Complementary Filter

### Step 3 — Write `imu_filter_node.py`

`imu_filter_node.py subscribes to `/imu/data`, runs your complementary filter, and republishes a new `sensor_msgs/Imu` message on `/imu/filtered` with a real orientation quaternion.

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/imu_filter_node.py
```

```python
#!/usr/bin/env python3
"""
imu_filter_node.py

Implements a complementary filter that fuses gyroscope and accelerometer
data from /imu/data into a stable roll/pitch estimate.

Publishes:
  /imu/filtered  — sensor_msgs/Imu with a valid orientation quaternion
  /imu/rpy       — std_msgs/Float32MultiArray with [roll, pitch, yaw] in degrees
                   (yaw is gyro-integrated only — it drifts, and that's expected)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


# Complementary filter coefficient.
# 0.98 means: 98% gyro integration, 2% accel correction every update.
# Higher = smoother but slower to correct drift.
# Lower = faster correction but noisier.
ALPHA = 0.98


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """
    Convert roll, pitch, yaw (radians) to a quaternion (x, y, z, w).
    Uses the ZYX / aerospace convention.
    """
    cr = math.cos(roll  / 2)
    sr = math.sin(roll  / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw   / 2)
    sy = math.sin(yaw   / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class ImuFilterNode(Node):

    def __init__(self):
        super().__init__('imu_filter_node')

        # Filter state — Euler angles in radians
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0   # gyro-integrated only, will drift

        self.last_stamp = None   # ROS2 timestamp of the previous message

        self.sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.pub_filtered = self.create_publisher(Imu, '/imu/filtered', 10)
        self.pub_rpy = self.create_publisher(Float32MultiArray, '/imu/rpy', 10)

        self.get_logger().info('ImuFilterNode started')

    def imu_callback(self, msg: Imu):
        # --- Compute dt from message timestamps ---
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is None:
            self.last_stamp = now
            return
        dt = now - self.last_stamp
        self.last_stamp = now

        # Guard against bad dt (e.g. first message, clock jump)
        if dt <= 0.0 or dt > 0.5:
            return

        # --- Read raw sensor values ---
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x   # rad/s
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Task: Compute roll and pitch from the accelerometer alone.
        # Same formulas as Step 2.
        accel_roll  = # Your code
        accel_pitch = # Your code

        # Task: Implement the complementary filter for roll and pitch.
        #
        # The filter equation is:
        #   angle = ALPHA * (angle + gyro_rate * dt) + (1 - ALPHA) * accel_angle
        #
        # For roll:  gyro rate is gx,  accel estimate is accel_roll
        # For pitch: gyro rate is gy,  accel estimate is accel_pitch
        #
        # Update self.roll and self.pitch using this formula.

        # Your code

        # Task: Update self.yaw by integrating the gyro z-rate only.
        # There is no accelerometer correction for yaw (we have no magnetometer).
        # Just: self.yaw += gz * dt

        # Your code

        # --- Convert to quaternion and publish ---
        qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)

        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.orientation.x = qx
        filtered_msg.orientation.y = qy
        filtered_msg.orientation.z = qz
        filtered_msg.orientation.w = qw
        # Small non-zero diagonal covariance — orientation is now valid
        filtered_msg.orientation_covariance[0] = 0.01
        filtered_msg.orientation_covariance[4] = 0.01
        filtered_msg.orientation_covariance[8] = 0.01
        filtered_msg.angular_velocity = msg.angular_velocity
        filtered_msg.linear_acceleration = msg.linear_acceleration
        self.pub_filtered.publish(filtered_msg)

        rpy_msg = Float32MultiArray()
        rpy_msg.data = [
            math.degrees(self.roll),
            math.degrees(self.pitch),
            math.degrees(self.yaw),
        ]
        self.pub_rpy.publish(rpy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register in `setup.py`:

```
'imu_filter_node = mini_pupper_labs.imu_filter_node:main',
```

Build and run:

```bash
colcon build --packages-select mini_pupper_labs --symlink-install
source install/setup.bash
ros2 run mini_pupper_labs imu_filter_node &
ros2 topic echo /imu/rpy
```

Tilt the robot slowly in roll and pitch while watching the output. Then leave it sitting still for 30 seconds and watch yaw — it will drift. That's expected and important.

**Task 3:** Tilt the robot to roughly 45° in roll (pick it up and hold one side down). What does `/imu/rpy` report? Is it close to 45°? Now set `ALPHA = 0.5` and repeat — describe the difference in noise and responsiveness. Reset to `ALPHA = 0.98` before continuing.

---

## Comparing Against Madgwick

### Step 4 — Install and Run `imu_filter_madgwick`

Install `imu_tools`:

```bash
sudo apt install ros-humble-imu-tools
```

`imu_filter_madgwick` is a ROS2 node that subscribes to `/imu/data` and publishes a filtered `/imu/data_out`. Run it alongside your filter node

```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
  -p use_mag:=false \
  -p publish_tf:=false \
  -r imu/data:=/imu/data \
  -r imu/data_out:=/imu/madgwick_out
```

!!! note "use_mag:=false"
    The Mini Pupper 2 has a magnetometer (compass) accessible via the ESP32 interface, but it's not currently wired into any ROS2 node. Setting `use_mag:=false` tells Madgwick to use gyro + accel only, the same inputs your complementary filter has. This makes the comparison fair.

Now run your filter in one terminal and Madgwick in another, and echo both `/imu/rpy` and `/imu/madgwick_out`.

To extract roll/pitch/yaw from the Madgwick quaternion output, you can pipe it through this quick one-liner:

```bash
ros2 topic echo /imu/madgwick_out --field orientation
```

Or write a short subscriber (optional) that calls a quaternion-to-euler conversion to print Madgwick's angles in the same format as your `/imu/rpy`.

**Task 4:** With the robot sitting completely still, compare the noise level of `/imu/rpy` (your filter) versus Madgwick on roll and pitch over 30 seconds. Then tilt the robot quickly back and forth several times — which filter responds faster, and which is smoother? What does this tell you about the different trade-offs each makes?

---

## Tilt Safety Node

### Step 5 — Write `tilt_safety_node.py`

Now use the filtered orientation. This node watches `/imu/filtered` and immediately publishes zero velocity to `/cmd_vel` if the robot's roll or pitch exceeds a safety threshold.

```bash
touch ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/tilt_safety_node.py
```

```python
#!/usr/bin/env python3
"""
tilt_safety_node.py

Watches /imu/filtered for roll and pitch. If either exceeds TILT_THRESHOLD_DEG,
publishes a zero-velocity Twist to /cmd_vel to cut the robot's motion.

This is a software emergency stop based on the robot's own orientation estimate.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


# Degrees of tilt in roll or pitch that triggers the estop.
# 30° is aggressive enough to catch a real fall but forgiving enough
# during a normal walking gait.
TILT_THRESHOLD_DEG = 30.0

TILT_THRESHOLD_RAD = math.radians(TILT_THRESHOLD_DEG)


def quaternion_to_euler(x, y, z, w):
    """Return (roll, pitch, yaw) in radians from a unit quaternion."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))   # clamp for numerical safety
    pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class TiltSafetyNode(Node):

    def __init__(self):
        super().__init__('tilt_safety')

        self.estop_active = False

        self.sub = self.create_subscription(
            Imu, '/imu/filtered', self.imu_callback, 10
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            f'TiltSafetyNode started — threshold: ±{TILT_THRESHOLD_DEG}°'
        )

    def imu_callback(self, msg: Imu):
        ox = msg.orientation.x
        oy = msg.orientation.y
        oz = msg.orientation.z
        ow = msg.orientation.w

        # Task: Call quaternion_to_euler(ox, oy, oz, ow) to get roll and pitch.
        roll, pitch, _ = # Your code

        roll_deg  = math.degrees(roll)
        pitch_deg = math.degrees(pitch)

        # Task: Check whether abs(roll) or abs(pitch) exceeds TILT_THRESHOLD_RAD.
        # If it does:
        #   - Set self.estop_active = True
        #   - Log a warning with the actual roll/pitch values
        #   - Publish a zero Twist to /cmd_vel (Twist() with all fields at 0.0)
        # If the robot is within the safe range and self.estop_active was True:
        #   - Set self.estop_active = False
        #   - Log an info message that the emergency stop has cleared

        # Your code


def main(args=None):
    rclpy.init(args=args)
    node = TiltSafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register in `setup.py`:

```
'tilt_safety = mini_pupper_labs.tilt_safety_node:main',
```

Run the full stack: bringup on the robot, your filter node, and the tilt safety node. Then drive the robot with teleop and manually tilt it past 30° while it's mid-walk.

**Task 5:** Screenshot the tilt safety node's log output at the moment it triggers — show the roll/pitch values that triggered it and the "estop cleared" message when you set it back down. What happens to the robot's motion when the estop fires?

---

## Looking Ahead

Also worth noting: `orientation_covariance[0] = -1` in the raw IMU data is why SLAM's `use_imu_data = false` exists. Now that you've published `/imu/filtered` with a real covariance, you have everything you'd need to re-enable it in Cartographer — change `tracking_frame` to use `/imu/filtered` instead and set `use_imu_data = true` in `slam.lua`. That's left as an optional extension.

**DELIVERABLE:** The tilt safety demo from Task 5, plus a explanation of why pure gyro integration drifts and why pure accelerometer tilt estimation is noisy during motion — and how the complementary filter addresses both.

---

## Tasks

1. `orientation_covariance` array from `/imu/data` and REP-145 interpretation (Step 1).
2. Peak-to-peak roll spread at rest vs. qualitative behavior while walking (Step 2).
3. 45° tilt test result and ALPHA comparison (Step 3).
4. Noise and responsiveness comparison between your filter and Madgwick (Step 4).
5. Tilt estop demo: screenshot of log with triggering values and cleared message (Step 5).
6. Deliverable writeup: gyro drift, accel noise, complementary filter explanation.

---

## Troubleshooting

??? question "`/imu/data` is in `ros2 topic list` but shows no messages"
    Same discovery-vs-transport issue from previous weeks. Try `ros2 daemon stop && ros2 daemon start` and confirm `ROS_DOMAIN_ID=42` is set on both machines. If `ros2 topic hz /imu/data` shows nothing from the PC, the DDS transport layer isn't bridging the topic despite the node being listed.

??? question "Roll and pitch are jumping ±180° randomly"
    This is a gimbal-lock symptom — `atan2` is discontinuous near its boundary values, and if `az` is near zero (robot tilted close to 90°) the output wraps. The complementary filter inherits this from the accel-angle formula. It's expected at extreme tilt angles. For normal operating range (±60°) it won't occur.

??? question "Filter output looks correct for roll but pitch seems inverted"
    The IMU is mounted with a specific orientation inside the robot and the `imu_interface.py` driver swaps and negates some axes to account for it. If pitch is negated relative to what you expect, negate `accel_pitch` in your filter — it's a sign convention issue, not a bug in the math.

??? question "`imu_filter_madgwick_node` crashes with a parameter error"
    The parameter name changed across `imu_tools` versions. Run `ros2 param list /imu_filter_madgwick_node` once it starts and check the exact names. The key ones to set are `use_mag` (disable magnetometer) and `publish_tf` (disable TF output to avoid conflicts with bringup's TF tree).

??? question "Tilt safety fires immediately when bringup starts"
    The IMU takes about 2 seconds to calibrate on startup (200 samples at 100 Hz). During that window it publishes garbage values. Add a short startup delay to `TiltSafetyNode.__init__` — `self.create_timer(2.0, self._enable)` with a flag `self.enabled = False` — and only check tilt once `self.enabled` is True.
