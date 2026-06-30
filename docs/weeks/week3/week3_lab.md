# Week 3 — Teleoperation & Gait

---

**Objectives:**

1. Drive the Mini Pupper 2 with keyboard teleop and understand what `/cmd_vel` actually carries.
2. Explain what the Stanford Quadruped controller does between receiving `/cmd_vel` and producing a walking gait.
3. Read and interpret a TF (transform) tree — frames, parent/child relationships, and live inspection tools.
4. Write a ROS2 node that uses TF to track the robot's position over time.

---

**Reference Material:**

- [teleop_twist_keyboard (ROS2 package)](https://github.com/ros2/teleop_twist_keyboard)
- [tf2 documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Stanford Quadruped controller (GitHub)](https://github.com/stanfordroboticsclub/StanfordQuadruped)
- Week 2 Lab — Nodes, Topics & cmd_vel (you'll reuse your `/cmd_vel` publisher knowledge here)

---

## Before You Start

Make sure your robot is up and bringup is running. If you need a refresher on starting things up, check the Verification Checklist from Week 1.

You should have `/cmd_vel`, `/odom`, and `/tf` all live before continuing.

```bash
ros2 topic list
```

---

## Background

Based on what we did last week, we got `/cmd_vel` working and a `geometry_msg/Twist` message carrying linear and angular velocity. For the Mini Pupper 2, only `linear.x` (forward/back) and `angular.z` (turning) actually matter as the robot can't fly (obvioulsy), so everything else gets ignored.

What were trying to do now is figure out what happens after the message gets published. Since the Pupper doesnt have whell it needs to have something that can cordinate commands into that leg gait. THis is where we can use the Stanfor Quadruped controller. This is a python based gait contrller that runs on the robot and subscribes to `/cmd_vel` and at every control cycle will:

1. Pick a gait pattern (the default is a trot, diagonal leg pairs move together)
2. Generate a foot trajectory for each leg based on the requested velocity
3. Solve inverse kinematics to convert each foot position into joint angles
4. Publishe those joint angles to the servo controllers

The gate itself is configured in a python file - `stanford_controller/Config.py` - which sets things like swing height, step time, and how many legs can be in the air at once. 

There is also TF, ROS2's transform tree. When the Stanford controller or any other node needs to know where the foot is relative to the body, or where the the robot is relative to the map, it doesnt recompute that chain from scratch every time. TF maintains a constantly updated tree of coordinate frames, where every frame knows its position relative to its parent. Any node can ask "Where is X relative to Y at this moment" whithout caring how many frames sit inbetween.

!!! note "Frames you'll see on the Mini Pupper 2"
    - `map` — fixed world frame (only exists once SLAM is running, Week 7)
    - `odom` — the robot's estimate of its own movement since startup; drifts over time
    - `base_link` — the robot's body, the frame everything else attaches to
    - Leg frames (`rf1`, `rf2`, `rf3`, `rffoot`, and the equivalent for the other three legs), `lidar_link`, `camera_link` — rigidly attached to `base_link` at fixed or moving offsets

---

## Driving the Robot

### Step 1 — Install and Launch Keyboard Teleop

If it's not already installed:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

Launch it with bringup running:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You'll get a terminal UI showing the key bindings. The basics: `i` forward, `,` backward, `j`/`l` turn left/right, `k` stop. Drive the robot around for a minute and get a feel for it.

### Step 2 — Watch What You're Actually Sending

Open a second terminal and watch the topic while you drive:

```bash
ros2 topic echo /cmd_vel
```

**Task 1:** Drive forward, then turn, then stop, while watching the echoed topic. Answer in your writeup:

- When you press `i`, what value shows up in `linear.x`? What about `,`?
- When you press `j` or `l`, which field changes, and is the sign what you'd expect for "turn left" vs "turn right"?
- When you press `k` to stop, does the topic stop publishing entirely, or does it keep publishing with everything zeroed out?

---

## Reading the TF Tree

### Step 3 — View It Live in RViz2

Launch RViz2 with the config from Week 1:

```bash
ros2 launch mini_pupper_description rviz.launch.py
```

Add a TF display if it isn't already there (Add, By Display Type, TF). You'll see the frame tree as colored axes, with `base_link` in the middle and everything else branching off it. Drive the robot with teleop while watching RViz — `base_link` moves, but the relationship between `base_link` and the leg/sensor frames stays fixed, since they're rigidly attached.

### Step 4 — Generate a Static Diagram and Query a Transform

RViz is good for watching live, but for understanding the tree structure, generate a static diagram:

```bash
ros2 run tf2_tools view_frames
xdg-open frames_*.pdf
```

This creates a PDF showing every frame, its parent, and its broadcast rate.

You can also ask TF directly for the relationship between any two frames:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

This prints the translation and rotation of `base_link` relative to `odom`, updating continuously.

**Task 2:** Sketch (or describe) the parent-child relationship for three frames of your choosing from the `view_frames` diagram. Then drive forward a short distance while watching `tf2_echo odom base_link`, and describe what changes.

---

## Building a Position Tracker

### Step 5 — Write a TF-Based Position Tracker Node

Now we're going to build a node that uses TF to track the robot's position over time and prints it out.

Create the file:

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/position_tracker.py
```

Fill in the starter code below. `# Task` marks what you need to write.

```python
#!/usr/bin/env python3
"""
position_tracker.py

Uses TF2 to look up the robot's position (base_link relative to odom)
at a fixed rate and prints how far it has traveled from its starting point.
"""

import math
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class PositionTrackerNode(Node):

    def __init__(self):
        super().__init__('position_tracker')

        # TF2 needs a Buffer to store incoming transforms and a
        # TransformListener to subscribe to /tf and /tf_static and fill it.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # We'll record the first position we see and treat it as the origin
        self.start_x = None
        self.start_y = None

        # Task: Create a timer that calls self.timer_callback every 0.5 seconds.
        # Hint: self.create_timer(period_in_seconds, callback_function)
        self.timer = # YOUR CODE HERE

        self.get_logger().info('PositionTrackerNode started!')

    def timer_callback(self):
        try:
            # Task: Look up the transform from 'odom' to 'base_link'.
            # Hint: self.tf_buffer.lookup_transform(target_frame, source_frame, time)
            # Pass rclpy.time.Time() as the time argument to mean "latest available"
            transform = # YOUR CODE HERE

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform: {e}')
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.get_logger().info(f'Starting position recorded: ({x:.3f}, {y:.3f})')
            return

        # Task: Compute the straight-line distance traveled from the start
        # position using x, y, self.start_x, and self.start_y.
        # Hint: standard distance formula — sqrt of the sum of squared
        # differences.
        distance = # YOUR CODE HERE

        self.get_logger().info(
            f'Position: ({x:.3f}, {y:.3f})  |  Distance from start: {distance:.3f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PositionTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register the node in `setup.py` under `entry_points`:

```
'position_tracker = mini_pupper_labs.position_tracker:main',
```

Build and run:

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_labs --symlink-install
source install/setup.bash
ros2 run mini_pupper_labs position_tracker
```

With the node running, drive the robot with teleop in another terminal and watch the distance value climb.

**Task 3:** With the node running, wait a few seconds without driving at all. Does the printed distance stay perfectly at 0.000, or does it drift slightly? What does that tell you about `/odom` as a sensor?

**Task 4:** The `lookup_transform` call is wrapped in a `try/except`. Why might a transform lookup fail right when the node first starts up, even though everything is working correctly?

---

## Looking Ahead: Why Odometry Isn't Enough

Drive the robot in a square — forward, turn 90°, repeat four times — trying to land back exactly where you started. Watch your `position_tracker` node's reported distance-from-start as you do it.

**Task 5:** Compare the `position_tracker` node's reported distance-from-start at the end to your best estimate of where you *actually* ended up (eyeball it against a tape measure or floor tile on the real robot, or compare to a ground-truth pose topic in simulation).

---

## Tasks

1. Teleop observations: `linear.x`/`angular.z` sign conventions and stop behavior (Step 2).
2. TF tree sketch (three frames) and `tf2_echo` observation while driving (Step 4).
3. Drift-at-rest observation and explanation (Step 5).
4. Explanation of why `lookup_transform` can fail on startup (Step 5).
5. Square-drive odometry drift comparison (Looking Ahead).

---

## Troubleshooting

??? question "`teleop_twist_keyboard` command not found"
    Confirm the install completed: `sudo apt install ros-humble-teleop-twist-keyboard`. If you're in a sourced workspace, make sure you haven't shadowed the system package — check `ros2 pkg list | grep teleop`.

??? question "RViz shows no TF frames at all"
    Confirm bringup is actually running and `robot_state_publisher` is active: `ros2 node list | grep robot_state_publisher`. No `robot_state_publisher` means no `/tf_static` for the rigid frames.

??? question "`position_tracker` immediately throws `LookupException` and never recovers"
    Check that `/tf` and `/tf_static` are both actually publishing: `ros2 topic hz /tf`. If nothing's there, bringup likely isn't fully up yet — restart it and wait a few seconds before launching the node.

??? question "Distance from start jumps to a huge, nonsensical number"
    This usually means the transform briefly returned a stale or extrapolated value. Confirm you're passing `rclpy.time.Time()` (latest available) rather than a fixed past timestamp to `lookup_transform`.

---
