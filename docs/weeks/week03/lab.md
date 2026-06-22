# Lab 3 — Teleop, RViz2 & TF Tree

## Before You Start

Make sure your robot is up and bringup is running (real robot or simulation — either works for this lab). If you need a refresher on starting things up, check the Verification Checklist from Week 1.

You should have `/cmd_vel`, `/odom`, and `/tf` all live before continuing. Quick check:

```bash
ros2 topic list | grep -E "cmd_vel|odom|tf"
```

---

## Part 1 — Driving with Keyboard Teleop

So far the only way you've sent `/cmd_vel` commands is through the publisher node you wrote last week. That's great for understanding the message, but it's a clunky way to actually drive the robot around. Today you'll use `teleop_twist_keyboard`, a standard ROS2 package made exactly for this.

### 1.1 — Install and Launch

If it's not already installed:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

Launch it:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You'll get a terminal UI showing the key bindings. The basics:

- `i` — forward
- `,` — backward
- `j` / `l` — turn left / right
- `k` — stop

Drive the robot around for a minute. Get a feel for it.

### 1.2 — Watch What You're Actually Sending

Open a second terminal and watch the topic while you drive:

```bash
ros2 topic echo /cmd_vel
```

Drive forward, then turn, then stop. Answer these in your lab notes:

1. When you press `i`, what value shows up in `linear.x`? What about when you press `,`?
2. When you press `j` or `l`, which field changes — and is the sign what you expected for "turn left" vs "turn right"?
3. What gets published when you press `k` to stop? Is it that the topic stops publishing, or does it publish a message with everything zeroed out? (Hint: this distinction matters a lot for safety — think about what happens if a teleop node crashes mid-drive.)

---

## Part 2 — Reading the TF Tree

### 2.1 — View It Live in RViz2

Launch RViz2 with the config from Week 1:

```bash
ros2 launch mini_pupper_description rviz.launch.py
```

Add a **TF** display if it isn't already there (Add → By Display Type → TF). You should see the frame tree as little colored axes, with `base_link` in the middle and everything else branching off it.

Drive the robot with teleop again while watching RViz. Notice that `base_link` moves, but the relationship between `base_link` and the leg/sensor frames stays the same — they're rigidly attached. What frame *doesn't* move at all, no matter how far you drive?

### 2.2 — Generate a TF Tree Diagram

RViz is good for watching live, but for actually understanding the tree structure, generate a static diagram:

```bash
ros2 run tf2_tools view_frames
```

This creates a PDF in your current directory showing every frame, its parent, and the broadcast rate. Open it:

```bash
xdg-open frames_*.pdf
```

In your lab notes, sketch (or just describe) the parent-child relationship for three frames of your choosing — for example, "`laser` is a child of `base_link`."

### 2.3 — Query a Specific Transform

You can ask TF directly for the relationship between any two frames using `tf2_echo`:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

This prints the translation and rotation of `base_link` relative to `odom`, updating continuously. Drive forward a short distance with teleop and watch the translation values change.

Question for your notes: if you drive in a perfect square and end up back where you started, would you expect `odom → base_link` to report you're back at the origin? Why or why not? (Hint: think about what `odom` actually is — re-read the Overview if you need to.)

---

## Part 3 — Writing a TF-Based Position Tracker

Time to write some code. You're going to build a node that uses TF to track the robot's position over time and prints it out — the same kind of lookup that Nav2 will rely on heavily starting in Week 8.

### 3.1 — Set Up the File

```bash
touch ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/position_tracker.py
```

### 3.2 — The Starter Code

Copy this in. As before, `# TODO` marks what you need to fill in, with a hint above each one.

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
        # TransformListener to actually subscribe to /tf and /tf_static
        # and fill that buffer.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # We'll record the first position we see and treat it as the origin
        self.start_x = None
        self.start_y = None

        # TODO: Create a timer that calls self.timer_callback every 0.5 seconds.
        # Hint: self.create_timer(period_in_seconds, callback_function)
        self.timer = # YOUR CODE HERE

        self.get_logger().info('PositionTrackerNode started!')

    def timer_callback(self):
        try:
            # TODO: Look up the transform from 'odom' to 'base_link'.
            # Hint: self.tf_buffer.lookup_transform(target_frame, source_frame, time)
            # The third argument is "what time" — pass rclpy.time.Time() to mean "latest available"
            transform = # YOUR CODE HERE

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform: {e}')
            return

        # The transform's translation tells us where base_link is relative to odom
        x = transform.transform.translation.x
        y = transform.transform.translation.y

        if self.start_x is None:
            # First reading — this becomes our reference point
            self.start_x = x
            self.start_y = y
            self.get_logger().info(f'Starting position recorded: ({x:.3f}, {y:.3f})')
            return

        # TODO: Compute the straight-line distance traveled from the start
        # position using x, y, self.start_x, and self.start_y.
        # Hint: this is the standard distance formula — sqrt of the sum of
        # squared differences. The math module is already imported.
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

### 3.3 — Register the Node

Same as last week — open `setup.py`:

```bash
cat ~/ros2_ws/src/mini_pupper_labs/setup.py
```

Find `entry_points`, and add a line for this node:

```
'position_tracker = mini_pupper_labs.position_tracker:main',
```

### 3.4 — Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_labs --symlink-install
source install/setup.bash
ros2 run mini_pupper_labs position_tracker
```

With the node running, open another terminal and drive the robot with teleop. Watch the distance value climb as you drive, and notice it should return close to zero if you drive back to where you started.

### 3.5 — Lab Notes

Answer these:

1. What happens to the printed distance if you wait a few seconds without driving at all — does it stay perfectly at 0.000, or does it drift slightly? What does that tell you about `/odom` as a sensor?
2. The `lookup_transform` call can throw exceptions, which is why it's wrapped in a `try/except`. Why might a transform lookup fail right when the node first starts up, even though everything is working correctly?

---

## Stretch Goal (Optional)

Drive the robot in a square — forward, turn 90°, repeat four times — trying to land back exactly where you started. Compare the `position_tracker` node's reported distance-from-start at the end to your best estimate of the *actual* distance (you can eyeball this against a tape measure or floor tile on the real robot, or compare to the ground-truth pose topic in simulation).

This gap is **odometry drift**, and it's the entire reason SLAM exists — you'll come back to this exact idea in Week 7.

---

## What's Next

Week 4 picks up right where the "CHAMP is a black box" idea left off. You'll model a single leg as a 2-link system and write the forward and inverse kinematics yourself — the actual math CHAMP is doing under the hood every time you send it a `/cmd_vel` command.
