# Week 8 — Nav2, AMCL, Costmaps & Waypoint Following

---

**Objectives:**

1. Explain Nav2's architecture: how `map_server`, AMCL, costmaps, a global planner, and a local controller fit together.
2. Launch Nav2 with your Week 7 map and localize the robot within it.
3. Inspect costmap layers and AMCL's particle cloud directly in RViz.
4. Send navigation goals via RViz and observe path planning and execution end-to-end.
5. Write a Python script using Nav2's Simple Commander API to follow a sequence of waypoints autonomously.
6. Investigate why this robot's navigation pipeline scales Nav2's velocity output before it reaches `/cmd_vel`.

---

**Reference Material:**

- [Nav2 documentation](https://docs.nav2.org/)
- [Simple Commander API](https://docs.nav2.org/commander_api/index.html)
- [mini_pupper_navigation package (GitHub)](https://github.com/mangdangroboticsclub/mini_pupper_ros/tree/ros2-dev/mini_pupper_navigation)
- Week 7 Lab — Cartographer & Map Save/Load (the map you saved there is this week's input)

---

## Background

Nav2 is a pipeline, not a single node. Here's the chain, in order:

1. **`map_server`** loads your saved `map.yaml` from Week 7 — the frozen, static map.
2. **AMCL** (Adaptive Monte Carlo Localization) figures out where the robot actually is *within* that map, using a particle filter: thousands of pose hypotheses, each scored against how well the live lidar scan matches the map from that hypothetical position. Hypotheses that don't match the scan get pruned; ones that do survive and multiply.
3. **Costmaps** layer obstacle information on top of the map — a *global* costmap (the static map, inflated outward from walls for safety margin) and a *local* costmap (a small rolling window centered on the robot, built from live sensor data, that reacts to things the static map doesn't know about).
4. A **global planner** computes a path across the global costmap from the robot's current pose to the goal.
5. A **local controller** executes that path moment-to-moment, adjusting around whatever the local costmap sees that the global plan didn't account for.

This course's actual configuration (`mini_pupper_navigation/param/real_table.yaml`):

| Parameter | Value | Notes |
|---|---|---|
| AMCL `laser_model_type` | `likelihood_field` | How particle weights are scored against lidar scans |
| AMCL `min_particles` / `max_particles` | 1000 / 3000 | Particle filter size |
| DWB `controller_frequency` | 20.0 Hz | How often the local controller re-evaluates |
| DWB `max_vel_x` / `max_vel_theta` | 0.20 m/s / 0.50 rad/s | Configured velocity limits |
| Costmap `footprint` | `[[-0.105,-0.055], [0.105,-0.055], [0.105,0.055], [-0.105,0.055]]` | A ~21×11cm rectangle, not a circular radius |
| Costmap `resolution` | 0.02 m | Finer than the ~0.05m map resolution from Week 7 — costmaps can resolve finer than the static map they're built on |

!!! note "Why a rectangular footprint, not a circle?"
    Many Nav2 tutorials use a single `robot_radius` value, which treats the robot as a circle for collision checking. This config uses an explicit `footprint` polygon instead — a better fit for a quadruped body that's clearly longer than it is wide.

!!! warning "Nav2's velocity output isn't what actually reaches the legs"
    Look at `mini_pupper_navigation/launch/navigation_smacplanner.launch.py`: Nav2's `cmd_vel` output is deliberately remapped to `/cmd_vel_navigation2`, and a separate node, `nav_vel_scaler` (in `mini_pupper_driver`), subscribes to that and republishes to the real `/cmd_vel` — scaled by **1.7× on `linear.x`, 2.0× on `linear.y`, and 2.0× on `angular.z`**. So if DWB decides to command 0.20 m/s forward (its configured `max_vel_x`), the robot actually receives a command for 0.34 m/s. You'll investigate this directly in Step 6.

---

## Localizing the Robot

### Step 1 — Launch Nav2 With Your Map

With bringup running on the robot, launch Nav2 on your PC using the map you saved last week:

```bash
ros2 launch mini_pupper_navigation navigation_smacplanner.launch.py map:=$HOME/map.yaml
```

For the best chance at automatic localization, start the robot at roughly the same physical position and orientation where you began SLAM mapping in Week 7.

### Step 2 — Check (and Fix) AMCL's Localization

In RViz, look for the particle cloud display (a scatter of small arrows around the robot). Each arrow is one pose hypothesis. A tight, converged cluster means AMCL is confident about the robot's position; a wide scatter means it isn't.

If the cloud looks scattered or clearly wrong, don't just hope it sorts itself out — use the **2D Pose Estimate** tool in RViz's toolbar: click it, then click-and-drag on the map at the robot's actual position and facing direction. This gives AMCL a strong initial guess to converge around.

**Task 1:** Screenshot the particle cloud right after launch, and again after it's converged (driving the robot a short distance with teleop usually helps it converge faster, since new scans give AMCL more to match against). Describe the visible difference.

---

## Costmaps

### Step 3 — Compare Global and Local Costmaps

In RViz, toggle the Global Costmap and Local Costmap displays on and off independently. The global costmap is the static map, inflated outward near walls — it won't change no matter what you put in front of the robot temporarily. The local costmap is a small rolling window of live sensor data — it *will* react.

Place a temporary obstacle (a book, your hand) somewhere in front of the robot that isn't part of the original mapped environment.

**Task 2:** Screenshot the local costmap reacting to your temporary obstacle, and confirm the global costmap does *not* show it. Why does Nav2 need both, rather than just live obstacle data alone?

---

## Sending Goals

### Step 4 — Send a Nav2 Goal

Use the **Nav2 Goal** tool in RViz's toolbar: click it, then click-and-drag at a destination within your mapped space. Watch the global path appear (a line from robot to goal) and the robot execute it.

**Task 3:** Record or screenshot a successful goal execution. Did the robot's actual path ever deviate from the originally-planned global path line? If so, describe what it was avoiding and why the local controller made that choice instead of just following the global plan exactly.

---

## Waypoint Following

### Step 5 — Write a Waypoint Patrol Script

RViz's goal tool only sends one goal at a time. To chain several together autonomously, you'll use Nav2's Simple Commander API — a Python library that wraps the action-server calls Nav2 expects.

Create the file:

```bash
touch ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/waypoint_patrol.py
```

Fill in the starter code. `# TODO` marks what you need to write.

```python
#!/usr/bin/env python3
"""
waypoint_patrol.py

Sends the robot through a fixed sequence of waypoints using Nav2's
Simple Commander API, then reports whether the route succeeded.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator, x, y, yaw_w=1.0, yaw_z=0.0):
    """Build a PoseStamped in the map frame from an (x, y) position."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = yaw_z
    pose.pose.orientation.w = yaw_w
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # TODO: Set this to the robot's actual starting pose in the map frame —
    # the same pose you started SLAM mapping from in Week 7.
    # Hint: make_pose(navigator, x, y)
    initial_pose = # YOUR CODE HERE
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # TODO: Build a list of at least 3 waypoints as PoseStamped objects
    # using make_pose(navigator, x, y). Pick coordinates inside your
    # mapped space that are actually reachable — check your map.pgm
    # from Week 7 before picking values, so you're not aiming at a wall.
    waypoints = # YOUR CODE HERE

    navigator.followWaypoints(waypoints)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            navigator.get_logger().info(
                f'Executing waypoint {feedback.current_waypoint + 1}/{len(waypoints)}'
            )

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Waypoint route complete!')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('Route was canceled.')
    elif result == TaskResult.FAILED:
        navigator.get_logger().info('Route failed.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register the node in `setup.py`:

```
'waypoint_patrol = mini_pupper_labs.waypoint_patrol:main',
```

Build and run it **in a separate terminal from Nav2** — this script doesn't launch Nav2 itself, it just sends commands to a Nav2 stack that's already running from Step 1:

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_labs --symlink-install
source install/setup.bash
ros2 run mini_pupper_labs waypoint_patrol
```

**Task 4:** Video or description of the robot completing your waypoint route. Did it succeed, get canceled, or fail? If it failed partway, what happened?

---

## Investigating the Velocity Scaler

### Step 6 — Confirm the Scaling Factors

With Nav2 actively navigating (either from Step 4 or Step 5), open two terminals and watch both sides of the scaler:

```bash
ros2 topic echo /cmd_vel_navigation2
ros2 topic echo /cmd_vel
```

**Task 5:** Compare a few matched pairs of values between the two topics. Do they match the 1.7×/2.0×/2.0× factors described in the Background? Then answer: DWB is configured with `max_vel_x: 0.20` — presumably a deliberately conservative limit for safe planning around the costmap's safety margins. If the *actual* command reaching the robot is scaled up to as much as 0.34 m/s, what risk does that introduce near obstacles that the costmap's inflation radius was tuned assuming the unscaled speed?

---

## Looking Ahead

You can now get the robot from one place to another autonomously. What you can't do yet is have it recognize *what's* at the destination — that's Week 9, using the OAK-D Lite camera and YOLOv8.

**DELIVERABLE:** In your lab writeup, briefly describe one limitation of this week's setup that bothered you while testing — for example, the requirement to start in the exact same pose as SLAM mapping, or the fixed/frozen map not updating as the room changes. Pick one, and propose (in a few sentences, no need to implement it) what would need to change in the pipeline to address it.

---

## Tasks

1. Particle cloud before/after convergence, with description (Step 2).
2. Local costmap reacting to a temporary obstacle the global costmap doesn't show (Step 3).
3. Successful Nav2 goal execution, with explanation of any path deviation (Step 4).
4. Completed `waypoint_patrol.py` and a recording/description of the route result (Step 5).
5. Velocity scaling verification and the inflation-radius risk discussion (Step 6).
6. Looking Ahead: one limitation and a proposed fix (Looking Ahead).

---

## Troubleshooting

??? question "Particle cloud never converges, robot seems 'lost'"
    Use the 2D Pose Estimate tool to manually correct it — don't assume it will sort itself out. Make sure you're actually starting from close to the same physical position as your Week 7 SLAM run.

??? question "Nav2 Goal is rejected, or the robot just doesn't move"
    Check that the goal and the robot's current position aren't too close to inflated costmap obstacles for the footprint to fit through. Also confirm Nav2's lifecycle nodes are active: `ros2 lifecycle get /amcl` (should report `active`).

??? question "`waypoint_patrol.py` hangs forever at `waitUntilNav2Active()`"
    This script doesn't start Nav2 — it only talks to a Nav2 stack that's already running. Confirm Step 1's launch command is running in a separate terminal first.

??? question "Robot drives through costmap obstacles like they're not there"
    Confirm scan data is actually reaching the costmap, not just being published — reuse your `ros2 topic hz /scan` check from Week 6/7. A costmap with no scan data to consume can't inflate anything.

---
