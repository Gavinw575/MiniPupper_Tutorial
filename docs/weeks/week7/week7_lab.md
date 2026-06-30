# Week 7 — Cartographer, Occupancy Grid, Nav2 & Waypoint Following

---

**Objectives:**

1. Explain how SLAM corrects for the odometry drift you observed in Week 3, using lidar scan matching.
2. Launch Cartographer and build a live occupancy grid map of a real environment.
3. Understand Cartographer's two-node architecture and why a separate node converts internal submaps into a `nav_msgs/OccupancyGrid`.
4. Inspect the actual `/map` topic and verify it against what the launch configuration claims to produce.
5. Save a completed map to disk and verify the saved files.
6. Explain Nav2's architecture: how `map_server`, AMCL, costmaps, a global planner, and a local controller fit together.
7. Launch Nav2 with your saved map and localize the robot within it.
8. Inspect costmap layers and AMCL's particle cloud directly in RViz.
9. Send navigation goals via RViz and observe path planning and execution end-to-end.
10. Write a Python script using Nav2's Simple Commander API to follow a sequence of waypoints autonomously.
11. Investigate why this robot's navigation pipeline scales Nav2's velocity output before it reaches `/cmd_vel`.

---

**Reference Material:**

- [Cartographer ROS documentation](https://google-cartographer-ros.readthedocs.io/)
- [nav_msgs/OccupancyGrid message definition](https://docs.ros2.org/latest/api/nav_msgs/msg/OccupancyGrid.html)
- [mini_pupper_slam package (GitHub)](https://github.com/mangdangroboticsclub/mini_pupper_ros/tree/ros2-dev/mini_pupper_slam)
- [Nav2 documentation](https://docs.nav2.org/)
- [Simple Commander API](https://docs.nav2.org/commander_api/index.html)
- [mini_pupper_navigation package (GitHub)](https://github.com/mangdangroboticsclub/mini_pupper_ros/tree/ros2-dev/mini_pupper_navigation)
- Week 3 Lab — Teleop, RViz2 & TF Tree (odometry drift — this week solves that problem)
- Week 6 Lab — LD19 Driver & LaserScan (Cartographer consumes the same `/scan` data you inspected there)

---

## Background

In Week 3 you drove a square and watched your `position_tracker` node's reported distance-from-start fail to return to zero. That gap was odometry drift — `/odom` alone has no way to correct itself, because it only ever integrates small errors forward in time.

SLAM (Simultaneous Localization and Mapping) fixes this by bringing the lidar into the loop. As the robot moves, Cartographer matches each new lidar scan against the map it's built so far. When the robot revisits a place it's seen before, the scan match reveals exactly how far the odometry estimate has drifted and Cartographer corrects for it.

Cartographer is split into two ROS2 nodes with different jobs:

| Node | Job |
|---|---|
| `cartographer_node` | The actual SLAM: scan matching, loop closure, pose graph optimization. Builds an internal map representation called *submaps*. |
| `cartographer_occupancy_grid_node` | Listens to those submaps and converts them into a `nav_msgs/OccupancyGrid` — the grid you actually see in RViz and the format the rest of the navigation stack expects. |

!!! note "Why two nodes?"
    Cartographer's own documentation notes that generating the occupancy grid from submaps is comparatively expensive, so map updates publish on the order of seconds, not every scan. This is by design — if your map in RViz seems to "catch up" in chunks rather than updating instantly, that's expected, not a bug.

Looking at `slam.lua` config, a few choices are worth understanding rather than just copying:

| Parameter | Value | Why it matters |
|---|---|---|
| `map_frame` | `"map"` | The fixed frame the whole map lives in |
| `tracking_frame` | `"imu_link"` | The frame Cartographer tracks the robot's pose "as" |
| `published_frame` | `"odom"` | The frame Cartographer publishes corrections relative to |
| `use_odometry` | `true` | Cartographer fuses your `/odom` estimate with scan matching |
| `use_imu_data` | `false` | The IMU is *not* used for scan matching on this robot |
| `min_range` / `max_range` | `0.02` / `12` | Matches the LD19's actual spec range exactly (Week 6) |

!!! warning "Run this on your PC, not the robot"
    `cartographer_node` is computationally heavy — scan matching and pose graph optimization are real-time optimization problems, not lightweight callbacks. The CM4 is already busy running bringup and servo control; the whole architecture assumes `/scan`, `/odom`, `/tf`, and `/imu/data` get streamed over the network to a separate machine running Cartographer.

---

## Setup

### Step 1 — Confirm Data Is Actually Flowing

Confirm that topics exist and are carrying data

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /tf
ros2 topic hz /imu/data
```

If any of these show nothing despite `ros2 topic list` listing them, that's a discovery-without-transport problem. Try `ros2 daemon stop && ros2 daemon start` first; if that doesn't fix it, it's likely a multicast/DDS configuration or wifi issue.

**Task 1:** Paste the `ros2 topic hz` output for all four topics above.

### Step 2 — Launch Cartographer

On your PC (not the robot), with bringup running on robot:

```bash
ros2 launch mini_pupper_slam slam.launch.py
```

This brings up `cartographer_node`, `cartographer_occupancy_grid_node`, and RViz with a SLAM-specific config, all in one launch file.

---

## Building the Map

### Step 3 — Drive and Watch the Map Build

With Cartographer running, drive the robot around with `teleop_twist_keyboard` (Week 3) and watch RViz. Try to cover the full space you want mapped including driving back through areas you've already seen, since that's what gives Cartographer loop-closure opportunities to correct drift.

**Task 2:** Screenshot the map after about 30 seconds of driving, and again once you've covered the full space. Describe what changed between the two — not just "more area covered," but anything that looks like it got *corrected* (walls that straightened up, edges that snapped into alignment).

---

## Investigating the Occupancy Grid

### Step 4 — Inspect the `/map` Message

```bash
ros2 interface show nav_msgs/msg/OccupancyGrid
ros2 topic echo /map --once
```

Look specifically at the `info` field — `resolution`, `width`, `height`, and `origin`.

**Task 3:** In your own words, explain what `resolution` means for this message, and what a single value in the `data` array represents.

### Step 5 — Verify the Launch File's Claims

Look at `mini_pupper_slam/launch/slam.launch.py`. The `cartographer_occupancy_grid_node` is configured like this:

```python
parameters=[
    {'use_sim_time': use_sim_time},
    {'-resolution': '0.05'},
    {'-publish_period_sec': '1.0'}
]
```

`-resolution` and `-publish_period_sec` are written as if they're command-line flags, but they're placed inside `parameters=[...]`, which is how you declare ROS2 parameters and parameter names don't normally start with a dash. Command-line-style flags for a node usually belong in a separate `arguments=[...]` list instead.

**Task 4:** Compare the `info.resolution` value from your Step 4 echo against the `0.05` the launch file claims to be setting. Do they match? If they match, is that necessarily proof the parameter is being applied, or could it just be the node's own default? What would you change in the launch file to pass this as an actual command-line argument instead?

---

## Saving the Map

### Step 6 — Save and Verify

Once you're happy with the map, save it from your PC:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

!!! warning "If this hangs or saves an empty map"
    This is a real gotcha from past runs of this course: `map_saver_cli` needs to subscribe to `/map` with the same QoS durability Cartographer publishes it with. If the basic command above doesn't work, add the matching QoS override explicitly:
    ```bash
    ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p map_subscribe_transient_local:=true
    ```

This produces two files in your home directory: `map.pgm` (the actual image of the occupancy grid) and `map.yaml` (metadata — resolution, origin, and a pointer to the `.pgm` file).

**Task 5:** Open `map.yaml` in a text editor and paste its contents. Match each field to what you saw in the `/map` message's `info` field back in Step 4 — which `map.yaml` field corresponds to which `OccupancyGrid.info` field?

**Task 6:** View `map.pgm` as an image (most image viewers open `.pgm` directly, or convert it: `convert map.pgm map.png`). Does the shape of the mapped space match what you actually drove through?

---

## Looking Ahead

You've now closed the loop on Week 3's odometry drift problem — Cartographer's scan matching corrects exactly the kind of error your `position_tracker` node exposed. Up next: Nav2 picks up `map.yaml` directly — its `map_server` loads it back in, AMCL localizes the robot within it, and costmaps get built on top of it for path planning.

**DELIVERABLE:** Answer the following in your lab writeup:

1. The robot has a working IMU, but `use_imu_data` is set to `false` in `slam.lua` for this setup. Look at the official Cartographer documentation's discussion of when IMU data helps 2D SLAM versus 3D SLAM. Based on that, propose a plausible reason this robot's config disables it for 2D scan matching, and what you'd want to check before turning it on.
2. Once Nav2 has loaded your saved map and the robot is localized within it, the map itself doesn't change anymore — it's frozen. What would have to be different about the SLAM pipeline if you wanted the robot to keep updating the map *while* also navigating autonomously?

---

## Background

Nav2 is a pipeline, not a single node. Here's the chain, in order:

1. **`map_server`** loads your saved `map.yaml` from above — the frozen, static map.
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
| Costmap `resolution` | 0.02 m | Finer than the ~0.05m map resolution from above — costmaps can resolve finer than the static map they're built on |

!!! note "Why a rectangular footprint, not a circle?"
    Many Nav2 tutorials use a single `robot_radius` value, which treats the robot as a circle for collision checking. This config uses an explicit `footprint` polygon instead — a better fit for a quadruped body that's clearly longer than it is wide.

!!! warning "Nav2's velocity output isn't what actually reaches the legs"
    Look at `mini_pupper_navigation/launch/navigation_smacplanner.launch.py`: Nav2's `cmd_vel` output is deliberately remapped to `/cmd_vel_navigation2`, and a separate node, `nav_vel_scaler` (in `mini_pupper_driver`), subscribes to that and republishes to the real `/cmd_vel` — scaled by **1.7× on `linear.x`, 2.0× on `linear.y`, and 2.0× on `angular.z`**. So if DWB decides to command 0.20 m/s forward (its configured `max_vel_x`), the robot actually receives a command for 0.34 m/s. You'll investigate this directly in Step 12.

---

## Localizing the Robot

### Step 7 — Launch Nav2 With Your Map

With bringup running on the robot, launch Nav2 on your PC using the map you saved:

```bash
ros2 launch mini_pupper_navigation navigation_smacplanner.launch.py map:=$HOME/map.yaml
```

For the best chance at automatic localization, start the robot at roughly the same physical position and orientation where you began SLAM mapping above.

### Step 8 — Check (and Fix) AMCL's Localization

In RViz, look for the particle cloud display (a scatter of small arrows around the robot). Each arrow is one pose hypothesis. A tight, converged cluster means AMCL is confident about the robot's position; a wide scatter means it isn't.

If the cloud looks scattered or wrong, use the 2D Pose Estimate tool in RViz's toolbar: click it, then click-and-drag on the map at the robot's actual position and facing direction. This gives AMCL a strong initial guess to converge around.

**Task 8:** Screenshot the particle cloud right after launch, and again after it's converged (driving the robot a short distance with teleop usually helps it converge faster, since new scans give AMCL more to match against).

---

## Costmaps

### Step 9 — Compare Global and Local Costmaps

In RViz, toggle the Global Costmap and Local Costmap displays on and off independently. The global costmap is the static map, inflated outward near walls — it won't change no matter what you put in front of the robot temporarily. The local costmap is a small rolling window of live sensor data, it will react.

Place a temporary obstacle (a book, your hand) somewhere in front of the robot that isn't part of the original mapped environment.

**Task 9:** Screenshot the local costmap reacting to your temporary obstacle, and confirm the global costmap does not show it. Why does Nav2 need both, rather than just live obstacle data alone?

---

## Sending Goals

### Step 10 — Send a Nav2 Goal

Use the Nav2 Goal tool in RViz's toolbar: click it, then click-and-drag at a destination within your mapped space. Watch the global path appear (a line from robot to goal) and the robot execute it.

**Task 10:** Record or screenshot a successful goal execution. Did the robot's actual path ever deviate from the originally-planned global path line? If so, describe what it was avoiding and why the local controller made that choice instead of just following the global plan exactly.

---

## Waypoint Following

### Step 11 — Write a Waypoint Patrol Script

RViz's goal tool only sends one goal at a time. To chain several together autonomously, you'll use Nav2's Simple Commander API. A Python library that wraps the action-server calls Nav2 expects.

Create the file:

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/waypoint_patrol.py
```

Fill in the starter code.

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

    # Task: Set this to the robot's actual starting pose in the map frame —
    # the same pose you started SLAM mapping from above.
    # Hint: make_pose(navigator, x, y)

    initial_pose = # Your code
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # Task: Build a list of at least 3 waypoints as PoseStamped objects
    # using make_pose(navigator, x, y). Pick coordinates inside your
    # mapped space that are actually reachable — check your map.pgm
    # before picking values, so you're not aiming at a wall.

    waypoints = # Your code

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

Build and run it in a separate terminal from Nav2. this script doesn't launch Nav2 itself, it just sends commands to a Nav2 stack that's already running from Step 7:

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_labs --symlink-install
source install/setup.bash
ros2 run mini_pupper_labs waypoint_patrol
```

**Task 11:** Video of the robot completing your waypoint route.

---

## Investigating the Velocity Scaler

### Step 12 — Confirm the Scaling Factors

With Nav2 actively navigating (either from Step 10 or Step 11), open two terminals and watch both sides of the scaler:

```bash
ros2 topic echo /cmd_vel_navigation2
ros2 topic echo /cmd_vel
```

**Task 12:** Compare a few matched pairs of values between the two topics. Do they match the 1.7×/2.0×/2.0× factors described in the Background? Then answer: DWB is configured with `max_vel_x: 0.20` — presumably a deliberately conservative limit for safe planning around the costmap's safety margins. If the actual command reaching the robot is scaled up to as much as 0.34 m/s, what risk does that introduce near obstacles that the costmap's inflation radius was tuned assuming the unscaled speed?

---

## Looking Ahead

You can now get the robot from one place to another autonomously. What you can't do yet is have it recognize *what's* at the destination — that's Week 9, using the OAK-D Lite camera and YOLOv8.

**DELIVERABLE:** In your lab writeup, briefly describe one limitation of this week's setup that bothered you while testing — for example, the requirement to start in the exact same pose as SLAM mapping, or the fixed/frozen map not updating as the room changes. Pick one, and propose (in a few sentences, no need to implement it) what would need to change in the pipeline to address it.

---

## Tasks

1. `ros2 topic hz` output for `/scan`, `/odom`, `/tf`, `/imu/data` (Step 1).
2. Before/after map screenshots with a description of visible corrections (Step 3).
3. Explanation of `resolution` and the `data` array (Step 4).
4. Resolution verification: does the launch file's claimed `0.05` match reality, and how would you fix the launch file if not (Step 5).
5. `map.yaml` contents, mapped to the corresponding `OccupancyGrid.info` fields (Step 6).
6. `map.pgm` viewed as an image, compared against the actual driven space (Step 6).
7. Looking Ahead discussion, 2 questions (IMU / frozen map).
8. Particle cloud before/after convergence (Step 8).
9. Local costmap reacting to a temporary obstacle the global costmap doesn't show (Step 9).
10. Successful Nav2 goal execution, with explanation of any path deviation (Step 10).
11. Completed `waypoint_patrol.py` and a video of the route result (Step 11).
12. Velocity scaling verification and the inflation-radius risk discussion (Step 12).
13. Looking Ahead: one limitation and a proposed fix (Looking Ahead).

---

## Troubleshooting

??? question "`ros2 topic hz` shows nothing even though `ros2 topic list` shows the topic"
    Classic discovery-without-transport symptom. Try `ros2 daemon stop && ros2 daemon start` first. If you're running inside a VM with bridged networking, this is very likely a multicast issue — ask your instructor about the CycloneDDS configuration fix.

??? question "RViz map doesn't update no matter how far you drive"
    Confirm `cartographer_node` actually shows as a running node (`ros2 node list`), and that `/tf` includes a path from `imu_link` to `base_link` — Cartographer's `tracking_frame` is `imu_link`, so if that transform is missing, scan matching can't run at all.

??? question "`map_saver_cli` hangs and never produces output"
    This is almost always the QoS durability mismatch described in Step 6 — add `--ros-args -p map_subscribe_transient_local:=true`.

??? question "Saved map.pgm looks mostly gray with no clear walls"
    Gray means "unknown" in an occupancy grid (as opposed to black for occupied or white for free). This usually means you didn't drive close enough to enough surfaces, not a bug — try mapping again and deliberately tracing along walls.

??? question "Particle cloud never converges, robot seems 'lost'"
    Use the 2D Pose Estimate tool to manually correct it — don't assume it will sort itself out. Make sure you're actually starting from close to the same physical position as your SLAM run above.

??? question "Nav2 Goal is rejected, or the robot just doesn't move"
    Check that the goal and the robot's current position aren't too close to inflated costmap obstacles for the footprint to fit through. Also confirm Nav2's lifecycle nodes are active: `ros2 lifecycle get /amcl` (should report `active`).

??? question "`waypoint_patrol.py` hangs forever at `waitUntilNav2Active()`"
    This script doesn't start Nav2 — it only talks to a Nav2 stack that's already running. Confirm Step 7's launch command is running in a separate terminal first.

??? question "Robot drives through costmap obstacles like they're not there"
    Confirm scan data is actually reaching the costmap, not just being published — reuse your `ros2 topic hz /scan` check from Week 6/above. A costmap with no scan data to consume can't inflate anything.

---
