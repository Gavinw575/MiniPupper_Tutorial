# Week 7 — Cartographer, Occupancy Grid & Map Save/Load

---

**Objectives:**

1. Explain how SLAM corrects for the odometry drift you observed in Week 3, using lidar scan matching.
2. Launch Cartographer and build a live occupancy grid map of a real environment.
3. Understand Cartographer's two-node architecture and why a separate node converts internal submaps into a `nav_msgs/OccupancyGrid`.
4. Inspect the actual `/map` topic and verify it against what the launch configuration claims to produce.
5. Save a completed map to disk and verify the saved files.

---

**Reference Material:**

- [Cartographer ROS documentation](https://google-cartographer-ros.readthedocs.io/)
- [nav_msgs/OccupancyGrid message definition](https://docs.ros2.org/latest/api/nav_msgs/msg/OccupancyGrid.html)
- [mini_pupper_slam package (GitHub)](https://github.com/mangdangroboticsclub/mini_pupper_ros/tree/ros2-dev/mini_pupper_slam)
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

You've now closed the loop on Week 3's odometry drift problem — Cartographer's scan matching corrects exactly the kind of error your `position_tracker` node exposed. Week 8 picks up `map.yaml` directly: Nav2's `map_server` loads it back in, AMCL localizes the robot within it, and costmaps get built on top of it for path planning.

**DELIVERABLE:** Answer the following in your lab writeup:

1. The robot has a working IMU, but `use_imu_data` is set to `false` in `slam.lua` for this setup. Look at the official Cartographer documentation's discussion of when IMU data helps 2D SLAM versus 3D SLAM. Based on that, propose a plausible reason this robot's config disables it for 2D scan matching, and what you'd want to check before turning it on.
2. Once Nav2 has loaded your saved map and the robot is localized within it, the map itself doesn't change anymore — it's frozen. What would have to be different about the SLAM pipeline if you wanted the robot to keep updating the map *while* also navigating autonomously?

---

## Tasks

1. `ros2 topic hz` output for `/scan`, `/odom`, `/tf`, `/imu/data` (Step 1).
2. Before/after map screenshots with a description of visible corrections (Step 3).
3. Explanation of `resolution` and the `data` array (Step 4).
4. Resolution verification: does the launch file's claimed `0.05` match reality, and how would you fix the launch file if not (Step 5).
5. `map.yaml` contents, mapped to the corresponding `OccupancyGrid.info` fields (Step 6).
6. `map.pgm` viewed as an image, compared against the actual driven space (Step 6).
7. Looking Ahead discussion, 2 questions.

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

---
