# Week 3 — Teleoperation & Gait

## What You'll Learn This Week

Last week you got nodes, topics, and `/cmd_vel` working — you wrote a publisher that drove the robot in a square. This week is about understanding *how* that velocity command actually turns into the robot walking, and giving you a proper way to drive the robot around: keyboard teleop.

By the end of this week you should be able to:

- Drive the Mini Pupper 2 with the keyboard using `teleop_twist_keyboard`
- Explain what CHAMP is doing between `/cmd_vel` and the leg joints
- Read and understand a TF (transform) tree — what frames are, why they're nested, and how to inspect them live
- Write a node that uses TF to track the robot's position over time

## A Quick Recap: `/cmd_vel`

You already know `/cmd_vel` carries a `geometry_msgs/Twist` message — linear velocity (x, y, z) and angular velocity (roll, pitch, yaw). For a ground robot like the Mini Pupper 2, only `linear.x` (forward/backward) and `angular.z` (turning) actually matter. Everything else is ignored.

What you *haven't* seen yet is what happens after that message is published. The Mini Pupper 2 doesn't have wheels — it has 12 servos across 4 legs. Something has to turn "move forward at 0.12 m/s" into a coordinated leg gait. That something is **CHAMP**.

## What CHAMP Is Actually Doing

CHAMP is a quadruped controller framework. It subscribes to `/cmd_vel`, and internally:

1. Picks a gait pattern (the default is a trot — diagonal leg pairs move together)
2. Generates a foot trajectory for each leg based on the requested velocity
3. Runs inverse kinematics to convert each foot position into joint angles
4. Publishes those joint angles to the servo controllers

The important thing this week: **you are not writing the gait logic yet**. That comes later — Week 4 you'll build the FK/IK math by hand, and Week 5 you'll get into the RL-trained alternative. For now, CHAMP is a black box that takes `/cmd_vel` in and produces walking out. Your job this week is to understand the *interface*, not the internals.

## Why TF Matters Here

When CHAMP — or anything else in the stack — needs to know "where is the robot's foot relative to its body" or "where is the robot relative to the map," it doesn't do that math from scratch every time. ROS2 maintains a constantly-updated tree of coordinate frames called **TF** (transform framework).

Some frames you'll see on the Mini Pupper 2:

- `map` — fixed world frame (only exists once you're running SLAM, later in the course)
- `odom` — the robot's estimate of its own movement since startup, drifts over time
- `base_link` — the robot's body, the frame everything else is attached to
- `base_footprint`, leg frames, `laser`, `camera_link` — attached to `base_link` at fixed or moving offsets

Every frame knows its position relative to its parent. TF lets any node ask "where is X relative to Y, right now" without caring how many frames are in between. This is exactly what `/odom` is built on, and it's the foundation for everything coming in Weeks 7–8 (SLAM and Nav2).

## Looking Ahead

Next week (Week 4) you'll build a 2-link leg model and write forward and inverse kinematics by hand — at that point CHAMP's black box starts to open up. The TF skills from this week carry forward directly: you'll be checking your IK math against the real foot positions TF reports.
