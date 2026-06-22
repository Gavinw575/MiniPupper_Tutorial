# Week 4 — Forward & Inverse Kinematics

## What You'll Learn This Week

Last week we called CHAMP a "black box" — it takes `/cmd_vel` in and produces walking out, and you didn't need to know how. This week you open that box partway. You'll learn the two core problems every legged robot's controller has to solve, and you'll solve them yourself, by hand, for one leg of the real Mini Pupper 2.

By the end of this week you should be able to:

- Explain the difference between forward kinematics (FK) and inverse kinematics (IK)
- Compute a Mini Pupper 2 foot position from its hip and knee joint angles
- Compute the hip and knee joint angles needed to put the foot at a target position
- Explain why the real robot's joint-angle convention isn't the "textbook" one, and adjust for it
- Cross-check your hand-derived math against what the real robot reports on `/joint_states` and TF

## The Two Problems

Picture one leg of the Mini Pupper 2, simplified to two rigid links connected by two joints — a **hip** joint and a **knee** joint, both rotating in the same plane. This is called an **RR model** ("R" for "revolute," meaning a rotating joint — two of them, one per link).

- **Forward kinematics (FK):** given the joint angles, where is the foot? This is the easy direction — straightforward trig, one unique answer.
- **Inverse kinematics (IK):** given a *target* foot position, what joint angles get the foot there? This is the harder direction — there can be zero, one, or two valid answers (a bent-knee and a straight-knee solution can sometimes both reach the same point), and sometimes the target is simply out of reach.

CHAMP is solving IK continuously, many times a second, for all four legs, every time you send a `/cmd_vel` command — it decides where each foot needs to be to produce a trot, then solves IK to get the joint angles, then sends those angles to the servos. You're about to do by hand, for one leg, one snapshot in time, what CHAMP does in a tight real-time loop for the whole robot.

## Real Geometry, Not a Toy Example

We're not going to invent fake numbers for this. We're using the actual measurements from the Mini Pupper 2's URDF, for the front-right leg:

- **l1** (hip joint → knee joint) = 5.02 cm
- **l2** (knee joint → foot contact point) = 5.6 cm
- **Standing pose** (this leg's actual resting joint values on the real robot): hip = 0.994 rad, knee = −1.767 rad

These numbers matter because the goal isn't just "learn FK/IK as abstract math" — it's to write code this week that you can later point at the *real robot's* `/joint_states` topic and get a foot position that matches what TF and RViz show you. That's the same cross-checking instinct from Week 3, applied to a new problem.

## The Catch: The Hip's Zero Angle Doesn't Point Where You'd Expect

If you've seen a 2-link RR arm before — in a textbook, or a robotics intro course — it's almost always drawn like this: joint 1 sits at the origin, and when its angle is zero, the first link points straight out along the x-axis (horizontal). The standard formula for the end-effector position looks like:

```
x = l1·cos(θ1) + l2·cos(θ1 + θ2)
y = l1·sin(θ1) + l2·sin(θ1 + θ2)
```

That formula assumes **θ1 = 0 means "link 1 points along +x."**

The Mini Pupper 2's hip joint doesn't work that way. It's a *hanging* leg, not a horizontal arm — when the hip angle is 0, the upper leg points straight **down**, not forward. That's a 90° difference in where "zero" points, and if you plug real `/joint_states` values straight into the textbook formula above, you'll get a wrong answer — not a crash, just silently wrong numbers, which is the worst kind of bug.

You're going to work out the corrected version of this formula yourself in the lab. It's a small fix — one well-defined substitution — but you should understand *why* it's needed, not just memorize it. The skill that matters here generalizes: any time you're handed a robot's joint convention, check what "zero" actually means physically before you trust a formula you learned somewhere else.

## Looking Ahead

This week's IK is the "classical" approach to legged locomotion — derive the exact equations by hand, solve them every control cycle. It works, and it's exactly what CHAMP does. But it's also one of two fundamentally different ways to solve this problem. Later in the course (Week 5 and beyond), you'll see the other way: a reinforcement-learned policy that never derives an equation at all — it just learns, through trial and lots of simulated trial-and-error, what joint angles tend to produce good walking. Keep this week's hand-derived formula in your back pocket; it'll be a useful point of comparison when you get there.
