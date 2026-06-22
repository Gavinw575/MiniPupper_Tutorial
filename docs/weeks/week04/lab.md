# Lab 4 — RR Model, FK & IK Scripts

## Before You Start

You'll need two scripts for this lab: `MiniPupper_RR_FK.py` and `MiniPupper_RR_IK.py`. If you don't already have them, ask your instructor — they're early drafts and need a few fixes before they're lab-ready, which is exactly what Part 1 is about.

You'll also want the robot up and reporting `/joint_states` and `/tf` by the end of this lab (real robot or simulation both work), so make sure bringup is running before Part 4.

---

## Part 1 — Fixing the FK Script

Open `MiniPupper_RR_FK.py`. As written, it has a few rough edges that need fixing before it's usable in a lab setting:

1. It uses `input()` to get joint angles interactively, and ends with `plt.show()` — both of those require an actual display. If you're SSH'd into the robot or running this somewhere headless, the script will just hang. We need it to take angles as arguments and save a plot to a file instead.
2. There's a `print()` statement meant to show the joint angles in degrees using `%d°` formatting, but the values never actually get substituted in — it just prints the raw tuple.
3. The link lengths in the script are leftover placeholders from early planning (`l1=5cm, l2=6cm`) — not the real Mini Pupper 2 geometry.

### 1.1 — Update the Geometry Constants

Find where the script defines link lengths and joint angles, and replace the placeholder values with the real ones:

```python
# Real Mini Pupper 2 front-right leg geometry, from the URDF
L1 = 0.0502  # hip joint -> knee joint, meters
L2 = 0.056   # knee joint -> foot contact, meters
```

### 1.2 — Fix the Headless Display Problem

Find the `input()` calls. Replace them with command-line arguments using `sys.argv`, or with a function that takes `hip_angle` and `knee_angle` as parameters directly — your choice, but function parameters are simpler and easier to test repeatedly. Then find `plt.show()` and replace it with:

```python
plt.savefig('fk_result.png')
print('Saved plot to fk_result.png')
```

### 1.3 — Fix the Broken Print Statement

Find the print statement with the `%d°` formatting bug and fix it so it actually substitutes the angle values. (Hint: with `%`-style formatting, the values to substitute go in a tuple *after* the `%` operator — check that the tuple is actually being passed in, not just sitting next to the string.)

---

## Part 2 — The Hip Convention Problem

Before you touch the FK *math* itself, let's confirm the problem the Overview described. The script likely has a forward kinematics function that looks something like this (textbook convention):

```python
def forward_kinematics(hip, knee, l1, l2):
    x = l1 * math.cos(hip) + l2 * math.cos(hip + knee)
    z = l1 * math.sin(hip) + l2 * math.sin(hip + knee)
    return x, z
```

Run it with the real standing pose — `hip=0.994`, `knee=-1.767` — and look at the output. You should get a foot position that does *not* look like a leg standing under a robot's body (if you get a positive z, or an x value that seems way too large, that's the tell).

**Lab notes — answer before continuing:**

1. What (x, z) does the unmodified script give you for the standing pose?
2. Does that match a robot leg that's bent at the knee and supporting weight underneath the body? Why or why not?

---

## Part 3 — Deriving the Corrected Formula

Here's the geometric fact from the Overview: at hip angle = 0, the upper leg points straight **down** (along −Z), not along +X like the textbook diagram assumes. We need a formula whose zero-reference actually matches that.

### 3.1 — Work Out Where the Knee Joint Is

Think of the upper leg as a vector of length `l1`, starting at the hip and currently pointing straight down (−Z direction) when hip=0. As the hip angle increases, the leg rotates away from straight-down.

If you drew this on paper with −Z as your reference direction and rotated counterclockwise by angle `hip`, you'd find the vector's components are:

```
knee_x = l1 · ( ? )
knee_z = l1 · ( ? )
```

**TODO:** Fill in the two blanks above. Hint: start from the straight-down vector `(0, -1)` and apply a standard 2D rotation matrix by angle `hip`. You should end up with one term using `sin(hip)` and one using `cos(hip)`, each with a sign.

### 3.2 — Extend to the Foot

The knee joint works the same way relative to the upper leg — when `knee=0`, the lower leg is collinear with the upper leg (fully extended). So the *absolute* angle of the lower leg, relative to straight-down, is `hip + knee` (this part **does** match the textbook's additive-angle idea — it's only the hip's zero-reference that's different).

**TODO:** Using the same pattern as 3.1, but with angle `(hip + knee)` instead of just `hip`, write the formula for the foot position relative to the knee, then add it to the knee position from 3.1 to get the full foot position relative to the hip:

```python
def forward_kinematics(hip, knee, l1, l2):
    knee_x = l1 * (___________)
    knee_z = l1 * (___________)

    foot_x = knee_x + l2 * (___________)
    foot_z = knee_z + l2 * (___________)

    return foot_x, foot_z
```

### 3.3 — Check Your Work

Plug the real standing pose back in: `hip=0.994`, `knee=-1.767`, `l1=0.0502`, `l2=0.056`. You should get approximately:

```
foot_x ≈ -0.003 m
foot_z ≈ -0.068 m
```

If you're off by more than a millimeter or two, check your signs — it's very easy to flip a `sin`/`cos` or drop a negative sign in this derivation, and that's a completely normal place to get stuck. Walk back through 3.1 step by step rather than guessing at sign flips.

**Lab notes:**

1. The foot ends up almost directly below the hip (x is tiny). Does that make sense for a standing robot? What would you expect x to look like instead if the robot were mid-stride, with this leg swung forward?
2. The corrected formula and the textbook formula both use the *same* `hip + knee` additive structure for the second joint, but differ on the first. Why does that asymmetry exist — what's actually different about how the hip and knee joints are defined?

---

## Part 4 — Fixing and Using the IK Script

### 4.1 — Fix the Validation Bug

Open `MiniPupper_RR_IK.py`. There's a leftover validation check that tests whether a value `!= float` after it's already been cast with `float()` — that check can never trigger, since the cast guarantees the type. Remove it, or replace it with a check that's actually meaningful (for example: checking whether the requested target position is even reachable, i.e. its distance from the hip is ≤ `l1 + l2`).

### 4.2 — Update the IK Math to Match Your FK Derivation

Just like the FK script, the IK script's law of cosines / geometry setup will assume the textbook hip convention unless you adjust it. The core IK approach (law of cosines to solve for the knee angle, then geometry to solve for the hip angle) stays the same — what changes is how the hip angle relates back to your corrected zero-reference from Part 3.

This is the harder direction, so work through it with your lab partner and don't hesitate to ask for help here. The graders care more about whether your IK solution round-trips correctly (see 4.3) than whether you got there cleanly on the first try.

### 4.3 — Round-Trip Test

The best way to check IK is correct: feed its output straight back into your fixed FK function.

```python
# Pick any target foot position within reach
target_x, target_z = -0.003, -0.068

hip_solved, knee_solved = inverse_kinematics(target_x, target_z, L1, L2)
foot_x, foot_z = forward_kinematics(hip_solved, knee_solved, L1, L2)

print(f'Target:  ({target_x:.4f}, {target_z:.4f})')
print(f'FK(IK):  ({foot_x:.4f}, {foot_z:.4f})')
```

These two should match to within a small floating-point tolerance. If they don't, your IK has a bug — go back and check it before moving on.

**Lab notes:** Try a target foot position that's farther forward (larger negative or positive x, your choice) and run the round-trip test again. Does it still work? Try a target that's clearly out of reach (farther than `l1 + l2` from the hip) — what does your script do? Does it crash, return garbage, or handle it gracefully?

---

## Part 5 — Cross-Checking Against the Real Robot

This is where Week 3's TF skills come back. With the robot standing still (real robot or simulation):

### 5.1 — Read the Real Joint Values

```bash
ros2 topic echo /joint_states --once
```

Find the entries for the front-right leg's hip and knee joints (check the `name` array to match them up — the order in `position` follows the order in `name`). Record the actual current values.

### 5.2 — Feed Them Into Your FK Script

Run your fixed FK script using these *live* values instead of the idealized standing pose from earlier. Note the foot position it computes.

### 5.3 — Compare Against TF

Use `tf2_echo` to check the real spatial relationship between the hip frame and the foot frame (check `view_frames` from last week's lab if you don't remember the exact frame names for this leg):

```bash
ros2 run tf2_ros tf2_echo rf1 rffoot
```

**Lab notes:**

1. How close is your FK script's computed foot position to what TF actually reports? They won't be identical — TF accounts for the small 3D offsets (abduction, the ~5mm y-offsets) that our simplified 2D planar model ignores. Is the gap roughly what you'd expect from ignoring those, or is it bigger than that?
2. If the gap were large and unexplained, what would that tell you — a bug in your FK derivation, or a wrong assumption about the geometry?

---

## Stretch Goal (Optional)

The standing-pose foot position we derived (6.75 cm below the hip) is a bit lower than you might expect for this robot's actual standing height. Investigate: does the foot link have its own radius or thickness that would add a bit more clearance? Look at the collision geometry in the URDF or the mesh file for the foot. This kind of "the simplified model gets you close, but not exact, and here's the physical reason why" gap is completely normal in robotics — the goal isn't to make the numbers match by force, but to understand *why* they don't match perfectly.

---

## What's Next

Week 5 shifts from "derive the exact equations" to a fundamentally different approach: training a control policy through reinforcement learning instead of solving IK by hand. Keep your FK function around — it'll be useful as a sanity check against the learned policy's behavior later in the course.
