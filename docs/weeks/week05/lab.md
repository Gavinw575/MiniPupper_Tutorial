# Week 5 — Reinforcement Learning: How to Train Your Robot

---

**Objectives:**

1. Understand how reinforcement learning is used to train quadruped locomotion policies.
2. Set up the MuJoCo/MJX training environment in Google Colab using the Mini Pupper 2 simulation model.
3. Train a walking policy using Proximal Policy Optimization (PPO) with reward shaping.
4. Analyze training curves and visualize policy behavior in simulation.
5. Understand the sim-to-real gap and domain randomization as a mitigation strategy.

---

**Reference Material:**

- [Stanford CS123 Lab 5 — How to Train Your Dog](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-25/lab-5.html)
- [Mini Pupper 2 MuJoCo Model](https://github.com/Gavinw575/mini_pupper_2_mjx)
- [pupperv3-mjx Training Code](https://github.com/JummerCloth/pupperv3-mjx)
- [Weights & Biases](https://wandb.ai)
- [MuJoCo MJX Documentation](https://mujoco.readthedocs.io/en/stable/mjx.html)

---

## Background

For this lab you will be letting the robot learn by trial and error using something called reinforcment learning (RL). To do this the robot is places inside a simulated environment and given a reward signal (e.g., move forward at 0.75 m/s), and trainted to maximize that reqard over millions of simulates steps. THis results in a neural network policy that serves as a function that maps sensor readings (joint angles, IMU data, velocity commands) to moter commands in the simulation. 

This is the same approach that is used by by many companies to train their quadruped and biped robots. By doing this through Google's Colab Pro we can save ourselves the pain of having to buy a $10,000 GPU and just do it through a cloud GPU for ~$10.

!!! note "Stanford CS123 Comparison"
    [Stanford CS123 Lab 5](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-25/lab-5.html) uses the same training pipeline on their custom Pupper v3 robot. This lab adapts that work for Mini Pupper 2 by providing a MuJoCo model converted from the Mini Pupper 2 URDF. The training code, reward structure, and learning concepts are roughly the same. 

---

## Setup

### Step 0 — Create a Weights & Biases Account

We use [Weights & Biases (wandb)](https://wandb.ai) to log all training runs. It's free and gives you live reward curves, training videos, and run comparisons. It is a great resource when trying to compare your training models.

1. Create a free account at [wandb.ai](https://wandb.ai)
2. After logging in, go to [wandb.ai/authorize](https://wandb.ai/authorize) and copy your API key

You will paste this key into the Colab notebook in the next step.

---

### Step 1 — Open the Colab Notebook

Open the [Mini Pupper 2 Lab 5 Colab](https://colab.research.google.com/drive/1w5c69BxMfCnkBinFAr7S13GcAnJ9dKbK?usp=sharing) and make a copy to your own Google Drive:

```
File → Save a copy in Drive
```

!!! warning "This requires a Pro Colab plan"
    This notebook requires a more powerful GPU then the free version allows for to run. Go to Runtime -> Change runtime type and select A100 (requires Colab Pro).

Paste your wandb API key into the first cell and run that cell to ensure it is connected.

---

### Step 2 — Install Dependencies

Run the*Install dependencies cell. This installs:

- `mujoco==3.2.7` and `mujoco-mjx==3.2.7` — physics simulator and GPU-accelerated version
- `brax==0.10.5` — RL environment wrapper
- `flax` and `jax` — neural network and GPU computation libraries
- `wandb` — experiment tracking

---

### Step 3 — Simulation Config (Mini Pupper 2)

This is where our setup differs from Stanford's. The Simulation Config section has been pre-adapted to use the Mini Pupper 2 MuJoCo model instead of the Pupper v3 model.

The key changes from Stanford's original notebook are:

```python
# Using the Mini Pupper repo instead of Stanford's Pupper v3 repo
simulation_config.model_repo = 'https://github.com/Gavinw575/mini_pupper_2_mjx'
simulation_config.model_branch = 'main'

# MJCF model
simulation_config.original_model_path = 'mini_pupper_2_mjx/mini_pupper_2.xml'

# Mini Pupper 2 body names
simulation_config.upper_leg_body_names = ["rf2", "lf2", "rb2", "lb2"]
simulation_config.lower_leg_body_names = ["rf3", "lf3", "rb3", "lb3"]

# Mini Pupper 2 foot sites
simulation_config.foot_site_names = ["rffoot_site", "lffoot_site", "rbfoot_site", "lbfoot_site"]

# Mini Pupper 2 default standing pose (from URDF)
training_config.default_pose = jp.array(
    [0.0, 0.994, -1.767,   # RF: abduction, hip, knee
     0.0, 0.994, -1.767,   # LF: abduction, hip, knee
     0.0, 0.994, -1.767,   # RB: abduction, hip, knee
     0.0, 0.994, -1.767]   # LB: abduction, hip, knee
)

# Mini Pupper 2 is smaller so we've lowered the height
training_config.terminal_body_z = 0.06  # [m]
```

Run the Simulation Config cell without modifying it.

---

### Step 4 — Benchmark the Model

Run the Benchmark pupper model cell. This tells you how many simulation steps per second the GPU can run:

```
Steps per sec:  1421043.7659230942
```

**DELIVERABLE:** Screenshot the benchmark output showing your steps/sec.

---

## Training

### Step 5 — Velocity Tracking (First Run)

Before training, you must set reward coefficients. All rewards start at `0` — a robot with no rewards will just flop randomly and learn nothing.

For the first run, well implement velocity tracking only. Find the Reward Configuration section and set:

```python
reward_config.rewards.scales.tracking_lin_vel = 1.5
reward_config.rewards.scales.tracking_ang_vel = 0.75
```

Leave all other rewards at `0`.

!!! note "Why these values?"
    The linear velocity coefficient is roughly double the angular velocity coefficient. This matches Stanford's recommendation - forward/backward motion is the primary objective, turning is secondary.

The velocity tracking reward uses an exponential function. When the robot's actual velocity matches the commanded velocity, the reward is maximized:

$$r_{vel} = \exp\left(-\frac{(v_{cmd} - v_{actual})^2}{\sigma}\right)$$

where `σ = 0.25` (set in `reward_config.rewards.tracking_sigma`).

**DELIVERABLE:** Before training, write 2–3 sentences predicting what you think the robot will look like after training with only velocity tracking. Will it walk naturally? What might go wrong?

Now run the entire notebook. Training will take approximately:

| GPU | 200M steps | Expected reward |
|-----|-----------|----------------|
| A100 | ~25 minutes | 15–20 |

Watch your [wandb dashboard](https://wandb.ai) for live reward curves. The reward should trend upward over training.

!!! warning
    If you need to stop training early, run `wandb.finish()` before starting a new run or the next run will fail to initialize.

**DELIVERABLE:** Screenshot your wandb reward curve after training completes.

**DELIVERABLE:** Access the training progress videos in the Colab file browser under `output_<run-name>/`. Describe what the robot looks like at 20M steps vs 200M steps. Does it look like natural walking?

---

### Step 6 — Full Reward (Second Run)

Your first policy probably moves forward but inefficiently — flopping, dragging legs, or using excessive motor torques. Now add regularization terms to encourage smoother, more energy-efficient movement.

!!! note "Sign convention"
    Positive coefficients encourage the behavior. Negative coefficients penalize it. Torque and action rate penalties are negative because we want to minimize them — they reduce unnecessary motor effort and jerky motion.

**DELIVERABLE:** Write out your full reward function in math notation. Why did you choose these terms? What behavior are you trying to encourage or discourage?

**DELIVERABLE:** Qualitatively compare the training video from this run vs your first run. How does the gait look different?


Now you have freedom to train the robot to the best of your abilities. Using what you know about training, tune the reward function to produce the best walking policy you can. You can use any combination of the available reward terms: 

| Term | Effect |
|------|--------|
| `tracking_lin_vel` | Follow commanded forward/backward velocity |
| `tracking_ang_vel` | Follow commanded yaw rate |
| `orientation` | Penalize body tilting |
| `torques` | Penalize excessive motor torques |
| `joint_acceleration` | Penalize jerky joint motion |
| `mechanical_work` | Penalize energy consumption |
| `action_rate` | Penalize rapid action changes |
| `feet_air_time` | Encourage long swing phases |
| `stand_still` | Stay still at zero command |
| `abduction_angle` | Keep legs from spreading too wide |
| `foot_slip` | Penalize feet sliding on ground |
| `knee_collision` | Penalize knees hitting the ground |

Increase `training_config.ppo.num_timesteps` to at least 300M whenever you are ready for your final run:

```python
training_config.ppo.num_timesteps = 300_000_000
```
Qualitatively compare the training video from this run to your first run. How does this gait look different.

**DELIVERABLE:** List all reward terms you used and their coefficients. Explain your reasoning for each.

**DELIVERABLE:** Training video showing your best policy walking in simulation.

**DELIVERABLE:** Can the robot stand still when given a zero velocity command? Record a video with `x_vel = 0, y_vel = 0, ang_vel = 0` in the Visualize Policy section.

---

### Step 8 — Domain Randomization

Your trained policy was optimized for fixed simulation. In the real world, the conditions can vary such as with the floor or just random knocking of the robot. Domain randomization exposes the policy to varied conditions during training so it learns to handle this variability.

Find the Domain randomization section of the training config and experiment with these parameters:

```python
# Kicks the robot randomly during training :(
training_config.kick_probability = 0.04
training_config.kick_vel = 0.10

# Randomize body mass (robot may be heavier/lighter than sim)
training_config.body_mass_scale_range = (0.8, 1.3)

# Randomize ground friction (carpet vs hardwood vs tile)
training_config.friction_range = (0.4, 1.8)

# Randomize motor gains (servos may not be perfectly calibrated)
training_config.position_control_kp_multiplier_range = (0.6, 1.2)
```

Re-run training with different domain randomization.

**DELIVERABLE:** What happens if you add too much domain randomization? Try setting `body_mass_scale_range = (0.1, 5.0)` and describe what happens to training.

**DELIVERABLE:** Training video comparing your policy with domain randomization. 

---

## Visualization

### Step 9 — Visualize Your Trained Policy

After training, run the Visualize Policy section to watch your policy in action. Set different velocity commands to test behavior:

```python
# Walking forward
x_vel = 0.75
y_vel = 0.0
ang_vel = 0.0

# Turning
x_vel = 0.0
y_vel = 0.0
ang_vel = 1.5

# Strafing
x_vel = 0.0
y_vel = 0.5
ang_vel = 0.0
```

To make the video larger and longer:

```python
n_steps = 500       # longer video
render_every = 1    # smoother playback

frames = visualization_env.render(rollout[::render_every], camera='tracking_cam')
media.show_video(frames, fps=1.0 / visualization_env.dt / render_every, width=640)
```

**DELIVERABLE:** Video of your best policy walking forward at `x_vel = 0.75`.

**DELIVERABLE:** Inspect the joint position plots generated after visualization. Compare the leg motion pattern to the triangular Raibert heuristic gait from Week 3. Do the legs follow a similar triangular path? Write a few sentences about similarities and differences.

---

## Sim-to-Real Discussion

We trained entirely in simulation this week. The Mini Pupper 2 deployment pipeline is future work — but it is worth understanding the gap between what we trained and what would happen on real hardware.

**DELIVERABLE:** Answer the following questions in your lab writeup (3–4 sentences each):

1. The Mini Pupper 2 uses serial smart servos controlled through an ESP32 layer — this introduces latency not present in our simulation. How might this affect policy performance on the real robot?

2. Our simulation uses position-controlled actuators (the servo commands a target angle). Most research RL work uses torque control. What are the tradeoffs?

3. What simulation parameters would you most want to randomize to improve real-world transfer for Mini Pupper 2 specifically?

---

## Tasks

1. Benchmark cell screenshot showing steps/sec on your GPU.

2. Pre-training prediction (Step 5 deliverable) written in your lab document.

3. Wandb reward curve screenshot from your velocity-tracking run (Step 5).

4. Training video comparison: 20M steps vs 200M steps (Step 5).

5. Full reward function writeup with math notation (Step 6).

6. Qualitative gait comparison: velocity tracking only vs with regularization (Step 6).

7. Best policy training video at 300M+ steps (Step 7).

8. Standing still video at zero command (Step 7).

9. Domain randomization experiment writeup (Step 8).

10. Policy visualization video: forward walk at 0.75 m/s (Step 9).

11. Joint position plot analysis comparing to Raibert heuristic (Step 9).

12. Sim-to-real discussion questions (3 questions, 3–4 sentences each).

---

## Troubleshooting

??? question "Training cell fails with `wandb.finish()` error"
    A previous run didn't close cleanly. Run this cell manually before starting a new run:
    ```python
    wandb.finish()
    ```

??? question "Reward stays at 0.000 throughout training"
    All reward coefficients are at 0. Make sure you set at least `tracking_lin_vel` and `tracking_ang_vel` to non-zero values in the Reward Configuration cell before running the training cell.

??? question "ParseError: junk after document element"
    The MJCF file in the repo is corrupted (two XML documents concatenated). Re-clone the repo:
    ```python
    !rm -rf mini_pupper_2_mjx
    !git clone {simulation_config.model_repo} -b {simulation_config.model_branch}
    ```

??? question "set_mjx_custom_options returns None"
    The MJCF is missing the `<custom>` block. This should be present in the current repo version. If you see this error, re-clone the repo as above.

??? question "KeyError: Invalid name 'home'"
    The MJCF is missing the `<keyframe>` block. Re-clone the repo.

??? question "Training is very slow on T4"
    The T4 GPU gives roughly 800k steps/sec vs 4.5M on A100. For 200M steps this is ~4 hours. Reduce `num_timesteps` to 50M for quick iteration:
    ```python
    training_config.ppo.num_timesteps = 50_000_000
    ```

??? question "Robot immediately falls over in simulation"
    The default pose may not match the MJCF home keyframe. Verify that `training_config.default_pose` matches the joint angles in the keyframe:
    ```python
    # Correct Mini Pupper 2 default pose
    training_config.default_pose = jp.array(
        [0.0, 0.994, -1.767, 0.0, 0.994, -1.767,
         0.0, 0.994, -1.767, 0.0, 0.994, -1.767]
    )
    ```

---

