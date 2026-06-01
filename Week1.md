# Week 1 — Mini Pupper Setup & Orientation

Ensure you have read through the README and installed all needed files before starting.
**Reference:** [Mini Pupper ROS2 Guide](https://minipupperdocs.readthedocs.io/en/latest/guide/ROS2Guide.html) — read through and use as a reference throughout the course.

---

## 1. Getting to Know the Mini Pupper

### 1.1 Compute Platform (Raspberry Pi 4B)

| Spec | Value |
|------|-------|
| Processor | Quad-core ARM Cortex-A72 @ 1.8GHz |
| Memory | 4GB LPDDR4 |
| GPU | VideoCore VI (no CUDA) |
| Storage | MicroSD card (32GB+) |
| Wired Network | Gigabit Ethernet |
| Wireless | Wi-Fi 5 (802.11ac), Bluetooth 5.0 |
| OS | Ubuntu 22.04 LTS |

### 1.2 Robot Specs

| Parameter | Value |
|-----------|-------|
| Height | ~180 mm standing |
| Weight | ~480 g |
| Total DoFs | 12 (3 per leg × 4 legs) |
| Servo Type | PWM hobby servo |
| Battery | 7.4V LiPo, ~1–2 hr runtime |
| Camera | Raspberry Pi AI Camera (IMX500) |
| Lidar | LD06 or LD19 (add-on) |

<!-- Add: photo of Mini Pupper with labeled joints and leg numbering -->

---

## 2. Leg Structure & Kinematics Overview

The Mini Pupper leg is modeled as an **RR (Revolute-Revolute) manipulator** — two servo joints in a single plane.

| Joint | Servo | Description |
|-------|-------|-------------|
| θ1 | Hip | Rotates the upper leg segment (l1 = 5 cm) |
| θ2 | Knee | Rotates the lower leg segment (l2 = 6 cm) |

- **Forward Kinematics (FK):** Given joint angles → compute foot position
- **Inverse Kinematics (IK):** Given foot position → compute joint angles

We will explore this in detail in Week 5. For now just know that all 12 servos are driven by PWM signals generated from the Pi.

<!-- Add: diagram of single leg RR model with l1, l2, theta1, theta2 labeled -->

---

## 3. Safety

- Mini Pupper powers on with servos relaxed — place it on a flat surface before enabling motion
- **DO NOT force the legs** while servos are powered — you will strip the gears
- Always power off before handling the servo board or wiring
- Do not leave the LiPo battery charging unsupervised
- Keep fingers clear of leg joints during motion commands

---

## 4. Power On / Off

### Powering On
1. Place the Mini Pupper on a flat surface with legs clear of obstacles
2. Confirm battery is charged and connected
3. Flip the power switch — the Pi will begin booting (green LED on Pi will blink)
4. Wait ~60 seconds for the Pi to fully boot before attempting SSH

### Powering Off
1. SSH in and run `sudo shutdown now` before cutting power
2. Wait for the Pi's green LED to stop blinking, then flip the power switch
3. Disconnect and charge the battery after each session

---

## 5. WiFi Setup

The Mini Pupper communicates over WiFi. You need to get it on your local network before anything else.

### Option A — Edit netplan directly on the Pi (via HDMI + keyboard)
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```
Add your network under `wifis:`:
```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "YourNetworkName":
          password: "YourPassword"
```
Then apply:
```bash
sudo netplan apply
sudo reboot
```

### Option B — nmcli (if already connected via Ethernet)
```bash
sudo nmcli dev wifi connect "YourNetworkName" password "YourPassword"
```

### Find the IP address
After connecting, on the Pi run:
```bash
hostname -I
```
Note the IP — you'll need it for SSH.

---

## 6. SSH Setup

SSH (Secure Shell) is a network protocol that lets you remotely access and control another computer over a network connection. When you run `ssh ubuntu@192.168.x.x` you authenticate with the remote machine and get placed into its shell session — any commands you run execute on the Pi, not your laptop.

### 6.1 Enable SSH on the Pi (first time only)
On the Pi directly:
```bash
sudo apt install openssh-server -y
sudo systemctl enable ssh
sudo systemctl start ssh
```

### 6.2 SSH in from your PC
```bash
ssh ubuntu@<robot-ip>
# Default password: mangdang
```

### 6.3 Confirm you're in
```bash
hostname
# Expected: ubuntu-desktop (or similar)
uname -a
# Expected: Linux ... aarch64 ...
```

<!-- Add: screenshot of successful SSH session -->

---

## 7. ROS2 Installation

The Mini Pupper runs a **ROS2 Humble** stack. Install it on the Pi:

```bash
sudo apt update
git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
~/ros2_setup_scripts_ubuntu/ros2-humble-ros-base-main.sh
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

This will take 15–20 minutes. Once done verify:
```bash
ros2 --version
# Expected: ros2cli 0.18.x
```

---

## 8. Mini Pupper ROS2 Package Setup

On the Pi, install the Mini Pupper ROS2 stack:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2-dev mini_pupper_ros
vcs import < mini_pupper_ros/.minipupper.repos --recursive

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

Add environment variables to `~/.bashrc`:
```bash
echo "export ROBOT_MODEL=mini_pupper" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

---

## 9. Verify Everything is Working

```bash
# ROS2
ros2 --version
# Expected: ros2cli 0.18.x

# Mini Pupper packages
ros2 pkg list | grep mini_pupper
# Expected: mini_pupper_bringup, mini_pupper_description, etc.

# Environment variables
echo $ROBOT_MODEL
# Expected: mini_pupper

echo $ROS_DOMAIN_ID
# Expected: 42

# ROS2 topics (run bringup first — see below)
ros2 topic list
```

---

## 10. First Bringup

Once everything is installed, test that the robot stack launches without errors:

```bash
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

You should see a series of nodes starting up with no red errors. If servo movement occurs — that's normal, the legs will move to their home position.

<!-- Add: screenshot of successful bringup terminal output -->

---

## Tasks

| # | Task | Points |
|---|------|--------|
| 1 | Connect the Mini Pupper to WiFi and find its IP address. Include a screenshot of `hostname -I` output. | 20 |
| 2 | SSH into the robot from your PC. Include a screenshot of the successful session. | 20 |
| 3 | Install ROS2 and verify with `ros2 --version`. Include a screenshot. | 30 |
| 4 | Run `ros2 launch mini_pupper_bringup bringup.launch.py` and confirm it launches without errors. Include a screenshot. | 30 |
