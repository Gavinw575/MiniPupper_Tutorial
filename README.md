# MiniPupper_Tutorial
A series of tutorials for getting started with the Mini Pupper 1 quadruped robot. Covers everything from hardware overview and SSH configuration to ROS2 navigation, SLAM, and computer vision. Designed for college-level students with little to no prior experience with quadruped robotics.

---

## Tutorial Overview

| Week | Topic |
|------|-------|
| 1 | Hardware Overview & Power On |
| 2 | SSH, Linux Basics & WiFi Setup |
| 3 | ROS2 Installation & Environment |
| 4 | Mini Pupper ROS2 Bringup |
| 5 | Forward & Inverse Kinematics |
| 6 | Teleoperation & Gait Control |
| 7 | Lidar Setup & SLAM |
| 8 | Nav2 Autonomous Navigation |
| 9 | Camera Setup & YOLO Vision |
| 10 | Capstone — Autonomous Mapping + Object Detection |

---

## Minimum Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |
| RAM | 8 GB | 16 GB |
| Robot | Mini Pupper 1 | Mini Pupper 1 |
| Camera | Raspberry Pi AI Camera (IMX500) | Raspberry Pi AI Camera (IMX500) |
| Lidar | LD06 or LD19 | LD19 |
| Network | WiFi (2.4 GHz or 5 GHz) | WiFi + Ethernet adapter |

---

## Pre-Course Setup

Run `install_all.sh` **once** on a fresh Ubuntu 22.04 machine before Week 1.
Takes approximately 20–40 minutes depending on your internet speed.

```bash
# 1. Clone the repo
git clone https://github.com/<your-username>/MiniPupper_Tutorial ~/MiniPupper_Tutorial

# 2. Run the setup script
bash ~/MiniPupper_Tutorial/setup/install_all.sh

# 3. Reload your shell environment
source ~/.bashrc
```

---

## What Gets Installed

| Step | Packages | Used In |
|------|----------|---------|
| 1 | `openssh-client`, `net-tools`, `git`, `curl` | Week 2 — SSH |
| 2 | `ros-humble-desktop`, `ros-humble-rmw-fastrtps-cpp` | Week 3 — ROS2 |
| 3 | `ros-humble-rviz2`, `ros-humble-tf2-tools`, `ros-humble-xacro` | Week 4 — Bringup |
| 4 | `mini_pupper_ros` (built from source) | Week 4–6 — Motion |
| 5 | `ros-humble-teleop-twist-keyboard` | Week 6 — Teleop |
| 6 | `ros-humble-slam-toolbox`, `ros-humble-nav2-bringup` | Week 7–8 — SLAM & Nav2 |
| 7 | `libcamera` (built from source), `python3-picamera2` | Week 9 — Camera |
| 8 | `ultralytics` (YOLOv8), `opencv-python`, `numpy` | Week 9–10 — YOLO Vision |
| 9 | `matplotlib`, `pandas` | Week 5 — Kinematics Viz |

**Disk usage (approximate):**
- ROS2 Humble desktop: ~2.5 GB
- mini_pupper_ros + deps: ~500 MB
- libcamera (built from source): ~300 MB
- YOLOv8n model (downloaded on first use): ~6 MB
- **Total: ~5–6 GB**

---

## Network Setup

The Mini Pupper communicates over **WiFi**.

### 1. Connect the Mini Pupper to your network

On the Pi (via HDMI monitor or keyboard):
```bash
sudo nmcli dev wifi connect "YourNetworkName" password "YourPassword"
```

Or edit the netplan config directly:
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
sudo netplan apply
sudo reboot
```

### 2. Find the robot's IP address

On the Pi:
```bash
hostname -I
```

### 3. Enable SSH (first time only)

On the Pi:
```bash
sudo apt install openssh-server -y
sudo systemctl enable ssh
sudo systemctl start ssh
```

### 4. SSH into the robot from your PC

```bash
ssh ubuntu@<robot-ip>
# Default password: mangdang
```

---

## Environment Variables

Add these to your `~/.bashrc` on your PC after setup:

```bash
export ROBOT_MODEL=mini_pupper
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## Verification Checklist

Run these after `source ~/.bashrc` to confirm everything is working:

```bash
# ROS2
ros2 --version
# Expected: ros2cli 0.18.x

# Mini Pupper packages
ros2 pkg list | grep mini_pupper
# Expected: mini_pupper_bringup, mini_pupper_navigation, etc.

# Python packages
python3 -c "import ultralytics; print('YOLO OK')"
python3 -c "import cv2; print('OpenCV OK')"

# Camera (on the Pi)
libcamera-hello --list-cameras
# Expected: Available cameras: imx500

# Robot connectivity
ping -c 3 <robot-ip>
ssh ubuntu@<robot-ip> "ros2 topic list | head -5"
```

---

## Troubleshooting

**`ros2: command not found`**
ROS2 isn't sourced. Run:
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

**`libcamera-hello: command not found` on Ubuntu**
Ubuntu's apt version of libcamera is too old. Build from source:
```bash
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
pip3 install meson --upgrade
meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp \
  -Dipas=rpi/vc4,rpi/pisp \
  -Dtest=false
ninja -C build
sudo ninja -C build install
```

**`ssh: connect to host <ip> port 22: Connection refused`**
SSH server isn't running. On the Pi directly:
```bash
sudo apt install openssh-server -y
sudo systemctl enable ssh
sudo systemctl start ssh
```

**`sudo apt` — waiting for cache lock**
A background process is holding the lock. Kill it and clear the locks:
```bash
sudo kill <PID>
sudo rm /var/lib/dpkg/lock-frontend
sudo rm /var/lib/dpkg/lock
sudo apt install <package>
```

**Mini Pupper not moving after bringup**
Check servo power and I2C bus:
```bash
ls /dev/i2c-*
i2cdetect -y 1
```

**Nav2 map not building**
Confirm the lidar topic is publishing:
```bash
ros2 topic echo /scan
```
If nothing appears, check the lidar driver is running and the correct port is configured.

---

## Repository Structure

```
MiniPupper_Tutorial/
├── setup/
│   └── install_all.sh
├── course/
│   ├── week1_hardware/
│   ├── week2_ssh/
│   ├── week3_ros2/
│   ├── week4_bringup/
│   ├── week5_kinematics/
│   ├── week6_teleop/
│   ├── week7_lidar_slam/
│   ├── week8_nav2/
│   ├── week9_camera_yolo/
│   └── week10_capstone/
└── README.md
```

---

## License

Apache 2.0 — see [LICENSE](LICENSE)
