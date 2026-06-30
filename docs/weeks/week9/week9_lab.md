# Week 9 — OAK-D Lite, On-Device YOLO & Person Tracking

---

**Objectives:**

1. Understand the difference between host-side and on-device inference, and why the OAK-D Lite's VPU changes the tradeoff.
2. Write a DepthAI pipeline that runs YOLOv6-nano on the camera's onboard VPU and publishes detections as a ROS2 topic.
3. Display live tracked detections on the robot's LCD screen.
4. Write a movement node that subscribes to detections and publishes `/cmd_vel` to make the robot follow a person.

---

**Reference Material:**

- [DepthAI Python API v3 documentation](https://docs.luxonis.com/software/depthai/manual/)
- [Luxonis Model Zoo](https://models.luxonis.com/)
- [mini_pupper_interfaces (GitHub)](https://github.com/mangdangroboticsclub/mini_pupper_ros/tree/ros2-dev/mini_pupper_interfaces)
- [geometry_msgs/Twist](https://docs.ros2.org/humble/api/geometry_msgs/msg/Twist.html)
- Week 3 Lab — Teleop & cmd_vel (the `/cmd_vel` interface you'll publish to this week)

---

## Background

Every week up to now has put inference workloads on the host machine — your PC runs Cartographer, your PC runs Nav2, your PC ran YOLO in the old Week 9 draft. The OAK-D Lite changes that option. It has an onboard **Myriad X VPU** (Vision Processing Unit) that can run compiled neural networks directly on the camera, before any frame data even travels over USB. The CM4 only receives small structured results — bounding boxes, class IDs, confidence scores — not video frames.

For the Mini Pupper 2, this matters a lot. The CM4 is a low-power ARM board. Streaming 640×400 frames off the camera and running inference on them on the CM4 would be a dead end — too slow to be useful. Running the same network on the VPU and sending only detection results over USB takes almost no CM4 CPU time at all.

This week's pipeline looks like this:

```
OAK-D VPU:   Camera → YOLOv6-nano → ObjectTracker → TrackingArray (USB)
Robot (CM4): oak_detection_publisher.py → publishes /tracking_array (ROS2)
PC:          movement_node.py → subscribes /tracking_array → publishes /cmd_vel
Robot:       stanford_controller receives /cmd_vel → moves
Robot LCD:   live annotated frame from VPU → ST7789 display
```

The `movement_node` runs on the PC, not the robot — the same reason SLAM and Nav2 ran on the PC. All computation heavier than "forward detections over ROS2" lives off the CM4.

!!! note "DepthAI v3 API"
    This lab uses the **DepthAI v3 Pipeline API** (`pipeline.create(node).build(...)`). The v2 API looks similar but is meaningfully different — if you find examples online that use `pipeline.createNeuralNetwork()` or `pipeline.createColorCamera()` without `.build()`, they're v2 and won't work here. Check the version: `python3 -c "import depthai; print(depthai.__version__)"` — you should see `3.x.x`.

---

## Setup

### Step 1 — Confirm DepthAI Is Installed and the Camera Is Recognized

On the robot:

```bash
python3 -c "import depthai; print(depthai.__version__)"
```

Check that the OAK-D Lite shows up over USB:

```bash
lsusb | grep 03e7
```

You should see a line with `03e7` — that's Luxonis's USB vendor ID. If it doesn't appear, check the udev rule is in place:

```bash
cat /etc/udev/rules.d/80-movidius.rules
```

It should contain: `SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"`

**Task 1:** Paste both the depthai version and the `lsusb` output confirming the camera is recognized.

---

## Building the OAK-D Detection Publisher

### Step 2 — Understand the Pipeline Structure

Before writing any code, look at the DepthAI node types you'll use:

```bash
python3 -c "import depthai as dai; help(dai.node.ColorCamera)" 2>/dev/null | head -30
```

The three DepthAI nodes you'll wire together:
- **ColorCamera** — captures frames from the RGB sensor
- **MobileNetDetectionNetwork** (or **YoloDetectionNetwork**) — runs a neural net on the VPU, outputs bounding boxes
- **ObjectTracker** — keeps track of individual detected objects across frames, assigns persistent IDs

They connect like a pipeline: camera frames → detection network → object tracker → your output queue.

### Step 3 — Write the OAK-D Detection Publisher

Create the file on the robot:

```bash
touch ~/oak_detection_publisher.py
```

Fill in the starter code below. `# Task:` marks what you need to write.

```python
#!/usr/bin/env python3
"""
oak_detection_publisher.py

Runs YOLOv6-nano on the OAK-D Lite's onboard VPU.
Publishes detections as a TrackingArray on /tracking_array.
Drives the ST7789 LCD directly with annotated frames.

Run on the robot (not the PC).
"""

import depthai as dai
import numpy as np
import rclpy
from rclpy.node import Node

from mini_pupper_interfaces.msg import TrackingArray, Tracking

# DepthAI model zoo slug — downloads and caches on first run
MODEL_SLUG  = "yolov6-nano"
PERSON_CLASS_ID = 0   # COCO class 0 = person

# LCD display size
LCD_W, LCD_H = 320, 240


class OakDetectionPublisher(Node):

    def __init__(self):
        super().__init__('oak_detection_publisher')

        # Task: Create a publisher for TrackingArray messages on '/tracking_array'
        # with a queue size of 10.
        self.publisher = # YOUR CODE HERE

        self._init_display()
        self._build_pipeline()
        self.get_logger().info('OakDetectionPublisher started — VPU inference running')

    # ------------------------------------------------------------------
    # Display setup
    # ------------------------------------------------------------------

    def _init_display(self):
        """Initialise the ST7789 LCD on the robot's front panel."""
        try:
            from MangDang.LCD.ST7789 import ST7789
            self.lcd = ST7789()
            self.lcd_available = True
        except Exception as e:
            self.get_logger().warn(f'LCD not available: {e}')
            self.lcd_available = False

    # ------------------------------------------------------------------
    # DepthAI pipeline
    # ------------------------------------------------------------------

    def _build_pipeline(self):
        pipeline = dai.Pipeline()

        # Task: Create a ColorCamera node on the pipeline.
        # Set its preview size to (640, 400) and set interleaved to False.
        # Hint: cam = pipeline.create(dai.node.ColorCamera)
        #        cam.setPreviewSize(w, h)
        #        cam.setInterleaved(False)
        cam = # YOUR CODE HERE

        # Task: Create a YoloDetectionNetwork node.
        # Load the model from the Luxonis model zoo using:
        #   nn = pipeline.create(dai.node.YoloDetectionNetwork)
        #   nn_config = dai.NNModelDescription(MODEL_SLUG)
        #   nn.build(cam.preview, nn_config)
        nn = # YOUR CODE HERE

        # Task: Create an ObjectTracker node and connect it to the
        # detection network output.
        # Hint:
        #   tracker = pipeline.create(dai.node.ObjectTracker)
        #   tracker.build(
        #       inputImageFrame=cam.preview,
        #       inputDetections=nn.out
        #   )
        tracker = # YOUR CODE HERE

        # Get the tracklet output queue from the object tracker.
        # passthroughTrackerFrame gives us annotated frames for the LCD.
        self.tracklets_queue = tracker.out.createOutputQueue()
        self.frame_queue     = tracker.passthroughTrackerFrame.createOutputQueue()

        self.device = dai.Device(pipeline)
        self.get_logger().info('DepthAI pipeline built and device opened')

    # ------------------------------------------------------------------
    # Main loop — call this from main() after rclpy.spin_once
    # ------------------------------------------------------------------

    def poll(self):
        """Pull one batch of results from the VPU and publish."""
        tracklets_msg = self.tracklets_queue.tryGet()
        frame_msg     = self.frame_queue.tryGet()

        if frame_msg is not None and self.lcd_available:
            self._update_lcd(frame_msg)

        if tracklets_msg is None:
            return

        tracklets = tracklets_msg.tracklets

        # Task: Build a TrackingArray message from the tracklets.
        # For each tracklet where tracklet.label == PERSON_CLASS_ID:
        #   - Create a Tracking() message
        #   - Set tracking.center_x  = (roi.xmin + roi.xmax) / 2   (normalized 0–1)
        #   - Set tracking.top_y     = roi.ymin                     (normalized 0–1)
        #   - Set tracking.bounding_area = (roi.xmax - roi.xmin) * (roi.ymax - roi.ymin)
        #   - Set tracking.confidence = tracklet.srcImgDetections[0].confidence
        #                               if tracklet.srcImgDetections else 0.0
        #   - Set tracking.track_id  = tracklet.id
        #   Append each Tracking to a list, then publish as TrackingArray.
        #
        # Hint: roi = tracklet.roi.denormalize(1.0, 1.0) gives you xmin/xmax/ymin/ymax
        #        TrackingArray has a field called 'trackings' (list of Tracking)

        # YOUR CODE HERE

    def _update_lcd(self, frame_msg):
        """Resize the VPU passthrough frame and push it to the LCD."""
        import cv2
        frame = frame_msg.getCvFrame()
        frame_resized = cv2.resize(frame, (LCD_W, LCD_H))
        # ST7789 expects RGB, OpenCV gives BGR
        frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        self.lcd.display(frame_rgb)


def main(args=None):
    rclpy.init(args=args)
    node = OakDetectionPublisher()

    try:
        while rclpy.ok():
            node.poll()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Run it on the robot to test before wiring up the movement node:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
python3 ~/oak_detection_publisher.py
```

On your PC, confirm detections are coming through:

```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /tracking_array
```

**Task 2:** Paste a sample `/tracking_array` message showing at least one person detection. What does `bounding_area` tell you about how far away the person is?

---

## LCD Display

### Step 4 — Confirm the Screen Is Updating

With `oak_detection_publisher.py` running, stand in front of the robot and move around. The LCD should show a live annotated frame with a bounding box around you.

!!! warning "Don't run `display_interface` at the same time"
    The `display_interface` ROS node and `oak_detection_publisher.py` both own the ST7789 hardware directly. Running both at the same time will cause them to conflict and neither will display correctly. The oak publisher owns the display when it's running.

**Task 3:** Take a photo of the robot's LCD showing a live detection bounding box. This is your proof the VPU pipeline and display are both working end-to-end.

---

## Building the Movement Node

### Step 5 — Write the Movement Node

This node runs on your **PC**, not the robot. It subscribes to `/tracking_array` and publishes `/cmd_vel` to make the robot turn toward and follow a detected person.

The control logic:
- `center_x` tells you where the person is horizontally in the frame (0 = left, 1 = right, 0.5 = centered).
- `bounding_area` tells you roughly how close the person is — a bigger box means closer.
- Use a simple proportional controller: error = `center_x - 0.5`, then `angular.z = -Kp * error`.
- Stop moving forward if `bounding_area` exceeds a threshold (person is close enough).

Create the file on your PC:

```bash
touch ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/movement_node.py
```

```python
#!/usr/bin/env python3
"""
movement_node.py

Subscribes to /tracking_array (person detections from the OAK-D Lite).
Publishes /cmd_vel to make the robot turn toward and follow the nearest person.

Run on the PC, not the robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mini_pupper_interfaces.msg import TrackingArray

# Tuning parameters — adjust these to change tracking behavior
Kp_YAW        = 1.0    # proportional gain for yaw correction
FORWARD_SPEED = 0.12   # m/s — how fast to walk toward the person
CLOSE_AREA    = 0.5    # stop walking forward when bounding_area exceeds this
YAW_DEADBAND  = 0.05   # ignore yaw error smaller than this (avoids jitter)


class MovementNode(Node):

    def __init__(self):
        super().__init__('movement_node')

        # Task: Create a subscriber to '/tracking_array' using TrackingArray
        # messages. Callback should be self.tracking_callback. Queue size 10.
        self.subscription = # YOUR CODE HERE

        # Task: Create a publisher for Twist messages on '/cmd_vel'.
        # Queue size 10.
        self.publisher = # YOUR CODE HERE

        self.get_logger().info('MovementNode started — waiting for detections')

    def tracking_callback(self, msg: TrackingArray):

        # Task: If there are no trackings in msg.trackings, publish a
        # zero Twist to stop the robot and return early.
        # YOUR CODE HERE

        # Pick the largest detection (closest person) to follow.
        # Task: Find the Tracking in msg.trackings with the highest
        # bounding_area and store it as `target`.
        target = # YOUR CODE HERE

        cmd = Twist()

        # Task: Compute the yaw error.
        # center_x is 0 (full left) to 1 (full right). Error is 0 when centered.
        # If the absolute error is smaller than YAW_DEADBAND, set angular.z to 0.
        # Otherwise set angular.z = -Kp_YAW * error  (negative because turning
        # left when person is to the right requires negative angular.z).
        # YOUR CODE HERE

        # Task: Set forward speed.
        # If target.bounding_area < CLOSE_AREA, set cmd.linear.x = FORWARD_SPEED.
        # Otherwise the person is close enough — set cmd.linear.x = 0.0.
        # YOUR CODE HERE

        # Task: Publish the Twist command.
        # YOUR CODE HERE

        self.get_logger().info(
            f'target center_x={target.center_x:.2f}  '
            f'area={target.bounding_area:.3f}  '
            f'lin={cmd.linear.x:.2f}  ang={cmd.angular.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register in `setup.py`:

```
'movement_node = mini_pupper_labs.movement_node:main',
```

Build:

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_labs --symlink-install
source install/setup.bash
```

### Step 6 — Run the Full Pipeline

Open three terminals:

**Terminal 1 — robot bringup (SSH):**
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

**Terminal 2 — OAK-D publisher (SSH):**
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
python3 ~/oak_detection_publisher.py
```

**Terminal 3 — movement node (PC):**
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run mini_pupper_labs movement_node
```

Stand in front of the robot and walk left and right slowly. It should turn to keep you centered. Walk further away and it should walk toward you. Walk close enough and it should stop.

!!! warning "Keep one hand near the power switch"
    The robot will try to move as soon as it sees a person. Have it on a flat surface with some space around it, and be ready to kill bringup (Ctrl+C in Terminal 1) if it starts moving unexpectedly.

**Task 4:** Describe the tracking behavior — does it turn to follow you smoothly, overshoot, or oscillate? If it's hunting back and forth, `Kp_YAW` is too high. If it barely responds, it's too low. Try at least two different `Kp_YAW` values and report what you observed.

---

## Tuning

### Step 7 — Tune the Controller

The three main parameters to experiment with:

| Parameter | Effect | Start here if... |
|---|---|---|
| `Kp_YAW` | How aggressively it turns to center the target | Oscillates → lower it. Sluggish → raise it. |
| `FORWARD_SPEED` | How fast it walks toward the person | Too fast/unstable → lower it |
| `CLOSE_AREA` | How close is "close enough" to stop walking | Stops too far away → lower it. Bumps into you → raise it. |

**Task 5:** Find a set of values where the robot tracks you smoothly across at least a 2-meter range. Report your final `Kp_YAW`, `FORWARD_SPEED`, and `CLOSE_AREA` values and describe the behavior at those settings.

---

## Looking Ahead

Right now the tracker follows whoever it sees first. If two people are in frame, `bounding_area` picks the larger (closer) one, but there's no concept of "lock on to this specific person and ignore others." The `track_id` field in `Tracking` is exactly what you'd use to fix that — once you've identified the person you want to follow, ignore all tracklets whose `track_id` doesn't match.

**DELIVERABLE:** In your writeup, describe how you'd modify `tracking_callback` to lock onto the first person seen and only follow that `track_id` for the rest of the session. You don't have to implement it — just describe the logic clearly enough that you could.

---

## Tasks

1. DepthAI version and `lsusb` output confirming camera is recognized (Step 1).
2. Sample `/tracking_array` message with at least one person detection, and explanation of `bounding_area` (Step 3).
3. Photo of the robot LCD showing a live detection bounding box (Step 4).
4. Tracking behavior description with at least two `Kp_YAW` values compared (Step 6).
5. Final tuned parameter values and behavior description (Step 7).
6. `track_id` lock-on logic description (Looking Ahead).

---

## Troubleshooting

??? question "`lsusb` doesn't show `03e7` at all"
    The udev rule is probably missing or not applied yet. Check `/etc/udev/rules.d/80-movidius.rules` — it should contain `SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"`. If it's missing, add it and run `sudo udevadm control --reload-rules && sudo udevadm trigger`, then unplug and replug the camera.

??? question "Pipeline builds but `tracklets_queue.tryGet()` always returns None"
    This usually means the model didn't download on first run — the robot needs internet access for the initial download. Run `python3 ~/oak_detection_publisher.py` once while connected to WiFi and wait for the download to complete before running offline. You can confirm the model cached by checking `~/.cache/` for Luxonis model files.

??? question "LCD shows nothing or freezes"
    Make sure `display_interface` isn't also running — check `ros2 node list` for a display node. If it's there, kill it before starting the oak publisher.

??? question "`/tracking_array` is publishing but movement_node isn't moving the robot"
    Confirm `ROS_DOMAIN_ID=42` is set in every terminal including the PC one. Run `ros2 topic echo /cmd_vel` to check if the movement node is actually publishing — if you see Twist messages there but the robot isn't moving, bringup on the robot may have died.

??? question "Robot spins continuously even with nobody in front of it"
    The tracker is probably picking up a false detection. Check `ros2 topic echo /tracking_array` — if detections keep coming with no person present, try raising the confidence threshold in `oak_detection_publisher.py` by filtering out tracklets where `srcImgDetections[0].confidence < 0.5`.

---
