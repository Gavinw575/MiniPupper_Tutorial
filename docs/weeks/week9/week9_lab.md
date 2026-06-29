# Week 9 — OAK-D Lite Camera & YOLOv8 Inference

---

**Objectives:**

1. Recap how the OAK-D Lite is already verified working (Week 1), and understand the architectural choice between host-side and on-device inference.
2. Write a ROS2 node that runs YOLOv8 object detection on the live camera feed.
3. Publish and visualize annotated detection results.
4. Investigate the OAK-D Lite's on-device inference alternative and compare it to host-side inference.

---

**Reference Material:**

- [Ultralytics YOLOv8 documentation](https://docs.ultralytics.com/)
- [cv_bridge documentation](https://docs.ros.org/en/humble/p/cv_bridge/)
- [depthai-ros (GitHub)](https://github.com/luxonis/depthai-ros)
- Week 1 Lab — Setup, Orientation & First Bringup (the camera bring-up this week builds on)

---

## Background

Week 1 got the OAK-D Lite publishing `/camera/image_raw`. This week is about doing something with that feed: detecting objects in it with YOLOv8.

There's a real architectural choice buried in "run YOLO on the camera feed," and it's worth being deliberate about rather than just picking the first thing that works:

- **Host-side inference** — a ROS2 node subscribes to `/camera/image_raw`, runs the YOLO model on each frame using a regular CPU (or GPU, if available), and publishes the result. Simple, standard ROS2 node structure, but the inference workload runs wherever the node runs.
- **On-device inference** — the OAK-D Lite's onboard Myriad X VPU runs a compiled neural network *directly on the camera*, before any frame data even reaches a host CPU. This is the camera's actual differentiating feature over a plain USB webcam — but it requires building a DepthAI pipeline with a model compiled specifically for the VPU, not just pointing it at a `.pt` file.

This week's required exercise uses host-side inference, but run it **on your PC, not the robot's CM4** — the same reasoning as SLAM and Nav2 in Weeks 7–8: the CM4 is an ARM board with limited compute, and `ultralytics` pulls in PyTorch as a dependency, which is a heavier, slower install on ARM than on a normal x86 PC. Your `/camera/image_raw` topic already streams over the network from the robot; this week's node just subscribes to it from the PC side, the same way Cartographer and Nav2 did.

!!! note "Why this matters as a real choice, not just an exercise"
    For a resource-constrained robot, the question of *where* inference happens isn't academic — it directly trades off latency, network bandwidth, and host compute load. Step 5 has you investigate the on-device path directly so you can compare, not just take this tradeoff on faith.

---

## Setup

### Step 1 — Confirm the Camera Feed Is Still Live

With bringup running on the robot:

```bash
ros2 topic hz /camera/image_raw
```

If this shows nothing from your PC, this is the same discovery-vs-transport issue you've hit before — check your DDS configuration before assuming the camera itself has a problem.

### Step 2 — Test YOLOv8 Standalone First

On your PC, install Ultralytics:

```bash
pip install ultralytics
```

Before writing any ROS2 code, test it standalone — this is the same debugging philosophy from Week 1's camera setup: confirm the library itself works in isolation before wiring it into ROS2, so you know which layer a problem belongs to if something breaks later.

```python
from ultralytics import YOLO

model = YOLO('yolov8n.pt')  # downloads automatically on first run
results = model('https://ultralytics.com/images/bus.jpg')
results[0].show()
```

**Task 1:** Confirm this runs and shows you a detection result. What objects did it find in the test image, and with what confidence scores?

---

## Building the Detector

### Step 3 — Write the YOLO Detector Node

Create the file:

```bash
touch ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/yolo_detector.py
```

Fill in the starter code below. `# TODO` marks what you need to write.

```python
#!/usr/bin/env python3
"""
yolo_detector.py

Subscribes to /camera/image_raw, runs YOLOv8 object detection on each
frame, draws the results, and republishes an annotated image.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.bridge = CvBridge()

        # TODO: Load a YOLOv8 model. The smallest/fastest model is a good
        # starting point for a CPU-only pipeline.
        # Hint: YOLO('yolov8n.pt') — already cached from Step 2.
        self.model = # YOUR CODE HERE

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.publisher = self.create_publisher(
            Image, '/yolo/image_annotated', 10
        )

        self.get_logger().info('YoloDetectorNode started!')

    def image_callback(self, msg: Image):
        # TODO: Convert the incoming ROS Image message into an OpenCV
        # (numpy) image so YOLO can process it.
        # Hint: self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image = # YOUR CODE HERE

        # TODO: Run the model on the frame. Calling the model directly
        # as a function — self.model(cv_image) — returns a list of
        # Results objects, one per image passed in. You only passed one.
        results = # YOUR CODE HERE

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]

                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'{class_name} {confidence:.2f}'
                cv2.putText(cv_image, label, (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        annotated_msg.header = msg.header
        self.publisher.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register the node in `setup.py`:

```
'yolo_detector = mini_pupper_labs.yolo_detector:main',
```

### Step 4 — Build, Run, and View

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_labs --symlink-install
source install/setup.bash
ros2 run mini_pupper_labs yolo_detector
```

View the annotated feed:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/yolo/image_annotated` from the topic dropdown, and point the camera at a few different objects.

**Task 2:** Screenshot a successful detection with at least one correctly labeled bounding box. What's the approximate frame rate you're seeing (watch `ros2 topic hz /yolo/image_annotated`), and does it feel noticeably slower than the raw `/camera/image_raw` rate from Step 1?

---

## Investigating On-Device Inference

### Step 5 — Compare Against the OAK-D Lite's Onboard VPU

`depthai_ros_driver` (the same package from Week 1) supports configuring an onboard neural network pipeline that runs directly on the camera's VPU instead of streaming raw frames for a host node to process. The exact parameter names for enabling this have changed across `depthai_ros_driver` versions, so check the package's own README/parameter docs for your installed version rather than assuming a specific flag — this is the same "verify against the actual installed version" habit from Week 6 and Week 7's launch-file investigations.

Once you've got it running, compare the two approaches directly:

**Task 3:** With your Step 3 node running, check CPU usage on whichever machine is running it (`top` or `htop`). Then try the on-device pipeline from `depthai_ros_driver` and check CPU usage again, on both the camera-side (nothing to check directly, it's on the VPU) and host side. What's the practical difference, and what did you have to give up (if anything) to get it — model flexibility, ease of setup, accuracy?

---

## Looking Ahead

Week 10's capstone pulls together navigation (Weeks 7–8) and vision (this week) into one system. Right now your detector only logs/visualizes — it doesn't feed a decision-making process. The natural next step is publishing structured detection data (class, confidence, bounding box) using `vision_msgs/Detection2DArray`, the standard ROS2 message type for object detector output, so a future state machine node can subscribe to it without caring which detector produced it.

**DELIVERABLE:** Look up the `vision_msgs/Detection2D` and `vision_msgs/Detection2DArray` message definitions (`ros2 interface show vision_msgs/msg/Detection2DArray`). Sketch out (in your writeup, not in code) how you'd restructure this week's detection loop to populate one of these messages per detected object, instead of just drawing on the image.

---

## Tasks

1. Standalone YOLOv8 test result and detected objects/confidences (Step 2).
2. Annotated detection screenshot and frame rate comparison (Step 4).
3. On-device vs. host-side CPU usage comparison and tradeoff discussion (Step 5).
4. `vision_msgs/Detection2DArray` restructuring sketch (Looking Ahead).

---

## Troubleshooting

??? question "`ultralytics` install is extremely slow or fails on the robot"
    This is expected if you tried installing it on the CM4 directly — PyTorch's ARM wheels are large and can be painfully slow on this hardware. Run the detector node on your PC instead, as described in the Background section.

??? question "`/camera/image_raw` shows up in `topic list` but `topic hz` shows nothing"
    Same discovery-vs-transport issue from previous weeks — check your DDS/CycloneDDS configuration before assuming the camera has a problem.

??? question "YOLO runs but never detects anything"
    Confirm `cv_image` actually has real image data — try `cv2.imwrite('debug.jpg', cv_image)` right after the conversion and check the saved file looks like a real camera frame, not a garbled or black image. A bad `desired_encoding` argument to `imgmsg_to_cv2` is a common cause of this.

??? question "`/yolo/image_annotated` topic exists but `rqt_image_view` shows nothing"
    Confirm the publisher's `encoding` in `cv2_to_imgmsg` matches what you actually have in `cv_image` (`bgr8` for a standard color OpenCV image). A mismatch here usually shows as a blank or corrupted-looking display rather than an error.

---
