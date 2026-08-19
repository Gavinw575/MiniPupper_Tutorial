# Week 10 — Capstone: Autonomous Object Inventory with Voice Control

---

**Objectives:**

1. Integrate SLAM, Nav2, and the OAK-D Lite detection pipeline into a single autonomous system.
2. Write a frontier-exploration node that drives the robot through an unmapped space.
3. Log each detected object with its 3D position in the map frame using the TF tree.
4. Wire the robot's onboard microphone and speaker into the ROS2 state machine so a voice command stops the sweep and prints the inventory.
5. Use the robot's capacitive touch pads (read directly via Raspberry Pi GPIO) as a physical estop that halts the robot at any point during a run.

---

**Reference Material:**

- [vosk offline speech recognition](https://alphacephei.com/vosk/)
- [tf2_ros Python API](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html)
- [Nav2 Simple Commander API](https://docs.nav2.org/commander_api/index.html)
- [sounddevice documentation](https://python-sounddevice.readthedocs.io/)
- [RPi.GPIO documentation](https://sourceforge.net/p/raspberry-gpio-python/wiki/BasicUsage/)
---

## Background

We want the robot to walk into a small room or a little setup of boxes, build a map, detect and locate objects in that space, respond to spoken commands, and use the touch sensor on the back to physically stop it at any time. 

### The full system

Five ROS2 nodes will need to run together, some of them can be reused or changed slightly from earlier labs:

| Node | Where it runs | What it does |
|---|---|---|
| `bringup` | Robot | Servos, lidar, IMU, odometry |
| `slam.launch.py` | PC | Builds the occupancy grid from `/scan` |
| `navigation_smacplanner.launch.py` | PC | Nav2 stack for path planning and execution |
| `explorer_node` | PC | Picks frontier goals and sends them to Nav2 (new) |
| `detector_node` | PC | Runs YOLOv8, gets depth from OAK-D Lite, transforms to map frame, logs inventory (new) |
| `voice_node` | Robot | Records from the onboard mic, runs vosk, publishes keyword events (new) |
| `touch_node` | Robot | Polls the touch pads directly via GPIO, publishes a `Bool` estop signal (new) |

The state machine that connects them lives in `explorer_node`. It listens to `/object_inventory` (from `detector_node`), `/voice_command` (from `voice_node`), and `/touch_estop` (from `touch_node`), and makes three decisions: keep exploring, stop cleanly, or emergency stop.

### The touch pad

The Mini Pupper's capacitive touch pad is wired to CM4 GPIO, active-low. The BSP's own demo folder (`mini_pupper_bsp/demos/touch_test.py`) reads them the same way. Your `touch_node` polls these pins directly and republishes the combined state as a standard `std_msgs/Bool` on `/touch_estop`.

BCM pin numbers:

| Pad | BCM pin |
|---|---|
| front | 6 |
| left | 3 |
| right | 16 |
| back | 2 |

### Object position pipeline

The OAK-D Lite gives you a 3D position in camera frame for each detection (X forward, Y left, Z up, in meters). To log where in the map an object is, you need to transform that point through the TF tree:

```
camera_optical_frame -> base_link -> odom -> map
```

`tf2_ros` handles this. Look up the transform from `camera_optical_frame` to `map` at the time of detection, and apply it to the XYZ point from the depth pipeline.

!!! note "Deduplication"
    The same chair will be detected many times as the robot walks past it. A simple rule, if an object of the same class already exists in the inventory within 0.5 m of this detection, skip it. 

---

## Setup

### Step 1 — Install vosk on the Robot

SSH into the robot and install vosk with its small English model:

```bash
pip install vosk --break-system-packages
cd ~
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
mv vosk-model-small-en-us-0.15 vosk-model
```

Confirm the microphone device is visible:

```bash
python3 -c "import sounddevice; print(sounddevice.query_devices())"
```

You should see an ALSA device listed. Note the device index, you'll need it in `voice_node`.

!!! warning "No HDMI cable during audio use"
    The BSP README notes this explicitly: the ALSA audio device is headphone output 0 only when no HDMI cable is connected. HDMI reassigns the headphone index and the microphone may also be affected.

**Task 1:** Paste the `sounddevice.query_devices()` output from your robot and identify which device index corresponds to the ALSA microphone.

### Step 2 — Verify the Touch Pads via GPIO

On the robot, run the BSP's own demo to confirm the pads respond and to see which pin corresponds to which pad:

```bash
python3 ~/mini_pupper_bsp/demos/touch_test.py
```

Touch each pad on the robot's body in turn and confirm the demo reports a state change. Finish the state section of the code below to see the raw GPIO reads directly instead of using the demo script.

```bash
python3 -c "
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
pins = {'front': 6, 'left': 3, 'right': 16, 'back': 2}
for p in pins.values():
    GPIO.setup(p, GPIO.IN)
print('Touch pads now — Ctrl+C to stop')
try:
    while True:
        state = # Your code
        print(state)
        time.sleep(0.2)
except KeyboardInterrupt:
    GPIO.cleanup()
"
```

**Task 2:** Create a python script that reads the GPIO pins directely instead of just running the demo. Paste the output showing a pad's state changing between touched and not-touched as well as your code.

---

## Building the Voice Node

### Step 3 — Write `voice_node.py`

Create the file on the robot:

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/voice_node.py
```

```python
#!/usr/bin/env python3
"""
voice_node.py

Listens to the onboard I2S microphone using sounddevice, runs vosk
offline speech recognition, and publishes recognized keyword events
on /voice_command as std_msgs/String.

Runs on the robot (CM4).
"""

import json
import queue
import sounddevice as sd
import rclpy
import audioop
from rclpy.node import Node
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer

# Keywords the state machine cares about.
# Any recognized phrase containing one of these strings triggers a publish.
KEYWORDS = ['stop', 'done', 'finished', 'halt']

# Sample rate must match the I2S hardware.
SAMPLE_RATE = 16000
# Block size controls latency vs. CPU trade-off.
BLOCK_SIZE = 4000
# Set this to the device index from Task 1 if auto-detection fails.
DEVICE_INDEX = None   # None = sounddevice default input
DEVICE_CHANELLS = 2


class VoiceNode(Node):

    def __init__(self):
        super().__init__('voice_node')

        self.publisher = self.create_publisher(String, '/voice_command', 10)

        self.audio_queue = queue.Queue()

        # Task: Load the vosk model from ~/vosk-model.
        # Hint: Model('/home/ubuntu/vosk-model') — adjust path if needed.

        self.model = # Your code

        # Task: Create a KaldiRecognizer with the model and SAMPLE_RATE.
        # Hint: KaldiRecognizer(self.model, SAMPLE_RATE)

        self.recognizer = # Your code

        self.get_logger().info('VoiceNode ready — listening for keywords')

        # Start the audio stream. The callback fills self.audio_queue.
        self.stream = sd.RawInputStream(
            samplerate=SAMPLE_RATE,
            blocksize=BLOCK_SIZE,
            device=DEVICE_INDEX,
            dtype='int16',
            channels=DEVICE_CHANNELS,
            callback=self._audio_callback,
        )
        self.stream.start()

        # Process audio in a ROS2 timer so the node stays spinnable.
        self.create_timer(0.05, self._process_audio)

    def _audio_callback(self, indata, frames, time, status):
        """Called by sounddevice from a background thread — just enqueue."""
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        mono = audioop.tomono(bytes(indata), 2, 0.5, 0.5)
        self.audio_queue.put(mono)

    def _process_audio(self):
        """Drain the audio queue and run vosk inference."""
        while not self.audio_queue.empty():
            data = self.audio_queue.get()

            # Task: Feed `data` to self.recognizer using AcceptWaveform().
            # If AcceptWaveform returns True, a complete utterance is ready.
            # Parse the JSON result with json.loads(self.recognizer.Result())
            # and check if result['text'] contains any word in KEYWORDS.
            # If it does, publish a String message with that keyword text.
            #
            # If AcceptWaveform returns False, a partial result is available.
            # You can optionally check self.recognizer.PartialResult() here
            # for faster response.

            # Your code

    def destroy_node(self):
        self.stream.stop()
        self.stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register in `setup.py`:

```
'voice_node = mini_pupper_labs.voice_node:main',
```

Build and test it standalone before integrating — say "stop" near the robot and confirm a message appears on `/voice_command`:

```bash
colcon build --packages-select mini_pupper_labs --symlink-install
source install/setup.bash
ros2 run mini_pupper_labs voice_node &
ros2 topic echo /voice_command
```

**Task 3:** Screenshot/record `/voice_command` receiving a message when you say a keyword.

---

## Building the Touch Node

### Step 4 — Write `touch_node.py`

Create the file on the robot:

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/touch_node.py
```

```python
#!/usr/bin/env python3
"""
touch_node.py

Reads the capacitive touch pads directly via Raspberry Pi GPIO and
publishes the combined touch state as a std_msgs/Bool on /touch_estop.

No ESP32/serial involvement — the touch pads are wired straight to
the CM4's GPIO, bypassing the ESP32 entirely.

Runs on the robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Task: import RPi.GPIO. Conventionally imported as GPIO.
# Your code

# BCM pin numbers for each touch pad, confirmed in Step 2.
# Change these if you find your pin numbers different.
PINS = {
    'front': 6,
    'left': 3,
    'right': 16,
    'back': 2,
}

POLL_INTERVAL = 0.05  # seconds


class TouchNode(Node):

    def __init__(self):
        super().__init__('touch_node')

        self.publisher = self.create_publisher(Bool, '/touch_estop', 10)

        # Task: set the GPIO mode to BCM (GPIO.setmode(GPIO.BCM)), then
        # configure each pin in PINS as an input (GPIO.setup(pin, GPIO.IN)).

        # Your code

        self._was_touched = False

        self.create_timer(POLL_INTERVAL, self._poll_gpio)
        self.get_logger().info('TouchNode listening on GPIO pins ' + str(PINS))

    def _poll_gpio(self):
        # Task: check whether any pad is currently touched. Pads are
        # active-low, so GPIO.input(pin) == GPIO.LOW means touched.
        # Use any(...) across all pins in PINS.
        #
        # Then: if this touched/not-touched state has CHANGED since the
        # last poll (compare against self._was_touched), publish a Bool
        # with the new state and update self._was_touched. Publish on
        # both transitions — press and release. This
        # matters: Task 4 needs to see a True -> False transition
        # on the topic.

        # Your code
        pass

    def destroy_node(self):
        # Task: release the GPIO pins on shutdown so a crashed or
        # restarted node doesn't leave them claimed.
        # Your code
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TouchNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register in `setup.py`:

```
'touch_node = mini_pupper_labs.touch_node:main',
```

**Task 4:** Run `touch_node`, then `ros2 topic echo /touch_estop` in another terminal. Press a pad and then release it — screenshot both the `True` message (on press) and the `False` message (on release).

---

## Building the LCD Camera Node

### Step 5 — Write `lcd_camera_node.py`

Create the file on the robot:

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/lcd_camera_node.py
```

```python
#!/usr/bin/env python3
"""
lcd_camera_node.py

Subscribes to /camera/image_raw, which the OAK-D Lite driver already
publishes locally on the robot, and mirrors a live resized feed to the
front ST7789 LCD panel.

Robot-only — no PC or network dependency. Works independently of
detector_node, explorer_node, voice_node, or touch_node.

Runs on the robot.
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from MangDang.LCD.ST7789 import ST7789

# LCD panel resolution.
LCD_W, LCD_H = 320, 240

# Cap how often we actually push a frame to the display. The camera may
# publish faster than this — extra frames are simply dropped rather than
# queued, since we only ever care about the most recent one. This keeps
# CPU and SPI bus load bounded on the CM4, which is also running vosk
# and GPIO polling at the same time.
DISPLAY_INTERVAL_SEC = 0.1  # ~10 fps


class LcdCameraNode(Node):

    def __init__(self):
        super().__init__('lcd_camera_node')

        self.bridge = CvBridge()

        # Task: Create an ST7789() instance and store it as self.lcd.
        # Hint: ST7789()

        self.lcd = # Your code

        self._last_display_time = 0.0

        # queue depth of 1 — we only want the newest frame, not a backlog.
        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self._on_frame, 1
        )

        self.get_logger().info('LcdCameraNode ready — mirroring camera to LCD')

    def _on_frame(self, msg: Image):
        now = time.monotonic()
        if now - self._last_display_time < DISPLAY_INTERVAL_SEC:
            return  # drop this frame, too soon since the last display push
        self._last_display_time = now

        # Task: Convert msg to a BGR OpenCV image.
        # Hint: self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        frame = # Your code

        # Task: Resize frame to (LCD_W, LCD_H) with cv2.resize().

        frame_resized = # Your code

        # Task: Convert BGR -> RGB. OpenCV gives BGR by default, but the
        # ST7789 driver expects RGB.
        # Hint: cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)

        frame_rgb = # Your code

        # Task: Push frame_rgb to the display.
        # Hint: self.lcd.display(frame_rgb)

        # Your code


def main(args=None):
    rclpy.init(args=args)
    node = LcdCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

!!! warning "Don't run `display_interface` at the same time"
    The stock `display_interface` service and `lcd_camera_node` both own the
    ST7789 hardware directly. Running both at once will make neither display
    correctly. Stop `display_interface` before running this node:
    ```bash
    sudo systemctl stop display_interface
    ```

Register in `setup.py`:

```python
'lcd_camera_node = mini_pupper_labs.lcd_camera_node:main',
```

Build and test standalone before folding it into the full launch:

```bash
colcon build --packages-select mini_pupper_labs --symlink-install
source install/setup.bash
ros2 run mini_pupper_labs lcd_camera_node
```

**Task 8:** Take a photo of the robot's LCD showing the live mirrored camera
feed — point the camera at something recognizable so it's clear the image is
live, not a static test pattern.

---

## Building the Detector Node

### Step 6 — Write `detector_node.py`

This node runs on the PC. It builds on the Week 9 YOLO detector but adds depth-to-map-frame transformation and inventory tracking. Need to install this system package as well.

```bash
pip install ultralytics
```

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/detector_node.py
```

```python
#!/usr/bin/env python3
"""
detector_node.py

Subscribes to /camera/image_raw and /stereo/depth (OAK-D Lite depth image),
runs YOLOv8 on each frame, estimates 3D position of each detection in the
map frame via TF, and maintains a deduplicated object inventory.

Publishes:
  /yolo/image_annotated — sensor_msgs/Image (annotated feed for rqt)
  /object_inventory — std_msgs/String (JSON manifest, updated live)
  /inventory_update — std_msgs/String (single-object event on new find)

Runs on the PC.
"""

import json
import math
import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # noqa: F401 — registers the transform type


# Only track classes the robot is likely to encounter indoors.
TRACKED_CLASSES = {
    'person', 'chair', 'bottle', 'cup', 'laptop',
    'cell phone', 'book', 'backpack', 'tv', 'couch', 'box',
}

# Minimum confidence to log a detection.
CONF_THRESHOLD = 0.55

# Deduplication radius in meters — same class within this distance = same object.
DEDUP_RADIUS = 0.5

# Camera intrinsics for the OAK-D Lite RGB sensor at 640x480.
# Used to project pixel (u, v) + depth d into 3D camera coordinates.
FX = 457.3   # focal length x (pixels)
FY = 457.3   # focal length y (pixels)
CX = 320.0   # principal point x
CY = 240.0   # principal point y


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector_node')

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Inventory: list of dicts with keys 'class', 'x', 'y', 'z', 'count'
        self.inventory = []

        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 5
        )
        self.sub_depth = self.create_subscription(
            Image, '/stereo/depth', self.depth_callback, 5
        )
        self.pub_annotated = self.create_publisher(Image, '/yolo/image_annotated', 5)
        self.pub_inventory = self.create_publisher(String, '/object_inventory', 5)
        self.pub_update = self.create_publisher(String, '/inventory_update', 5)

        self.latest_depth = None

        self.get_logger().info('DetectorNode started')

    def depth_callback(self, msg: Image):
        """Cache the most recent depth frame for use in image_callback."""
        self.latest_depth = msg

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image, verbose=False)

        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                class_name = self.model.names[int(box.cls[0])]

                if conf < CONF_THRESHOLD or class_name not in TRACKED_CLASSES:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx_px = (x1 + x2) // 2
                cy_px = (y1 + y2) // 2

                # Task: Get the depth value at pixel (cx_px, cy_px).
                # self.latest_depth is a 32FC1 depth image (meters as float32).
                # Convert it with self.bridge.imgmsg_to_cv2(self.latest_depth, '32FC1')
                # then index into it at [cy_px, cx_px].
 
                depth_m = # Your code
                if depth_m is None:
                    continue

                # Task: Project (cx_px, cy_px, depth_m) into 3D camera coordinates.
                # The standard pinhole projection formula:
                #   X = (cx_px - CX) * depth_m / FX
                #   Y = (cy_px - CY) * depth_m / FY
                #   Z = depth_m
                # Build a PointStamped in 'camera_optical_frame' with these values
                # and stamp it with msg.header.stamp.

                pt_cam = # Your code

                # Task: Transform pt_cam from camera_optical_frame to 'map' using
                # self.tf_buffer.transform(pt_cam, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
                # Wrap this in a try/except tf2_ros.LookupException block — if the
                # transform isn't available yet (SLAM just starting), just skip.

                pt_map = # Your code
                if pt_map is None:
                    continue

                mx = pt_map.point.x
                my = pt_map.point.y
                mz = pt_map.point.z

                self._update_inventory(class_name, mx, my, mz)

                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'{class_name} {conf:.2f} ({depth_m:.1f}m)'
                cv2.putText(cv_image, label, (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

        annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        annotated_msg.header = msg.header
        self.pub_annotated.publish(annotated_msg)

        inventory_msg = String()
        inventory_msg.data = json.dumps(self.inventory, indent=2)
        self.pub_inventory.publish(inventory_msg)

    def _update_inventory(self, class_name: str, x: float, y: float, z: float):
        """Add a detection to the inventory, deduplicating by class + position."""
        for entry in self.inventory:
            if entry['class'] != class_name:
                continue
            dist = math.sqrt(
                (entry['x'] - x) ** 2 +
                (entry['y'] - y) ** 2
            )
            if dist < DEDUP_RADIUS:
                # Task: Update the existing entry's position as a running average,
                # and increment entry['count'] by 1.
                # Running average: new_x = (old_x * count + x) / (count + 1)
                # Do this for x, y, and z separately, then increment count.

                # Your code
                return

        # New object — add to inventory and announce it.
        entry = {'class': class_name, 'x': round(x, 2),
                 'y': round(y, 2), 'z': round(z, 2), 'count': 1}
        self.inventory.append(entry)
        self.get_logger().info(f'New object: {class_name} at ({x:.2f}, {y:.2f})')

        update_msg = String()
        update_msg.data = json.dumps(entry)
        self.pub_update.publish(update_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register in `setup.py`:

```
'detector_node = mini_pupper_labs.detector_node:main',
```

**Task 5:** Screenshot `/object_inventory` publishing a non-empty JSON list with at least two different object classes detected at different positions.

---

## Building the Explorer Node

### Step 7 — Write `explorer_node.py`

This is the state machine. It runs on the PC and owns the top-level behavior.

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/explorer_node.py
```

```python
#!/usr/bin/env python3
"""
explorer_node.py

State machine that drives the capstone demo:
  IDLE - waiting for the run to start
  EXPLORE — sends frontier goals to Nav2, watching for voice/touch events
  TRACKING — (optional extension) switches to person-following mode
  STOPPED — final state: cancels Nav2, plays a chirp, prints the manifest

State transitions:
  IDLE - EXPLORE on startup (or a configurable delay)
  EXPLORE - STOPPED on /voice_command containing a keyword
  EXPLORE - STOPPED on /touch_estop True
  EXPLORE - EXPLORE on Nav2 goal completion (pick next frontier)

Runs on the PC.
"""

import json
import math
import random
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import numpy as np


class State:
    IDLE = 'IDLE'
    EXPLORE = 'EXPLORE'
    STOPPED = 'STOPPED'


class ExplorerNode(Node):

    def __init__(self):
        super().__init__('explorer_node')

        self.state = State.IDLE
        self.inventory = []
        self.current_goal = None
        self.map_data = None

        self.navigator = BasicNavigator()

        self.sub_voice = self.create_subscription(
            String, '/voice_command', self.voice_callback, 10
        )
        self.sub_touch = self.create_subscription(
            Bool, '/touch_estop', self.touch_callback, 10
        )
        self.sub_inventory = self.create_subscription(
            String, '/object_inventory', self.inventory_callback, 10
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        # Main loop at 2 Hz — checks Nav2 goal status and picks new frontiers.
        self.create_timer(0.5, self.update)

        self.get_logger().info('ExplorerNode ready. Waiting for Nav2...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 active. Starting exploration.')
        self.state = State.EXPLORE

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def voice_callback(self, msg: String):
        if self.state == State.EXPLORE:
            self.get_logger().info(f'Voice command received: "{msg.data}" — stopping.')
            self._stop()

    def touch_callback(self, msg: Bool):
        if msg.data and self.state == State.EXPLORE:
            self.get_logger().warn('Touch estop triggered — halting immediately.')
            self._stop()

    def inventory_callback(self, msg: String):
        try:
            self.inventory = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    # ------------------------------------------------------------------
    # State machine update (runs every 0.5s)
    # ------------------------------------------------------------------

    def update(self):
        if self.state != State.EXPLORE:
            return

        # Task: Check whether the current Nav2 goal is still in progress.
        # Use self.navigator.isTaskComplete() — if it returns True, the
        # robot reached (or failed to reach) the last frontier.
        # If it's complete, call self._pick_next_frontier() to send a new goal.
        # If isTaskComplete() returns False, the robot is still moving — do nothing.

        # Your code

    # ------------------------------------------------------------------
    # Frontier selection
    # ------------------------------------------------------------------

    def _pick_next_frontier(self):
        """Find a frontier cell in the occupancy grid and send it to Nav2."""
        if self.map_data is None:
            self.get_logger().warn('No map yet — waiting.')
            return

        frontier = self._find_frontier(self.map_data)
        if frontier is None:
            self.get_logger().info('No frontiers found — exploration complete.')
            self._stop()
            return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = frontier[0]
        goal.pose.position.y = frontier[1]
        goal.pose.orientation.w = 1.0

        self.get_logger().info(f'New frontier goal: ({frontier[0]:.2f}, {frontier[1]:.2f})')

        # Task: Send this goal to Nav2 using self.navigator.goToPose(goal).
        # Your code

    def _find_frontier(self, map_msg: OccupancyGrid):
        """
        Return (world_x, world_y) of a frontier cell, or None if no frontiers exist.

        A frontier cell is an unknown cell (-1) that is adjacent to a known-free cell (0).
        This is the classic definition from Yamauchi (1997).
        """
        res = map_msg.info.resolution
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y
        w = map_msg.info.width
        h = map_msg.info.height

        # Task: Convert map_msg.data (a flat list of int8) into a 2D numpy array
        # of shape (h, w). Values: 0 = free, 100 = occupied, -1 = unknown.
        # Hint: np.array(map_msg.data, dtype=np.int8).reshape((h, w))

        grid = # Your code

        # Task: Find all frontier cells.
        # A cell at (row, col) is a frontier if:
        #   grid[row, col] == -1   (unknown)
        #   AND at least one of its 4-connected neighbors has grid value == 0 (free)
        #
        # Collect all frontier (row, col) pairs.
        # If none are found, return None.
        # Otherwise pick one at random (random.choice(frontiers)) and convert it
        # to world coordinates:
        #   world_x = col * res + ox
        #   world_y = row * res + oy
        # Return (world_x, world_y).

        # Your code

    # ------------------------------------------------------------------
    # Stop and report
    # ------------------------------------------------------------------

    def _stop(self):
        self.state = State.STOPPED
        self.navigator.cancelTask()

        self.get_logger().info('\n' + '=' * 50)
        self.get_logger().info('OBJECT INVENTORY')
        self.get_logger().info('=' * 50)

        if not self.inventory:
            self.get_logger().info('No objects detected.')
        else:
            for entry in self.inventory:
                self.get_logger().info(
                    f"  {entry['class']:15s}  "
                    f"x={entry['x']:+.2f}  y={entry['y']:+.2f}  "
                    f"(seen {entry['count']}x)"
                )

        self.get_logger().info('=' * 50)

        # Task: Play a short audio chirp using the robot's speaker.
        # The BSP installs `mpg321` and `sounddevice` / `amixer` on the robot.
        # The simplest approach: use subprocess.run to call amixer to set volume,
        # then play a .wav file with `aplay`. You can generate a 0.5s sine wave
        # .wav file in advance and place it at ~/chirp.wav.
        #
        # Alternatively: generate the chirp in Python with sounddevice and numpy,
        # write it to /tmp/chirp.wav with soundfile, then play it.
        #
        # Note: this runs on the PC, so you'll need to SSH-invoke a command on
        # the robot to play from its speaker, or move the chirp play logic into
        # voice_node.py which already runs on the robot and can subscribe to
        # /inventory_update to trigger playback.
        
        # Your code


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register in `setup.py`:

```
'explorer_node = mini_pupper_labs.explorer_node:main',
```

---

## Putting It All Together

### Step 8 — Full System Launch

Start everything in order. Use separate terminals.

**Terminal 1 — Robot bringup (on robot):**

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

**Terminal 2 — SLAM (on PC):**

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_slam slam.launch.py
```

**Terminal 3 — Nav2 (on PC):**

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_navigation navigation_smacplanner.launch.py
```

**Terminal 4 — Voice and touch nodes (on robot):**

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run mini_pupper_labs voice_node & ros2 run mini_pupper_labs touch_node
```

**Terminal 5 — Detector and explorer (on PC):**

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run mini_pupper_labs detector_node & ros2 run mini_pupper_labs explorer_node
```

**Task 6:** Run the full system. Let the robot explore the room and detect at least 3 distinct objects (you can place them). Then say "stop" to trigger the voice command. Screenshot the inventory manifest printed in the explorer node's log output and record the robot walking around.

**Task 7:** Trigger the touch estop mid-run. Confirm the robot halts after the touch event. Screenshot the `/touch_estop` topic and the explorer node log showing the transition to STOPPED.

---

## Looking Ahead

You've now built a complete autonomous robot system from scratch: hardware bringup, sensor fusion, SLAM, navigation, computer vision, speech recognition, and physical human-robot interaction. Each week's lab was one layer of this stack.

A few natural extensions if you want to keep pushing:

- **RL locomotion:** Swap the Stanford gait controller for the trained neural policy from the MJX Colab. The object inventory and exploration logic is completely independent of the locomotion layer — you'd only change how `cmd_vel` gets executed below the navigation stack.
- **LCD live inventory:** Push the current object count and last-detected class to the ST7789 display so the robot's screen acts as a live status panel during the sweep.
- **Speaker feedback on each new find:** Add a short bark or chirp (via `sounddevice` on the robot) every time `detector_node` publishes a new `/inventory_update` event, so you can hear the robot "finding" things without watching a terminal.

---

## Tasks

1. `sounddevice.query_devices()` output with I2S microphone identified (Step 1).
2. GPIO touch pad output showing a pad's state changing between touched and not-touched, with pin-to-pad mapping confirmed (Step 2).
3. `/voice_command` receiving a message on keyword detection (Step 3).
4. `/touch_estop` publishing both `True` on press and `False` on release (Step 4).
5. `/object_inventory` with at least two distinct object classes and positions (Step 5).
6. Full system run: 60+ seconds of exploration, 3+ objects, voice stop, manifest (Step 7).
7. Touch estop demo: halt confirmed within 1 second, log showing STOPPED transition (Step 7).

---
