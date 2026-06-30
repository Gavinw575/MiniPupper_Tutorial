# Week 10 — Capstone: Autonomous Object Inventory with Voice Control

---

**Objectives:**

1. Integrate SLAM, Nav2, and the OAK-D Lite detection pipeline into a single autonomous system.
2. Write a frontier-exploration node that drives the robot through an unmapped space.
3. Log each detected object with its 3D position in the map frame using the TF tree.
4. Wire the robot's onboard microphone and speaker into the ROS2 state machine so a voice command stops the sweep and prints the inventory.
5. Use the robot's capacitive touch pad (via the ESP32) as a physical estop that halts the robot at any point during a run.

---

**Reference Material:**

- [vosk offline speech recognition](https://alphacephei.com/vosk/)
- [tf2_ros Python API](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html)
- [Nav2 Simple Commander API](https://docs.nav2.org/commander_api/index.html)
- [sounddevice documentation](https://python-sounddevice.readthedocs.io/)
- Week 7 Lab — Cartographer & Map Save/Load
- Week 8 Lab — Nav2, AMCL, Costmaps & Waypoint Following
- Week 9 Lab — OAK-D Lite Camera & YOLOv8 Inference

---

## Background

This lab ties every major piece of the course together. The robot will walk into a room it hasn't seen before, build a map, detect and locate objects in that space, respond to a spoken command, and let a person physically stop it at any time. That last part — the touch pad — is the piece you haven't touched yet.

### The full system

Five ROS2 nodes run together, most of them reused or lightly adapted from earlier labs:

| Node | Where it runs | What it does |
|---|---|---|
| `bringup` | Robot | Servos, lidar, IMU, odometry |
| `slam.launch.py` | PC | Builds the occupancy grid from `/scan` |
| `navigation_smacplanner.launch.py` | PC | Nav2 stack for path planning and execution |
| `explorer_node` | PC | Picks frontier goals and sends them to Nav2 (new) |
| `detector_node` | PC | Runs YOLOv8, gets depth from OAK-D Lite, transforms to map frame, logs inventory (new) |
| `voice_node` | Robot | Records from the onboard mic, runs vosk, publishes keyword events (new) |
| `touch_node` | Robot | Polls the ESP32's touch pad, publishes a `Bool` estop signal (new) |

The state machine that connects them lives in `explorer_node`. It listens to `/object_inventory` (from `detector_node`), `/voice_command` (from `voice_node`), and `/touch_estop` (from `touch_node`), and makes three decisions: keep exploring, stop cleanly, or emergency-freeze.

### Why the voice node runs on the robot

The microphone is a hardware I2S device on the robot's CM4. You can't stream raw audio over ROS2 without significant latency and bandwidth cost — it's much cleaner to run `vosk` locally on the robot where the audio device lives, and only publish small string messages over the network when a keyword is recognized. The trade-off is that `vosk`'s small model takes about 200 MB of RAM on the CM4, which is tight but workable since inference only runs between detections, not continuously.

### The touch pad

The Mini Pupper 2's capacitive touch pad is connected to the ESP32, which communicates with the CM4 over USB serial. The BSP installs `screen /dev/ttyUSB0 115200` as the `esp32-cli` alias — that's the same serial port the servo commands flow through. The touch state is a separate message type in the ESP32's serial protocol. Your `touch_node` reads that serial stream, parses the touch packet, and republishes it as a standard `std_msgs/Bool` on `/touch_estop`.

!!! warning "ttyUSB0 vs ttyUSB1"
    If bringup is already running when you test `touch_node`, the servo interface may already hold `/dev/ttyUSB0`. The touch pad may appear on `/dev/ttyUSB1` in that case. Check `ls /dev/ttyUSB*` on the robot before hardcoding the port.

### Object position pipeline

The OAK-D Lite gives you a 3D position in camera frame for each detection (X forward, Y left, Z up, in meters). To log where in the *map* an object is, you need to transform that point through the TF tree:

```
camera_optical_frame → base_link → odom → map
```

`tf2_ros` handles this — look up the transform from `camera_optical_frame` to `map` at the time of detection, and apply it to the XYZ point from the depth pipeline.

!!! note "Deduplication"
    The same chair will be detected many times as the robot walks past it. A simple rule — if an object of the same class already exists in the inventory within 0.5 m of this detection, skip it — keeps the manifest readable. You'll implement this in Step 5.

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

You should see an I2S or ALSA device listed. Note the device index — you'll need it in `voice_node`.

!!! warning "No HDMI cable during audio use"
    The BSP README notes this explicitly: the I2S audio device is headphone output 0 only when no HDMI cable is connected. HDMI reassigns the headphone index and the microphone may also be affected. Unplug HDMI before running any audio node.

**Task 1:** Paste the `sounddevice.query_devices()` output from your robot and identify which device index corresponds to the I2S microphone.

### Step 2 — Verify the Touch Pad via Serial

On the robot, open the ESP32 serial monitor:

```bash
esp32-cli   # alias for: screen /dev/ttyUSB0 115200
```

Touch the capacitive pad on the back of the robot. You should see a message in the serial output when it registers contact. Note exactly what the serial packet looks like — your `touch_node` will parse this string.

**Task 2:** Screenshot the serial output when the touch pad is pressed. What does the packet look like, and what changes between "touched" and "not touched"?

---

## Building the Voice Node

### Step 3 — Write `voice_node.py`

Create the file on the robot (this node runs on the robot, not the PC):

```bash
touch ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/voice_node.py
```

```python
#!/usr/bin/env python3
"""
voice_node.py

Listens to the onboard I2S microphone using sounddevice, runs vosk
offline speech recognition, and publishes recognized keyword events
on /voice_command as std_msgs/String.

Runs on the robot (CM4), not the PC.
"""

import json
import queue
import sounddevice as sd
import rclpy
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
            channels=1,
            callback=self._audio_callback,
        )
        self.stream.start()

        # Process audio in a ROS2 timer so the node stays spinnable.
        self.create_timer(0.05, self._process_audio)

    def _audio_callback(self, indata, frames, time, status):
        """Called by sounddevice from a background thread — just enqueue."""
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        self.audio_queue.put(bytes(indata))

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
            # for faster response, but it's not required.

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

**Task 3:** Screenshot `/voice_command` receiving a message when you say a keyword.

---

## Building the Touch Node

### Step 4 — Write `touch_node.py`

Create the file on the robot:

```bash
touch ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/touch_node.py
```

```python
#!/usr/bin/env python3
"""
touch_node.py

Reads the ESP32 serial stream and publishes a True on /touch_estop
whenever the capacitive touch pad is pressed.

Runs on the robot (CM4).
"""

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Adjust if the touch pad appears on ttyUSB1 when bringup is running.
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200


class TouchNode(Node):

    def __init__(self):
        super().__init__('touch_node')

        self.publisher = self.create_publisher(Bool, '/touch_estop', 10)

        # Task: Open a serial.Serial connection to SERIAL_PORT at BAUD_RATE.
        # Set timeout=0.1 so readline() doesn't block the node permanently.
        self.ser = # Your code

        self.create_timer(0.05, self._poll_serial)
        self.get_logger().info(f'TouchNode listening on {SERIAL_PORT}')

    def _poll_serial(self):
        # Task: Read a line from self.ser using readline(), decode it as
        # 'utf-8' with errors='ignore', and strip whitespace.
        # Check if the line matches the touch packet format you found in Task 2.
        # If it indicates a touch event, publish Bool(data=True) on /touch_estop.

        # Your code
        pass

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TouchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register in `setup.py`:

```
'touch_node = mini_pupper_labs.touch_node:main',
```

**Task 4:** Screenshot `/touch_estop` publishing `True` when you press the touch pad.

---

## Building the Detector Node

### Step 5 — Write `detector_node.py`

This node runs on the PC. It builds on the Week 9 YOLO detector but adds depth-to-map-frame transformation and inventory tracking.

```bash
touch ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/detector_node.py
```

```python
#!/usr/bin/env python3
"""
detector_node.py

Subscribes to /camera/image_raw and /stereo/depth (OAK-D Lite depth image),
runs YOLOv8 on each frame, estimates 3D position of each detection in the
map frame via TF, and maintains a deduplicated object inventory.

Publishes:
  /yolo/image_annotated  — sensor_msgs/Image (annotated feed for rqt)
  /object_inventory      — std_msgs/String   (JSON manifest, updated live)
  /inventory_update      — std_msgs/String   (single-object event on new find)

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


# Only track COCO classes the robot is likely to encounter indoors.
TRACKED_CLASSES = {
    'person', 'chair', 'bottle', 'cup', 'laptop',
    'cell phone', 'book', 'backpack', 'tv', 'couch',
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
                # Skip this detection if latest_depth is None or the depth value
                # is 0.0 or nan (the OAK-D Lite returns 0 for unmeasurable pixels).

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

### Step 6 — Write `explorer_node.py`

This is the state machine. It runs on the PC and owns the top-level behavior.

```bash
touch ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/explorer_node.py
```

```python
#!/usr/bin/env python3
"""
explorer_node.py

State machine that drives the capstone demo:
  IDLE     — waiting for the run to start
  EXPLORE  — sends frontier goals to Nav2, watching for voice/touch events
  TRACKING — (optional extension) switches to person-following mode
  STOPPED  — final state: cancels Nav2, plays a chirp, prints the manifest

State transitions:
  IDLE     → EXPLORE   on startup (or a configurable delay)
  EXPLORE  → STOPPED   on /voice_command containing a keyword
  EXPLORE  → STOPPED   on /touch_estop True
  EXPLORE  → EXPLORE   on Nav2 goal completion (pick next frontier)

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
        # .wav file in advance and place it at ~/chirp.wav, or use any system sound.
        #
        # Alternatively: generate the chirp in Python with sounddevice and numpy,
        # write it to /tmp/chirp.wav with soundfile, then play it.
        #
        # Note: this runs on the PC, so you'll need to SSH-invoke a command on
        # the robot to play from its speaker, OR move the chirp play logic into
        # voice_node.py which already runs on the robot and can subscribe to
        # /inventory_update to trigger playback.
        #
        # Document your approach in your writeup.

        # Your code (or implementation note)


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

### Step 7 — Full System Launch

Start everything in order. Use separate terminals.

**Terminal 1 — Robot bringup (on robot or via SSH):**

```bash
export ROBOT_MODEL=mini_pupper_2
export ROS_DOMAIN_ID=42
ros2 launch mini_pupper_bringup bringup.launch.py
```

**Terminal 2 — SLAM (on PC):**

```bash
export ROS_DOMAIN_ID=42
ros2 launch mini_pupper_slam slam.launch.py
```

**Terminal 3 — Nav2 (on PC):**

```bash
export ROS_DOMAIN_ID=42
ros2 launch mini_pupper_navigation navigation_smacplanner.launch.py map:=$HOME/map.yaml
```

!!! note "Blank map vs. live mapping"
    If you have a pre-built map from Week 7, you can run Nav2 with it for localization. If you want the robot to build the map from scratch during the demo, skip Nav2's map argument and let SLAM build one live — Nav2 will localize against the growing map. The second approach is more impressive but requires SLAM and Nav2 to run simultaneously, which is the standard configuration.

**Terminal 4 — Voice and touch nodes (on robot via SSH):**

```bash
export ROS_DOMAIN_ID=42
ros2 run mini_pupper_labs voice_node &
ros2 run mini_pupper_labs touch_node
```

**Terminal 5 — Detector and explorer (on PC):**

```bash
export ROS_DOMAIN_ID=42
ros2 run mini_pupper_labs detector_node &
ros2 run mini_pupper_labs explorer_node
```

**Task 6:** Run the full system. Let the robot explore for at least 60 seconds and detect at least 3 distinct objects. Then say "stop" to trigger the voice command. Screenshot the inventory manifest printed in the explorer node's log output.

**Task 7:** Trigger the touch estop mid-run (during active navigation). Confirm the robot halts within 1 second of the touch event. Screenshot the `/touch_estop` topic and the explorer node log showing the transition to STOPPED.

---

## Looking Ahead

You've now built a complete autonomous robot system from scratch: hardware bringup, sensor fusion, SLAM, navigation, computer vision, speech recognition, and physical human-robot interaction. Each week's lab was one layer of this stack.

A few natural extensions if you want to keep pushing:

- **RL locomotion:** Swap the Stanford gait controller for the trained neural policy from the MJX Colab. The object inventory and exploration logic is completely independent of the locomotion layer — you'd only change how `cmd_vel` gets executed below the navigation stack.
- **LCD live inventory:** Push the current object count and last-detected class to the ST7789 display so the robot's screen acts as a live status panel during the sweep.
- **Speaker feedback on each new find:** Add a short bark or chirp (via `sounddevice` on the robot) every time `detector_node` publishes a new `/inventory_update` event, so you can hear the robot "finding" things without watching a terminal.

**DELIVERABLE:** Full demo video (phone camera is fine) showing: robot exploring autonomously, at least 3 objects detected and logged, voice stop command received, manifest printed, touch estop demonstrated.

---

## Tasks

1. `sounddevice.query_devices()` output with I2S microphone identified (Step 1).
2. ESP32 serial output showing touch pad press/release packet format (Step 2).
3. `/voice_command` receiving a message on keyword detection (Step 3).
4. `/touch_estop` publishing `True` on pad press (Step 4).
5. `/object_inventory` with at least two distinct object classes and positions (Step 5).
6. Full system run: 60+ seconds of exploration, 3+ objects, voice stop, manifest (Step 7).
7. Touch estop demo: halt confirmed within 1 second, log showing STOPPED transition (Step 7).

---

## Troubleshooting

??? question "vosk loads but `AcceptWaveform` never returns True"
    Check that `SAMPLE_RATE = 16000` in `voice_node.py` matches the actual hardware rate. Run `python3 -c "import sounddevice; print(sounddevice.query_devices())"` and look for the default samplerate on your I2S device. If it's 48000, change `SAMPLE_RATE` to 48000 and create the `KaldiRecognizer` with that value instead.

??? question "TF transform lookup fails with `LookupException`"
    The TF tree needs time to populate after SLAM starts. The first few seconds of a run will always fail lookups — the `try/except` in `detector_node` handles this gracefully. If it keeps failing after 30+ seconds, check that `ROS_DOMAIN_ID=42` is set on both machines and run `ros2 run tf2_tools view_frames` to see what's actually in the tree.

??? question "Depth image (`/stereo/depth`) is not publishing"
    The OAK-D Lite depth stream requires both stereo cameras to be running. Confirm the depthai pipeline is configured for stereo mode in your Week 9 setup. Run `ros2 topic list | grep stereo` — if the topic is absent entirely, the pipeline is in RGB-only mode.

??? question "`/touch_estop` never publishes"
    The touch pad serial packet format varies by BSP version — re-run `esp32-cli` from Task 2 and confirm exactly what string the ESP32 sends. If bringup is already holding `ttyUSB0`, change `SERIAL_PORT` to `/dev/ttyUSB1` in `touch_node.py`.

??? question "Explorer node sends goals but robot doesn't move"
    Confirm Nav2 is fully initialized before the explorer starts sending goals — the `waitUntilNav2Active()` call in `__init__` handles this, but if Nav2 is started after the explorer, the wait may time out. Restart the explorer node after Nav2 is up.

??? question "Frontier detection finds no frontiers even in a new room"
    This usually means the map isn't publishing yet (SLAM takes a few seconds to produce the first `/map` message) or `map_callback` isn't receiving it. Run `ros2 topic hz /map` to confirm the grid is updating, and check `ROS_DOMAIN_ID` on the PC running SLAM matches the PC running the explorer.

---
