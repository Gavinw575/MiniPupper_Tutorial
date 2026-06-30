# Week 10 Answer Key

Complete implementations for every `# Your code` gap in `week10_lab.md`.

---

## `voice_node.py` — Complete

```python
#!/usr/bin/env python3
"""
voice_node.py — ANSWER KEY
"""

import json
import queue
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer

KEYWORDS = ['stop', 'done', 'finished', 'halt']
SAMPLE_RATE = 16000
BLOCK_SIZE = 4000
DEVICE_INDEX = None


class VoiceNode(Node):

    def __init__(self):
        super().__init__('voice_node')

        self.publisher = self.create_publisher(String, '/voice_command', 10)
        self.audio_queue = queue.Queue()

        # Load the vosk small English model from ~/vosk-model
        self.model = Model('/home/ubuntu/vosk-model')

        # KaldiRecognizer takes (model, sample_rate)
        self.recognizer = KaldiRecognizer(self.model, SAMPLE_RATE)

        self.get_logger().info('VoiceNode ready — listening for keywords')

        self.stream = sd.RawInputStream(
            samplerate=SAMPLE_RATE,
            blocksize=BLOCK_SIZE,
            device=DEVICE_INDEX,
            dtype='int16',
            channels=1,
            callback=self._audio_callback,
        )
        self.stream.start()
        self.create_timer(0.05, self._process_audio)

    def _audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        self.audio_queue.put(bytes(indata))

    def _process_audio(self):
        while not self.audio_queue.empty():
            data = self.audio_queue.get()

            if self.recognizer.AcceptWaveform(data):
                # Complete utterance — parse the final result
                result = json.loads(self.recognizer.Result())
                text = result.get('text', '')
                self.get_logger().info(f'Heard: "{text}"')

                for keyword in KEYWORDS:
                    if keyword in text:
                        msg = String()
                        msg.data = keyword
                        self.publisher.publish(msg)
                        self.get_logger().info(f'Published keyword: {keyword}')
                        break
            else:
                # Partial result — optional fast-path check
                partial = json.loads(self.recognizer.PartialResult())
                partial_text = partial.get('partial', '')
                for keyword in KEYWORDS:
                    if keyword in partial_text:
                        msg = String()
                        msg.data = keyword
                        self.publisher.publish(msg)
                        self.get_logger().info(f'Published keyword (partial): {keyword}')
                        break

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

---

## `touch_node.py` — Complete

```python
#!/usr/bin/env python3
"""
touch_node.py — ANSWER KEY

The ESP32 serial protocol on Mini Pupper 2 sends lines like:
  TOUCH:1   (pad pressed)
  TOUCH:0   (pad released)

Adjust the `_is_touch_event()` method if your BSP sends a different format —
Task 2 tells students to note the exact packet, so this should be adapted to
match what they observed.
"""

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200


class TouchNode(Node):

    def __init__(self):
        super().__init__('touch_node')

        self.publisher = self.create_publisher(Bool, '/touch_estop', 10)

        # timeout=0.1 so readline() doesn't block the ROS2 executor
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

        self.create_timer(0.05, self._poll_serial)
        self.get_logger().info(f'TouchNode listening on {SERIAL_PORT}')

    def _poll_serial(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return

        if not line:
            return

        if self._is_touch_event(line):
            msg = Bool()
            msg.data = True
            self.publisher.publish(msg)
            self.get_logger().info(f'Touch estop triggered (line: "{line}")')

    def _is_touch_event(self, line: str) -> bool:
        """
        Parse the ESP32 serial line and return True if it indicates a touch press.

        The exact format depends on the BSP version and ESP32 firmware.
        Common formats observed on Mini Pupper 2:
          'TOUCH:1'        — direct touch state
          'BTN:1'          — button press variant
          'touch pressed'  — verbose text variant

        Students should fill this in based on what they saw in Task 2.
        This implementation handles the most common format.
        """
        line_lower = line.lower()
        # Handle 'TOUCH:1' or 'touch:1' format
        if 'touch:1' in line_lower:
            return True
        # Handle 'BTN:1' format
        if 'btn:1' in line_lower:
            return True
        # Handle verbose 'touch pressed' format
        if 'touch' in line_lower and 'press' in line_lower:
            return True
        return False

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

---

## `detector_node.py` — Gap Fills

### Gap 1: Get depth at pixel (cx_px, cy_px)

```python
# Replace: depth_m = # Your code
try:
    depth_img = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='32FC1')
    depth_m = float(depth_img[cy_px, cx_px])
    if depth_m == 0.0 or math.isnan(depth_m) or math.isinf(depth_m):
        continue
except Exception:
    continue
```

The guard against 0.0 is important — the OAK-D Lite returns 0.0 for pixels where stereo matching failed (typically very close objects or textureless surfaces).

### Gap 2: Project to 3D camera coordinates

```python
# Replace: pt_cam = # Your code
import rclpy.duration

pt_cam = PointStamped()
pt_cam.header.frame_id = 'camera_optical_frame'
pt_cam.header.stamp = msg.header.stamp
pt_cam.point.x = (cx_px - CX) * depth_m / FX
pt_cam.point.y = (cy_px - CY) * depth_m / FY
pt_cam.point.z = depth_m
```

### Gap 3: Transform to map frame

```python
# Replace: pt_map = # Your code
try:
    pt_map = self.tf_buffer.transform(
        pt_cam, 'map',
        timeout=rclpy.duration.Duration(seconds=0.1)
    )
except (tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException) as e:
    self.get_logger().debug(f'TF lookup failed: {e}')
    pt_map = None
```

### Gap 4: Running average deduplication update

```python
# Replace: # Your code (inside the dedup for-loop)
n = entry['count']
entry['x'] = round((entry['x'] * n + x) / (n + 1), 2)
entry['y'] = round((entry['y'] * n + y) / (n + 1), 2)
entry['z'] = round((entry['z'] * n + z) / (n + 1), 2)
entry['count'] += 1
return
```

---

## `explorer_node.py` — Gap Fills

### Gap 1: Check Nav2 goal completion and pick next frontier

```python
# Replace: # Your code (in update())
if self.navigator.isTaskComplete():
    result = self.navigator.getResult()
    if result == TaskResult.FAILED:
        self.get_logger().warn('Goal failed — picking another frontier.')
    self._pick_next_frontier()
```

### Gap 2: Send goal to Nav2

```python
# Replace: # Your code (in _pick_next_frontier())
self.navigator.goToPose(goal)
```

### Gap 3: Convert map.data to 2D numpy array

```python
# Replace: grid = # Your code
grid = np.array(map_msg.data, dtype=np.int8).reshape((h, w))
```

### Gap 4: Find frontier cells and return one

```python
# Replace: # Your code (in _find_frontier())
frontiers = []
for row in range(1, h - 1):
    for col in range(1, w - 1):
        if grid[row, col] != -1:
            continue
        # Check 4-connected neighbors for a free cell
        neighbors = [
            grid[row - 1, col],
            grid[row + 1, col],
            grid[row, col - 1],
            grid[row, col + 1],
        ]
        if 0 in neighbors:
            frontiers.append((row, col))

if not frontiers:
    return None

row, col = random.choice(frontiers)
world_x = col * res + ox
world_y = row * res + oy
return (world_x, world_y)
```

**Performance note for students:** This O(h × w) scan runs every time a new frontier is needed, which on a 500×500 map is 250,000 iterations. It's fine for this use case since it only runs once per Nav2 goal (every 30–60 seconds), but a production explorer would use a faster frontier-detection algorithm.

### Gap 5: Play a chirp on the robot's speaker

The simplest approach — generate a sine wave and play it via `sounddevice` in `voice_node.py` (which already runs on the robot) triggered by a `/inventory_update` subscription:

```python
# Add to voice_node.py alongside the voice logic:
import numpy as np
import sounddevice as sd

def play_chirp(freq=880.0, duration=0.3, samplerate=48000):
    t = np.linspace(0, duration, int(samplerate * duration), endpoint=False)
    wave = (0.5 * np.sin(2 * np.pi * freq * t)).astype(np.float32)
    sd.play(wave, samplerate=samplerate)
    sd.wait()
```

Then in `VoiceNode.__init__`, add:

```python
self.sub_update = self.create_subscription(
    String, '/inventory_update', self._on_new_object, 10
)
```

And the callback:

```python
def _on_new_object(self, msg: String):
    try:
        entry = json.loads(msg.data)
        self.get_logger().info(f"New object found: {entry['class']}")
        play_chirp()
    except Exception:
        pass
```

Alternatively, from the PC you can SSH-invoke playback:

```python
# In explorer_node._stop():
import subprocess
subprocess.run([
    'ssh', 'ubuntu@10.218.0.141',
    'aplay /home/ubuntu/chirp.wav'
])
```

---

## Notes on the Touch Packet Format

The ESP32 firmware's exact serial output format isn't documented in the public BSP. The `_is_touch_event()` implementation above covers the three most common variants. If a student's Task 2 shows something different (e.g., a raw binary byte or a JSON packet), the parsing logic should be updated to match. The pattern `'TOUCH:1'` is the most commonly reported format in MangDang Discord posts as of early 2025.

If the touch pad serial output completely conflicts with the servo command stream on `ttyUSB0`, the cleanest solution is to run `touch_node` before bringup and cache the device file descriptor, or reconfigure the ESP32 firmware to use a separate UART channel. For the course this is unlikely to be an issue since bringup and touch polling can coexist on the same serial port as long as `touch_node` only reads (never writes).

---

## Camera Intrinsics Note

The `FX`, `FY`, `CX`, `CY` values in `detector_node.py` are approximations for the OAK-D Lite RGB camera at 640×480. The actual calibrated values live in the camera's EEPROM and are published by the depthai pipeline on `/stereo/camera_info`. For a more accurate implementation, students could subscribe to `CameraInfo` and read `.k[0]` (FX), `.k[4]` (FY), `.k[2]` (CX), `.k[5]` (CY) dynamically rather than hardcoding them. This is a good extension task for students who finish early.

---
