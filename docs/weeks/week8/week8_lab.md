# Lab 8 — Natural Language Voice Control with an LLM

---

**Objectives:**

1. Build a full-dictation voice pipeline that accepts free-form spoken commands, rather than matching a fixed keyword list.
2. Build a small action wrapper around `/cmd_vel` that exposes named movement primitives (`move_forward`, `turn_left`, etc.).
3. Send the transcribed command to the OpenAI API and constrain its output to a fixed action vocabulary.
4. Parse the model's response into a sequence of actions and execute them in order, with a physical touch estop able to interrupt at any point.

---

**Reference Material:**

- [OpenAI API — Chat Completions](https://platform.openai.com/docs/api-reference/chat)
- [vosk offline speech recognition](https://alphacephei.com/vosk/)
- [geometry_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
- Week 3 Lab — Teleop, RViz2 & TF Tree (your `linear.x`/`angular.z` values)

---

## Background

This lab replaces fixed grammar with free dictation, and adds a reasoning step in between: instead of matching keywords directly, you transcribe whatever was said, hand the full sentence to an LLM, and have the model translate it into a short sequence of actions the robot actually knows how to perform.

### Why an LLM in the middle instead of matching phrases yourself

You could write a pile of `if "forward" in text` conditionals, but that has its own issues — "go ahead", "walk that way", and "move forward" all mean the same thing, and a multi-step command like "turn left then stop" needs to be split into an ordered sequence. An LLM handles that translation robustly as long as you constrain its output tightly.

### Why this node runs on the robot

The voice pipeline runs on the robot because the microphone is a hardware I2S device there — streaming raw audio off the robot would be slow and bandwidth-heavy. What's new here is that the LLM call itself also happens on the robot. 

The upside of this design compared to something like a live voice-to-voice API: you're sending a short line of transcribed text per command, not a continuous audio stream.

### The pipeline

| Node | Where it runs | What it does |
|---|---|---|
| `voice_transcript_node` | Robot | Records from the onboard mic, runs vosk in full-dictation mode, publishes complete sentences |
| `llm_command_node` | Robot | Sends each transcript to the OpenAI API, parses the structured response, executes actions via `pupper_actions` |
| `pupper_actions` | Robot (imported module, not a node) | Thin wrapper exposing `move_forward()`, `turn_left()`, etc. as `/cmd_vel` Twist publishes |

---

## Setup

### Step 1 — Install the OpenAI package and store your API key

On the robot:

```bash
pip install google-genai --break-system-packages
```

```bash
echo 'export GEMINI_API_KEY="your-key-here"' >> ~/.bashrc
source ~/.bashrc
```

**Task 1:** Confirm the key is visible with `python3 -c "import os; print(bool(os.environ.get('OPENAI_API_KEY')))"`.
---

## Building the Transcript Node

### Step 2 — Build a full-dictation voice node

Create the file:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python mini_pupper_labs
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/voice_transcript_node.py
```

Unlike a keyword-filtered voice node, this one should publish every completed utterance, whatever it says — there's no fixed word list to match against here.

```python
#!/usr/bin/env python3
"""
voice_transcript_node.py

Streams microphone audio through vosk in full-dictation mode and
publishes every completed utterance — no fixed keyword filtering.

Runs on the robot (CM4).
"""

import json
import queue
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer

SAMPLE_RATE = 16000
BLOCK_SIZE = 4000
DEVICE_INDEX = None


class VoiceTranscriptNode(Node):

    def __init__(self):
        super().__init__('voice_transcript_node')
        self.publisher = self.create_publisher(String, '/voice_transcript', 10)
        self.audio_queue = queue.Queue()

        self.model = Model('/home/ubuntu/demo_assets/vosk-model-small-en-us-0.15')
        self.recognizer = KaldiRecognizer(self.model, SAMPLE_RATE)

        self.get_logger().info('VoiceTranscriptNode ready — listening')

        self.stream = sd.RawInputStream(
            samplerate=SAMPLE_RATE, blocksize=BLOCK_SIZE, dtype='int16',
            channels=1, device=DEVICE_INDEX, callback=self._audio_callback,
        )
        self.stream.start()
        self.timer = self.create_timer(0.1, self._process_audio)

    def _audio_callback(self, indata, frames, time_info, status):
        self.audio_queue.put(bytes(indata))

    def _process_audio(self):
        while not self.audio_queue.empty():
            data = self.audio_queue.get()
            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                text = result.get('text', '').strip()
                # Task: if `text` is non-empty, publish it on /voice_transcript
                # and log it. Don't filter for specific words here — that's
                # the LLM's job now, not this node's.

                # Your code


def main(args=None):
    rclpy.init(args=args)
    node = VoiceTranscriptNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Task 2:** Register `voice_transcript_node` in `setup.py`, build, and confirm `ros2 topic echo /voice_transcript` shows full sentences as you speak. Take screenshot.

---

## Building the Action Wrapper

### Step 3 — Write `pupper_actions.py`

This is the equivalent of a Karel-style API: a small set of named methods that hide the `Twist` details.

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/pupper_actions.py
```

```python
"""
pupper_actions.py

Thin wrapper around /cmd_vel exposing named movement primitives.
Not a node itself — instantiate with a reference to a running node
so it can create its own publisher.
"""

import time
from geometry_msgs.msg import Twist

ACTION_DURATION = 1.0   # seconds per discrete action


class PupperActions:

    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self.estopped = False

    def _run(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        end_time = time.time() + ACTION_DURATION
        while time.time() < end_time:
            if self.estopped:
                break
            self.pub.publish(twist)
            time.sleep(0.1)
        self.stop()

    def move_forward(self):
        # Task: publish a forward Twist using the linear.x value you
        # settled on in Week 3's teleop lab.
        # Your code

    def move_backward(self):
        # Task: same idea, negative linear.x. Check your Week 3 notes
        # on why reverse should be slower than forward.
        # Your code

    def turn_left(self):
        # Task: publish an angular.z Twist for an in-place left turn.
        # Your code

    def turn_right(self):
        # Your code

    def stop(self):
        self.pub.publish(Twist())
```

**Task 3:** Fill in the four movement methods and test each one directly from a Python shell to confirm the robot moves as expected.
```python
import rclpy
from rclpy.node import Node
from mini_pupper_labs.pupper_actions import PupperActions

rclpy.init()
node = Node('test_pupper_actions')
actions = PupperActions(node)

actions.#your code
...



node.destroy_node
rclpy.shutdown()
```

---

## Building the Command Node

### Step 4 — Write the system prompt

Create `llm_command_node.py` and write a system prompt that forces the model to output a JSON list drawn from the fixed action vocabulary. This is the part that determines whether your parser downstream stays simple or turns into a mess of edge cases.

Your prompt should:

- State the exact allowed tokens: `move_forward`, `move_backward`, `turn_left`, `turn_right`, `stop`
- Require a JSON array as the *entire* response — no explanation, no markdown code fences, no extra words
- Give a few example transcript → output pairs, including at least one multi-step example (e.g. "turn around and come back" → `["turn_left", "turn_left", "move_forward"]`)
- Tell the model what to output if the transcript doesn't match any known action (an empty list `[]`, not a guess)

**Task 4:** Write this system prompt (aim for a fairly short). Test it directly against the OpenAI API with a few example sentences before wiring it into the node, and note any transcripts that produced output outside your expected format.

### Step 5 — Implement the node

```python
#!/usr/bin/env python3
"""
llm_command_node.py

Subscribes to /voice_transcript, sends each transcript to the Gemini
API constrained to a fixed action vocabulary, and executes the
resulting action sequence via PupperActions. Subscribes to
/touch_estop and aborts any in-progress sequence immediately.

Runs on the robot (CM4) — requires internet access.
use model="gemini-flash-latest"
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from google import genai
from google.genai import types
from .pupper_actions import PupperActions

VALID_ACTIONS = {'move_forward', 'move_backward', 'turn_left', 'turn_right', 'stop'}

SYSTEM_PROMPT = """
# Task: paste the system prompt you wrote in Step 4 here.
"""


class LLMCommandNode(Node):

    def __init__(self):
        super().__init__('llm_command_node')
        self.actions = PupperActions(self)
        self.client = genai.Client()  # reads GEMINI_API_KEY from the environment

        self.create_subscription(String, '/voice_transcript', self._on_transcript, 10)
        self.create_subscription(Bool, '/touch_estop', self._on_estop, 10)

        self.get_logger().info('LLMCommandNode ready')

    def _on_estop(self, msg):
        self.actions.estopped = msg.data
        if msg.data:
            self.get_logger().warn('Touch estop triggered — aborting action queue')

    def _on_transcript(self, msg):
        if self.actions.estopped:
            return

        transcript = msg.data
        self.get_logger().info(f'Heard: "{transcript}"')

        # Task: call self.client.models.generate_content(), passing the
        # transcript as `contents` and your SYSTEM_PROMPT inside a
        # types.GenerateContentConfig as `system_instruction`. Set
        # temperature=0 for consistency. Wrap the call in a try/except —
        # on any failure, log a warning and fall back to raw_response = '[]'
        # rather than letting the node crash.
        # Your code
        raw_response = None  # replace with the model's text output

        # Task: parse raw_response as JSON. Handle the case where the
        # model wraps it in a code fence (```json ... ```) or adds
        # stray text despite the prompt — strip and retry before
        # giving up. On any parse failure, log a warning and treat it
        # as an empty action list rather than crashing the node.
        # Your code
        actions = []

        for action in actions:
            if self.actions.estopped:
                self.get_logger().warn('Estop during sequence — stopping remaining actions')
                break
            if action not in VALID_ACTIONS:
                self.get_logger().warn(f'Ignoring unknown action: {action}')
                continue
            getattr(self.actions, action)()


def main(args=None):
    rclpy.init(args=args)
    node = LLMCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Task 5:** Implement the two `# Task` sections above — the API call and the defensive JSON parsing. Register the node in `setup.py`.

---

## Putting It All Together

### Step 6 — Full system test

Run everything on the robot via SSH (separate terminals):

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run mini_pupper_labs voice_transcript_node
ros2 run mini_pupper_labs llm_command_node
```

**Task 6:** Test a single-step command ("move forward"), a multi-step command ("turn left, then move forward"), and a nonsense phrase that shouldn't match anything. Record what the robot does in each case.

---

## Tasks

1. Confirm `GEMINI_API_KEY` visibility difference between interactive and non-interactive SSH sessions (Step 1).
2. `/voice_transcript` publishing full sentences, not single keywords (Step 2).
3. Four movement methods in `pupper_actions.py` implemented and manually tested (Step 3).
4. System prompt written and tested directly against the API with example transcripts (Step 4).
5. API call and defensive JSON parsing implemented in `llm_command_node.py` (Step 5).
6. Full run: single-step command, multi-step command, unmatched phrase, and a mid-sequence touch estop (Step 6).

---
