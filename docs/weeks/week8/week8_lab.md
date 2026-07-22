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
pip install openai --break-system-packages
```

Store your API key as an environment variable rather than hardcoding it anywhere that could end up on GitHub:

```bash
echo 'export OPENAI_API_KEY="your-key-here"' >> ~/.bashrc
source ~/.bashrc
```

!!! warning "Non-interactive SSH sessions don't source `.bashrc`"
    This is the same gotcha as `ROS_DOMAIN_ID` — if you launch `llm_command_node` via a one-line `ssh ubuntu@<robot-ip> "ros2 run ..."` command, the environment variable won't be there even though it works fine in an interactive session. Export it explicitly in the same command, e.g. `ssh ubuntu@<robot-ip> "export OPENAI_API_KEY=... && ros2 run mini_pupper_labs llm_command_node"`, or add the export directly to your startup script.

**Task 1:** Confirm the key is visible with `python3 -c "import os; print(bool(os.environ.get('OPENAI_API_KEY')))"` — both in an interactive SSH session and in a non-interactive one (`ssh ubuntu@<robot-ip> "python3 -c ..."`). Note which one fails and why.

---

## Building the Transcript Node

### Step 2 — Build a full-dictation voice node

Create the file:

```bash
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

        self.model = Model('/home/ubuntu/vosk-model')
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

**Task 2:** Register `voice_transcript_node` in `setup.py`, build, and confirm `ros2 topic echo /voice_transcript` shows full sentences (not single words) as you speak.

---

## Building the Action Wrapper

### Step 3 — Write `pupper_actions.py`

This is the equivalent of a Karel-style API: a small set of named methods that hide the `Twist` details.

```bash
touch ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/pupper_actions.py
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

**Task 3:** Fill in the four movement methods and test each one directly from a Python shell (no LLM involved yet) to confirm the robot moves as expected before wiring it to anything else.

---

## Building the Command Node

### Step 4 — Write the system prompt

Open `llm_command_node.py` (create it in the same package) and write a system prompt that forces the model to output *only* a JSON list drawn from the fixed action vocabulary — nothing else. This is the part that determines whether your parser downstream stays simple or turns into a mess of edge cases.

Your prompt should:

- State the exact allowed tokens: `move_forward`, `move_backward`, `turn_left`, `turn_right`, `stop`
- Require a JSON array as the *entire* response — no explanation, no markdown code fences, no extra words
- Give a few example transcript → output pairs, including at least one multi-step example (e.g. "turn around and come back" → `["turn_left", "turn_left", "move_forward"]`)
- Tell the model what to output if the transcript doesn't match any known action (an empty list `[]`, not a guess)

**Task 4:** Write this system prompt (aim for a fairly short, tightly scoped prompt — you don't need CS123's ~50 lines here since the vocabulary is smaller). Test it directly against the OpenAI API with a few example sentences before wiring it into the node, and note any transcripts that produced output outside your expected format.

### Step 5 — Implement the node

```python
#!/usr/bin/env python3
"""
llm_command_node.py

Subscribes to /voice_transcript, sends each transcript to the OpenAI
API constrained to a fixed action vocabulary, and executes the
resulting action sequence via PupperActions. Subscribes to
/touch_estop and aborts any in-progress sequence immediately.

Runs on the robot (CM4) — requires internet access.
"""

import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from openai import OpenAI
from .pupper_actions import PupperActions

VALID_ACTIONS = {'move_forward', 'move_backward', 'turn_left', 'turn_right', 'stop'}

SYSTEM_PROMPT = """
# Task: paste the system prompt you wrote in Step 4 here.
"""


class LLMCommandNode(Node):

    def __init__(self):
        super().__init__('llm_command_node')
        self.actions = PupperActions(self)
        self.client = OpenAI()  # reads OPENAI_API_KEY from the environment

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

        # Task: call self.client.chat.completions.create() with the
        # system prompt and the transcript as the user message.
        # Use a small, fast model — check OpenAI's current model list
        # for the cheapest general-purpose option, pricing and model
        # names change often. Set temperature=0 for consistency.
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

Run everything on the robot via SSH (separate terminals or `&`):

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export OPENAI_API_KEY="your-key-here"
ros2 run mini_pupper_labs voice_transcript_node & \
ros2 run mini_pupper_labs llm_command_node & \
ros2 run mini_pupper_labs touch_node
```

**Task 6:** Test a single-step command ("move forward"), a multi-step command ("turn left, then move forward, then stop"), and a nonsense phrase that shouldn't match anything. Record what the robot does in each case, and confirm the touch estop halts an in-progress sequence.

---

## Tasks

1. Confirm `OPENAI_API_KEY` visibility difference between interactive and non-interactive SSH sessions (Step 1).
2. `/voice_transcript` publishing full sentences, not single keywords (Step 2).
3. Four movement methods in `pupper_actions.py` implemented and manually tested (Step 3).
4. System prompt written and tested directly against the API with example transcripts (Step 4).
5. API call and defensive JSON parsing implemented in `llm_command_node.py` (Step 5).
6. Full run: single-step command, multi-step command, unmatched phrase, and a mid-sequence touch estop (Step 6).

---

## Troubleshooting

??? question "The model's response isn't valid JSON"
    Usually it's wrapping the array in a markdown code fence or adding a sentence like "Sure, here's the list:" despite the system prompt. Strip anything before the first `[` and after the last `]` before calling `json.loads()`, and consider lowering `temperature` to 0 if you haven't already — higher temperatures make the model more likely to drift from the exact format.

??? question "`OPENAI_API_KEY` is set but the node still fails to authenticate"
    Confirm it's actually present in the process that launched the node, not just your login shell — see the Step 1 warning. Run `ros2 run mini_pupper_labs llm_command_node` from the same terminal where you exported the key, not a fresh SSH session.

??? question "The robot keeps moving briefly after a touch estop"
    Check that `PupperActions._run()` is checking `self.estopped` *inside* the sleep loop, not just once before it starts. A check only at the top of the method won't interrupt an action that's already running.

??? question "vosk mishears a command and the robot does the wrong thing"
    This is the real tradeoff of dropping a fixed keyword grammar for free dictation — it's fuzzier than a small closed vocabulary. If this happens often, consider having `llm_command_node` speak back a short confirmation before executing (a natural next extension using the robot's speaker), or falling back to a smaller set of trigger phrases for anything safety-critical.

??? question "API calls succeed sometimes and time out other times"
    Classroom wifi can be inconsistent. Wrap the API call in a `try/except` with a short timeout, and on failure, log it and treat the transcript as producing no actions rather than letting the node hang or crash.

---
