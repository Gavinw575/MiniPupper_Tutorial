# Week 8 Answer Key

Complete implementations for every `# Your code` gap in `week8_lab.md`.

---

## `imu_raw_angles.py` — Gap Fill

```python
# Roll and pitch from raw accelerometer (Step 2)
roll_deg  = math.degrees(math.atan2(ay, az))
pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
```

---

## `imu_filter_node.py` — Gap Fills

### Gap 1: Accel-derived roll and pitch

```python
accel_roll  = math.atan2(ay, az)
accel_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
```

### Gap 2: Complementary filter update for roll and pitch

```python
self.roll  = ALPHA * (self.roll  + gx * dt) + (1.0 - ALPHA) * accel_roll
self.pitch = ALPHA * (self.pitch + gy * dt) + (1.0 - ALPHA) * accel_pitch
```

### Gap 3: Yaw integration (gyro only)

```python
self.yaw += gz * dt
```

---

## `tilt_safety_node.py` — Gap Fills

### Gap 1: Quaternion to Euler call

```python
roll, pitch, _ = quaternion_to_euler(ox, oy, oz, ow)
```

### Gap 2: Tilt check and estop logic

```python
tipping = abs(roll) > TILT_THRESHOLD_RAD or abs(pitch) > TILT_THRESHOLD_RAD

if tipping:
    if not self.estop_active:
        self.estop_active = True
        self.get_logger().warn(
            f'TILT ESTOP: roll={roll_deg:+.1f}°  pitch={pitch_deg:+.1f}°  '
            f'— cutting /cmd_vel'
        )
    # Keep publishing zeros every callback while tipped — prevents any
    # other node from sneaking a non-zero command through.
    self.pub_cmd.publish(Twist())
else:
    if self.estop_active:
        self.estop_active = False
        self.get_logger().info(
            f'Tilt estop cleared: roll={roll_deg:+.1f}°  pitch={pitch_deg:+.1f}°'
        )
```

**Note on the "keep publishing zeros" pattern:** publishing a zero Twist on every callback while tipped is intentional — it preempts any teleop or navigation command that arrives at the same time. A single one-shot publish could be overridden by the next teleop message. This is a valid approach for a simple safety node; a production system would use a hardware-level interlock or a proper ROS2 lifecycle node.

---

## Notes on IMU Axis Convention

The `imu_interface.py` driver swaps and negates raw MPU axes to match ROS convention (REP-103: x forward, y left, z up). The relevant lines:

```python
return [
    raw_data['ay'] * GRAVITY,    # ax → ay (swap)
    raw_data['ax'] * GRAVITY,    # ay → ax (swap)
    raw_data['az'] * -GRAVITY,   # az negated
    raw_data['gy'] * DEG2RAD,
    raw_data['gx'] * DEG2RAD,
    raw_data['gz'] * -DEG2RAD,   # gz negated
]
```

If students find that pitch is inverted from what they expect (pitching the nose down gives positive instead of negative pitch), they should negate `accel_pitch` in the complementary filter. This is a physical mounting orientation issue, not a math error.

---

## Notes on the IMU Calibration Delay

The `imu_interface.py` startup calibration averages 200 samples (2 seconds at 100 Hz) before publishing. During that window no messages appear on `/imu/data`. If a student's filter node or tilt safety node seems to not receive anything for the first couple seconds, this is why — not a DDS issue.

The tilt safety node's optional startup delay (mentioned in the troubleshooting section) can be implemented like this:

```python
def __init__(self):
    ...
    self.enabled = False
    self.create_timer(3.0, self._enable_once)  # one-shot via cancel

def _enable_once(self):
    self.enabled = True
    self.get_logger().info('Tilt safety armed.')
    # Cancel the timer so it only fires once
    # (ROS2 doesn't have a built-in one-shot timer, so just set the flag and ignore repeats)

def imu_callback(self, msg: Imu):
    if not self.enabled:
        return
    # ... rest of callback
```

---

## Expected Task Answers (for grading reference)

**Task 1:** `orientation_covariance[0]` will be `-1.0`. Per REP-145, this means the orientation field is invalid/unavailable and should be ignored by downstream consumers. `angular_velocity` and `linear_acceleration` should show small non-zero values (sensor noise at rest) plus gravity on the z-axis (~9.8 m/s²).

**Task 2:** At rest, peak-to-peak roll noise is typically 0.5–2° depending on surface vibration and nearby machinery. While walking, the accelerometer-only estimate should be visibly erratic — swinging several degrees with each step cycle because the servo impacts register as lateral accelerations indistinguishable from tilt.

**Task 3:** At 45° tilt, `/imu/rpy` should report close to 45° in roll (within a few degrees). With `ALPHA = 0.5`: much noisier output (bounces noticeably with each step) but reacts almost instantly to a tilt change. With `ALPHA = 0.98`: smooth and stable but takes 1–2 seconds to fully converge after a step change in tilt.

**Task 4:** At rest, Madgwick and the complementary filter should be similar in noise level since they're using the same inputs with no magnetometer. During rapid tilting, Madgwick tends to converge slightly faster due to its gradient descent approach in quaternion space, but the difference is subtle without a magnetometer. The main observable difference is that Madgwick handles gimbal-lock-adjacent cases more gracefully because it never converts through Euler angles internally.

**Task 5:** The tilt estop log should show the WARN line with roll/pitch values exceeding 30°, followed within a second or two by the INFO cleared message once the robot is upright. The robot's motion should stop immediately on the WARN — the zero Twist preempts the teleop command.
