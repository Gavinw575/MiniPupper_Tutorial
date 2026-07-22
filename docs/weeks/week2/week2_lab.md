# Mini Pupper ROS2 Robotics Course
## Lab 2 — ROS2 Fundamentals: Nodes, Topics, and Moving the Robot

---

**Objectives:**

1. Understand the relationship between ROS2 nodes and topics.
2. Drive the Mini Pupper 2 using the keyboard and observe how velocity commands work.
3. Inspect live robot data using ROS2 command line tools.
4. Create a ROS2 Python package from scratch.
5. Write a publisher node that moves the robot with code.
6. Write a subscriber node that reads and prints live sensor data.

---

**Reference Material:**

- [ROS2 Humble — Understanding Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [ROS2 Humble — Understanding Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [geometry_msgs/Twist](https://docs.ros2.org/humble/api/geometry_msgs/msg/Twist.html)
- [sensor_msgs/Imu](https://docs.ros2.org/humble/api/sensor_msgs/msg/Imu.html)

---

## Background

In ROS2, a node is a single process that can do multiple things like reading a sensor, controlling a motor, running an algorithm, etc... Nodes talk to each other by passing messages over topics. A node that sends a message is called a publisher. A node that receives them is called a subscriber. A single node can be both at the same time.

Topics have different types and every topic carries one specific message type. `/cmd_vel` always carries `geometry_msgs/Twist` messages. `/imu/data` always carries `sensor_msgs/Imu` messages. A node has to use the correct type or ROS2 won't let it connect.

This architecture means nodes are completely seperate from other nodes. The teleop keyboard node has no idea the [Stanford Quadruped controller](https://github.com/stanfordroboticsclub/StanfordQuadruped) exists, all it does us publishe to `/cmd_vel`. The controller subscribes to `/cmd_vel`. They never talk directly, which makes it easy to swap one out without touching the other. This is the core idea that makes ROS2 so powerful for robotics.

---

## Lab Tasks
### 1.0 - Installing Ros2 Packages
On your computer terminal install the base ROS2 Humble  
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```
Adding the Gazebo+control packages
```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs gazebo \
  ros-humble-xacro ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard \
  ros-humble-cartographer ros-humble-cartographer-ros \
  ros-humble-navigation2 ros-humble-nav2-bringup
```
Creating and building the workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
Adding our environmental variables to ~/.bashrc
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export ROBOT_MODEL=mini_pupper_2" >> ~/.bashrc
source ~/.bashrc
```
Now that all that is installed we should be able to continue further down the line


### 1.1 — Robot Bringup

SSH into the robot and launch bringup:

```bash
ssh ubuntu@<Pupper_IP>
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

Leave this terminal running. Open a second terminal on your PC and source ROS2:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Verify the robot is live by checking this topic is publishing:

```bash
ros2 topic hz /joint_states
```

Should show a steady publish rate. If it shows nothing, bringup is not fully running and teleop will not work.

---

### 1.2 — Driving with Teleop


Now drive the robot manually to see how `/cmd_vel` works.

Open a second terminal on the PC, run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

This is a controller build by stanford.

You should see a control layout printed in the terminal. Use the keys to drive the robot around:

```
Moving around:
   u    i    o
   j    k    l
   m    ,    .
```

`i` moves forward, `k` stops, `j` and `l` turn left and right.

While keeping everything open, open a third terminal and watch what teleop is sending:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /cmd_vel
```

You will see `geometry_msgs/Twist` messages streaming in real time as you press keys.  
Notice the structure:

```yaml
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

`linear.x` controls forward/backward speed in metres per second. `angular.z` controls rotation in radians per second. All other fields are zero as the robot cannot fly (obviously).

Mess around with the other button options and see what they all do. 

Press `k` to stop the robot, then `Ctrl+C` to exit teleop.

!!! note
    The Stanford Quadruped controller translates these simple velocity commands into coordinated 12-servo leg movements automatically. From ROS2's perspective, controlling a quadruped looks identical to controlling a wheeled robot as they both just subscribe to `/cmd_vel`.

**Task 1:** Screenshot of `ros2 topic echo /cmd_vel` showing live Twist messages while driving with teleop. As well as a video of the robot walking using teleop.

---

### 1.3 — Inspecting the ROS2 Graph

In a terminal on the PC:

```bash
ros2 node list
```

You'll see a long list of nodes — the lidar driver, the controller, the IMU, and others. Pick one that you think will be interesting and inspect it:

```bash
ros2 node info /mini_pupper_hardware
```

This shows you every topic the node publishes to and subscribes from, and what message types it uses. This can be a very helpful tool for ROS2. For example, if osmething is not working you can look at the node info and it will tell you what the node is expecting to receive and what it is sending out.

To look at a topic directly:

```bash
ros2 topic info /cmd_vel
```

Expected output:
```
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1
```

Notice there are 0 publishers right now because teleop isn't running anymore. The controller is still subscribed and waiting. When teleop was running there was 1 publisher. When your code runs later, your node will be that publisher.

Check the full message definition for the Twist (vector that describes the velocity of a rigid body through 3D spcae) type:

```bash
ros2 interface show geometry_msgs/msg/Twist
```

Keep this in mind as you'll need to know the field names when you write your publisher node.

**Task 2:** Take a screenshot of `ros2 topic info /cmd_vel` while running teleop and with teleop closed.

---

### 2.1 — Creating a ROS2 Package

Now create the Python package that your nodes will live in. On your PC:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --dependencies rclpy geometry_msgs sensor_msgs mini_pupper_labs
```

This creates a package called `mini_pupper_labs` with all the things needed for a Python ROS2 package. Let's look at what was created:

```bash
ls mini_pupper_labs/
```

You should see:

```
creating folder./mini_pupper_labs
creating ./mini_pupper_labs/package.xml
... cont.
```

package.xml declares the package name, version, and dependencies. Open it and take a look:

```bash
cat mini_pupper_labs/package.xml
```

setup.py is how Python knows what executables to build. Every node you write needs to be registered here. Open it:

```bash
cat mini_pupper_labs/setup.py
```

Find the `entry_points` section. It looks like this:

```python
entry_points={
    'console_scripts': [
    ],
},
```

This is where you'll add and register the nodes. Every time you write a new node file, you add a line here in the format:

```
'node_name = package_name.file_name:main',
```

You'll do this for each node you write. You can leave it empty for now and come back to it later.

---

### 2.2 — Writing a Publisher Node (cmd_vel)

Create a new file for your first node:

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/move_robot.py
```

Open it in your editor and copy in the following starter code. The `# Task` comments mark the lines you need to fill in:

```python
#!/usr/bin/env python3
"""
move_robot.py
A publisher node that sends velocity commands to the Mini Pupper 2.
Publishes to /cmd_vel at a fixed rate to make the robot move in a square.
Uses the IMU to correct heading drift while driving forward.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot')
        # Task: Create a publisher that publishes Twist messages to '/cmd_vel'
        # with a queue size of 10.
        # Hint: self.create_publisher(MessageType, 'topic_name', queue_size)
        self.publisher = # Your code

        # Task: Create a subscriber to '/imu/data' that calls self.imu_callback
        # with a queue size of 10.
        # Hint: self.create_subscription(MessageType, 'topic_name', callback, queue_size)
        self.imu_sub = # Your code

        # This timer calls self.timer_callback every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        # Track how long we've been in the current movement phase
        self.phase_time = 0.0
        self.phase = 'forward'

        # Heading tracking (the IMU orientation field is not valid on this
        # robot, so we integrate the raw gyro z-axis rate ourselves)
        self.yaw = 0.0
        self.last_imu_time = None
        self.Kp = 1.2             # correction gain, feel free to tune
        self.max_correction = 0.3  # rad/s cap on correction

        self.get_logger().info('MoveRobotNode started!')

    def imu_callback(self, msg):
        now = self.get_clock().now()
        if self.last_imu_time is not None:
            # Task: compute dt (in seconds) between this reading and the last
            # Hint: (now - self.last_imu_time).nanoseconds / 1e9
            dt = # Your code
            # Task: integrate angular_velocity.z into self.yaw
            self.yaw += # Your code
        self.last_imu_time = now

    def timer_callback(self):
        msg = Twist()
        self.phase_time += 0.1
        if self.phase == 'forward':
            # Task: Set the forward speed
            # Hint: which field of msg.linear controls forward movement?
            msg.linear.x = # Your code

            # Task: Compute a correction term that steers back toward yaw = 0
            # Hint: correction = -Kp * yaw, then clip it to [-max_correction, max_correction]
            correction = # Your code
            msg.angular.z = correction

            if self.phase_time >= ##:   # set drive forward time in seconds
                self.phase = 'turn'
                self.phase_time = 0.0
        elif self.phase == 'turn':
            msg.linear.x = 0.0
            # Task: Set the rotation speed to turn left
            # Hint: which field of msg.angular controls rotation?
            # Your code
            if self.phase_time >= ##:  # set drive turning time in seconds
                self.phase = 'forward'
                self.phase_time = 0.0
                # Task: reset self.yaw so the next forward leg starts fresh
                # Your code
        # Task: Publish the message using self.publisher
        # Your code


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot cleanly on exit
        stop = Twist()
        node.publisher.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Once you've filled in the tasks, register the node in `setup.py`. Open `~/ros2_ws/src/mini_pupper_labs/setup.py` and update the `entry_points` section:

```python
entry_points={
    'console_scripts': [
        'move_robot = mini_pupper_labs.move_robot:main',
    ],
},
```
Place the robot on the floor and build and run:

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_labs
source install/setup.bash
ros2 run mini_pupper_labs move_robot
```

The robot should drive forward, rotate, and repeat. Press `Ctrl+C` to stop — it will send a zero velocity command before exiting.

!!! note
    Make sure bringup is still running on the robot in your first terminal before running this. The node publishes to `/cmd_vel` over the network — the robot receives it automatically as long as both machines are on the same network with `ROS_DOMAIN_ID` matching.

**Task 3:** Show your completed `move_robot.py` and a video of the robot moving in a square using the node you just made

---

### 2.3 — Writing a Subscriber Node (IMU data)

Now you'll write a subscriber that listens to the robot's IMU and prints the orientation data to the terminal. This is useful for understanding what the robot is sensing in real time.

Create a new file:

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/read_imu.py
```

Open it and copy in the starter code:

```python
#!/usr/bin/env python3
"""
read_imu.py

A subscriber node that listens to /imu/data and prints
the robot's linear acceleration and angular velocity to the terminal.
"""

#!/usr/bin/env python3
"""
read_imu.py

A subscriber node that listens to /imu/data and prints
the robot's linear acceleration and angular velocity to the terminal.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ReadImuNode(Node):

    def __init__(self):
        super().__init__('read_imu')

        # Task: Create a subscription to '/imu/data' of type Imu,
        # with callback self.imu_callback and queue size of 10.
        # Hint: self.create_subscription(MessageType, 'topic_name', callback, queue_size)
        self.subscription = # Your code

        # Smoothed values, updated on every message but only printed
        # occasionally (see timer below)
        self.smoothed_accel = [0.0, 0.0, 0.0]
        self.smoothed_gyro = [0.0, 0.0, 0.0]
        self.alpha = 0.2  # smoothing factor: lower = smoother/slower to react

        # Task: Create a timer that calls self.print_callback every 0.5 seconds.
        # Hint: self.create_timer(period_seconds, callback)
        self.print_timer = # Your code

        self.get_logger().info('ReadImuNode started — listening to /imu/data')

    def imu_callback(self, msg):
        accel = msg.linear_acceleration
        gyro = msg.angular_velocity

        raw_accel = [accel.x, accel.y, accel.z]
        raw_gyro = [gyro.x, gyro.y, gyro.z]

        for i in range(3):
            # TODO: Apply exponential smoothing to each axis.
            # Formula: new_smoothed = alpha * raw + (1 - alpha) * old_smoothed
            self.smoothed_accel[i] = # Your code
            self.smoothed_gyro[i] = # Your code

    def print_callback(self):
        a = self.smoothed_accel
        g = self.smoothed_gyro
        self.get_logger().info(
            f'Accel   — x: {a[0]:6.3f}  y: {a[1]:6.3f}  z: {a[2]:6.3f}'
        )
        self.get_logger().info(
            f'Gyro    — x: {g[0]:6.3f}  y: {g[1]:6.3f}  z: {g[2]:6.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ReadImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register it in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'move_robot = mini_pupper_labs.move_robot:main',
        'read_imu = mini_pupper_labs.read_imu:main',
    ],
},
```

Build and run:

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_labs
source install/setup.bash
ros2 run mini_pupper_labs read_imu
```

You should see acceleration and gyroscope values streaming in your terminal. Try picking the robot up and tilting it — watch how the values change. Try running your `move_robot` node at the same time in a second terminal and observe how the IMU values change as the robot walks.

**Task 4:** Submit your completed `read_imu` code. Video of the code running showing changing in the IMU

---

### 2.4 — Read Joint States

Write a third node called `read_joints.py` that subscribes to `/joint_states` and prints the position of each of the 12 servos. 

```bash
nano ~/ros2_ws/src/mini_pupper_labs/mini_pupper_labs/read_joints.py
```

You'll need to look up the `sensor_msgs/JointState` message type to know what fields are available:

```bash
ros2 interface show sensor_msgs/msg/JointState
```

The `name` field is a list of joint names and the `position` field is a matching list of angles in radians. Try to print them together so the output looks like:

```
[INFO] fl_leg1_joint:  0.123  fl_leg2_joint: -0.456  ...
```

**Task 5:** Submit your completed `read_joints` code and a screenshot of it running

---

### Tasks

1. Screenshot of `ros2 topic echo /cmd_vel` showing live Twist messages while driving with teleop. As well as a video of the robot walking using teleop.

2. Take a screenshot of `ros2 topic info /cmd_vel` while running teleop and with teleop closed.

3. Show your completed `move_robot.py` and a video of the robot moving using the node you just made

4. Submit your completed `read_imu` code. Video of the code running showing changing in the IMU

5. Submit your completed `read_joints` code and a screenshot of it running.

---
