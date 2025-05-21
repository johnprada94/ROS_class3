# 📘 Class 3 – UR10 Manipulation in ROS Noetic (Python Only)

## 🧱 Initial Setup Instructions

### ✅ Create the Package

```bash
cd ~/catkin_ws/src
catkin_create_pkg class3 rospy std_msgs moveit_commander geometry_msgs tf
```

### ✅ Organize Your Files

```bash
mkdir -p class3/src class3/launch
cd class3/src
touch joint_goal.py cartesian_path.py pick_and_place.py p_controller_orientation.py auto_motion.py
chmod +x *.py
```

### ✅ Update `CMakeLists.txt`

Add to the bottom:

```cmake
catkin_install_python(PROGRAMS
  src/joint_goal.py
  src/cartesian_path.py
  src/pick_and_place.py
  src/p_controller_orientation.py
  src/auto_motion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### ✅ Build Your Workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

# ⚙️ Exercise 1 – Joint Space Motion Planning

### 🎯 Objective
Move the UR10 to a predefined set of joint angles using MoveIt.

### 🔧 Code: `joint_goal.py`

```python
#!/usr/bin/env python
import rospy, sys
import moveit_commander
from math import pi

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_joint_goal', anonymous=True)

    group = moveit_commander.MoveGroupCommander("manipulator")

    joint_goal = [0, -pi/2, pi/2, 0, pi/2, 0]
    group.go(joint_goal, wait=True)
    group.stop()

if __name__ == '__main__':
    main()
```

### 🧠 Explanation
This script initializes MoveIt, sets a target joint configuration for the UR10, and executes the motion plan.

### 🖥 Run Instructions

Terminal 1:
```bash
roslaunch ur10_moveit_config demo.launch
```

Terminal 2:
```bash
rosrun class3 joint_goal.py
```

### ✅ Expected Result
UR10 moves to a joint configuration where the arm is bent upward and forward.

---

# ⚙️ Exercise 2 – Cartesian Path Planning

### 🎯 Objective
Move the UR10 end-effector in a straight line along the X-axis.

### 🔧 Code: `cartesian_path.py`

```python
#!/usr/bin/env python
import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_cartesian_path', anonymous=True)

    group = moveit_commander.MoveGroupCommander("manipulator")

    waypoints = []
    start_pose = group.get_current_pose().pose
    waypoints.append(start_pose)

    new_pose = Pose()
    new_pose.position.x = start_pose.position.x + 0.2
    new_pose.position.y = start_pose.position.y
    new_pose.position.z = start_pose.position.z
    new_pose.orientation = start_pose.orientation
    waypoints.append(new_pose)

    (plan, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,
        jump_threshold=True
    )
    group.execute(plan, wait=True)

if __name__ == '__main__':
    main()
```

### 🧠 Explanation
The robot's tool0 is moved forward 20 cm in the X direction using a Cartesian trajectory.

### 🖥 Run Instructions

Terminal 1:
```bash
roslaunch ur10_moveit_config demo.launch
```

Terminal 2:
```bash
rosrun class3 cartesian_path.py
```

### ✅ Expected Result
The robot end-effector moves forward in a straight line along X by 20 cm.

---

# ⚙️ Exercise 3 – Pick and Place Simulation

### 🎯 Objective
Simulate a vertical pick-lift-place task using motion planning.

### 🔧 Code: `pick_and_place.py`

```python
#!/usr/bin/env python
import rospy, sys, copy
import moveit_commander

def move_to_pose(group, pose):
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_pick_place', anonymous=True)
    group = moveit_commander.MoveGroupCommander("manipulator")

    start_pose = group.get_current_pose().pose

    pick_pose = copy.deepcopy(start_pose)
    pick_pose.position.z -= 0.1
    move_to_pose(group, pick_pose)

    rospy.sleep(1.0)
    rospy.loginfo("Simulated grasp...")

    lift_pose = copy.deepcopy(pick_pose)
    lift_pose.position.z += 0.1
    move_to_pose(group, lift_pose)

    place_pose = copy.deepcopy(lift_pose)
    place_pose.position.y += 0.2
    move_to_pose(group, place_pose)

if __name__ == '__main__':
    main()
```

### 🧠 Explanation
Simulates moving down to pick, lifting the object, and placing it to the side.

### 🖥 Run Instructions

Terminal 1:
```bash
roslaunch ur10_moveit_config demo.launch
```

Terminal 2:
```bash
rosrun class3 pick_and_place.py
```

### ✅ Expected Result
The robot lowers, simulates a grasp, lifts, then places the object to the right.

---

# ⚙️ Exercise 4 – Orientation Control with P Controller

### 🎯 Objective
Rotate the UR10 wrist to a target yaw using proportional control.

### 🔧 Code: `p_controller_orientation.py`

```python
#!/usr/bin/env python
import rospy, sys, math
import moveit_commander
import tf

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_orientation_control', anonymous=True)
    group = moveit_commander.MoveGroupCommander("manipulator")

    target_yaw = math.radians(90)
    Kp = rospy.get_param("~Kp", 0.5)
    listener = tf.TransformListener()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))
            current_yaw = tf.transformations.euler_from_quaternion(rot)[2]
            error = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))
            angular_z = Kp * error

            joints = group.get_current_joint_values()
            joints[5] += angular_z
            group.set_joint_value_target(joints)
            group.go(wait=True)
            group.stop()

            if abs(error) < 0.01:
                rospy.loginfo("Orientation corrected.")
                break

            rate.sleep()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

if __name__ == '__main__':
    main()
```

### 🧠 Explanation
Applies angular velocity to joint 6 based on yaw error to achieve target orientation.

### 🖥 Run Instructions

Terminal 1:
```bash
roslaunch ur10_moveit_config demo.launch
```

Terminal 2:
```bash
rosrun class3 p_controller_orientation.py
```

### ✅ Expected Result
The wrist rotates to 90° yaw until error is negligible.

---

# ⚙️ Exercise 5 – Automated Random Motions via Launch File

### 🎯 Objective
Move UR10 to random joint configurations in a loop.

### 🔧 Code: `auto_motion.py`

```python
#!/usr/bin/env python
import rospy, sys, random
import moveit_commander

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_auto_motion', anonymous=True)
    group = moveit_commander.MoveGroupCommander("manipulator")

    safe_positions = [
        [0, -1.57, 1.57, 0, 1.57, 0],
        [0.5, -1.0, 1.2, 0, 1.57, 0.3],
        [-0.5, -1.0, 1.2, 0, 1.57, -0.3]
    ]

    group.set_max_velocity_scaling_factor(0.3)

    while not rospy.is_shutdown():
        goal = random.choice(safe_positions)
        group.go(goal, wait=True)
        group.stop()
        rospy.sleep(10.0)

if __name__ == '__main__':
    main()
```

### 🔧 Launch File: `auto_motion.launch`

```xml
<launch>
  <node pkg="class3" type="auto_motion.py" name="auto_motion" output="screen"/>
</launch>
```

### 🖥 Run Instructions

Terminal 1:
```bash
roslaunch ur10_moveit_config demo.launch
```

Terminal 2:
```bash
roslaunch class3 auto_motion.launch
```

### ✅ Expected Result
UR10 cycles through random joint configurations every 10 seconds.

---