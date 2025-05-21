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

## ⚙️ Exercise 1 – Joint Space Motion Planning

### 🎯 Objective
Move the UR10 to a predefined set of joint angles using MoveIt.

### 🔧 Code: `joint_goal.py`
...

# For brevity, full content is included in the actual export