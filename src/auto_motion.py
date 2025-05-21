#!/usr/bin/env python
import rospy
import moveit_commander
import random
import sys
import time

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
