#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from math import pi

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_joint_goal_node', anonymous=True)

    group = moveit_commander.MoveGroupCommander("manipulator")

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/2
    joint_goal[2] = pi/2
    joint_goal[3] = 0
    joint_goal[4] = pi/2
    joint_goal[5] = 0

    group.go(joint_goal, wait=True)
    group.stop()

if __name__ == '__main__':
    main()
