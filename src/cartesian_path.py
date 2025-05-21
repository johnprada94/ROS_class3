#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_cartesian_path_node', anonymous=True)

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
    waypoints, 0.01, True)


    group.execute(plan, wait=True)

if __name__ == '__main__':
    main()
