#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
import copy

def move_to_pose(group, pose):
    group.set_pose_target(pose)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_pick_and_place_node', anonymous=True)

    group = moveit_commander.MoveGroupCommander("manipulator")

    start_pose = group.get_current_pose().pose

    # Pick Position (above object)
    pick_pose = copy.deepcopy(start_pose)
    pick_pose.position.z -= 0.1
    move_to_pose(group, pick_pose)

    rospy.sleep(1.0)
    rospy.loginfo("Grasping simulated...")

    # Lift
    lift_pose = copy.deepcopy(pick_pose)
    lift_pose.position.z += 0.1
    move_to_pose(group, lift_pose)

    # Place Position
    place_pose = copy.deepcopy(lift_pose)
    place_pose.position.y += 0.2
    move_to_pose(group, place_pose)

if __name__ == '__main__':
    main()
