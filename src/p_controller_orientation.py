#!/usr/bin/env python
import rospy
import moveit_commander
import sys
import tf
import math

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_orientation_control', anonymous=True)
    group = moveit_commander.MoveGroupCommander("manipulator")

    target_yaw = math.radians(90)  # desired yaw = 90 degrees
    Kp = rospy.get_param("~Kp", 0.8)

    listener = tf.TransformListener()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            current_yaw = euler[2]
            error = target_yaw - current_yaw

            # Normalize angle to [-pi, pi]
            error = math.atan2(math.sin(error), math.cos(error))
            angular_z = Kp * error

            # Apply correction to the last joint (wrist_3_joint)
            current_joints = group.get_current_joint_values()
            current_joints[5] += angular_z  # joint_6 (yaw rotation)

            group.set_joint_value_target(current_joints)
            group.set_max_velocity_scaling_factor(0.1)
            group.set_max_acceleration_scaling_factor(0.1)
            group.go(wait=True)

            if abs(error) < 0.01:
                rospy.loginfo("Orientation corrected.")
                break

            rate.sleep()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

if __name__ == '__main__':
    main()
