#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf2_ros
import numpy as np
from tf.transformations import quaternion_from_matrix
import tf.transformations as tft


def main():
    # Initialize moveit_commander and the ROS node.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_tf2_ik_base_link_to_item_planner', anonymous=True)

    # Create a tf2 buffer and listener to get the transform.
    tfBuffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tfBuffer)

    # Instantiate RobotCommander and MoveGroupCommander.
    robot = moveit_commander.RobotCommander()
    group_name = "ur5e"  # Update with your planning group name.
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Define the source and target frames.
    from_frame = 'base_link'
    to_frame = 'item::marker3'
    
    # Wait for the transform to be available.
    try:
        transformStamped = tfBuffer.lookup_transform(from_frame, to_frame, rospy.Time(0), rospy.Duration(5.0))
    except Exception as e:
        rospy.logerr("TF2 Exception: {}".format(e))
        return

    # Extract translation and rotation from the transform.
    trans = [transformStamped.transform.translation.x,
             transformStamped.transform.translation.y,
             transformStamped.transform.translation.z]
    rot = [transformStamped.transform.rotation.x,
           transformStamped.transform.rotation.y,
           transformStamped.transform.rotation.z,
           transformStamped.transform.rotation.w]

    # Build the homogeneous transformation matrix for the object frame.
    T_object = tf_transformations_matrix(rot, trans)

    # PLACEHOLDER: Define an offset from the object frame to the end-effector frame.
    # This will be provided by the motion planning team because it differs based on the item type and its pose.
    offset_pose = geometry_msgs.msg.Pose()
    offset_pose.position.x = 0.0
    offset_pose.position.y = 0.0
    offset_pose.position.z = 0.0
    # Identity orientation for the offset.
    offset_pose.orientation.x = 0.0
    offset_pose.orientation.y = 0.0
    offset_pose.orientation.z = 0.0
    offset_pose.orientation.w = 1.0

    offset_quat = [offset_pose.orientation.x,
                   offset_pose.orientation.y,
                   offset_pose.orientation.z,
                   offset_pose.orientation.w]
    T_offset = tf_transformations_matrix(offset_quat, [offset_pose.position.x,
                                                       offset_pose.position.y,
                                                       offset_pose.position.z])

    # Compose the final target transform: T_target = T_object * T_offset.
    T_target = np.dot(T_object, T_offset)

    # Extract translation and quaternion from T_target.
    target_translation = T_target[0:3, 3]
    target_quaternion = quaternion_from_matrix(T_target)

    # Create a PoseStamped message for the target pose.
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = from_frame
    target_pose.pose.position.x = target_translation[0]
    target_pose.pose.position.y = target_translation[1]
    target_pose.pose.position.z = target_translation[2]
    target_pose.pose.orientation.x = target_quaternion[0]
    target_pose.pose.orientation.y = target_quaternion[1]
    target_pose.pose.orientation.z = target_quaternion[2]
    target_pose.pose.orientation.w = target_quaternion[3]

    rospy.loginfo("Computed target pose:\n%s", target_pose)

    # Set the target pose for the MoveIt move group.
    move_group.set_pose_target(target_pose)
    rospy.sleep(1)  # Allow time for RViz to update.

    # Plan the trajectory to the target pose.
    rospy.loginfo("Planning trajectory...")
    plan = move_group.plan()

    # Execute the plan if planning was successful.
    if plan:
        rospy.loginfo("Plan found. Executing trajectory...")
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
    else:
        rospy.logerr("No valid plan found!")

    # Shutdown moveit_commander.
    moveit_commander.roscpp_shutdown()
    
def tf_transformations_matrix(quat, trans):
    """
    Helper function to build a 4x4 homogeneous transformation matrix from a quaternion and translation.
    """
    # Create rotation matrix from quaternion.
    T = np.eye(4)
    T = np.array(tft.quaternion_matrix(quat))
    # Set translation.
    T[0:3, 3] = trans
    return T

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

