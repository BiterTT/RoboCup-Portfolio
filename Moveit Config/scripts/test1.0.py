#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import math
import tf.transformations as tfs 
import actionlib
from math import pi
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def check_success(move_group, target_pose, tolerance=0.005):
    """检查最终位置是否在误差范围内"""
    final_pose = move_group.get_current_pose().pose
    position_error = math.sqrt(
        (final_pose.position.x - target_pose.position.x) ** 2 +
        (final_pose.position.y - target_pose.position.y) ** 2 +
        (final_pose.position.z - target_pose.position.z) ** 2
    )
    rospy.loginfo(f"Final position error: {position_error} m")
    return position_error < tolerance


def control_gripper(gripper_client, pos, open_gripper=True):
    """控制夹爪开闭，open_gripper为True时打开夹爪，否则夹爪闭合到指定位置"""
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["robotiq_85_left_knuckle_joint"]
    point = JointTrajectoryPoint()
    if open_gripper:
        point.positions = [0.0]
    else:
        point.positions = [pos]
    point.time_from_start = rospy.Duration(1.0)
    goal.trajectory.points.append(point)
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()


def init_move_group():
    """初始化机械臂规划组，并设置规划时间与容忍度"""
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planning_time(20.0)
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.05)
    return move_group


def get_current_pose(move_group):
    """获取当前位姿并打印"""
    current_pose = move_group.get_current_pose().pose
    rospy.loginfo("Current pose is:\n{}".format(current_pose))
    return current_pose


def create_pose_goal(x, y, z, roll=0.0, pitch=pi, yaw=0.0):
    """创建目标位姿，内部将欧拉角转换为四元数"""
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    qx, qy, qz, qw = tfs.quaternion_from_euler(roll, pitch, yaw)
    pose_goal.orientation.x = qx
    pose_goal.orientation.y = qy
    pose_goal.orientation.z = qz
    pose_goal.orientation.w = qw
    return pose_goal


def plan_and_execute(move_group, pose_goal, max_attempts=5, tolerance=0.005):
    """规划并执行运动，尝试多次直到运动成功并在容差内"""
    move_group.set_pose_target(pose_goal)
    success = False
    for i in range(max_attempts):
        move_group.set_start_state_to_current_state()
        rospy.loginfo("Attempt #{} planning...".format(i+1))
        plan = move_group.plan()
        if plan[1]:
            rospy.loginfo("Planning succeeded, executing...")
            move_group.execute(plan[1], wait=True)
            success = True
        else:
            rospy.logwarn("Attempt #{} planning failed. Retrying...".format(i+1))
            continue

        # 检查目标位置是否达到要求
        if check_success(move_group, pose_goal, tolerance):
            rospy.loginfo("Final position is within tolerance. Accepting result.")
            break
        else:
            rospy.logwarn("Final position out of tolerance, retrying...")

    if not success:
        rospy.logerr("All {} attempts failed. No valid plan found.".format(max_attempts))

    move_group.stop()
    move_group.clear_pose_targets()
    rospy.loginfo("Motion finished!")
    final_pose = move_group.get_current_pose().pose
    rospy.loginfo("Current pose (after moving):\n{}".format(final_pose))
    return success


def init_gripper_client():
    """初始化夹爪客户端并等待连接"""
    gripper_client = actionlib.SimpleActionClient(
        "/gripper_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction
    )
    rospy.loginfo("Waiting for gripper controller...")
    gripper_client.wait_for_server()
    rospy.loginfo("Gripper controller connected.")
    return gripper_client


def shutdown_moveit():
    """关闭MoveIt相关模块"""
    moveit_commander.roscpp_shutdown()


def main():
    rospy.init_node("moveit_arm_demo", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    # 初始化机械臂与夹爪客户端
    move_group = init_move_group()
    gripper_client = init_gripper_client()

    # 获取当前位姿
    get_current_pose(move_group)

    # 创建目标位姿：可根据需要修改目标坐标和姿态
    pose_goal = create_pose_goal(x=-0.4652, y=-0.5901, z=0.0403, roll=0.0, pitch=pi, yaw=0.0)

    # 规划并执行运动
    plan_and_execute(move_group, pose_goal)

    # 调用夹爪控制函数：例如，闭合夹爪到位置0.5
    control_gripper(gripper_client, pos=0.5, open_gripper=False)

    shutdown_moveit()


if __name__ == "__main__":
    main()
