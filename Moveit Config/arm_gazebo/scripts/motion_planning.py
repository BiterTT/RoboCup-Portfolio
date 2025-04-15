#!/usr/bin/env python3

import rospy
import moveit_commander
import os
import sys
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler
from parse_arm0 import parse_arm0_file  # 解析 ARM0.txt

# 初始化 ROS 及 MoveIt
rospy.init_node("motion_planning", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander("arm")  # 确保规划组名称正确

# ---- 在这里手动提高规划时间 ----
# group.set_planning_time(5)          # 给 MoveIt 至少 20秒进行规划
# group.set_num_planning_attempts(20)    # 每次plan()内部最多尝试20次
# group.allow_replanning(True)  # 允许重新规划
# group.set_max_velocity_scaling_factor(0.2)
# group.set_max_acceleration_scaling_factor(0.2)

# Parsing ARM0.txt to get the position information of the target object
arm0_path = os.path.join(os.path.dirname(__file__), "../config/ARM0.txt")
objects_data = parse_arm0_file(arm0_path)

from moveit_msgs.msg import OrientationConstraint, Constraints
def move_to_pose(x, y, z, roll, pitch, yaw, max_attempts=10):
    """
    Plan the robot arm's target pose (x, y, z, roll, pitch, yaw) and execute the motion.
    If Cartesian path planning fails, it automatically switches to normal path planning.
    :param max_attempts: Maximum number of attempts to avoid getting stuck.
    """

    # Set the target pose
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    quaternion = quaternion_from_euler(roll, pitch, yaw)
    pose_target.orientation.x = quaternion[0]
    pose_target.orientation.y = quaternion[1]
    pose_target.orientation.z = quaternion[2]
    pose_target.orientation.w = quaternion[3]

    rospy.loginfo(f"Target pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")

    success = False
    attempts = 0

    while not success and attempts < max_attempts:
        attempts += 1
        rospy.loginfo(f"Attempt {attempts} to plan, target={x:.3f}, {y:.3f}, {z:.3f})")

        # Clear old targets
        group.stop()
        group.clear_pose_targets()
        group.set_pose_target(pose_target)

        # Compute the Cartesian path
        waypoints = [pose_target]
        (cartesian_plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, True)

        if fraction > 0.8:
            rospy.loginfo("Cartesian path planning succeeded, executing...")
            exec_result = group.execute(cartesian_plan, wait=True)

        # Check error
        current_pose = group.get_current_pose().pose
        position_tolerance = 0.02
        position_error = math.sqrt(
            (current_pose.position.x - pose_target.position.x) ** 2 +
            (current_pose.position.y - pose_target.position.y) ** 2 +
            (current_pose.position.z - pose_target.position.z) ** 2
        )

        if position_error < position_tolerance:
            rospy.loginfo("Pose reached the target position (ignoring roll, pitch, yaw), continuing...")
            success = True
        else:
            rospy.logwarn("Trajectory execution failed, retrying...")

    if not success:
        rospy.logerr(f"Planning failed, cannot reach target position:({x:.3f}, {y:.3f}, {z:.3f})")

# def move_to_pose(x, y, z, roll, pitch, yaw, max_attempts=9999999):
#     """
#     Send a 6DOF target pose (x, y, z, roll, pitch, yaw) to MoveIt for path planning.
#     规划失败时，会不断重试，直到成功为止（不能跳过）。
#     max_attempts: 允许的最大尝试次数（一个很大的数字，防止无限死循环）
#     """
#     # # 确保 roll, pitch, yaw 是弧度
#     # roll = math.radians(roll) if abs(roll) > math.pi else roll
#     # pitch = math.radians(pitch) if abs(pitch) > math.pi else pitch
#     # yaw = math.radians(yaw) if abs(yaw) > math.pi else yaw
    
#     # 设置目标位姿
#     pose_target = geometry_msgs.msg.Pose()

#     # 设置目标位置
#     pose_target.position.x = x
#     pose_target.position.y = y
#     pose_target.position.z = z

#     # 计算四元数
#     from tf.transformations import quaternion_from_euler
#     quaternion = quaternion_from_euler(roll, pitch, yaw)
#     pose_target.orientation.x = quaternion[0]
#     pose_target.orientation.y = quaternion[1]
#     pose_target.orientation.z = quaternion[2]
#     pose_target.orientation.w = quaternion[3]

#     # 归一化四元数，防止错误
#     norm = math.sqrt(
#         pose_target.orientation.x**2 +
#         pose_target.orientation.y**2 +
#         pose_target.orientation.z**2 +
#         pose_target.orientation.w**2
#     )
#     pose_target.orientation.x /= norm
#     pose_target.orientation.y /= norm
#     pose_target.orientation.z /= norm
#     pose_target.orientation.w /= norm
#     rospy.loginfo(f"目标位姿: x={x:.3f}, y={y:.3f}, z={z:.3f}, 四元数: {quaternion}")
    
#     rospy.loginfo(f"目标姿态: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")


#     # 循环尝试规划，直到成功或超过max_attempts
#     success = False
#     attempts = 0
#     while not success and attempts < max_attempts and not rospy.is_shutdown():
#         attempts += 1
#         rospy.loginfo(f"尝试第 {attempts} 次规划, 目标=({x:.3f}, {y:.3f}, {z:.3f}), roll={roll:.2f}")

#         # 强制设定当前状态（可选，确保起始状态正确）
#         #current_state = group.get_current_state()
#         #group.set_start_state(current_state)

#         # 清除旧目标
#         group.stop()
#         group.clear_pose_targets()

#         # 设置新的目标位姿
#         group.set_pose_target(pose_target)

#         # 执行规划
#         plan = group.plan()
#         if plan and plan[0]:  # 如果规划成功
#             rospy.loginfo("规划成功，开始执行轨迹...")
#             exec_result = group.execute(plan[1], wait=True)
#             if exec_result:
#                 rospy.loginfo(f"成功移动到目标位置 ({x:.2f}, {y:.2f}, {z:.2f})")
#                 success = True
#             else:
#                 current_pose = group.get_current_pose().pose

#                 # 计算位置误差（只看 x, y, z，不管 roll, pitch, yaw）
#                 position_tolerance = 0.01  # 允许的误差范围（单位：米）
#                 position_error = math.sqrt(
#                     (current_pose.position.x - pose_target.position.x) ** 2 +
#                     (current_pose.position.y - pose_target.position.y) ** 2 +
#                     (current_pose.position.z - pose_target.position.z) ** 2
#                 )

#                 rospy.logwarn(f"当前位置: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
#                 rospy.logwarn(f"目标位置: x={pose_target.position.x:.3f}, y={pose_target.position.y:.3f}, z={pose_target.position.z:.3f}")
    
#                 if position_error < position_tolerance:
#                     rospy.loginfo("位姿已达目标位置 (忽略 roll, pitch, yaw)，继续执行下一步...")
#                     success = True
#                 else:
#                     rospy.logwarn("轨迹执行失败，继续尝试规划...")
#                 # current_pose = group.get_current_pose().pose
#                 # rospy.logwarn(f"当前位姿: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
#                 # rospy.logwarn(f"目标位姿: x={pose_target.position.x:.3f}, y={pose_target.position.y:.3f}, z={pose_target.position.z:.3f}")

#                 # rospy.logwarn("轨迹执行失败，继续尝试规划...")
#         else:
#             rospy.logwarn("规划失败，尝试笛卡尔路径...")

#             # 尝试笛卡尔路径
#             # 创建 waypoints 列表
#             waypoints = []
#             waypoints.append(pose_target)

#             # 计算笛卡尔路径
#             (cartesian_plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, True)

#             if fraction > 0.8:
#                 rospy.loginfo("Cartesian path planning succeeded, executing...")
#                 exec_result = group.execute(cartesian_plan, wait=True)

#             # Check error
#             current_pose = group.get_current_pose().pose
#             position_tolerance = 0.02
#             position_error = math.sqrt(
#                 (current_pose.position.x - pose_target.position.x) ** 2 +
#                 (current_pose.position.y - pose_target.position.y) ** 2 +
#                 (current_pose.position.z - pose_target.position.z) ** 2
#             )

#             if position_error < position_tolerance:
#                 rospy.loginfo("Pose reached the target position (ignoring roll, pitch, yaw), continuing...")
#                 success = True
#             else:
#                 rospy.logwarn("Trajectory execution failed, retrying...")

#     if not success:
#         rospy.logerr(f"Planning failed, cannot reach target position:({x:.3f}, {y:.3f}, {z:.3f})")

#     # 清除目标，防止影响下次规划
#     group.clear_pose_targets()

#     # 如果超过 max_attempts 还没成功，就会退出函数
#     if not success:
#         rospy.logerr(f"尝试 {max_attempts} 次仍无法移动到 ({x:.2f}, {y:.2f}, {z:.2f})。")

def execute_task(objects):
    """
    Execute the task:
    1. Move above the object
    2. Move to the trash bin to place the object
    """
    total_score = 0  # Record total score

    for obj in objects:
        pos = obj["position"][:3]  # 取 x, y, z
        ori = obj["position"][3:]  # 取 roll, pitch, yaw
        bin_pos = obj["bin"]["position"]  # 获取垃圾箱的位置
        bin_ori = obj["bin"]["orientation"]  # 获取垃圾箱的角度
        score = obj["score"]    # 物体得分

        rospy.loginfo(f"开始处理物体: {obj['id']} ({obj['type']})")

        # 让夹爪始终向下，设定 roll=π, pitch=0, yaw=?
        # roll=π, pitch=0, yaw=某个适当值，以确保夹爪朝下。
        # fixed_ori = [math.pi / 2, 0, 0]
        # fixed_ori = [math.pi, 0, 0]  # 设定 roll=π, pitch=0, yaw=0
        fixed_ori = [math.pi, 0, 0]  # Predefined orientation: roll=π, pitch=0, yaw=1.58

        #fixed_ori = [0, -math.pi/2, 0]

        # 先移动到目标物体正上方 (距离 z+0.1)
        move_to_pose(pos[0], pos[1], pos[2] + 0.1, fixed_ori[0], fixed_ori[1], fixed_ori[2])

        # 移动到垃圾箱正上方
        move_to_pose(bin_pos[0], bin_pos[1], bin_pos[2] + 0.2, fixed_ori[0], fixed_ori[1], fixed_ori[2])

        rospy.loginfo(f"{obj['id']} ({obj['type']}) 放置完成, 得分: {score}")
        total_score += score

    rospy.loginfo(f"所有任务完成，总得分: {total_score}")

execute_task(objects_data)
rospy.spin()
