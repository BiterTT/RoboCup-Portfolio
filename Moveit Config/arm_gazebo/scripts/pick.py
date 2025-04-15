#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rospy
import moveit_commander
import geometry_msgs.msg
from math import sqrt
import math
from geometry_msgs.msg import PoseStamped

def check_success(move_group, target_pose, tolerance=0.005):
    """检查最终位置是否在误差范围内"""
    final_pose = move_group.get_current_pose().pose
    position_error = sqrt(
        (final_pose.position.x - target_pose.position.x)**2 +
        (final_pose.position.y - target_pose.position.y)**2 +
        (final_pose.position.z - target_pose.position.z)**2
    )
    # 输出减少：不再打印每次误差信息
    return position_error < tolerance

def read_targets_from_file(file_path):
    """
    从文件中读取目标位置，只提取x, y, z三个数值。
    支持两种格式：
      1. 纯数字格式（以空白字符分隔）：x y z
      2. 带标签格式：name type ... 后跟至少三个数字，取后面前三个为 x, y, z
    忽略以 "#" 开头的注释行。
    返回的列表中，每个目标为 (x, y, z) 的元组。
    """
    targets = []
    try:
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue  # 忽略空行与注释
                if ',' in line:
                    tokens = [token.strip() for token in line.split(',') if token.strip()]
                else:
                    tokens = line.split()
                try:
                    # 如果第一项能转换为数字，认为是纯数字格式
                    float(tokens[0])
                    x, y, z = map(float, tokens[:3])
                except ValueError:
                    # 否则假设前两项为标签，取后面三个数字
                    if len(tokens) >= 5:
                        x, y, z = map(float, tokens[2:5])
                    else:
                        rospy.logwarn("行数据格式不正确: " + line)
                        continue
                targets.append((x, y, z))
    except Exception as e:
        rospy.logerr(f"读取文件 {file_path} 失败: {e}")
    return targets

def main():
    rospy.init_node("moveit_arm_demo", anonymous=True, log_level=rospy.ERROR)
    # rospy.init_node("moveit_arm_demo", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_planning_time(20.0)
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.05)

    # 读取目标位置文件（例如 ARM0.txt）
    arm0_path = os.path.join(os.path.dirname(__file__), "../config/ARM0.txt")
    targets = read_targets_from_file(arm0_path)
    rospy.loginfo("加载到 {} 个目标".format(len(targets)))

    for idx, target in enumerate(targets):
        x, y, z = target
        rospy.loginfo("移动到目标 {}: x={}, y={}, z={}".format(idx+1, x, y, z))

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        # 设置抓夹朝下的朝向（假设末端执行器默认抓夹朝上）
        # 这里设置为：roll=π, pitch=0, yaw=0
        roll = math.pi    # 绕 X 轴旋转180度
        pitch = 0.0
        yaw = 0.0

        qx, qy, qz, qw = tfs.quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw

        # # 固定朝向（无旋转）
        # pose_goal.orientation.x = 0.0
        # pose_goal.orientation.y = 0.0
        # pose_goal.orientation.z = 0.0
        # pose_goal.orientation.w = 1.0

        move_group.set_pose_target(pose_goal)

        success = False
        # 最多尝试5次规划与执行
        for i in range(5):
            move_group.set_start_state_to_current_state()
            plan = move_group.plan()
            if plan[1]:
                move_group.execute(plan[1], wait=True)
                success = True
            else:
                continue

            if check_success(move_group, pose_goal, tolerance=0.005):
                break

        if not success:
            rospy.logerr("目标 {} 运动失败，跳过".format(idx+1))
        else:
            rospy.loginfo("目标 {} 到达，停留2秒".format(idx+1))
            rospy.sleep(2.0)

        move_group.stop()
        move_group.clear_pose_targets()

    rospy.loginfo("所有目标处理完成！")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
