import rospy
import moveit_commander
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

import sys
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import actionlib

from trajectory_msgs.msg import *
from control_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler
from copy import deepcopy


# 初始化 ROS 节点和 MoveIt
rospy.init_node("load_gazebo_objects")
scene = PlanningSceneInterface()
scene.clear()
rospy.sleep(1)  # 确保 MoveIt 场景接口初始化完成




# def control_gripper(object_catagory):
#     if object_catagory == "bottle":





# 添加 Gazebo 环境中的桌子（仅桌面）
def add_table(table_name, x, y, z=-0.1, yaw=1.57):  # 桌面在 0.5m 高度，旋转90°  # 桌面在 0.5m 高度
    table_pose = PoseStamped()
    table_pose.header.frame_id = "base_link"
    table_pose.pose.position.x = x
    table_pose.pose.position.y = y
    table_pose.pose.position.z = z  # 桌子高度的一半
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    table_pose.pose.orientation.x = quaternion[0]
    table_pose.pose.orientation.y = quaternion[1]
    table_pose.pose.orientation.z = quaternion[2]
    table_pose.pose.orientation.w = quaternion[3]
    # 添加桌面（尺寸根据 Gazebo 模型设置）
    scene.add_box(table_name, table_pose, size=(1.5, 0.8, 0.03))  # (长, 宽, 高)
    rospy.sleep(2)  # 等待场景稳定


# 解析 ARM0.txt 文件
arm0_path = "./ARM0.txt"  # 确保路径正确
objects = []

with open(arm0_path, "r") as file:
    for line in file:
        if line.startswith("#") or line.strip() == "":
            continue  # 跳过注释和空行

        parts = line.split()
        if len(parts) < 7:
            continue  # 确保行数据完整
        
        obj_name = parts[0]
        obj_type = parts[1]
        x, y, z = map(float, parts[2:5])  # 位置
        roll, pitch, yaw = map(float, parts[5:8])  # 旋转

        # 只加载 "bottle" 和 "can" 相关物体
        if "bottle" in obj_type.lower() or "pouch" in obj_type.lower():
            objects.append({
                "name": obj_name,
                "type": obj_type,
                "position": (x, y, z-0.62),
                "rotation": (roll, pitch, yaw)
            })
# 将物体添加到 MoveIt 规划场景
add_table("table1", 0.14, 0.0)  # Gazebo 中的桌子1
add_table("table2", 0.94, 0.0)  # Gazebo 中的桌子2

# 将物体添加到 MoveIt 规划场景
for obj in objects:
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = obj["position"]

    # 计算四元数（将欧拉角转换为四元数）
    quaternion = tf.transformations.quaternion_from_euler(*obj["rotation"])
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    # 物体大小假设
    size = (0.033, 0.033, 0.1)  # 假设瓶子/罐子的尺寸

    # 添加到 MoveIt 规划场景
    scene.add_box(obj["name"], pose, size=size)
    rospy.sleep(0.2)  # 等待 MoveIt 处理

print(f"成功加载 {len(objects)} 个瓶子/罐子 到 MoveIt 规划场景！")
