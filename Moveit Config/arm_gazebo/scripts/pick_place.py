#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import moveit_msgs.msg
import sys
from geometry_msgs.msg import PoseStamped, TwistStamped
from shape_msgs.msg import SolidPrimitive  # 修改导入
from sensor_msgs.msg import JointState
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
import tf

class SceneObject:
    def __init__(self, ID, name, position, axis):
        self.ID = ID
        self.name = name
        self.position = position
        self.axis = axis

    def preGrasp(self):
        targetPose = PoseStamped()
        targetPose.header.frame_id = "base_link"
        targetPose.pose.position.x = self.position[0] + 0.1
        targetPose.pose.position.y = self.position[1]
        targetPose.pose.position.z = self.position[2] - 0.02
        q = tf.transformations.quaternion_from_euler(0, np.pi/2, np.pi)  # 绕z180, y轴旋转90度
        targetPose.pose.orientation.x = q[0]
        targetPose.pose.orientation.y = q[1]
        targetPose.pose.orientation.z = q[2]
        targetPose.pose.orientation.w = q[3]
        self.executeForTime(targetPose, 7)
        return

    def open(self):
        goal = GripperCommandGoal()
        goal.command.position = 0.085  # Max open for Robotiq-85
        goal.command.max_effort = 10.0
        client = actionlib.SimpleActionClient('/gripper_controller/gripper_cmd', GripperCommandAction)
#        client.wait_for_server()
        client.send_goal(goal)
#        client.wait_for_result()
        return client.get_result()

    def moveToGrasp(self):
        targetPose = PoseStamped()
        targetPose.header.frame_id = "base_link"
        targetPose.pose.position.x = self.position[0] + 0.1
        targetPose.pose.position.y = self.position[1]
        targetPose.pose.position.z = self.position[2] - 0.02
        self.executeForTime(targetPose, 5)
        return

    def grasp(self):
        goal = GripperCommandGoal()
        goal.command.position = 0.02  # Close for gripping
        goal.command.max_effort = 40.0
        client = actionlib.SimpleActionClient('/gripper_controller/gripper_cmd', GripperCommandAction)
#        client.wait_for_server()
        client.send_goal(goal)
#        client.wait_for_result()
        return client.get_result()


    def postGrasp(self):
        targetPose = PoseStamped()
        targetPose.header.frame_id = "base_link"
        targetPose.pose.position.x = -0.4
        targetPose.pose.position.y = 0.5 if self.position[1] > 0 else -0.5
        targetPose.pose.position.z = 0.3
        q = tf.transformations.quaternion_from_euler(0, np.pi/2, np.pi)  # 绕y轴旋转90度
        targetPose.pose.orientation.x = q[0]
        targetPose.pose.orientation.y = q[1]
        targetPose.pose.orientation.z = q[2]
        targetPose.pose.orientation.w = q[3]
        self.executeForTime(targetPose, 5)

    def attach_object_to_gripper(self):
        # 物体尺寸 (圆柱体转换为近似方形包围盒)
        height = 0.1       # 罐子高度
        radius = 0.033 / 2  # 罐子半径
        box_size = [radius * 2, radius * 2, height]  # 转换为盒子尺寸

        # 物体在夹爪中的位姿
        pose = PoseStamped()
        pose.header.frame_id = "gripper_tip_link"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = height / 2  # 罐子中心点偏移

        # 将物体附加至夹爪
        scene.attach_box("gripper_tip_link", self.ID, pose, box_size)
        rospy.loginfo(f"{self.ID} 已成功附加至夹爪")


    def pickPlacePipeline(self):
#        self.open()
        control_gripper(0.0, duration = 5)
        self.preGrasp()
#        self.moveToGrasp()
        control_gripper(0.23, duration = 5)
        self.attach_object_to_gripper()   # 新增：将物体附加到夹爪
#        self.preGrasp()
        self.postGrasp()
        return

    def executeForTime(self, targetPose, delay):
        time = rospy.Time.now()
        rate = rospy.Rate(10)

        # 打印当前位置，验证是否正确获取
 #       current_pose = move_group.get_current_pose().pose
 #       print("Current Pose Before Planning:", current_pose)

        move_group.clear_pose_targets()
        current_state = move_group.get_current_state()
        move_group.set_start_state_to_current_state()
#        move_group.set_pose_target(targetPose,end_effector_link='gripper_tip_link')
#        success = move_group.go(wait=True)
        move_group.set_end_effector_link('gripper_tip_link')
        move_group.set_goal_tolerance(0.01)
        plan = move_group.plan(targetPose)
        success = move_group.execute(plan[1], wait=True)
        while rospy.Time.now() - time < rospy.Duration(delay):
            rate.sleep()
        return

def control_gripper(pos, duration=5.0):
    """
    控制 Robotiq 2F-85 夹爪开合。

    :param position: 目标位置 (范围 0.0 ~ 0.8, 0.0 为完全打开, 0.8 为完全闭合)
    :param duration: 运动时间（秒）
    """

    # 连接到 FollowJointTrajectory action server
    gripper_client = actionlib.SimpleActionClient('/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    
    rospy.loginfo("等待夹爪控制器...")
    gripper_client.wait_for_server()

    # 创建 FollowJointTrajectoryGoal 目标
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["robotiq_85_left_knuckle_joint"]  # 夹爪主关节

    # 设置目标位置
    point = JointTrajectoryPoint()
    point.positions = [pos]  # 设定关节目标位置
    point.time_from_start = rospy.Duration(duration)  # 运动时间

    goal.trajectory.points.append(point)

    rospy.loginfo(f"发送夹爪目标位置: {pos}")
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()
    rospy.loginfo("夹爪控制完成！")

if __name__ == '__main__':
    rospy.init_node('ur5e_pick_place', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    knownObjectNames = scene.get_known_object_names()
    filteredObjectNames = [name for name in knownObjectNames if name not in ["table1", "table2"]] 
    knownObjectPoses = scene.get_object_poses(filteredObjectNames)
    allSceneObjects = []

    for name, pose in knownObjectPoses.items():
        allSceneObjects.append(SceneObject(name, name, [pose.position.x, pose.position.y, pose.position.z], 'z'))

    allSceneObjects.sort(key=lambda SceneObject: (SceneObject.position[0]))

    for obj in allSceneObjects:
        obj.pickPlacePipeline()
        scene.remove_world_object(obj.ID)
