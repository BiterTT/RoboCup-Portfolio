#!/usr/bin/env python3

import rospy
import sys
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import actionlib

from trajectory_msgs.msg import *
from control_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from tf.transformations import *
from copy import deepcopy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from math import pi

home_joint_positions = [0, 0.0, 1.57, -1.57, 0, 0]

lay_down = 0
stand = 1
pouch_width = 22
pouch_offset_z  = 0.007
bottle_offset_z_down = 0.018
bottle_offset_z_up = -0.15

can_offset_z_up = -0.01
can_offset_z_down = 0.001

bottle_width = 50
bottle_cap_width = 26

can_width = 60


world_frame_offset_z = 0.615
world_frame_offset_x = -0.1

safety_z_offset = 0.2
safety_back_z_offset = 0.2

class UR5GraspSystem:
    def __init__(self):
        rospy.init_node("ur5_grasp_system")

        # MoveIt setup
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm = MoveGroupCommander("arm")
        self.arm.set_planning_time(10.0)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_goal_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_end_effector_link("gripper_tip_link")

        self.gripper_group = MoveGroupCommander("gripper")


        # Gripper setup
        self.gripper_client = actionlib.SimpleActionClient("/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for gripper controller...")
        self.gripper_client.wait_for_server()
        rospy.loginfo("Gripper controller connected.")

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
       
        #initialize Objects
        # x y z roll pitch yaw
        self.bluebin_position = [-0.465, -0.5, 0]
        self.greenbin_position = [-0.465, 0.5, 0]
        self.pouch1_position = [ 0.434, -0.033, 0.524, 0, 0, 0, "pouch"]
        self.pouch2_position = [ 0.433, 0.041, 0.524, 0, 0, 0, "pouch" ]
        self.pouch3_position = [ 0.521, -0.031, 0.524, 0, 0, 0, "pouch" ]
        self.pouch4_position = [ 0.522, 0.042, 0.524, 0, 0, 0, "pouch" ]
        self.pouch5_position = [ 0.786, 0.204, 0.524, 0, 0, 0.25,"pouch" ]
        self.pouch6_position = [ 0.766, 0.284, 0.524, 0, 0, 0.3, "pouch" ]
        self.pouch7_position = [ 0.521, -0.131, 0.524, 0, 0, 1.0,"pouch" ]
        self.pouch8_position = [ 0.433, 0.131, 0.524, 0, 0, 0.8, "pouch" ]

        self.gCan1_position = [-0.1067, 0.3634, 0.69,  0.00,  -0.01,  0.00, "can"]
        self.gCan2_position = [0.2191, -0.0220, 0.547,  1.57,  -1.55,  0.01, "can"]
        self.gCan3_position = [-0.0809, -0.6600, 0.5733,  0.0,  -0.0,  -1.73, "can"]
        self.gCan4_position = [0.3690, -0.5296, 0.5904,  0.0,  -0.0, -2.99, "can"]
    
        self.rCan1_position = [0.295, 0.5031, 0.5729, 3.14, -0.00, 3.00, "can"]
        self.rCan2_position = [-0.0809, -0.6600, 0.6896, 3.14, -0.00, 0.0, "can"]
        self.rCan3_position = [0.7, 0.0322, 0.5730, 3.14, -0.00, 0.66, "can"]
   
        self.yCan1_position = [0.6, 0.1722, 0.5729, 3.14, -0.00, 1.3, "can"]
        self.yCan2_position = [0.55, -0.4821, 0.5648, -1.57, 0.10, -0.83, "can"]  
        self.yCan3_position = [0.36, -0.45, 0.596, 0.0, 0.0, -2.95, "can"]  
        self.yCan4_position = [0.26, -0.42, 0.5648, -1.57, -1.09, 2.147, "can"]  
        
        self.rBottle1_position = [0.22, 0.62, 0.55, 1.57, -0.42, 1.53, "bottle"]
        self.rBottle2_position = [0.820, 0.0, 0.5485, 1.57, -0.10, 0.0, "bottle"]

        self.bBottle1_position = [-0.17, -0.46, 0.6135,  0.00,  -0.00,  -0.00, "bottle"]
        self.bBottle2_position = [0.0734, 0.2300, 0.6136, 0.00, -0.00, 0.00, "bottle"]
        self.bBottle3_position = [0.2630, -0.2251, 0.5488, 1.61, 1.56, 1.61, "bottle"]
        
        self.yBottle1_position = [-0.1430, 0.5264, 0.5485, 1.57, 1.47, 1.58, "bottle"]
        self.yBottle2_position = [0.680, -0.1600, 0.61, 0.00, -0.00, 0.00, "bottle"]
        self.yBottle3_position = [0.3064, 0.1509, 0.6136, 0.00, 0.00, -0.01, "bottle"]
        self.yBottle4_position = [0.234, -0.6251, 0.5659, 1.69, 1.47, 1.8, "bottle"]

        self. Grasp_Order = []

        self.position_dict = {
        "pouch1": self.pouch1_position,
        "pouch2": self.pouch2_position,
        "pouch3": self.pouch3_position,
        "pouch4": self.pouch4_position,
        "pouch5": self.pouch5_position,
        "pouch6": self.pouch6_position,
        "pouch7": self.pouch7_position,
        "pouch8": self.pouch8_position,
        "gCan1": self.gCan1_position,
        "gCan2": self.gCan2_position,
        "gCan3": self.gCan3_position,
        "gCan4": self.gCan4_position,
        "rCan1": self.rCan1_position,
        "rCan2": self.rCan2_position,
        "rCan3": self.rCan3_position,
        "yCan1": self.yCan1_position,
        "yCan2": self.yCan2_position,
        "yCan3": self.yCan3_position,
        "yCan4": self.yCan4_position,
        "rBottle1": self.rBottle1_position,
        "rBottle2": self.rBottle2_position,
        "bBottle1": self.bBottle1_position,
        "bBottle2": self.bBottle2_position,
        "bBottle3": self.bBottle3_position,
        "yBottle1": self.yBottle1_position,
        "yBottle2": self.yBottle2_position,
        "yBottle3": self.yBottle3_position,
        "yBottle4": self.yBottle4_position
    }


        self.move_to_home()

    # def Object_Init(self):
        
    #print arm information
    def print_arm_reference_info(self):
        print("Planning frame:", self.arm.get_planning_frame())
        print("End effector link:", self.arm.get_end_effector_link())
        print("Pose reference frame  :", self.arm.get_pose_reference_frame())
        
    #pos ≈ 0.8 × (1 - 开口宽度 / 85)
    def estimate_gripper_pos_from_object_width(self, width_mm):
        return max(0.0, min(0.8, 0.8 - (width_mm / 85.0)))  # 简化估算


    def Judge_Object_Pose(self, Object_Pose):
        #roll pitch yaw
        Object_Pose_roll = Object_Pose[3]
        Object_Pose_pitch = Object_Pose[4]
        Object_Pose_yaw = Object_Pose[5]

        if ((((Object_Pose_roll < -1.5) | (Object_Pose_roll > 1.5)) & (Object_Pose_roll < 2)) | ((Object_Pose_roll<0.2) & (Object_Pose_roll > -0.2))):
            object_pose = lay_down
        else:
            object_pose = stand
        return object_pose
    
    def Judge_Gripper_Pose(self, Object_Pose):
        object_pose = self.Judge_Object_Pose(Object_Pose)
        if (object_pose == lay_down):
            pitch=pi/2
            roll = 0
            return pitch,roll
        if (object_pose == stand):
            roll=-pi
            pitch = 0
            return pitch,roll

    def get_current_pose(self, arm):
        """获取当前位姿并打印"""
        current_pose = arm.get_current_pose().pose
        rospy.loginfo("Current pose is:\n{}".format(current_pose))
        return current_pose
    
    def quaternion_from_euler_Setpoints(self, x, y, z, roll, pitch, yaw):
        """创建目标位姿，内部将欧拉角转换为四元数"""
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw
        return pose_goal
    
    import math

    def cartesian_move(self, start_pose, target_pose, 
                    eef_step=0.1, 
                    jump_threshold=False, 
                    fraction_threshold=0.9,
                    max_planning_attempts=10):
        # 1. 生成 waypoints（起点和目标）
        waypoints = [deepcopy(start_pose), deepcopy(target_pose)]

        plan = None
        fraction = 0.0

        # 尝试多次规划，直到达到成功率或用完次数
        for attempt in range(1, max_planning_attempts + 1):
            (plan, fraction) = self.arm.compute_cartesian_path(
                waypoints, 
                eef_step, 
                jump_threshold
            )
            rospy.loginfo("Attempt %d: fraction=%.2f" % (attempt, fraction))
            if fraction >= fraction_threshold:
                rospy.loginfo("笛卡尔路径规划第 %d 次尝试成功，fraction=%.2f" % (attempt, fraction))
                break
            else:
                rospy.logwarn("笛卡尔路径规划第 %d 次尝试失败，fraction=%.2f，继续重试..." % (attempt, fraction))

        # 判断最终结果
        if plan is None:
            rospy.logerr("未生成任何规划轨迹！")
            return
    
        if fraction < fraction_threshold:
            rospy.logerr("在 %d 次尝试后，仍未达到规划成功率阈值（%.2f），最终 fraction=%.2f" 
                        % (max_planning_attempts, fraction_threshold, fraction))
            rospy.loginfo("执行当前已生成的部分轨迹...")
        else:
            rospy.loginfo("笛卡尔路径规划成功，执行完整轨迹...")

        # 使用 retime_trajectory 对轨迹进行时间重参数化，自动分配合理的时间戳
        #plan = self.arm.retime_trajectory(self.robot.get_current_state(), plan)

        # 只修正前两个点的时间戳
        self._fix_first_two_timestamps(plan, min_dt=0.001)

        # 对轨迹进行降采样，过滤掉冗余点
        #plan = self._downsample_trajectory(plan, joint_threshold=0.01)

        #for i, point in enumerate(plan.joint_trajectory.points):
            #rospy.loginfo("Point %d, time_from_start = %.3f", i, point.time_from_start.to_sec())

    
        execute_result = self.arm.execute(plan, wait=True)
        rospy.loginfo("Trajectory execution result: %s" % str(execute_result))
        
        # 清理
        rospy.sleep(1.0)
        self.arm.clear_pose_targets()


    def _fix_first_two_timestamps(self, plan, min_dt=0.001):
        """
        如果轨迹中第 0 个和第 1 个路点的 time_from_start 相同，
        则将第 1 个路点的时间戳设置为第 0 个时间戳加上 min_dt。
        :param plan: RobotTrajectory 对象
        :param min_dt: 最小时间间隔（单位：秒）
        """
        if plan is None or not plan.joint_trajectory.points:
            rospy.logwarn("轨迹为空，无法修正时间戳。")
            return

        points = plan.joint_trajectory.points
        if len(points) >= 2:
            t0 = points[0].time_from_start.to_sec()
            t1 = points[1].time_from_start.to_sec()
            if t1 <= t0:
                new_t1 = t0 + min_dt
                points[1].time_from_start = rospy.Duration(new_t1)
                rospy.logwarn("修正第 1 个点的时间戳为 %.3f 秒" % new_t1)

    def _downsample_trajectory(self, plan, joint_threshold=0.01):
        """
        对轨迹进行降采样，去掉相邻变化太小的轨迹点。
        :param plan: 由 compute_cartesian_path 或 retime_trajectory 生成的 RobotTrajectory 对象
        :param joint_threshold: 两个轨迹点关节值变化的最小阈值（欧式距离），低于该值的点会被舍弃
        :return: 降采样后的轨迹 plan
        """
        if plan is None or not plan.joint_trajectory.points:
            rospy.logwarn("轨迹为空，无法降采样。")
            return plan

        new_points = []
        prev_point = None
        for point in plan.joint_trajectory.points:
            if prev_point is None:
                new_points.append(point)
                prev_point = point
            else:
                # 计算欧式距离差异
                diff = sum((p - q)**2 for p, q in zip(point.positions, prev_point.positions))**0.5
                if diff >= joint_threshold:
                    new_points.append(point)
                    prev_point = point
                else:
                    rospy.loginfo("降采样：舍弃冗余点，关节差异 = %.3f" % diff)
        plan.joint_trajectory.points = new_points
        rospy.loginfo("降采样后轨迹点数量：%d" % len(new_points))
        return plan


    def move_to_pregrasp_pose(self, x, y, z, roll, pitch, yaw):
        # 1. 获取当前位姿
        start_pose = self.arm.get_current_pose().pose

        offset_z = 0.1
        z = z + offset_z

        # 2. 构造目标位姿
        target_pose = self.quaternion_from_euler_Setpoints(x, y, z, roll, pitch, yaw)

        # 4. 调用cartesian_move
        self.cartesian_move(start_pose, target_pose)
     

    def move_to_grasp_pose(self, x, y, z,roll, pitch, yaw):
         # 1. 获取当前位姿
        start_pose = self.arm.get_current_pose().pose

        # 2. 构造目标位姿
        target_pose = self.quaternion_from_euler_Setpoints(x, y, z, roll, pitch, yaw)

        # 调用 cartesian_move
        self.cartesian_move(start_pose, target_pose)

    def move_to_back_pose(self, x, y, z,roll, pitch, yaw):
        # 1. 获取当前位姿
        start_pose = self.arm.get_current_pose().pose

        # 2. 构造目标位姿，目标位姿在 z 方向上加 0.2m
        target_pose = self.quaternion_from_euler_Setpoints(x, y, z + 0.6, roll, pitch, yaw)

        # 调用 cartesian_move
        self.cartesian_move(start_pose, target_pose)


    def control_gripper(self, gripper_client, pos, open_gripper=True):
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

    def attach_object(self, robot_model, robot_link, object_model, object_link):
        """
        Attach an object to the robot's link via the link_attacher_node service.
        """
        rospy.wait_for_service("/link_attacher_node/attach")
        try:
            attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
            req = AttachRequest()
            req.model_name_1 = robot_model
            req.link_name_1  = robot_link
            req.model_name_2 = object_model
            req.link_name_2  = object_link
            resp = attach_srv.call(req)
            return resp.ok
        except rospy.ServiceException as e:
            rospy.logerr("Attach service call failed: %s", e)
            return False
    
    def detach_object(self, robot_model, robot_link, object_model, object_link):
        """
        Detach an object from the robot's link via the link_attacher_node service.
        """
        rospy.wait_for_service("/link_attacher_node/detach")
        detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
        req = AttachRequest()
        req.model_name_1 = robot_model
        req.link_name_1  = robot_link
        req.model_name_2 = object_model
        req.link_name_2  = object_link
        resp = detach_srv.call(req)
        return resp.ok
        
    def lift_arm(self):
        pose = self.arm.get_current_pose().pose
        pose.position.z += 0.5
        self.arm.set_pose_target(pose)
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

    def move_to_home(self):
        self.arm.set_joint_value_target(home_joint_positions)
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        #self.control_gripper(self.gripper_client, 0.0, open_gripper=True)
        rospy.sleep(1)

    def Drop_Greenbin(self, Object_ID):
        roll=0.0 
        pitch=pi/2
        yaw=0.0
        target_position = self.greenbin_position 
        Greenbin_pose = self.quaternion_from_euler_Setpoints(target_position[0], target_position[1], target_position[2] +0.1, roll, pitch, yaw)
        
        # 获取当前位姿
        start_pose = self.arm.get_current_pose().pose

        # 使用笛卡尔路径规划移动到目标位姿
        self.cartesian_move(start_pose, Greenbin_pose)

        rospy.logwarn("Drop_Greenbin: 执行丢弃操作...")
        rospy.sleep(1.0)
        self.arm.clear_pose_targets()

        self.detach_object("robot", "wrist_3_link",Object_ID, "link")
        self.control_gripper(self.gripper_client, 0.0, open_gripper=True)
        

    def Drop_Bluebin(self, Object_ID):
        roll=0.0 
        pitch=pi/2
        yaw=0.0
        target_position = self.bluebin_position 
        Bluebin_pose = self.quaternion_from_euler_Setpoints(target_position[0], target_position[1], target_position[2] +0.1, roll, pitch, yaw)
        
        # 获取当前位姿
        start_pose = self.arm.get_current_pose().pose

        # 调用笛卡尔路径规划函数进行运动
        self.cartesian_move(start_pose, Bluebin_pose)
    
        rospy.logwarn("Drop_Bluebin: 执行丢弃操作...")
        rospy.sleep(1.0)
        self.arm.clear_pose_targets()

        # 分离物体，并打开夹爪
        self.detach_object("robot", "wrist_3_link",Object_ID, "link")
        self.control_gripper(self.gripper_client, 0.0, open_gripper=True)

    #caculate bottle_offset_z, bottle_grasp_width in different pose
    def Bottle_Grasp_Judge(self, Object_ID):
        bottle_ori = self.position_dict[Object_ID]
        object_pos = self.Judge_Object_Pose(bottle_ori)
        if object_pos == lay_down:
            bottle_offset_z = bottle_offset_z_down
            bottle_grasp_width = bottle_width
        if object_pos == stand:
            bottle_offset_z = bottle_offset_z_up
            bottle_grasp_width = bottle_cap_width
        return bottle_offset_z, bottle_grasp_width

    def Can_Grasp_Judge(self, Object_ID):
        can_ori = self.position_dict[Object_ID]
        object_pos = self.Judge_Object_Pose(can_ori)
        if object_pos == lay_down:
            bottle_offset_z = can_offset_z_down
            bottle_grasp_width = can_width
        if object_pos == stand:
            bottle_offset_z = can_offset_z_up
            bottle_grasp_width = can_width
        return bottle_offset_z, bottle_grasp_width

    def Object_Seek(self, Object_ID):
        Target_pos = self.position_dict[Object_ID]
        return Target_pos

    def Judge_Object_Ori(self, Object_ID):
        Target_Pos = self.Object_Seek(Object_ID)
        if Target_Pos[6] == "bottle":
            print("bottle")
            height_offset_z, grasp_width = self.Bottle_Grasp_Judge(Object_ID)
        elif Target_Pos[6] == "can":
            height_offset_z, grasp_width = self.Can_Grasp_Judge(Object_ID)
            print("can")
        else:
            height_offset_z = pouch_offset_z
            grasp_width = pouch_width
            print("pouch")
        
        return height_offset_z, grasp_width



    def test(self, Object_ID):
        #Gain information
        Target_Pos = self.Object_Seek(Object_ID)
        #Gain height_offset_z grasp_width
        height_offset_z, grasp_width = self.Judge_Object_Ori(Object_ID)

        #pitch, roll = self.Judge_Gripper_Pose(Target_Pos)
        x=Target_Pos[0] - world_frame_offset_x
        y=Target_Pos[1]
        z=Target_Pos[2] - world_frame_offset_z - height_offset_z
        roll = Target_Pos[3]
        pitch = pi/2
        yaw = Target_Pos[5]+ pi/2
        #pregrasp_pos
        self.move_to_pregrasp_pose(x, y, z,roll, pitch, yaw )
        #grasp_pos
        self.move_to_grasp_pose(x, y, z,roll, pitch, yaw )

        #caculate gripper pos
        pos = self.estimate_gripper_pos_from_object_width(grasp_width)
        #close gripper
        self.control_gripper(self.gripper_client, pos, open_gripper=False)
        success = self.attach_object("robot", "wrist_3_link", Object_ID, "link")
        print(success)
        #rospy.sleep(2)
        self.move_to_back_pose(0.5, 0, 0.3,roll, pitch, yaw )
        self.move_to_home()
        self.Drop_Greenbin(Object_ID)
        self.move_to_home()

    def test3(self, Object_ID):
        #Gain information
        Target_Pos = self.Object_Seek(Object_ID)
        #Gain height_offset_z grasp_width
        height_offset_z, grasp_width = self.Judge_Object_Ori(Object_ID)

        #pitch, roll = self.Judge_Gripper_Pose(Target_Pos)
        x=Target_Pos[0] - world_frame_offset_x
        y=Target_Pos[1]+ 0.005
        z=Target_Pos[2] - world_frame_offset_z - height_offset_z
        roll = Target_Pos[3]
        pitch = pi/2
        yaw = Target_Pos[5]
        #pregrasp_pos
        self.move_to_pregrasp_pose(x, y, z,roll, pitch, yaw )
        #grasp_pos
        self.move_to_grasp_pose(x, y, z,roll, pitch, yaw )

        #caculate gripper pos
        pos = self.estimate_gripper_pos_from_object_width(grasp_width)
        #close gripper
        self.control_gripper(self.gripper_client, pos, open_gripper=False)
        success = self.attach_object("robot", "wrist_3_link", Object_ID, "link")
        print(success)
        #rospy.sleep(2)
        self.move_to_back_pose(0.5, 0, 0.3,roll, pitch, yaw )
        self.move_to_home()
        self.Drop_Greenbin(Object_ID)
        self.move_to_home()

    def test4(self, Object_ID):
        #Gain information
        Target_Pos = self.Object_Seek(Object_ID)
        #Gain height_offset_z grasp_width
        height_offset_z, grasp_width = self.Judge_Object_Ori(Object_ID)

        #pitch, roll = self.Judge_Gripper_Pose(Target_Pos)
        x=Target_Pos[0] - world_frame_offset_x
        y=Target_Pos[1] - 0.002
        z=Target_Pos[2] - world_frame_offset_z - height_offset_z
        roll = Target_Pos[3]
        pitch = pi/2
        yaw = Target_Pos[5]
        #pregrasp_pos
        self.move_to_pregrasp_pose(x, y, z,roll, pitch, yaw )
        #grasp_pos
        self.move_to_grasp_pose(x, y, z,roll, pitch, yaw )

        #caculate gripper pos
        pos = self.estimate_gripper_pos_from_object_width(grasp_width)
        #close gripper
        self.control_gripper(self.gripper_client, pos, open_gripper=False)
        success = self.attach_object("robot", "wrist_3_link", Object_ID, "link")
        print(success)
        #rospy.sleep(2)
        self.move_to_back_pose(0.5, 0, 0.3,roll, pitch, yaw )
        self.move_to_home()
        self.Drop_Greenbin(Object_ID)
        self.move_to_home()

    def test2(self, Object_ID):
        #Gain information
        Target_Pos = self.Object_Seek(Object_ID)
        #Gain height_offset_z grasp_width
        height_offset_z, grasp_width = self.Judge_Object_Ori(Object_ID)

        #pitch, roll = self.Judge_Gripper_Pose(Target_Pos)
        x=Target_Pos[0] - world_frame_offset_x
        y=Target_Pos[1] + 0.018
        z=Target_Pos[2] - world_frame_offset_z - height_offset_z
        roll = Target_Pos[3]
        pitch = pi/2
        yaw = Target_Pos[5]
        #pregrasp_pos
        self.move_to_pregrasp_pose(x, y, z,roll, pitch, yaw )
        #grasp_pos
        self.move_to_grasp_pose(x, y, z,roll, pitch, yaw )

        #caculate gripper pos
        pos = self.estimate_gripper_pos_from_object_width(grasp_width)
        #close gripper
        self.control_gripper(self.gripper_client, pos, open_gripper=False)
        success = self.attach_object("robot", "wrist_3_link", Object_ID, "link")
        print(success)
        #rospy.sleep(2)
        self.move_to_back_pose(0.5, 0, 0.3,roll, pitch, yaw )
        self.move_to_home()
        self.Drop_Bluebin(Object_ID)
        self.move_to_home()

    #def test_
    def run(self):
        rate = rospy.Rate(10)

        self.test("yCan3")
        self.test("gCan4")
        self.test3("yCan2")
        self.test4("yCan4")
        self.test2("yBottle4")
        

if __name__ == "__main__":
    try:
        grasp_system = UR5GraspSystem()
        grasp_system.run()
    except rospy.ROSInterruptException:
        pass
