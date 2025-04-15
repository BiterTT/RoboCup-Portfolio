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
bottle_offset_z_up = -0.08

can_offset_z_up = -0.01
can_offset_z_down = 0.01

bottle_width = 50
bottle_cap_width = 26

can_width = 50


world_frame_offset_z = 0.615
world_frame_offset_x = -0.1

safety_z_offset_outbound = 0.2
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
        self.yCan3_position = [0.36, -0.45, 0.59, 0.0, 0.0, -2.95, "can"]  
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

        if(((Object_Pose_roll< 0.1) & (Object_Pose_roll > -0.1)) | (Object_Pose_roll > 3.1)):
            object_pose = stand
        else:
            object_pose = lay_down
        # if ((((Object_Pose_roll < -1.5) | (Object_Pose_roll > 1.5)) & (Object_Pose_roll < 2)) | ((Object_Pose_roll<0.2) & (Object_Pose_roll > -0.2))):
        #     object_pose = lay_down
        # else:
        #     object_pose = stand
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

    def move_to_pregrasp_pose(self, x, y, z, roll, pitch, yaw, safety_z_offset):
        #Gain target pose
        offset_z = safety_z_offset
        z = z + offset_z
        pose = self.quaternion_from_euler_Setpoints(x, y, z, roll, pitch, yaw)
        self.arm.set_pose_target(pose)
        plan = self.arm.plan()
        success = self.arm.execute(plan[1], wait=True)

        rospy.logwarn("Planning to pre-grasp pose.")
        rospy.sleep(1)
        # self.get_current_pose(self.arm) 
        self.arm.clear_pose_targets()
     

    def move_to_grasp_pose(self, x, y, z,roll, pitch, yaw):
        pose = self.quaternion_from_euler_Setpoints(x, y, z, roll, pitch, yaw)
        self.arm.set_pose_target(pose)
        plan = self.arm.plan()
        success = self.arm.execute(plan[1], wait=True)

        rospy.logwarn("Planning to grasp pose .")
        rospy.sleep(1)
        #self.get_current_pose(self.arm)
        self.arm.clear_pose_targets()

    def move_to_back_pose(self, x, y, z,roll, pitch, yaw):
        pose = self.quaternion_from_euler_Setpoints(x, y, z + safety_back_z_offset, roll, pitch, yaw)
        self.arm.set_pose_target(pose)
        plan = self.arm.plan()
        success = self.arm.execute(plan[1], wait=True)

        rospy.logwarn("Planning to back pose .")
        rospy.sleep(2)
        #self.get_current_pose(self.arm)
        self.arm.clear_pose_targets()


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
        rospy.sleep(5)

    def Drop_Greenbin(self, Object_ID):
        roll=0.0 
        pitch=pi/2
        yaw=0.0
        target_position = self.greenbin_position 
        Greenbin_pose = self.quaternion_from_euler_Setpoints(target_position[0], target_position[1], target_position[2] +0.3, roll, pitch, yaw)
        self.arm.set_pose_target(Greenbin_pose)
        plan = self.arm.plan()
        success = self.arm.execute(plan[1], wait=True)
        rospy.logwarn("Planning to Drop_Greenbin.")
        rospy.sleep(2)
        # self.get_current_pose(self.arm)
        self.arm.clear_pose_targets()

        self.detach_object("robot", "wrist_3_link",Object_ID, "link")
        self.control_gripper(self.gripper_client, 0.0, open_gripper=True)
        

    def Drop_Bluebin(self, Object_ID):
        roll=0.0 
        pitch=pi/2
        yaw=0.0
        target_position = self.bluebin_position 
        Bluebin_pose = self.quaternion_from_euler_Setpoints(target_position[0], target_position[1], target_position[2] +0.3, roll, pitch, yaw)
        self.arm.set_pose_target(Bluebin_pose)
        plan = self.arm.plan()
        success = self.arm.execute(plan[1], wait=True)
        rospy.logwarn("Planning to Drop_Bluebin.")
        rospy.sleep(2)
        #self.get_current_pose(self.arm)
        self.arm.clear_pose_targets()
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
        
    # def grasp_object(self, Object_position):
    #     x = Object_position[0]
    #     y = Object_position[1]
    #     z = Object_position[2]
    #     roll = Object_position[3]
    #     pitch = Object_position[4]

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

    def Area_Judge(self, Object_ID):
        pos = self.position_dict[Object_ID]
        x = pos[0] - world_frame_offset_x
        y = pos[1]

        if x > 0.55: 
            pre_grasp_z_offset = safety_z_offset_outbound
        else:
            pre_grasp_z_offset = 0.1
        return pre_grasp_z_offset

    def test(self, Object_ID):
        #pitch  up down pitch=pi/2 down 90
        #pitch=pi/2, yaw=pi/2  hengxiang
        #pitch=pi/2, yaw=0  zongxiang
        #roll = 
        # self.move_to_pregrasp_pose(x=0.4, y=0.1, z=0.3,roll=0.0, pitch=pi/2, yaw=pi/2)

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
        yaw = Target_Pos[5]

        safety_z_offset = self.Area_Judge(Object_ID)

        #pregrasp_pos
        self.move_to_pregrasp_pose(x, y, z,roll, pitch, yaw, safety_z_offset )
        #grasp_pos
        self.move_to_grasp_pose(x, y, z,roll, pitch, yaw )

        #caculate gripper pos
        pos = self.estimate_gripper_pos_from_object_width(grasp_width)
        #close gripper
        self.control_gripper(self.gripper_client, pos, open_gripper=False)
        success = self.attach_object("robot", "wrist_3_link", Object_ID, "link")
        print(success)
        rospy.sleep(2)
        self.move_to_back_pose(x, y, 0.3,roll, pitch, yaw )
        self.move_to_home()
        Object = Target_Pos[6]
        if ((Object == "pouch") | (Object == "can")):
            self.Drop_Greenbin(Object_ID)
        else:
            self.Drop_Bluebin(Object_ID)
        self.move_to_home()

    def test2(self):
        pitch, roll = self.Judge_Gripper_Pose(self.pouch2_position)
        x=self.pouch2_position[0] - world_frame_offset_x
        y=self.pouch2_position[1]
        z=self.pouch2_position[2] - world_frame_offset_z -0.007
        self.move_to_pregrasp_pose(x, y, z,roll, pitch, yaw = 0)
        self.move_to_grasp_pose(x, y, z,roll, pitch, yaw = 0)
        pos = self.estimate_gripper_pos_from_object_width(22)
        self.control_gripper(self.gripper_client, pos, open_gripper=False)
        success = self.attach_object("robot", "wrist_3_link","pouch2", "link")
        print(success)
        rospy.sleep(2)
        #self.move_to_back_pose(x, y, z,roll, pitch, yaw = 0)
        self.move_to_home()
        self.Drop_Greenbin()
        self.move_to_home()
        self.control_gripper(self.gripper_client, 0.0, open_gripper=True)

    



    #def test_
    def run(self):
        rate = rospy.Rate(10)
        # self.print_arm_reference_info()
        # self.move_to_pregrasp_pose(x=0.4, y=0.1, z=0.3,roll=0.0, pitch=pi/2, yaw=0.0)
        #self.move_to_grasp_pose(x=0.4, y=0.1, z=0.3,roll=0.0, pitch=pi/2, yaw=0.25)

        self.test("pouch1")
        self.test("pouch2")
        # # self.test("pouch3")
        # # self.test("pouch4")
        # self.test("pouch5")
        # self.test("pouch6")
        self.test("pouch7")
        self.test("pouch8")

        self.test("rBottle1")
        self.test("rBottle2")
        
        self.test("bBottle2")
        self.test("bBottle3")

        self.test("yBottle2")
        self.test("yBottle3")

        self.test("bBottle1")
        # self.test("yBottle1")
        # self.test("yBottle4")
        self.test("rCan1")
        self.test("rCan2")
        self.test("rCan3")

        self.test("gCan1")
        self.test("gCan2")
        self.test("gCan4")



        # rospy.sleep(0.5)
        # self.test2()
        # self.gripper_group.set_named_target("close")
        # print(self.gripper_group.get_named_targets())
        # success = self.gripper_group.go(wait=True)
        # print("Direct joint command success:", success)

        # goal = FollowJointTrajectoryGoal()
        # goal.trajectory.joint_names = ["robotiq_85_left_knuckle_joint"]
        # point = JointTrajectoryPoint()

        # point = JointTrajectoryPoint()
        # point.positions = [0.53]
        # point.time_from_start = rospy.Duration(1.0)

        # goal.trajectory.points.append(point)
        # goal.trajectory.header.stamp = rospy.Time.now()

        # self.gripper_client.send_goal(goal)
        # self.gripper_client.wait_for_result()
        # print("Gripper result:", self.gripper_client.get_result())

        #self.gripper_group.go(wait=True)
        

        # target_pose = self.quaternion_from_euler_Setpoints(x=0.4, y=0.1, z=0.2,roll=0.0, pitch=pi, yaw=0.0)
        # self.arm.set_pose_target(target_pose)
        # plan = self.arm.plan()
        # if plan and plan[0]:  # 关键判断：是否规划成功
        #     success = self.arm.execute(plan[1], wait=True)
        #     print("Execution success:", success)
        # else:
        #     print("Planning failed.")
        

if __name__ == "__main__":
    try:
        grasp_system = UR5GraspSystem()
        grasp_system.run()
    except rospy.ROSInterruptException:
        pass
