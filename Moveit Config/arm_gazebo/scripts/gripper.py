#!/usr/bin/env python3
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

def control_gripper(position, duration=1.0):
    """
    控制 Robotiq 2F-85 夹爪开合。

    :param position: 目标位置 (范围 0.0 ~ 0.8, 0.8 为完全打开, 0.0 为完全闭合)
    :param duration: 运动时间（秒）
    """
    rospy.init_node('gripper_control', anonymous=True)

    # 连接到 FollowJointTrajectory action server
    gripper_client = actionlib.SimpleActionClient('/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    
    rospy.loginfo("等待夹爪控制器...")
    gripper_client.wait_for_server()

    # 创建 FollowJointTrajectoryGoal 目标
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["robotiq_85_left_knuckle_joint"]  # 夹爪主关节

    # 设置目标位置
    point = JointTrajectoryPoint()
    point.positions = [position]  # 设定关节目标位置
    point.time_from_start = rospy.Duration(duration)  # 运动时间

    goal.trajectory.points.append(point)

    rospy.loginfo(f"发送夹爪目标位置: {position}")
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()
    rospy.loginfo("夹爪控制完成！")

if __name__ == '__main__':
    try:
        # 夹爪闭合（0.0）
        control_gripper(0.0, duration=10.0)
        rospy.sleep(2)

        # 夹爪张开（0.8）
        control_gripper(0.8, duration=10.0)
    except rospy.ROSInterruptException:
        pass
