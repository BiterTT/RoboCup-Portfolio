# README

The code comments are all in the code, making clear what each function does and the input and output parameters. The Control_System.py tests the whole grasping system work flow and control algorithms with valid parameters, cooperating with transformation,  motion planning and detection

At the same time will MoveIt_config configure the file to filter out ros_controllers.yaml, which contains the configuration based on MoveIt to control the robot arm in the Gazebo with PID and constraints.

```python
controller_list:
  - name: arm_controller
    action_ns: follow_joint_trajectory
    default: True
    type: FollowJointTrajectory
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
  - name: gripper_controller
    action_ns: follow_joint_trajectory
    default: True
    type: FollowJointTrajectory
    joints:
      - robotiq_85_left_knuckle_joint
```

