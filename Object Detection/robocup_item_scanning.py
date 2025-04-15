#!/usr/bin/env python

import cv2
import moveit_commander
import numpy as np
import open3d as o3d
import rospy
import sensor_msgs.point_cloud2 as pc2
import sys
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, PointCloud2
from ultralytics import YOLO

pointcloud_buffer = []
MAX_FRAMES = 5 

def pointcloud_callback(msg):
    global pointcloud_buffer
    points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
    pointcloud_buffer.append(points)

    if len(pointcloud_buffer) > MAX_FRAMES:
        pointcloud_buffer.pop(0)

def merge_pointclouds():
    global pointcloud_buffer
    if len(pointcloud_buffer) == 0:
        rospy.logwarn("No point cloud data available!")
        return None

    merged_points = np.vstack(pointcloud_buffer)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(merged_points)

    downsampled_pcd = pcd.voxel_down_sample(voxel_size=0.01)

    clean_pcd, _ = downsampled_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)

    return np.asarray(clean_pcd.points)

def get_xyz_uv():
    listener = tf.TransformListener()
    listener.waitForTransform("base_link", "camera_depth_link", rospy.Time(0), rospy.Duration(4.0))
    
    '''
        how did you get these P values?;
    '''
    P = [554.3827128226441, 0.0, 320.5, -38.80678989758509, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    P = np.array(P).reshape(3, 4)
    pose = listener.lookupTransform("base_link", "camera_depth_link", rospy.Time(0))
    pos = pose[0]
    ori = pose[1]
    quat = [ori[0], ori[1], ori[2], ori[3]]
    tf_mtx = tf.transformations.quaternion_matrix(quat)
    tf_mtx[0][3] = pos[0]
    tf_mtx[1][3] = pos[1]
    tf_mtx[2][3] = pos[2]
    
    #data = rospy.wait_for_message("/camera/depth/points", PointCloud2, timeout=5.0)
    rospy.Subscriber("/camera/depth/points", PointCloud2, pointcloud_callback)
    rospy.sleep(1) 
    points = merge_pointclouds()
    xyz_uv = np.zeros((640, 480, 3), dtype=np.float32)
    
    for point in points:
        xyz = np.array([point[0], point[1], point[2], 1]).T
        uv1 = np.dot(P, xyz) / xyz[2]
        if uv1[0] < 0 or uv1[0] >= 480 or uv1[1] < 0 or uv1[1] >= 640:
            continue
        xyz = np.dot(tf_mtx, xyz)
        xyz_uv[int(uv1[1]), int(uv1[0])] = [xyz[0], xyz[1], xyz[2]]
    
    print("\n ------- \n\n xyz_uv has been calculated")
    return xyz_uv

def find_table_plane(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    
    table_points = np.asarray(pcd.select_by_index(inliers).points) # points the item contacts the table
    table_height = np.mean(table_points[:, 2])  # mean of all z coordinates of table points to find table height
    
    return plane_model, table_height

def get_object_center_3d(object_points, plane_model, threshold=0.02):
    a, b, c, d = plane_model
    norm = np.sqrt(a*a + b*b + c*c)
    
    # Calculate distance of each point to the plane
    distances = np.abs(np.dot(object_points, [a, b, c]) + d) / norm
    
    # points that are within threshold (0.02) to the table points are interpreted as contact points between item and table
    base_indices = np.where(distances < threshold)[0]
    
    if len(base_indices) < 3:
        return None
    
    base_points = object_points[base_indices]
    base_center_xy = np.mean(base_points[:, :2], axis=0)

    if len(object_points) > 0:
        z_axis_dir = np.array([a, b, c]) / norm
        # Project points onto z-axis to find height
        heights = np.dot(object_points, z_axis_dir)
        min_height = np.min(heights)
        max_height = np.max(heights)
        
        center_z = (min_height + max_height) / 2
        
        # For very flat objects, use the base z with a small offset
        if max_height - min_height < 0.02:  # If object is very flat
            center_z = min_height + 0.01
    else:
        # default
        center_z = np.mean(base_points[:, 2])
    
    full_center = np.array([base_center_xy[0], base_center_xy[1], center_z])
    
    return full_center

def build_world(xyz_uv):
    model = YOLO('/home/robot/catkin_ws/src/Robocup_Project/best.pt')
    
    rgb = rospy.wait_for_message("camera/rgb/image_raw", Image, timeout=1.0)
    bridge = CvBridge()
    rgb = bridge.imgmsg_to_cv2(rgb, 'bgr8')
    results = model(rgb)
    detections = results[0].boxes
    
    all_points = xyz_uv.reshape(-1, 3)
    all_points = all_points[~np.any(all_points == 0, axis=1)]
    plane_model, table_height = find_table_plane(all_points)
    rospy.loginfo(f"Detected table plane at approximate height: {table_height:.4f}m")
    
    for name, bb, conf in zip(detections.cls, detections.xyxy, detections.conf):
        name = str(name.cpu().numpy())
        bb = bb.cpu().numpy()
        conf = conf.cpu().numpy()
        x1, y1, x2, y2 = map(int, bb[:4])
        xlb = 1
        xrb = 635
        ylb = 1
        yrb = 455
        spawn = True
        print("Bounding box coordinates:", bb)
        if xlb<=x1<=xrb and xlb<=x2<=xrb and ylb<=y1<=yrb and ylb<=y2<=yrb and conf>0.69:
            xyz_crop = xyz_uv[y1:y2, x1:x2].reshape(-1, 3)
            xyz_crop = xyz_crop[~np.any(xyz_crop == 0, axis=1)]
            
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz_crop)

            if len(xyz_crop) < 4:
                rospy.logwarn("Insufficient points for bounding box generation. Skipping this object.")
                continue 

            bbox = pcd.get_oriented_bounding_box()
            dims = bbox.get_max_bound() - bbox.get_min_bound()
            rotation_matrix = bbox.R

            center = get_object_center_3d(xyz_crop, plane_model)
            if center is None:
                center = bbox.get_center()
                rospy.logwarn("3D center detection failed, using bounding box center instead.")

            yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]) + np.pi / 2
            roll = np.pi / 2  # 90° 
            pitch = 0
            r = R.from_euler('xyz', [roll, pitch, yaw])
            quat = r.as_quat()

            max_index = np.argmax(dims)
            if max_index == 0:
                orientation = "x"
            elif max_index == 1:
                orientation = "y"
            else:
                orientation = "z"
            
            knownObjectNames = scene.get_known_object_names()
            knownObjectPoses = scene.get_object_poses(knownObjectNames)

            for _, pose in knownObjectPoses.items():
                center_diff = np.array([(pose.position.x) - center[0],
                            pose.position.y - center[1],
                            (pose.position.z) - center[2]])
                dist = np.linalg.norm(center_diff)
                if dist < 0.06:
                    print("Object already exists in the scene, Not adding it again.")
                    spawn = False
                    break
                
            if spawn:    
                objPose = PoseStamped()
                objPose.header.frame_id = "base_link"
                objPose.pose.position.x = center[0]
                objPose.pose.position.y = center[1]
                objPose.pose.position.z = center[2]
                
                if orientation == "x":
                    objPose.pose.orientation.x = quat[0]
                    objPose.pose.orientation.y = quat[1]
                    objPose.pose.orientation.z = quat[2]
                    objPose.pose.orientation.w = quat[3]
                elif orientation == "y":
                    objPose.pose.orientation.x = quat[0]
                    objPose.pose.orientation.y = quat[1]
                    objPose.pose.orientation.z = quat[2]
                    objPose.pose.orientation.w = quat[3]

                else:
                    objPose.pose.orientation.x = 0.0
                    objPose.pose.orientation.y = 0.0
                    objPose.pose.orientation.z = 0.0
                    objPose.pose.orientation.w = 1.0
                
                global uniqueId
                uniqueName = str(uniqueId)+"_"+name+"_"+orientation
                uniqueId += 1

                # Does this still need to be changed?
                # change size
                if name == "0.0": #bottle
                    size = [0.068, 0.0668, 0.1970]
                elif name == "1.0": # can
                    size = [0.066, 0.066, 0.1168]
                else: # pouch
                    size = [0.03, 0.03, 0.02]
                
                print("Adding object to the scene")
                scene.add_box(uniqueName, objPose, size)
            
    

def makePose(coordinates):
    targetPose = PoseStamped()
    targetPose.header.stamp = rospy.Time.now()
    targetPose.header.frame_id = "base_link"
    
    targetPose.pose.position.x = coordinates[0]
    targetPose.pose.position.y = coordinates[1]
    targetPose.pose.position.z = coordinates[2]
    
    q = tf.transformations.quaternion_from_euler(0, np.pi/2, np.pi) 
    targetPose.pose.orientation.x = q[0]
    targetPose.pose.orientation.y = q[1]
    targetPose.pose.orientation.z = q[2]
    targetPose.pose.orientation.w = q[3]
    
    return targetPose

def move_to_pose(move_group, pose_goal):
    move_group.clear_pose_targets()
    move_group.set_start_state_to_current_state()
    move_group.set_end_effector_link('gripper_tip_link')
    move_group.set_goal_tolerance(0.01)
    move_group.set_pose_target(pose_goal)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    return success

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")


if __name__ == '__main__':
    
    rospy.init_node('move_ur5e_bot', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    scene.clear()
    move_group = moveit_commander.MoveGroupCommander("arm")
    uniqueId = 0
    
    coordinatesList = [[0.05, -0.40, 0.65], [0.25, 0.00, 0.70], [0.05, 0.40, 0.65],
                       [0.60, -0.40, 0.45], [0.70, 0.00, 0.45], [0.60, 0.40, 0.45]]
    try:
        ImageSubscriber()
        for coordinate in coordinatesList:
            pose = makePose(coordinate)
            rospy.loginfo(f"Moving to pose: {pose.pose.position}")
            move_to_pose(move_group, pose)
            xyz_uv = get_xyz_uv()
            build_world(xyz_uv)
        knownObjectNames = scene.get_known_object_names()
        knownObjectPoses = scene.get_object_poses(knownObjectNames)
        print(knownObjectNames, knownObjectPoses)
    except rospy.ROSInterruptException:
        pass