#!/usr/bin/env python3

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class UR5eCameraTransformation:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ur5e_camera_transformation')
        
        # TF2 buffer and listener for transformations
        self.tf_cache_time = rospy.Duration(30.0)
        self.tf_buffer = tf2_ros.Buffer(self.tf_cache_time)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Store the oldest and newest timestamps in the buffer for monitoring
        self.oldest_transform_time = rospy.Time.now()
        self.newest_transform_time = rospy.Time.now()
        
        # Set up a timer to monitor the TF buffer
        rospy.Timer(rospy.Duration(5.0), self.monitor_tf_buffer)
        
        # Publishers
        self.object_pose_pub = rospy.Publisher('/object_poses_base_frame', PoseStamped, queue_size=10)
        self.markers_pub = rospy.Publisher('/object_markers', MarkerArray, queue_size=10)
        
        # Frame IDs
        self.camera_frame = "camera_color_optical_frame"  # Camera optical frame
        self.robot_base_frame = "base_link"  # UR5e base frame
        self.tool_frame = "tool0"  # UR5e tool frame (end flange)
        
        # Subscribe to object detections (output from detection node)
        # Detection should publish PoseStamped messages with object poses in camera frame
        self.detection_sub = rospy.Subscriber(
            '/camera_frame_detections', 
            PoseStamped, 
            self.detection_callback
        )
        
        # Dictionary to store object information by ID
        self.objects = {}
        
        # Verify the TF tree
        rospy.Timer(rospy.Duration(5.0), self.check_tf_tree)
        
        rospy.loginfo("UR5e Camera Transformation node initialized")
    
    def check_tf_tree(self, event=None):
        try:
            # Check that we can transform from camera to robot base
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                self.camera_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            rospy.loginfo(f"Transform from {self.camera_frame} to {self.robot_base_frame} is available")
            
            # Check the transform from tool to camera
            tool_to_camera = self.tf_buffer.lookup_transform(
                self.tool_frame,
                self.camera_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            # Log the transformation for verification
            t = tool_to_camera.transform.translation
            r = tool_to_camera.transform.rotation
            rospy.loginfo(f"Camera is mounted at position [{t.x:.3f}, {t.y:.3f}, {t.z:.3f}] " +
                         f"with orientation [{r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f}] relative to tool frame")
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF tree check failed: {e}")
            rospy.logwarn("Ensure all transforms are being published correctly")
    
    def monitor_tf_buffer(self, event=None):
        """Monitor the TF buffer to ensure it contains transforms for the required time range"""
        try:
            now = rospy.Time.now()
            
            # Try to get the oldest transform in the buffer
            try:
                # We use a very old timestamp and see how far back we can go
                # The lookup will fail but will set the stamp to the oldest available time
                transform = self.tf_buffer.lookup_transform(
                    self.robot_base_frame,
                    self.camera_frame,
                    now - rospy.Duration(3600),  # Look 1 hour back (will fail, but that's expected)
                    rospy.Duration(0.1)
                )
            except tf2_ros.ExtrapolationException as e:
                # Extract the oldest available time from the error message
                error_msg = str(e)
                if "Lookup would require extrapolation into the past.  Requested time" in error_msg:
                    # Parse the error message to extract the oldest available time
                    parts = error_msg.split("Requested time ")
                    if len(parts) > 1:
                        time_parts = parts[1].split(" ")
                        if len(time_parts) > 0:
                            try:
                                oldest_time_sec = float(time_parts[0])
                                self.oldest_transform_time = rospy.Time(oldest_time_sec)
                            except ValueError:
                                pass
            
            # Get the newest transform (should be very close to now)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_base_frame,
                    self.camera_frame,
                    rospy.Time(0),  # Latest available
                    rospy.Duration(0.1)
                )
                self.newest_transform_time = transform.header.stamp
            except:
                pass
            
            # Calculate buffer size
            buffer_size = (self.newest_transform_time - self.oldest_transform_time).to_sec()
            
            # Log buffer information
            rospy.loginfo(f"TF Buffer information:")
            rospy.loginfo(f"  Buffer time capacity: {self.tf_cache_time.to_sec():.1f} seconds")
            rospy.loginfo(f"  Oldest transform: {self.oldest_transform_time.to_sec():.3f} seconds ago")
            rospy.loginfo(f"  Newest transform: {self.newest_transform_time.to_sec():.3f} seconds ago")
            rospy.loginfo(f"  Actual buffer size: {buffer_size:.3f} seconds")
                      
        except Exception as e:
            rospy.logwarn(f"Error monitoring TF buffer: {e}")
    
    def detection_callback(self, msg):
        """
        IMPORTANT: This assumes that the timestamp in the message header
        matches the time when the image was captured, which is essential
        for accurate transformation in a moving robot.
        """
        try:
            # Get object ID (assuming it's part of the message or generate one)
            object_id = msg.header.seq  # Example: use sequence number as ID
            object_type = "unknown"  # This would come from Detection
            
            pose_in_camera = msg
            image_timestamp = msg.header.stamp
            
            # Transform the pose from camera frame to robot base frame using the image timestamp
            base_pose = self.transform_pose_to_base_frame(pose_in_camera, image_timestamp)
            
            if base_pose:
                # Store object information
                self.objects[object_id] = {
                    'pose_camera': pose_in_camera,
                    'pose_base': base_pose,
                    'type': object_type,
                    'last_seen': rospy.Time.now()
                }
                
                # Publish the transformed pose
                self.object_pose_pub.publish(base_pose)
                
                # Create and publish visualization markers
                markers = self.create_object_markers()
                self.markers_pub.publish(markers)
                
                rospy.loginfo(f"Object {object_id} transformed from camera to base frame")
                
                # Print the transformation for debugging
                pc = pose_in_camera.pose.position
                pb = base_pose.pose.position
                rospy.logdebug(f"Position in camera frame: [{pc.x:.3f}, {pc.y:.3f}, {pc.z:.3f}]")
                rospy.logdebug(f"Position in base frame: [{pb.x:.3f}, {pb.y:.3f}, {pb.z:.3f}]")
                
        except Exception as e:
            rospy.logerr(f"Error transforming pose: {e}")
    
    def transform_pose_to_base_frame(self, pose_stamped, timestamp=None):
        try:
            # Use the provided timestamp or the one from the message
            transform_time = timestamp if timestamp is not None else pose_stamped.header.stamp
            
            # CRITICAL: Ensure we're using the timestamp from when the image was captured
            # This is essential for moving robots to get the correct transformation
            # If we still don't have a valid timestamp, warn and use latest available
            if transform_time.to_sec() == 0:
                rospy.logwarn("No valid timestamp for transformation. Using latest available transform.")
                transform_time = rospy.Time(0)  # Uses latest available transform
            else:
                rospy.loginfo(f"Using transform at time: {transform_time.to_sec()}")
            
            # Check if the transform is likely to be in the buffer
            now = rospy.Time.now()
            time_diff = (now - transform_time).to_sec()
            
            if time_diff > self.tf_cache_time.to_sec() * 0.8:  # If older than 80% of buffer capacity
                rospy.logwarn(f"Transform timestamp ({time_diff:.2f}s ago) is close to or beyond the buffer limit ({self.tf_cache_time.to_sec():.1f}s).")
                rospy.logwarn("The requested transform may not be in the buffer. Consider increasing buffer size.")
            
            # Check if the timestamp is older than our oldest known transform
            if transform_time < self.oldest_transform_time:
                rospy.logwarn(f"Requested transform time ({transform_time.to_sec()}) is older than the oldest transform in the buffer ({self.oldest_transform_time.to_sec()}).")
                transform_time = self.oldest_transform_time
            
            # Create a copy of the pose_stamped to avoid modifying the original
            pose_copy = PoseStamped()
            pose_copy.header = pose_stamped.header
            pose_copy.pose = pose_stamped.pose
            
            # Use TF2 to transform the pose to base frame at the specific timestamp
            # This utilizes the full TF tree with all intermediate links
            transformed_pose = self.tf_buffer.transform(
                pose_copy,
                self.robot_base_frame,
                transform_time,
                rospy.Duration(0.5)
            )
            
            rospy.logdebug(f"Transformed pose using timestamp: {transform_time.to_sec()}")
            
            return transformed_pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to transform pose to base frame: {e}")
            return None
    
    def create_object_markers(self):
        marker_array = MarkerArray()
        
        current_time = rospy.Time.now()
        timeout = rospy.Duration(5.0)  # Remove objects not seen for 5 seconds
        
        # Clean up old objects
        to_delete = []
        for obj_id, obj_data in self.objects.items():
            if current_time - obj_data['last_seen'] > timeout:
                to_delete.append(obj_id)
        
        for obj_id in to_delete:
            del self.objects[obj_id]
        
        # Create markers for current objects
        for i, (obj_id, obj_data) in enumerate(self.objects.items()):
            # Add a marker for the object in base frame
            marker = Marker()
            marker.header = obj_data['pose_base'].header
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set the pose
            marker.pose = obj_data['pose_base'].pose
            
            # Set scale - adjust based on your objects
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            # Set color based on object type
            if obj_data['type'] == "box":
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif obj_data['type'] == "cylinder":
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            
            marker.color.a = 0.8  # Opacity
            
            # Set lifetime
            marker.lifetime = rospy.Duration(1.0)
            
            marker_array.markers.append(marker)
            
            # Add a text marker with the object ID
            text_marker = Marker()
            text_marker.header = obj_data['pose_base'].header
            text_marker.ns = "object_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Position the text above the object
            text_marker.pose = obj_data['pose_base'].pose
            text_marker.pose.position.z += 0.1
            
            # Set text properties
            text_marker.text = f"ID: {obj_id}, Type: {obj_data['type']}"
            text_marker.scale.z = 0.03  # Text height
            
            # Set color (white)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.lifetime = rospy.Duration(1.0)
            
            marker_array.markers.append(text_marker)
        
        return marker_array

if __name__ == '__main__':
    try:
        # Initialize the node
        node = UR5eCameraTransformation()
        
        # Wait for TF tree to be populated
        rospy.sleep(2.0)
               
        rospy.loginfo("UR5e Camera Transformation node is running. Press Ctrl+C to terminate.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
