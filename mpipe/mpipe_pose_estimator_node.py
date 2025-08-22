import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import Point32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from message_filters import ApproximateTimeSynchronizer, Subscriber

class MediaPipePoseEstimator(Node):
    '''
        Node that processes webcam frames to detect key body locations.
        Subscribes to 'videostream' topic (raw  zedcam images), 'depthstream' (depth zedcam images)
        Publishes to 'pose_skeleton' topic (webcam images overlaid with detected landmarks/skeleton),
        'pose_3d_coordinates' topic (Point3D point cloud with 33 points - see list of landmarks below)

        0 - nose
        1 - left eye (inner)
        2 - left eye
        3 - left eye (outer)
        4 - right eye (inner)
        5 - right eye
        6 - right eye (outer)
        7 - left ear
        8 - right ear
        9 - mouth (left)
        10 - mouth (right)
        11 - left shoulder
        12 - right shoulder
        13 - left elbow
        14 - right elbow
        15 - left wrist
        16 - right wrist
        17 - left pinky
        18 - right pinky
        19 - left index
        20 - right index
        21 - left thumb
        22 - right thumb
        23 - left hip
        24 - right hip
        25 - left knee
        26 - right knee
        27 - left ankle
        28 - right ankle
        29 - left heel
        30 - right heel
        31 - left foot index
        32 - right foot index
    '''

    def __init__(self):
        super().__init__('mpipe_pose_estimator')

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1
                                 )
        
        # QoS profile for point cloud publisher
        qos_profile_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # Increased depth to handle potential bursts of points
        )

        self.image_subscription = Subscriber(self, Image, 'videostream', qos_profile=qos_profile)
        self.depth_subscription = Subscriber(self, Image, 'depthstream', qos_profile=qos_profile)
        self.camera_info_subsubscription = Subscriber(self, CameraInfo, 'zed_camera_info', qos_profile=qos_profile)
        self.ts = ApproximateTimeSynchronizer(
            [self.image_subscription, self.depth_subscription, self.camera_info_subsubscription], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        self.image_publisher = self.create_publisher(Image, 'pose_skeleton', qos_profile=qos_profile)
        self.points_publisher = self.create_publisher(PointCloud, 'pose_3d_coordinates', qos_profile=qos_profile_reliable)
        self.bridge = CvBridge()

        # Import mediapipe model and drawing utilities
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.base_options = python.BaseOptions(model_asset_path="/ros2_ws/src/sense_pose/mediapipe_models/pose_landmarker_full.task", delegate=python.BaseOptions.Delegate.GPU) 
        self.options = vision.PoseLandmarkerOptions(base_options=self.base_options, output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(self.options)

        # Load mediapipe model
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0

    def synced_callback(self, image_msg, depth_msg, camera_info_msg):
        # Update intrinsics
        self.fx = camera_info_msg.k[0]
        self.fy = camera_info_msg.k[4]
        self.cx = camera_info_msg.k[2]
        self.cy = camera_info_msg.k[5]
        if any(v == 0.0 for v in [self.fx, self.fy, self.cx, self.cy]):
            self.get_logger().warn("Skipping processing: Invalid camera intrinsics")
            return

        try:
            cv_frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            depth_data = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
            return

        # Recolor image to RGB
        image = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False            
 
        # Make detection
        results = self.pose.process(image)

        # Color back to BGR
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Create PointCloud message for 3D coordinates
        point_cloud_msg = PointCloud()
        point_cloud_msg.header = image_msg.header
        point_cloud_msg.header.frame_id = camera_info_msg.header.frame_id
        # Initialize 33 points with default values (0.0, 0.0, 0.0)
        point_cloud_msg.points = [Point32(x=0.0, y=0.0, z=0.0) for _ in range(33)]
        
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            frame_h, frame_w = cv_frame.shape[:2]
            for idx, landmark in enumerate(landmarks):
                x, y = landmark.x, landmark.y
                pixel_x = int(x * frame_w)
                pixel_y = int(y * frame_h)
                if 0 <= pixel_x < frame_w and 0 <= pixel_y < frame_h:
                    depth_value = float(depth_data[pixel_y, pixel_x])
                    if np.isfinite(depth_value):
                        Z = depth_value
                        X = (pixel_x - self.cx) * Z / self.fx
                        Y = (pixel_y - self.cy) * Z / self.fy
                        # Add 3D point to PointCloud
                        point_cloud_msg.points[idx].x = X
                        point_cloud_msg.points[idx].y = Y
                        point_cloud_msg.points[idx].z = Z
    
        # Publish the PointCloud message
        self.points_publisher.publish(point_cloud_msg)

        # Publish the zed camera image with landmarks drawn over it
        self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
        try:
            output_image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            output_image_msg.header = image_msg.header
            self.image_publisher.publish(output_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipePoseEstimator()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()