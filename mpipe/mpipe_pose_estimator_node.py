import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
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
        Subscribes to 'videostream' topic (raw webcam images)
        Publishes to 'pose_skeleton' topic (webcam images overlaid with detected landmarks/skeleton)
    '''

    def __init__(self):
        super().__init__('mpipe_pose_estimator')

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1
                                 )

        self.image_subscription = Subscriber(self, Image, 'videostream', qos_profile=qos_profile)
        self.depth_subscription = Subscriber(self, Image, 'depthstream', qos_profile=qos_profile)
        self.camera_info_subsubscription = Subscriber(self, CameraInfo, 'zed_camera_info', qos_profile=qos_profile)
        self.ts = ApproximateTimeSynchronizer(
            [self.image_subscription, self.depth_subscription, self.camera_info_subsubscription], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        self.publisher= self.create_publisher(Image, 'pose_skeleton', qos_profile = qos_profile)
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
        
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            frame_h, frame_w = cv_frame.shape[:2]
            for idx, landmark in enumerate(landmarks):
                x, y = landmark.x, landmark.y
                pixel_x = int(x * frame_w)
                pixel_y = int(y * frame_h)
                if 0 <= pixel_x < frame_w and 0 <= pixel_y < frame_h:
                    depth_value = depth_data[pixel_y, pixel_x]
                    if np.isfinite(depth_value):
                        Z = depth_value
                        X = (pixel_x - self.cx) * Z / self.fx
                        Y = (pixel_y - self.cy) * Z / self.fy
                        if idx == 13:
                            self.get_logger().info(f'Left elbow: 3D (X, Y, Z) = ({X:.3f}, {Y:.3f}, {Z:.3f}) m')
                    elif idx == 13:
                        self.get_logger().warn(f'Left elbow: Invalid depth at ({pixel_x}, {pixel_y})')
                elif idx == 13:
                    self.get_logger().warn(f'Left elbow:Out of bounds at ({pixel_x}, {pixel_y})')
    

        self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
        try:
            output_image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            output_image_msg.header = image_msg.header
            self.publisher.publish(output_image_msg)
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