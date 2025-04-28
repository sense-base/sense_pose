import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import time
import cv2

import mediapipe as mp
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

class MediaPipePoseEstimator(Node):
    '''
        Node that processes webcam frames to detect key body locations.
        Subscribes to 'videostream' topic (raw webcam images)
        Publishes to 'pose_skeleton' topic (webcam images overlaid with detected landmarks/skeleton)
    '''

    def __init__(self):
        super().__init__('mpipe_pose_estimator')

        # Import mediapipe model and drawing utilities
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils

        self.base_options = python.BaseOptions(model_asset_path="src/mpipe/mediapipe_models/pose_landmarker_full.task", delegate=python.BaseOptions.Delegate.GPU) 
        self.options = vision.PoseLandmarkerOptions(base_options=self.base_options, output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(self.options)

        # Load mediapipe model
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1
                                 )

        self.subscription = self.create_subscription(
            Image,
            'videostream',
            self.listener_callback,
            qos_profile = qos_profile)

        self.publisher= self.create_publisher(Image, 'pose_skeleton', qos_profile = qos_profile)
        self.bridge = CvBridge()

        self.create_timer(1 / 30.0, self.publish_frame)
        self.latest_frame = None


    def listener_callback(self, data):

        self.latest_frame = data

    def publish_frame(self):

        if self.latest_frame:
        
            #start_time = time.time()

            # Convert image to CV2 
            cv_frame = self.bridge.imgmsg_to_cv2(self.latest_frame)

            # Recolor image to RGB
            image = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False            
 
            # Make detection
            mp_frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
            results1 = self.pose.process(image)
            results2 = self.detector.detect(mp_frame)

            self._logger.info(str(results1))
            self._logger.info((str(type(results2))))
            

            # Color back to BGR
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Extract landmarks
            #try:
            #    landmarks = results.pose_landmarks.landmark
            #except:
            #    pass


            # Render detections
            self.mp_drawing.draw_landmarks(image, results1.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

            # Calculate the delay
            """ end_time = time.time()
            delay = end_time - start_time
            self.get_logger().info(f'Processing time: {delay:.2f} seconds')
            """
            output_image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.publisher.publish(output_image_msg)
            #self.get_logger().info('Publishing pose estimation image')

            # Calculate the delay
            #end_time = time.time()
            #delay = end_time - start_time
            #self.get_logger().info(f'Processing time: {delay:.2f} seconds')


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipePoseEstimator()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()