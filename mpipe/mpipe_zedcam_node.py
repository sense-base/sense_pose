import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import pyzed.sl as sl
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ZedcamImagePublisher(Node):
    # Node that publishes raw zed camera images to the 'videostream' topic and the depth information to the 'depthstream' topic

    def __init__(self):
        super().__init__('zedcam_image_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.image_publisher_ = self.create_publisher(Image, 'videostream', qos_profile=qos_profile)
        self.depth_publisher_ = self.create_publisher(Image, 'depthstream', qos_profile=qos_profile)

        self.bridge = CvBridge()

        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"ZED camera failed to open: {status}")
            raise RuntimeError(f"ZED camera error: {status}")
        
        # Create runtime parameters and Mat to hold the image
        self.runtime_params = sl.RuntimeParameters()
        self.zed_image = sl.Mat()
        self.zed_depth = sl.Mat()

        self.create_timer(1.0 / 30.0, self.publish_frame)


    def publish_frame(self):
        #self.get_logger().info('DEBUGGER 5')
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Retrieve the image from left camera
            self.zed.retrieve_image(self.zed_image, sl.VIEW.LEFT)
            frame = self.zed_image.get_data()  # Get image as numpy array
            
            # Convert from BGRA to BGR if needed
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_publisher_.publish(img_msg)

                # Retrieve depth map
                self.zed.retrieve_measure(self.zed_depth, sl.MEASURE.DEPTH)
                depth_data = self.zed_depth.get_data()
                # Convert depth (float32, meters) to ROS Image message
                depth_msg = self.bridge.cv2_to_imgmsg(depth_data, encoding='32FC1')
                depth_msg.header.stamp = img_msg.header.stamp
                depth_msg.header.frame_id = 'zed_left_camera'
                self.depth_publisher_.publish(depth_msg)

            except CvBridgeError as error:
                 self.get_logger().error(f"CvBridgeError: {error}")
            

def main(args=None):
    rclpy.init(args=args)
    node = ZedcamImagePublisher()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
