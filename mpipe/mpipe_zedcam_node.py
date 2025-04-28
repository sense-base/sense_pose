import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import pyzed.sl as sl

class ZedcamImagePublisher(Node):
    # Node that publishes raw webcam images to the 'videostream' topic

    def __init__(self):
        super().__init__('zedcam_image_publisher')
        self.publisher_ = self.create_publisher(Image, 'videostream', 10)

        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"Error: {status}")
            exit()
        # Create runtime parameters and Mat to hold the image
        runtime_params = sl.RuntimeParameters()
        zed_image = sl.Mat()

        self.bridge = CvBridge()


        while True:
            # Capture image from ZED camera
            if self.zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Retrieve the image from left camera
                self.zed.retrieve_image(zed_image, sl.VIEW.LEFT)
                frame = zed_image.get_data()  # Get image as numpy array

            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(img_msg)

            except CvBridgeError as error:
                 print(error)
            

def main(args=None):
    rclpy.init(args=args)
    node = ZedcamImagePublisher()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
