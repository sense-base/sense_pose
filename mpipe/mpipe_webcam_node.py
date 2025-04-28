import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__("mpipe_webcam")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.image_publisher = self.create_publisher(Image, "/videostream", qos_profile=qos_profile)

        self._cap = cv2.VideoCapture(0)
        self._cvbridge = CvBridge()

        self.create_timer(1 / 30.0, self.publish_frame)


    def publish_frame(self):
        ret, frame = self._cap.read()
        if ret:
            img_msg = self._cvbridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.frame_id = "camera"
            self.image_publisher.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
