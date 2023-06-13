import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class ImageRepublisherNode(Node):
    def __init__(self):
        super().__init__("image_republisher_node")
        self.subscription = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            CompressedImage,
            "/sign_detection/compressed",
            10
        )
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        # Convert the image message to OpenCV format
        cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Process the image here (optional)

        # Republish the image on the output topic
        output_msg = self.cv_bridge.cv2_to_compressed_imgmsg(cv_image)
        self.publisher.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    image_republisher_node = ImageRepublisherNode()
    rclpy.spin(image_republisher_node)

    image_republisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()