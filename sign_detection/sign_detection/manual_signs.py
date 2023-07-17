# import packages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import pynput


# create node class
class SignDetection(Node):
    def __init__(self):
        super().__init__("sign_detection")
        self.subscription = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.image_callback,
            10
        )
        self.publisher_image = self.create_publisher(
            CompressedImage,
            "/sign_detection/compressed",
            10
        )
        self.publisher_sign = self.create_publisher(
            String,
            "/sign_detection/sign",
            10
        )
        

    def publish_sign(self, key):
        self.publisher_sign.publish(key)

    def image_callback(self, msg):
        pass
    
def main(args=None):
    legend()
    

    rclpy.init(args=None)
    global sign_detection_node
    sign_detection_node = SignDetection()

    with pynput.keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

        
    sign_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

def on_press(key):
    pressed_key = String()
    pressed_key.data = sign(str(key))

    sign_detection_node.publish_sign(pressed_key)
    rclpy.spin_once(sign_detection_node)
    

def on_release(key):
    rclpy.spin_once(sign_detection_node)
    
def sign(key):
    match key:
        case "'1'":
            return "stop"
        case "'2'":
            return "train"
        case "'3'":
            return "crossing"
        case "'4'":
            return "construction"
        case "'5'":
            return "parking"
        case "'6'":
            return "tunnel"
        case "'l'":
            return "left"
        case "'r'":
            return "right"
        case "'7'":
            return "red"
        case "'8'":
            return "yellow"
        case "'9'":
            return "green"
        case "'o'":
            return "opened"
        case "'c'":
            return "closed" 
        case _:
            return "no assignment"
        
def legend():
    print(
        "1: Stop\n"
        "2: Train\n"
        "3: Crossing\n"
        "4: Construction\n"
        "5: Parking\n"
        "6: Tunnel\n"
        "L: Left\n"
        "R: Right\n"
        "7: Red Ligh\n"
        "8: Yellow Light\n"
        "9: Green Light\n"
        "O: Barrier opened\n"
        "C: Barrier closed\n"
    )
