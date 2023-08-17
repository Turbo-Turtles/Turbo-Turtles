# import packages
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import pynput

# load cascades
cascade_construction = cv2.CascadeClassifier('/home/marvin/ros2_ws/Turbo-Turtles/sign_detection/signs/11/construction.xml')
cascade_crossing = cv2.CascadeClassifier('/home/marvin/ros2_ws/Turbo-Turtles/sign_detection/signs/crossing_sign2.xml')
cascade_left = cv2.CascadeClassifier('/home/marvin/ros2_ws/Turbo-Turtles/sign_detection/signs/left.xml')
cascade_parking = cv2.CascadeClassifier('/home/marvin/ros2_ws/Turbo-Turtles/sign_detection/signs/parking.xml')
cascade_right = cv2.CascadeClassifier('/home/marvin/ros2_ws/Turbo-Turtles/sign_detection/signs/right.xml')
cascade_stop = cv2.CascadeClassifier('/home/marvin/ros2_ws/Turbo-Turtles/sign_detection/signs/stop.xml')
cascade_train = cv2.CascadeClassifier('/home/marvin/ros2_ws/Turbo-Turtles/sign_detection/signs/train.xml')
cascade_tunnel = cv2.CascadeClassifier('/home/marvin/ros2_ws/Turbo-Turtles/sign_detection/signs/tunnel.xml')



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
        
        self.cv_bridge = CvBridge()

    def publish_sign(self, key):
        self.publisher_sign.publish(key)

    def image_callback(self, msg):
        # Convert the image message to OpenCV format
        cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Process the image here
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        sign_text = cv_image
        output_sign = String()
        
        construction_sign_scaled = cascade_construction.detectMultiScale(image=gray, scaleFactor=1.3, minNeighbors=5)
        crossing_sign_scaled = cascade_crossing.detectMultiScale(image=gray, scaleFactor=1.3, minNeighbors=5)
        left_sign_scaled = cascade_left.detectMultiScale(image=gray, scaleFactor=1.3, minNeighbors=5)
        parking_sign_scaled = cascade_parking.detectMultiScale(image=gray, scaleFactor=1.3, minNeighbors=5)
        right_sign_scaled = cascade_right.detectMultiScale(image=gray, scaleFactor=1.3, minNeighbors=5)
        stop_sign_scaled = cascade_stop.detectMultiScale(image=gray, scaleFactor=1.3, minNeighbors=5)
        train_sign_scaled = cascade_train.detectMultiScale(image=gray, scaleFactor=1.3, minNeighbors=5)
        tunnel_sign_scaled = cascade_tunnel.detectMultiScale(image=gray, scaleFactor=1.3, minNeighbors=5)


        
        for (x, y, w, h) in construction_sign_scaled:
            construction_sign_rectangle = cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
            sign_text = cv2.putText(construction_sign_rectangle,"Construction Sign",(x, y+h+30),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2,cv2.LINE_4)
            output_sign.data = "construction"
            
        for (x, y, w, h) in crossing_sign_scaled:
            crossing_sign_rectangle = cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
            sign_text = cv2.putText(crossing_sign_rectangle,"Crossing Sign",(x, y+h+30),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2,cv2.LINE_4)
            output_sign.data = "crossing"
            
        for (x, y, w, h) in left_sign_scaled:
            left_sign_rectangle = cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
            sign_text = cv2.putText(left_sign_rectangle,"Left Sign",(x, y+h+30),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2,cv2.LINE_4)
            output_sign.data = "left"
            
        for (x, y, w, h) in parking_sign_scaled:
            parking_sign_rectangle = cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
            sign_text = cv2.putText(parking_sign_rectangle,"Parking Sign",(x, y+h+30),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2,cv2.LINE_4)
            output_sign.data = "parking"
            
        for (x, y, w, h) in right_sign_scaled:
            right_sign_rectangle = cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
            sign_text = cv2.putText(right_sign_rectangle,"Right Sign",(x, y+h+30),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2,cv2.LINE_4)
            output_sign.data = "right"
            
        for (x, y, w, h) in stop_sign_scaled:
            stop_sign_rectangle = cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
            sign_text = cv2.putText(stop_sign_rectangle,"Stop Sign",(x, y+h+30),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2,cv2.LINE_4)
            output_sign.data = "stop"
            
        for (x, y, w, h) in train_sign_scaled:
            train_sign_rectangle = cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
            sign_text = cv2.putText(train_sign_rectangle,"Train Sign",(x, y+h+30),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2,cv2.LINE_4)
            output_sign.data = "train"
            
        for (x, y, w, h) in tunnel_sign_scaled:
            tunnel_sign_rectangle = cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
            sign_text = cv2.putText(tunnel_sign_rectangle,"Tunnel Sign",(x, y+h+30),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2,cv2.LINE_4)
            output_sign.data = "tunnel"
            
        

        # Republish the image on the output topic
        output_msg = self.cv_bridge.cv2_to_compressed_imgmsg(sign_text)
        self.publisher_image.publish(output_msg)
        # self.publisher_sign.publish(output_sign)



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
    taste = String()
    taste.data = sign(str(key))

    sign_detection_node.publish_sign(taste)
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
    )