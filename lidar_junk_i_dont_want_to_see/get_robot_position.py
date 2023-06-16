import rclpy
from rclpy.node import Node

from functools import partial

from geometry_msgs.msg import PoseStamped
from turtlebot3_interfaces.srv import GetPosition

class FrameListener(Node):
    def __init__(self):
        super().__init__("robot_pos")

        self.location = PoseStamped()

        # Call on_timer function every second
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.get_location()

        print(self.location)

    # service client, calls position listener
    def get_location(self):
        client = self.create_client(GetPosition, 'current_position')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        future = client.call_async(GetPosition.Request())
        # waits for response from service and the passes the received data on to a callback function
        future.add_done_callback(partial(self.location_callback))

    # callback function, to work with the received data from the service
    def location_callback(self, future):
        try:
            response = future.result()
            self.location.pose = response.position.pose
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))


def main():
    rclpy.init()
    node = FrameListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()