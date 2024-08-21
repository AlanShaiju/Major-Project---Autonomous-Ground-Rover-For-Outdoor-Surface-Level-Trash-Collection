# localization.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid


class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'boundary_map', self.map_callback, 10)
        self.position_pub = self.create_publisher(Pose, 'robot_position', 10)

        self.map = None
        self.position = Pose()

    def map_callback(self, msg):
        self.map = msg

    def odom_callback(self, msg):
        if self.map is None:
            return

        # Localization logic: for now, just simulating simple odometry-based localization
        self.position.position.x = msg.pose.pose.position.x
        self.position.position.y = msg.pose.pose.position.y
        self.position.orientation = msg.pose.pose.orientation

        self.position_pub.publish(self.position)


def main(args=None):
    print("Starting localization node...")
    rclpy.init(args=args)
    node = Localization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
