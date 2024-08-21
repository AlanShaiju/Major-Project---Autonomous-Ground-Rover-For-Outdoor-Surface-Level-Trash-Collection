# obstacle_avoidance.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.safe_distance = 0.5  # Minimum safe distance from obstacles in meters

    def lidar_callback(self, msg):
        min_distance = min(msg.ranges)
        twist = Twist()

        if min_distance < self.safe_distance:
            twist.linear.x = -0.2  # Move backward
            twist.angular.z = 0.5  # Turn away
            self.get_logger().info("Obstacle detected! Avoiding...")
        else:
            twist.linear.x = 0.5  # Move forward
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    print("Starting obstacle avoidance node...")
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()