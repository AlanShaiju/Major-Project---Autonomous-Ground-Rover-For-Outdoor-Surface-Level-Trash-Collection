# sweep_cleaner.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String
import numpy as np

class SweepCleaner(Node):
    def __init__(self):
        super().__init__('sweep_cleaner')
        self.map_sub = self.create_subscription(OccupancyGrid, 'boundary_map', self.map_callback, 10)
        self.position_sub = self.create_subscription(Pose, 'robot_position', self.position_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cleaning_state_pub = self.create_publisher(String, 'cleaning_state', 10)
        
        self.map = None
        self.position = None
        self.cleaned_areas = np.zeros((100, 100), dtype=bool)  # Track cleaned areas
        self.cleaning_complete = False

    def map_callback(self, msg):
        self.map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.get_logger().info("Received map, beginning cleaning process")

    def position_callback(self, msg):
        self.position = msg
        if self.map is None:
            return
        
        if not self.cleaning_complete:
            self.sweep_cleaning()

    def sweep_cleaning(self):
        if self.position is None:
            return

        x = int((self.position.position.x - self.map.info.origin.position.x) / self.map.info.resolution)
        y = int((self.position.position.y - self.map.info.origin.position.y) / self.map.info.resolution)

        if 0 <= x < self.cleaned_areas.shape[0] and 0 <= y < self.cleaned_areas.shape[1]:
            if not self.cleaned_areas[x, y]:
                self.cleaned_areas[x, y] = True
                self.cleaning_state_pub.publish(String(data="Cleaning..."))

                twist = Twist()
                twist.linear.x = 0.5  # Moving forward
                twist.angular.z = 0.1  # Slight turn for coverage
                self.cmd_vel_pub.publish(twist)
            else:
                self.cleaning_state_pub.publish(String(data="Area already cleaned, moving to next area"))
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.5  # Turn in place to find uncleaned area
                self.cmd_vel_pub.publish(twist)
        
        if np.all(self.cleaned_areas):
            self.cleaning_state_pub.publish(String(data="Cleaning complete"))
            self.cleaning_complete = True
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0  # Stop
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    print("Starting Sweep Cleaner...")
    rclpy.init(args=args)
    node = SweepCleaner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
