# boundary_mapper.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

class BoundaryMapper(Node):
    def __init__(self):
        super().__init__('boundary_mapper')
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'boundary_map', 10)

        # Occupancy Grid Setup
        self.grid_size = 100  # 100x100 grid
        self.resolution = 0.1  # 10 cm per grid cell
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

        # Initializing OccupancyGrid message
        self.map = OccupancyGrid()
        self.map.header.frame_id = "map"
        self.map.info.resolution = self.resolution
        self.map.info.width = self.grid_size
        self.map.info.height = self.grid_size
        self.map.info.origin.position.x = -self.grid_size * self.resolution / 2
        self.map.info.origin.position.y = -self.grid_size * self.resolution / 2

    def lidar_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i, distance in enumerate(msg.ranges):
            if distance > msg.range_min and distance < msg.range_max:
                angle = angle_min + i * angle_increment
                x = int((distance * math.cos(angle)) / self.resolution) + self.grid_size // 2
                y = int((distance * math.sin(angle)) / self.resolution) + self.grid_size // 2
                if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                    self.grid[x, y] = 100  # Marking as occupied

        self.map.data = self.grid.flatten().tolist()
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map)

def main(args=None):
    print("Starting boundary mapper node...")
    rclpy.init(args=args)
    node = BoundaryMapper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
