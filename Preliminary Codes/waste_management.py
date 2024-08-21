# waste_management.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class WasteManagement(Node):
    def __init__(self):
        super().__init__('waste_management')
        self.capacity_sensor_sub = self.create_subscription(Float32, 'capacity_sensor', self.sensor_callback, 10)
        self.dumping_state_pub = self.create_publisher(String, 'dumping_state', 10)
        
        self.waste_threshold = 80.0  # Threshold for triggering waste dump
        self.current_capacity = 0.0

    def sensor_callback(self, msg):
        self.current_capacity = msg.data
        if self.current_capacity >= self.waste_threshold:
            self.dump_waste()

    def dump_waste(self):
        self.dumping_state_pub.publish(String(data="Waste capacity reached. Dumping waste..."))
        # Insert logic to navigate to waste dump station and empty the container
        self.get_logger().info("Dumping waste at station...")
        # Once waste is dumped, reset the capacity
        self.current_capacity = 0.0

def main(args=None):
    print("Starting waste management node...")
    rclpy.init(args=args)
    node = WasteManagement()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
