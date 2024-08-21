# master_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MasterController(Node):
    def __init__(self):
        super().__init__('master_controller')
        self.create_timer(1.0, self.check_systems)

        # Subscribers to monitor different systems
        self.cleaning_state_sub = self.create_subscription(String, 'cleaning_state', self.cleaning_state_callback, 10)
        self.dumping_state_sub = self.create_subscription(String, 'dumping_state', self.dumping_state_callback, 10)
        self.charging_state_sub = self.create_subscription(String, 'charging_state', self.charging_state_callback, 10)

        self.system_states = {
            'cleaning': 'idle',
            'dumping': 'idle',
            'charging': 'idle'
        }

    def check_systems(self):
        # Central logic to coordinate all tasks
        if self.system_states['cleaning'] == 'in_progress':
            self.get_logger().info("Currently cleaning the area")
        elif self.system_states['dumping'] == 'in_progress':
            self.get_logger().info("Dumping waste")
        elif self.system_states['charging'] == 'in_progress':
            self.get_logger().info("Charging at the dock")
        else:
            self.get_logger().info("System idle or task completed")

    def cleaning_state_callback(self, msg):
        self.system_states['cleaning'] = msg.data

    def dumping_state_callback(self, msg):
        self.system_states['dumping'] = msg.data

    def charging_state_callback(self, msg):
        self.system_states['charging'] = msg.data


def main(args=None):
    print("Starting master controller node...")
    rclpy.init(args=args)
    node = MasterController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()