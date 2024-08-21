# charging_management.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class ChargingManagement(Node):
    def __init__(self):
        super().__init__('charging_management')
        self.battery_sub = self.create_subscription(BatteryState, 'battery_state', self.battery_callback, 10)
        self.charging_state_pub = self.create_publisher(String, 'charging_state', 10)

        self.charge_threshold = 20.0  # Charge level to trigger return to dock
        self.current_charge = 100.0

    def battery_callback(self, msg):
        self.current_charge = msg.percentage
        if self.current_charge <= self.charge_threshold:
            self.return_to_charging_dock()

    def return_to_charging_dock(self):
        self.charging_state_pub.publish(String(data="Battery low. Returning to charging dock..."))
        # Insert logic to navigate to charging dock
        self.get_logger().info("Navigating to charging dock...")
        # Once at charging dock, charging state can be handled
        # After charging, reset charge state
        self.current_charge = 100.0


def main(args=None):
    print("Starting charging management node...")
    rclpy.init(args=args)
    node = ChargingManagement()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
