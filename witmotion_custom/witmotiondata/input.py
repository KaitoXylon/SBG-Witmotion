import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import sys

class Input(Node):
    def __init__(self):
        super().__init__('input_node')
        self.publisher_ = self.create_publisher(Vector3, '/user_command', 10)
        self.timer = self.create_timer(0.1, self.get_user_input)
        self.get_logger().info("--- GPS WAYPOINT INPUT STARTED ---")

    def get_user_input(self):
        self.timer.cancel()
        try:
            print("\n" + "="*30)
            print(" ENTER TARGET GPS COORDINATES")
            print("="*30)
            lat_in = input("Target Latitude: ")
            lon_in = input("Target Longitude: ")

            msg = Vector3()
            msg.x = float(lat_in)  # X = Latitude
            msg.y = float(lon_in)  # Y = Longitude
            msg.z = 0.0

            self.publisher_.publish(msg)
            self.get_logger().info(f"Target Sent: {msg.x}, {msg.y}")
            
        except ValueError:
            self.get_logger().error("Invalid Number! Try again.")
        except KeyboardInterrupt:
            sys.exit(0)
            
        self.timer.reset()

def main(args=None):
    rclpy.init(args=args)
    node = Input()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()