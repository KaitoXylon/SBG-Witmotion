import rclpy
from rclpy.node import Node
from sbg_driver.msg import SbgEkfNav


class sbg(Node):
    def __init__(self):
        super().__init__('sbg_gg')

        self.sbg_sub = self.create_subscription(SbgEkfNav,"sbg/ekf_nav", self.sgg_callback,10)

        self.get_logger().info("Listening to sbg/ekf_nav")

    def sgg_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude


        self.get_logger().info(f"latitude : {lat}, Longitude : {lon} , Altitude : {alt}")



def main(args=None):
    rclpy.init(args=args)
    node = sbg()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()