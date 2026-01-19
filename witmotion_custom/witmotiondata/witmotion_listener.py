#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
import math

def euler_from_quaternion(x, y, z, w):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python -> 
    from tf_transformations import euler_from_quaternion
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

class WitmotionListener(Node):
    def __init__(self):
        super().__init__('witmotion_listener')

        # Subscribe to IMU topic
        self.subscription = self.create_subscription(
            Quaternion, 
            "/orientation", 
            self.imu_callback, 
            10
        )

        self.get_logger().info("Listener Started. Rotate sensor to check North...")

    def imu_callback(self, msg):
        # 1. Get Quaternion
        x = msg.x
        y = msg.y
        z = msg.z
        w = msg.w

        # 2. Convert to Euler (Roll, Pitch, Yaw) using the helper function
        (roll, pitch, yaw) = euler_from_quaternion(x, y, z, w)

        # DEBUG: Print raw values
        # self.get_logger().info(f"roll: {math.degrees(roll):.1f}°")
        # self.get_logger().info(f"pitch: {math.degrees(pitch):.1f}°")
        # self.get_logger().info(f"yaw: {math.degrees(yaw):.1f}°")

        # 3. Convert Yaw to Degrees (-180 to +180)
        # FIX: Changed 'roll' to 'yaw' here because you want heading
        yaw_deg = math.degrees(roll) 

        # 4. Convert to Compass Heading (0-360 degrees)
        # Normalizes -180/180 to 0-360 range
        head = yaw_deg+180

        compass_heading = (head) % 360
        

        self.get_logger().info(
            f"Heading: {compass_heading:.1f}° (Raw Yaw: {yaw_deg:.1f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = WitmotionListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()