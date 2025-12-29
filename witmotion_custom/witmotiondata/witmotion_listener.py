#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import tf_transformations
import math

class WitmotionListener(Node):
    def __init__(self):
        super().__init__('witmotion_listener')

        # Subscribe to IMU topic (Fused orientation)
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu', 
            self.imu_callback,
            10)

        # Subscribe to Magnetometer topic (Raw magnetic data)
        self.mag_sub = self.create_subscription(
            MagneticField,
            '/magnetometer',
            self.mag_callback,
            10)

        self.get_logger().info("Listener Started. Rotate sensor to check North...")

    def imu_callback(self, msg):
        # 1. Get Quaternion
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # 2. Convert to Euler (Roll, Pitch, Yaw)
        euler = tf_transformations.euler_from_quaternion([x, y, z, w])
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2] # This is in Radians (-pi to +pi)

        # 3. Convert Yaw to Degrees (-180 to +180)
        yaw_deg = math.degrees(yaw)

        # 4. Convert to Compass Heading (0-360 degrees from North)
        # ROS ENU standard: 0=East, 90=North. 
        # Compass standard: 0=North, 90=East.
        # This formula aligns them:
        # compass_heading = (yaw_deg) % 360
        correct = yaw_deg +360
        compass_heading = correct%360

        # NOTE: If your sensor mounts X-axis forward, '0' might be East. 
        # If the value is 90 degrees off, use this formula instead:
        # compass_heading = (450 - yaw_deg) % 360 

        self.get_logger().info(
            f"Heading: {compass_heading:.1f}° (Raw Yaw: {yaw_deg:.1f})"
        )

    def mag_callback(self, msg):
        # Calculate rough heading from raw magnetometer (Backup method)
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        
        # Standard atan2 for magnetic heading
        heading_rad = math.atan2(my, mx)
        heading_deg = math.degrees(heading_rad)
        
        # Normalize to 0-360
        if heading_deg < 0:
            heading_deg += 360
            
        # self.get_logger().info(f"Raw Mag Heading: {heading_deg:.1f}")

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