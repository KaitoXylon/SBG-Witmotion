import rclpy
from rclpy.node import Node
from sbg_driver.msg import SbgGpsPos
from geometry_msgs.msg import Quaternion, Vector3
import tf_transformations
import math

class Human(Node):
    def __init__(self):
        super().__init__('human')

        self.sbg_sub = self.create_subscription(SbgGpsPos, "sbg/gps_pos", self.sgg_callback, 10)
        self.wit_sub = self.create_subscription(Quaternion, "/orientation", self.wit_callback, 10)
        self.cmd_sub = self.create_subscription(Vector3, "/user_command", self.cmd_callback, 10)

        self.get_logger().info("Listening started, human rover is ready")

        # State variables
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.target_heading = 0.0
        self.has_new_command = False
        
        # New variable to share distance between callbacks
        self.distance_left = None 

    def cmd_callback(self, msg):
        self.target_lat = msg.x
        self.target_lon = msg.y
        self.has_new_command = True
        self.distance_left = None # Reset distance on new command
        self.get_logger().info(f"NEW GOAL -> Lat: {self.target_lat}, Lon: {self.target_lon}")

    def sgg_callback(self, msg):
        if not self.has_new_command:
            return

        curr_lat = msg.latitude
        curr_lon = msg.longitude

        # 1. Calculate Distance and store it in self variable
        self.distance_left = self.heversine(curr_lat, curr_lon, self.target_lat, self.target_lon)
        
        # 2. Calculate Direction
        self.target_heading = self.calculate_bearing(curr_lat, curr_lon, self.target_lat, self.target_lon)

        # 3. Check for arrival (Priority over printing distance)
        if self.distance_left <= 2.0:
             self.get_logger().info("☑️☑️☑️☑️☑️☑️ Goal reached! Gimme the real rover")
             self.has_new_command = False # Stop processing until new command

    def wit_callback(self, msg):
        if not self.has_new_command or self.distance_left is None:
            return
        
        x = msg.x
        y = msg.y
        z = msg.z
        w = msg.w

        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion([x, y, z, w])
        yaw_deg = math.degrees(roll)

        # Calculate turn needed
        turn = self.shortest(self.target_heading, yaw_deg)

        # --- LOGIC CHANGED HERE ---
        
        # Increased tolerance slightly to 8 degrees to make it easier to trigger the distance message
        if abs(turn) < 8.0: 
            # ONLY show distance when aligned
            self.get_logger().info(f"✅ ALIGNED! Walk Forward. | Distance: {self.distance_left:.2f}m", throttle_duration_sec=1.0)
                
        elif turn > 0:
            self.get_logger().info(f"🔄 TURN RIGHT by {turn:.0f}°", throttle_duration_sec=1.0)
                
        else: 
            self.get_logger().info(f"🔄 TURN LEFT by {abs(turn):.0f}°", throttle_duration_sec=1.0)

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_l = math.radians(lon2 - lon1)

        y = math.sin(delta_l) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_l)
        
        theta = math.atan2(y, x)
        bearing = math.degrees(theta)
        return (bearing + 360) % 360

    def heversine(self, lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_p = math.radians(lat2 - lat1)
        delta_l = math.radians(lon2 - lon1)

        a = math.sin(delta_p / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_l / 2.0)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def shortest(self, target_heading, yaw_deg):
        error = target_heading - yaw_deg
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        return error

def main(args=None):
    rclpy.init(args=args)
    node = Human()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()