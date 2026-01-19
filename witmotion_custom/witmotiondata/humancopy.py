import rclpy
from rclpy.node import Node
from sbg_driver.msg import SbgGpsPos
from geometry_msgs.msg import Quaternion , Vector3
import tf_transformations
import math



class Human(Node):
    def __init__(self):
        super().__init__('human')

        self.sbg_sub = self.create_subscription(SbgGpsPos,"sbg/gps_pos", self.sgg_callback,10)
        self.wit_sub = self.create_subscription(Quaternion, "/orientation", self.wit_callback,10)

        self.get_logger().info("Listening started, human rover is ready")

        self.cmd_sub = self.create_subscription(Vector3, "/user_command", self.cmd_callback, 10)


        self.ref_lat = None
        self.ref_lon = None
        self.has_ref = False

        self.target_distance = 0.0
        self.target_heading = 0.0
        self.has_new_command = False


    def cmd_callback(self, msg):
        # When user sends input, this function runs automatically
        self.target_lat= msg.x
        self.target_lon = msg.y
        self.has_new_command = True
        
        self.get_logger().info(f"NEW GOAL -> Lat: {self.target_lat}, Lon: {self.target_lon}")

    def sgg_callback(self,msg):
        if not self.has_new_command:
            return

        curr_lat = msg.latitude
        curr_lon = msg.longitude

        # 1. Calculate Distance TO THE TARGET (not from start)
        distance_left = self.heversine(curr_lat, curr_lon, self.target_lat, self.target_lon)
        
        # 2. Calculate Direction TO THE TARGET
        # We save this to 'self.target_heading' so the compass callback can use it
        self.target_heading = self.calculate_bearing(curr_lat, curr_lon, self.target_lat, self.target_lon)

        # 3. Print status
        if distance_left > 2.0:
             # Added throttle so it doesn't spam your terminal
             self.get_logger().info(f"Go {distance_left:.2f} m more (Heading needed: {self.target_heading:.0f}°)", throttle_duration_sec=1.0)
        else:
             self.get_logger().info("Goal reached gimme the real rover")



    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        # Convert to radians
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_l = math.radians(lon2 - lon1)

        # Formula to find angle
        y = math.sin(delta_l) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_l)
        
        theta = math.atan2(y, x)
        bearing = math.degrees(theta)
        
        # Normalize to 0-360
        return (bearing + 360) % 360


    def heversine(self, lat1 ,lon1, lat2, lon2):

        R = 6371000.0

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_p = math.radians(lat2-lat1)
        delta_l = math.radians(lon2-lon1)


        a = math.sin(delta_p / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_l/2.0)**2

        c = 2* math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R *c
    
    def wit_callback(self,msg):
        if not self.has_new_command:
            return
        
        x = msg.x
        y = msg.y
        z = msg.z
        w = msg.w

        (roll , pitch , yaw) = tf_transformations.euler_from_quaternion([x,y,z,w])

        yaw_deg = math.degrees(yaw)

        # self.get_logger().info(f"Heading: {yaw_deg:.2f}°")


        turn = self.shortest(self.target_heading , yaw_deg)

        if abs(turn) < 5.0: # Increased tolerance to 5 deg for human use
            self.get_logger().info("✅ ALIGNED! Walk Forward.", throttle_duration_sec=1.0)
                
        elif turn > 0:
            # FIX 4: Removed self.turn_left() (doesn't exist)
            # FIX 5: Changed 'turn_needed' to 'turn'
            self.get_logger().info(f"🔄 TURN RIGHT by {turn:.0f}°", throttle_duration_sec=1.0)
                
        else: 
            # FIX 6: Changed 'turn_needed' to 'turn'
            self.get_logger().info(f"🔄 TURN LEFT by {abs(turn):.0f}°", throttle_duration_sec=1.0)

    def shortest(self , target_heading , yaw_deg):
        error = target_heading - yaw_deg

        if error >180:
            error-=360
        elif error <-180:
            error +=360


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


        
    