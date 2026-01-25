import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
from std_msgs.msg import Int32


class unity(Node):
    def __init__(self):
        super().__init__('unity_publisher')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.led_pub = self.create_publisher(Int32, '/rover/led_status', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.led_timer = 0

        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_received = False

        self.timer = self.create_timer(0.0333, self.control_command)
        msg = Int32()
        msg.data = 0
        self.led_pub.publish(msg)


        self.get_logger().info("Unity Publisher Started.")

    def odom_callback(self, msg):
        
        
        
        

        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y


        orientation = msg.pose.pose.orientation
        self.curr_orient_x = orientation.x
        self.curr_orient_y = orientation.y
        self.curr_orient_z = orientation.z
        self.curr_orient_w = orientation.w

        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion([
            self.curr_orient_x,
            self.curr_orient_y,
            self.curr_orient_z,
            self.curr_orient_w
        ])


        self.curr_yaw = yaw
        # Broadcast TF
        t = TransformStamped()

        # Give the transform a timestamp
        t.header.stamp = self.get_clock().now().to_msg()
        
        # Define the Frame Names
        t.header.frame_id = 'odom'       # The fixed world frame
        t.child_frame_id = 'base_link'   # Your robot's frame

        # Copy Position
        t.transform.translation.x = self.curr_x
        t.transform.translation.y = self.curr_y
        t.transform.translation.z = 0.0  # Assuming rover stays on ground

        # Copy Rotation (Use the raw quaternion from the message)
        t.transform.rotation.x = orientation.x
        t.transform.rotation.y = orientation.y
        t.transform.rotation.z = orientation.z
        t.transform.rotation.w = orientation.w

        # Send it!
        self.tf_broadcaster.sendTransform(t)


        # self.get_logger().info(f"Current Pose: X={self.curr_x}, Y={self.curr_y}")


    def goal_callback(self, msg):

        
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        self.goal_received = True



        self.get_logger().info(f"Received Goal Pose: X={self.goal_x}, Y={self.goal_y}")

    def control_command(self):

        msg  = Int32()

        if not self.goal_received:
            if self.led_timer > 0:
                # BLINKING LOGIC:
                # We check the remainder of the timer divided by 10.
                # This makes it blink roughly every 0.25 seconds.
                # Change "10" to a higher number to blink slower.
                if (self.led_timer % 10) > 5:
                    msg.data = 2  # Green (ON)
                else:
                    msg.data = 0  # Grey (OFF)
                
                self.led_timer -= 1  # Count down
            else:
                msg.data = 0  # Idle (Grey)
            
            self.led_pub.publish(msg)
            return

        self.x = self.goal_x - self.curr_x
        self.y = self.goal_y - self.curr_y

        distance = math.sqrt(self.x**2 + self.y**2)

        target_heading = math.atan2(self.y, self.x)

        heading_error = target_heading - self.curr_yaw

        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        cmd = Twist()

        turn_speed = 5.0 

        if distance > 0.3:
            
            msg.data = 1
            self.led_pub.publish(msg)
            
            if heading_error > 0.1:
               
                cmd.angular.z = turn_speed
                cmd.linear.x = 0.0 

            elif heading_error < -0.1:
                
                cmd.angular.z = -turn_speed
                cmd.linear.x = 0.0 

            else:
                
                cmd.angular.z = 0.0 
                cmd.linear.x = 1.2 

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("✅✅✅✅✅✅ Reached Goal Position. Time to get the real rover running!!!!!!!")
            self.goal_received = False
            self.led_timer = 100
            msg.data = 2
            self.led_pub.publish(msg)

        self.vel_pub.publish(cmd)
        #self.get_logger().info(f"Published Velocity Command: Linear={cmd.linear.x}, Angular ={cmd.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = unity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()









    
    


