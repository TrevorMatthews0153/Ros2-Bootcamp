import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class SpiralPublisher(Node):
    def __init__(self):
        super().__init__('turtle_spiral_publisher')

        #Initialize linear and angular velocities
        self.linear_velocity = 0.5
        self.angular_velocity = 0.5

        #subscribe to odometry sensor and retrieve positional data from the sensor
        self.subscription = self.create_subscription(Odometry, '/robot_5/odom',self.odom_callback,10)

        self.current_position = None
        self.current_orientation = 0.0

        self.initial_position = None
        self.initial_orientation = 0.0

        #Create publisher to send velocity commands to cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/robot_5/cmd_vel',10)

        #Create timer to call update_velocity method
        self.timer = self.create_timer(0.1,self.publish_velocity)
    
    def odom_callback(self, odom_msg):
        #retrieve current x and y positions from the odometry
        self.current_position = odom_msg.pose.pose.position

        #retrieve the quaternion data for orientation from the odometry sensor
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        # calculate yaw of the current position
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_orientation = yaw

        if self.initial_position is None:
            self.initial_position = self.current_position
            self.initial_orientation = self.current_orientation
            self.get_logger().info(f"Initial Position: {self.initial_position}, Orientation:{self.current_orientation}")

    def publish_velocity(self):
        if self.current_position is None:
            self.get_logger().warn('Odometry data not yet recieved, waiting...')
            return

        velocity_msg = Twist()
        velocity_msg.linear.x = self.linear_velocity
        velocity_msg.angular.z = self.angular_velocity

        self.publisher_.publish(velocity_msg)
        self.get_logger().info(f"Publishing: Linear ={velocity_msg.linear.x}, Angular ={velocity_msg.angular.z}")


def main(args=None):
    rclpy.init(args=args)

    spiral_publisher = SpiralPublisher()

    rclpy.spin(spiral_publisher)

    spiral_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
