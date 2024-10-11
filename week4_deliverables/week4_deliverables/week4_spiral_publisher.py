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
        self.linear_velocity = 0.01
        self.angular_velocity = 0.2
        self.radius_increment = 0.001

        #subscribe to odometry sensor and retrieve positional data from the sensor
        self.subscription = self.create_subscription(Odometry, '/odom',self.odom_callback,10)
        self.current_position = {'x': 0.0,'y': 0.0, 'theta': 0.0}

        #Create publisher to send velocity commands to cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel',10)

        #Create timer to call update_velocity method
        self.timer = self.create_timer(0.1,self.update_velocity)
    
    def odom_callback(self, odom_msg):
        #retrieve current x and y positions from the odometry
        self.current_position['x'] = odom_msg.pose.pose.position.x
        self.current_position['y'] = odom_msg.pose.pose.position.y

        #retrieve the quaternion data for orientation from the odometry sensor
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        # calculate and save the yaw of the current position
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_position['theta'] = yaw

    def update_velocity(self):
        #Check to see if the current position of the turtle is within the boundaries of the field. 
        #If so, have the turtle move in a spiral pattern with specified velocities
        #If not, stop the turtle
        if abs(self.current_position['x']) > 10.5 or abs(self.current_position['y']) > 10.5:
            # self.get_logger().info('Limit reached! Stopping the turtle.')

            velocity_msg = Twist()

            velocity_msg.linear.x = 0.0

            velocity_msg.angular.z = 0.0

            self.publisher_.publish(velocity_msg)

        else:
            #Define a velocity message as the Twist data type
            velocity_msg = Twist()

            # Define the linear and angular velocity message as the initialized velocity
            velocity_msg.linear.x = self.linear_velocity
            velocity_msg.angular.z = self.angular_velocity

            self.publisher_.publish(velocity_msg)

            #Increase the linear velocity to move out in the spiral
            self.linear_velocity += self.radius_increment


def main(args=None):
    rclpy.init(args=args)

    spiral_publisher = SpiralPublisher()

    rclpy.spin(spiral_publisher)

    spiral_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
