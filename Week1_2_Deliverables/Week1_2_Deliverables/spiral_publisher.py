import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class SpiralPublisher(Node):
    def __init__(self):
        super().__init__('turtle_spiral_publisher')

        #Initialize linear and angular velocities
        self.linear_velocity = 1.0
        self.angular_velocity = 2.0
        self.radius_increment = 0.05

        #Create publisher to send velocity commands to cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel',10)

        #Create subscriber to retrieve the current position of the turtle
        self.subscription = self.create_subscription(Pose,'turtle1/pose',self.pose_callback,10)

        self.current_position = Pose()

        #Create timer to call update_velocity method
        self.timer = self.create_timer(0.1,self.update_velocity)

    def pose_callback(self,pose_data):
        # store the positional data of the turtle in current_position
        self.current_position = pose_data

    def update_velocity(self):
        #Check to see if the current position of the turtle is within the boundaries of the field. 
        #If so, have the turtle move in a spiral pattern with specified velocities
        #If not, stop the turtle
        if abs(self.current_position.x) > 10.5 or abs(self.current_position.y) > 10.5:
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
