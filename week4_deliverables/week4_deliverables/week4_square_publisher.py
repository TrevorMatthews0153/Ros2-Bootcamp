import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
#from turtlesim.srv import TeleportAbsolute


class DefinedGoalPublisher(Node):

    def __init__(self):
        super().__init__('Defined_goal_state_machine')

        #Define the states for the state machine
        self.MOVING = 0
        self.TURNING = 1
        self.GOAL_REACHED = 2
        self.state = self.MOVING

        #Define the initial position of the turtle
        self.initial_position = None
        self.initial_orientation = None

        #Define the current position of the turtle
        self.current_position = None
        self.current_orientation = 0.0

        self.start_position = None
        self.start_orientation = 0.0

        self.move_distance = 2.0
        self.turn_angle = math.pi / 2

        #subscribe to topic '/odom' to get the current positional data of the turtle
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        #publish to topic 'turtle1/cmd_vel' a message of data type Twist
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        #timer to update turtle's state and movement
        self.timer = self.create_timer(0.1, self.update_state)

    def odom_callback(self, msg): 
        """call the current position and save it in pose_data"""
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_orientation = yaw

        if self.initial_position == None:
            self.initial_position = self.current_position
            self.initial_orientation = self.current_orientation
            self.get_logger().info(f"Initial Position: {self.initial_position}, Orientation:{self.current_position}")

        if self.start_position == None:
            self.start_position = self.current_position
            self.start_orientation = self.current_orientation

    
    def update_state(self):
        """Timer callback to update turtle's movement based on its state"""
        if self.current_position is None or self.start_position is None:
            self.get_logger().warn('Odometry data not yet recieved, waiting...')
            return

        if self.state == self.MOVING:
            self.move_forward()

        elif self.state == self.TURNING:
            self.turn_turtle()

        elif self.state == self.GOAL_REACHED:
            self.stop_turtle()

    
    def move_forward(self):
        """Move turtle forward by distance self.move_distance (in meters)."""

        if self.current_position is None or self.start_position is None:
            self.get_logger().warn('Waiting for odometry data in move_forward()...')
            return
        
        self.velocity_message = Twist()

        try:
            self.distance_moved = math.sqrt( (self.current_position.x - self.start_position.x)**2 + (self.current_position.y - self.start_position.y)**2)
            self.distance_from_initial = math.sqrt( (self.current_position.x - self.initial_position.x)**2 + (self.current_position.y - self.initial_position.y)**2)
        except AttributeError as e:
            self.get_logger().error(f'Error calculating distance: {e}')
            return
        

        if self.distance_moved < self.move_distance:
            self.velocity_message.linear.x = 0.15
            self.velocity_message.angular.z = 0.0

        elif self.distance_from_initial >= 0.95*math.sqrt(8): #define the stop condition as once the distance equals hypotenuse of triangle with sides 2m
            self.state = self.GOAL_REACHED
            self.velocity_message.linear.x = 0.0

        else:
            #Change the state once condition is reached. Set the velocity to 0 and define the start orientation of the next maneuver
            self.state = self.TURNING
            self.velocity_message.linear.x = 0.0
            self.start_orientation = self.current_orientation

        #Publish velocity command to move turtle
        self.cmd_vel_publisher.publish(self.velocity_message)
        self.get_logger().info(f"Current State: {self.state}")
        self.get_logger().info(f"Publishing velocity: linear.x={self.velocity_message.linear.x}, angular.z={self.velocity_message.angular.z}")
        self.get_logger().info(f"Current Position: X:{self.current_position.x},Y:{self.current_position.y}")

    def turn_turtle(self):
        """Turn the turtle 90 degrees"""
        if self.current_position is None or self.start_position is None:
            self.get_logger().warn('Waiting for odometry data in turn_turtle()...')
            return
        
        angle_turned = abs(self.current_orientation - self.start_orientation)

        if angle_turned < self.turn_angle:
            self.velocity_message.linear.x = 0.0
            self.velocity_message.angular.z = 0.2
        else:
            self.state = self.MOVING
            self.velocity_message.angular.z = 0.0
            self.start_position = self.current_position

        #Publish velocity command to move turtle
        self.cmd_vel_publisher.publish(self.velocity_message)
        self.get_logger().info(f"Current State: {self.state}")
        self.get_logger().info(f"Publishing velocity: linear.x={self.velocity_message.linear.x}, angular.z={self.velocity_message.angular.z}")
        self.get_logger().info(f"Current Position: X:{self.current_position.x},Y:{self.current_position.y}")

    def stop_turtle(self):
        "Stop the turtle once goal is reached"
        self.get_logger().info('Goal reached! Stopping the turtle.')

        self.velocity_message.linear.x = 0.0

        self.velocity_message.angular.z = 0.0

        self.cmd_vel_publisher.publish(self.velocity_message)
        self.get_logger().info(f"Current State: {self.state}")
        self.get_logger().info(f"Publishing velocity: linear.x={self.velocity_message.linear.x}, angular.z={self.velocity_message.angular.z}")
        self.get_logger().info(f"Current Position: X:{self.current_position.x},Y:{self.current_position.y}")


def main(args=None):
    rclpy.init(args=args)

    defined_goal_publisher = DefinedGoalPublisher()

    rclpy.spin(defined_goal_publisher)

    defined_goal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()