import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
import math
#from turtlesim.srv import TeleportAbsolute


class RandomGoalSubscriber(Node):

    def __init__(self):
        super().__init__('random_goal_subscriber')

        #Define the states for the state machine
        self.MOVING = 0
        self.GOAL_REACHED = 1
        self.state = self.GOAL_REACHED

        #Define the initial position of the turtle and the current goal
        self.current_position = Pose()
        self.current_goal = None

        #subscribe to node 'turtle1/pose' to get the current positional data of the turtle
        self.pose_subscription = self.create_subscription(Pose , 'turtle1/pose' , self.pose_callback , 10)

        #subscribe to node 'random_goal' to get the new goal location from the publisher
        self.goal_subscription = self.create_subscription(Point , 'random_goal' , self.goal_callback , 10)

        #publish to node 'turtle1/cmd_vel' a message of data type Twist
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel',10)

        #timer to update turtle's state and movement
        self.timer = self.create_timer(0.1,self.update_turtle_movement)


        # self.teleport_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')

        # while not self.teleport_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for teleport service to become avaliable...')

    def pose_callback(self, pose_data): 
        """call the current position and save it in pose_data"""
        self.current_position = pose_data

    def goal_callback(self,random_goal): 
        """Call out the new goal and initiate movement to the new goal"""
        self.get_logger().info(f"Recieved new goal: {random_goal}")

        self.current_goal = random_goal

        #Switch state to MOVING when a new goal is recieved
        self.state = self.MOVING
        # self.teleport_turtle(random_goal.x, random_goal.y)
    
    def update_turtle_movement(self):
        """Timer callback to update turtle's movement based on its state"""
        if self.current_goal is None:
            return
        
        if self.state == self.MOVING:
            self.move_towards_goal()

        elif self.state == self.GOAL_REACHED:
            self.stop_turtle()
    
    def move_towards_goal(self):
        """Move turtle towards the goal."""
        goal = self.current_goal

        #Calculate distance and angle to goal
        distance = math.sqrt( (goal.x - self.current_position.x)**2 + (goal.y - self.current_position.y)**2)

        angle_to_goal = math.atan2(goal.y - self.current_position.y, goal.x - self.current_position.x)

        #Calculate angular difference to goal
        angle_diff = angle_to_goal - self.current_position.theta

        #Normalize angle
        angle_diff = (angle_diff + math.pi) % (2*math.pi)-math.pi

        #"""Check to see if turtle has reached its goal"""
        if distance < 0.1:
            #Calculate the difference between the current position at goal and the desired position
            angle_diff_orientation = goal.z - self.current_position.theta
            angle_diff_orientation = (angle_diff_orientation + math.pi) % (2*math.pi - math.pi)

            #Check if the two orientations are within tolerance and break out if no correction required
            if abs(angle_diff_orientation) < 0.1:
                self.state = self.GOAL_REACHED
                return
            
            #command a purely angular velocity to rotate the robot to the desired orientation
            velocity_message = Twist()
            velocity_message.linear.x = 0.0
            velocity_message.angular.z = 2.0 * angle_diff_orientation
            self.cmd_vel_publisher.publish(velocity_message)
            return
        

        #Set linear and angular velocity
        velocity_message = Twist()

        velocity_message.linear.x = 1.5 * distance

        velocity_message.angular.z = 4.0 * angle_diff

        #Publish velocity command to move turtle
        self.cmd_vel_publisher.publish(velocity_message)

    def stop_turtle(self):
        self.get_logger().info('Goal reached! Stopping the turtle.')

        velocity_message = Twist()

        velocity_message.linear.x = 0.0

        velocity_message.angular.z = 0.0

        self.cmd_vel_publisher.publish(velocity_message)


    # def teleport_turtle(self, x, y):
    #     """Teleports the turtle to the specified (x, y) coordinates."""
    #     teleport_request = TeleportAbsolute.Request()
    #     teleport_request.x = x
    #     teleport_request.y = y
    #     teleport_request.theta = 0.0  # You can choose a specific orientation if needed

    #     # Call the teleport service
    #     self.get_logger().info(f"Teleporting turtle to: x={x}, y={y}")
    #     future = self.teleport_client.call_async(teleport_request)
    #     future.add_done_callback(self.teleport_response_callback)

    # def teleport_response_callback(self, future):
    #     """Callback to log the result of the teleport service."""
    #     try:
    #         response = future.result()
    #         self.get_logger().info('Teleportation successful!')
    #     except Exception as e:
    #         self.get_logger().error(f'Service call failed: {e}')

        # self.move_turtle(random_goal)

    # def move_turtle(self,random_goal): # define how the turtle will move from its current position to the new goal
    #     velocity_message = Twist()
        
    #     while rclpy.ok(): #continue running until node is manually shut down
    #         distance = math.sqrt((random_goal.x - self.current_position.x)**2 + (random_goal.y - self.current_position.y)**2 )
    #         steering_angle = math.atan2(random_goal.y - self.current_position.y, random_goal.x - self.current_position.x)
    #         velocity_message.linear.x = 1.5 * distance
    #         velocity_message.angular.z = 4 * (steering_angle - self.current_position.theta)

    #         self.cmd_vel_publisher.publish(velocity_message)

    #         if distance < 0.1:
    #             velocity_message.linear.x = 0
    #             velocity_message.angular.z = 0
    #             self.cmd_vel_publisher.publish(velocity_message)
    #             break

    #         rclpy.spin_once(self, timeout_sec=0.1)






def main(args=None):
    rclpy.init(args=args)

    random_goal_subscriber = RandomGoalSubscriber()

    rclpy.spin(random_goal_subscriber)

    random_goal_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()