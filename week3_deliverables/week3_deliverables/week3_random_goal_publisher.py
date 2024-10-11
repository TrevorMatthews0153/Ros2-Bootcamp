import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random
import math


class RandomGoalPub(Node):

    def __init__(self):
        super().__init__('random_goal_publisher') #define the name of the publisher
        self.goal_publisher = self.create_publisher(Point, 'random_goal', 10) #create publisher with message type Point and publish to topic 'random_goal'
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.publish_goal) #create a timer to send a new goal every 10 seconds
        self.i = 0

    def publish_goal(self): #define the message published to topic
        random_goal = Point() #define random_goal to be a Point data type, with parameters x and y
        random_goal.x = random.uniform(-3,3) #define the new x coordinate for goal
        random_goal.y = random.uniform(-3,3) #define the new y coordinate for goal
        random_goal.z = random.uniform(-math.pi,math.pi) #define new theta coordinate for goal
        self.get_logger().info(f"Publishing random goal {self.i}: {random_goal}") #record the random goal published
        self.goal_publisher.publish(random_goal) #publish random_goal of type Point to 'random_goal'
        self.i += 1 #keep a counter to keep track of goals relative to the subscriber

#run the Node and begin publishing
def main(args=None):
    rclpy.init(args=args)

    random_goal_publisher = RandomGoalPub()

    rclpy.spin(random_goal_publisher)

    random_goal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()