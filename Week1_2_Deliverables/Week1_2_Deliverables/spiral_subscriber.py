#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class SpiralSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_pose_subscriber')

        #Create subscriber to retrieve the positional data of the turtle
        self.subscription = self.create_subscription(Pose,'turtle1/pose',self.pose_callback,10)

    def pose_callback(self,msg):
        #Callback the position of the turtle
        self.get_logger().info(f"Turtle Position -> X: {msg.x:.2f}, Y: {msg.y:.2f}, Theta: {msg.theta:.2f}")

def main(args=None):
    rclpy.init(args=args)

    turtle_vacuum_subscriber = SpiralSubscriber()

    rclpy.spin(turtle_vacuum_subscriber)

    turtle_vacuum_subscriber.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
