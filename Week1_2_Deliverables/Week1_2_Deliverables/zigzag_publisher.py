import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class ZigZagPublisher(Node):
    def __init__(self):
        super().__init__('zigzag_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose',self.pose_callback,10)

        self.pose = None
        self.state = 'MOVE_FORWARD'
        self.state_timer = 0
        self.zigzag_count = 0

    def pose_callback(self,msg):
        self.pose = msg
        
    def move_turtle(self, linear_vel, angular_vel):
        velocity_msg = Twist()
        velocity_msg.linear.x = linear_vel
        velocity_msg.angular.z = angular_vel
        self.publisher_.publish(velocity_msg)

    def update_state_machine(self):

