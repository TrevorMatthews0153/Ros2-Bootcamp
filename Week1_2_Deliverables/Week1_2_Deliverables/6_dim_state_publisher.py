import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import Float32MultiArray


class SixDimState(Node):

    def __init__(self):
        super().__init__('Seven_dim_state')
        self.publisher = self.create_publisher(Float32MultiArray, 'topic', 10)
        timer_period = 1.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float32MultiArray()
        # State_vector = [np.random.randint(1, 50, size=7)]
        msg.data = [random.uniform(0.0, 10.0) for _ in range(7)]
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data} - {self.i}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    Six_Dim_State = SixDimState()

    rclpy.spin(Six_Dim_State)

    # Destroy the node explicitly
    SixDimState.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()