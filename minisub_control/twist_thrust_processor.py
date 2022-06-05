import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray

class ThrustControl(Node):
    def __init__(self):
        super().__init__('ms_thr_ctl')

        self.thr_pwr_publisher_ = self.create_publisher(
            Int64MultiArray, 
            'thrusters_power', 
            10
        )

        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_subscription_cb,
            10
        )
        self.twist_subscription  # prevent unused variable warning

    def twist_subscription_cb(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        
        self.thr_pwr_publish(0.5, 0.5)


    def thr_pwr_publish(self, x_left, x_right):
        msg = Int64MultiArray
        msg.data[0] = x_left
        msg.data[1] = x_right
        
        self.thr_pwr_publisher_.publish(msg)
        
        self.get_logger().info('Publishing: "%s"' % msg.data[0])
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    thrust_control_node = ThrustControl()

    rclpy.spin(thrust_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thrust_control_node.destroy_node()
    rclpy.shutdown()