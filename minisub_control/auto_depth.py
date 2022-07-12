import enum
import numpy, enum
from simple_pid import PID
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32


class TwistThrustProcessor(Node):
    def __init__(self):
        super().__init__('twist_thrust_processor')

        self.twist_publisher = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10
        )

        self.depth_subscription = self.create_subscription(
            Float32,
            'desired_depth',
            self.depth_subscription_cb,
            10
        )
        
        self.pressure_subscription = self.create_subscription(
            Float32,
            'outside_pressure',
            self.pressure_subscription_cb,
            10
        )

        self.pid = PID(1, 0.1, 0.05, setpoint=0)
        self.pid.output_limits = (-1, 1)
        self.pid.sample_time = 0.1


    def depth_subscription_cb(self, msg):
        self.get_logger().debug('I heard: "%s"' % msg)

        self.desired_depth = msg.data

        self.pid.setpoint = self.desired_depth


    def pressure_subscription_cb(self, msg):
        self.get_logger().debug('I heard: "%s"' % msg)

        self.current_depth = msg.fluid_pressure / 9780;

        # Setup & sent message
        msg = Twist()
        msg.linear.z = self.pid(self.current_depth)
                
        self.twist_publisher.publish(msg)
        
        self.get_logger().debug('Publishing: "%s"' % msg)

        
def main(args=None):
    rclpy.init(args=args)

    thrust_control_node = TwistThrustProcessor()

    rclpy.spin(thrust_control_node)

    thrust_control_node.destroy_node()

    rclpy.shutdown()