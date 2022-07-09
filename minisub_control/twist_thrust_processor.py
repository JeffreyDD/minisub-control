import enum
import numpy, enum
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class ThrusterType(enum.Enum):
    Horizontal = 1
    Vertical = 2

class ThrusterConfig():
    type = ThrusterType.Horizontal

class TwistThrustProcessor(Node):
    def __init__(self):
        super().__init__('twist_thrust_processor')

        self.thr_pwr_publisher_ = self.create_publisher(
            Float32MultiArray, 
            'thrusters_power', 
            10
        )

        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_subscription_cb,
            10
        )

        # FIXME: Use parameter, allow to be configured per thruster
        self.inverse_thrust = True
        
    def twist_subscription_cb(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)

        speed = numpy.clamp(msg.linear.x, -1, 1)
        rotation = msg.angular.z

        if(self.inverse_thrust):
            speed = -speed
        
        left = speed
        right = speed

        if(rotation != 0):
            if(speed != 0):
                if(rotation > 0):
                    left = speed
                    right = speed * rotation
                elif(rotation < 0):
                    left = speed * -rotation
                    right = speed
            else:
                # These come out way to high
                left = -rotation
                right = rotation
        
        msg = Float32MultiArray()
        msg.data = [left, right]

        self.thr_pwr_publisher_.publish(msg)
        
        self.get_logger().info('Publishing: "%s"' % msg.data[0])
        
def main(args=None):
    rclpy.init(args=args)

    thrust_control_node = TwistThrustProcessor()

    rclpy.spin(thrust_control_node)

    thrust_control_node.destroy_node()

    rclpy.shutdown()