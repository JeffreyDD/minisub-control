import enum
import numpy, enum
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class ThrusterType(enum.Enum):
    Horizontal = 1
    Vertical = 2

class ThrusterHorizontalPosition(enum.Enum):
    Left = 1
    Middle = 2
    Right = 3

class ThrusterVerticalPosition(enum.Enum):
    Top = 1
    Middle = 2
    Bottom = 3

def ThrusterConfig(thruster_type, h_pos, v_pos):
    return {
        # In future, this could automatically be determined based on rotation of transform
        "thruster_type": thruster_type, 
        # In future, these could automatically be determined based on position in relation to the center of the vessel
        "position": { 
            "horizontal" : h_pos,
            "vertical": v_pos
        }
    }

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

        self.thrusters = [
            ThrusterConfig(ThrusterType.Horizontal, ThrusterHorizontalPosition.Left, ThrusterVerticalPosition.Bottom),
            ThrusterConfig(ThrusterType.Horizontal, ThrusterHorizontalPosition.Right, ThrusterVerticalPosition.Bottom),
            ThrusterConfig(ThrusterType.Vertical, ThrusterHorizontalPosition.Left, ThrusterVerticalPosition.Middle),
            ThrusterConfig(ThrusterType.Vertical, ThrusterHorizontalPosition.Right, ThrusterVerticalPosition.Middle),
        ]

        # FIXME: Use parameter, allow to be configured per thruster
        self.inverse_thrust = True
        
    def twist_subscription_cb(self, msg):
        self.get_logger().debug('I heard: "%s"' % msg)

        speed = numpy.clip(msg.linear.x, -0.99, 0.99)
        rotation = numpy.clip(msg.angular.z, -0.99, 0.99)

        self.get_logger().debug("Parsed speed: {} rotation: {}".format(speed, rotation))

        left = speed
        right = speed

        if(rotation != 0):
            if(speed > 0):
                if(rotation > 0):
                    left = speed
                    right = speed + (speed * rotation)
                elif(rotation < 0):
                    left = speed + (speed * -rotation)
                    right = speed
            elif(speed < 0):
                if(rotation > 0):
                    left = speed + (speed * rotation)
                    right = speed
                elif(rotation < 0):
                    left = speed
                    right = speed + (speed * -rotation)
            else:
                # These come out way to high
                left = -rotation * 0.5
                right = rotation * 0.5
        
        if(self.inverse_thrust):
            left = -left
            right = -right
        
        # Vertical calculations
        vspeed = numpy.clip(msg.linear.z, -1, 1)

        # Setup & sent message
        msg = Float32MultiArray()
        for thruster_config in self.thrusters:
            if thruster_config["thruster_type"] == ThrusterType.Horizontal:
                if thruster_config["position"]["horizontal"] == ThrusterHorizontalPosition.Left:
                    msg.data.append(left)
                elif thruster_config["position"]["horizontal"] == ThrusterHorizontalPosition.Right:
                    msg.data.append(right)
                else:
                    msg.data.append(0)
            else:
                if thruster_config["position"]["horizontal"] == ThrusterHorizontalPosition.Left:
                    msg.data.append(vspeed)
                elif thruster_config["position"]["horizontal"] == ThrusterHorizontalPosition.Right:
                    msg.data.append(vspeed)
                else:
                    msg.data.append(0)
                
        self.thr_pwr_publisher_.publish(msg)
        
        self.get_logger().debug('Publishing: "%s"' % msg.data[0])
        
def main(args=None):
    rclpy.init(args=args)

    thrust_control_node = TwistThrustProcessor()

    rclpy.spin(thrust_control_node)

    thrust_control_node.destroy_node()

    rclpy.shutdown()