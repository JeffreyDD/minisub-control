from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from tf2_ros import TransformBroadcaster

# import tf_transformations

from sensor_msgs.msg import Imu


class FramePublisher(Node):

    def __init__(self):
        super().__init__('imu_tf2_frame_publisher')

        # Declare and acquire `turtlename` parameter
        # self.declare_parameter('turtlename', 'turtle')
        # self.turtlename = self.get_parameter(
        #     'turtlename').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        # Subscribe to imu topic and call handle_imu_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.handle_imu,
            qos_profile_sensor_data)
        self.subscription

    def handle_imu(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'imu'

        # IMU doesn't know about translation, so set to 0
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Assign rotation from IMU orientation
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        # Send the transformation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()