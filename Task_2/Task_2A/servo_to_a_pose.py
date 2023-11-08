#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to reach a target 3D Cartesian pose.
`ros2 run pymoveit2 ex_servo.py`
"""
from std_srvs.srv import Trigger 
from copy import deepcopy
import rclpy
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import QoSProfile
import tf2_ros
import math

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.tf_buffer = tf2_ros.Buffer(Duration(seconds=1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.target_pose =   [0.35, 0.1, 0.68]
        self.stop_threshold = 0.1  # Adjust this threshold as needed

        self.timer = self.create_timer(0.02, self.servo_reach_target_pose, ReentrantCallbackGroup())

    def servo_reach_target_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform("base_link", "ee_link", rclpy.time.Time().to_msg())
            translation = transform.transform.translation

            current_pose = [translation.x, translation.y, translation.z]

            distance = math.sqrt((self.target_pose[0] - current_pose[0])**2 +
                                 (self.target_pose[1] - current_pose[1])**2 +
                                 (self.target_pose[2] - current_pose[2])**2)

            gain = 3.0

            if distance > self.stop_threshold:
                linear_velocity = [gain * (self.target_pose[0] - current_pose[0]),
                                  gain * (self.target_pose[1] - current_pose[1]),
                                  gain * (self.target_pose[2] - current_pose[2])]

                twist_msg = TwistStamped()
                twist_msg.header.frame_id = 'base_link'
                twist_msg.twist.linear.x = linear_velocity[0]
                twist_msg.twist.linear.y = linear_velocity[1]
                twist_msg.twist.linear.z = linear_velocity[2]

                twist_msg.header.stamp = self.get_clock().now().to_msg()

                self.twist_pub.publish(twist_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def start_servo_service(self):
        # Create a client for the start_servo service
        start_servo_client = self.create_client(Trigger, '/servo_node/start_servo')

        # Wait for the service to be available
        if not start_servo_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Service '/servo_node/start_servo' not available. Is Servo properly configured?")
        else:
            # Create a request for the start_servo service
            request = Trigger.Request()

            # Call the start_servo service
            future = start_servo_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info("Servo controller started successfully.")
                else:
                    self.get_logger().error("Failed to start Servo controller: %s", future.result().message)
            else:
                self.get_logger().error("Service call failed. Unable to start Servo controller.")


def main():
    rclpy.init()
    node = ServoController()
    node.start_servo_service()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()