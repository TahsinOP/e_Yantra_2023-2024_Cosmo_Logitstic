#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics

xy_tolerance = 0.25 
pre_dock_x = 0.50
pre_dock_y = 4.55

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Add another one here

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize all flags and parameters here
        self.is_docking = False
        self.robot_pose = [0.0, 0.0, 0.0]
        self.dock_pose = [0.0,0.0]
        self.usrleft_value = 0.0
        self.usrright_value = 0.0
        self.dock_aligned = False

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    # Callback function for the left ultrasonic sensor
    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range
    # Callback function for the right ultrasonic sensor
    def ultrasonic_rr_callback(self, msg):
    # Extract and update the value from the right ultrasonic sensor
        self.usrright_value = msg.range



    # Callback function for the right ultrasonic sensor
    # Implement the right ultrasonic sensor callback here

    # Utility function to normalize angles within the range of -π to π (OPTIONAL)
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # Main control loop for managing docking behavior
    def controller_loop(self):

        angular_speed = 0.0
        if self.is_docking :
            if (self.robot_pose[0] >= pre_dock_x - xy_tolerance or self.robot_pose[0] <= pre_dock_x + xy_tolerance ) and (self.robot_pose[1] >= pre_dock_y - xy_tolerance or self.robot_pose[1] <= pre_dock_y + xy_tolerance) : 

                # Calculate angular correction to align the robot with the desired orientation
                target_angle = self.dock_pose[1] 
                angular_error = self.normalize_angle(target_angle - self.robot_pose[2])
                angular_speed = 0.7 * angular_error  # P-controller for angular correction
                velocity_msg = Twist()
                velocity_msg.angular.z = angular_speed

                self.velocity_pub.publish(velocity_msg)
                

                # Check if the robot is aligned within a threshold
                if abs(angular_error) < 0.1:
                   self.dock_aligned = True
                   self.get_logger().info("Robot is aligned for docking.")
                else:
                   self.get_logger().info("Aligning robot...")

            # else:
            #     # Calculate linear correction based on ultrasonic sensor data
            #     linear_speed = self.calculate_linear_correction()
            #     velocity_msg = Twist()
            #     velocity_msg.angular.z = angular_speed
            #     velocity_msg.linear.x = linear_speed
            #     self.velocity_pub.publish(velocity_msg)
                
    def calculate_linear_correction(self):
        # Implement linear correction based on ultrasonic sensor data
        # Use ultrasonic sensor data to find the rear distance and adjust linear_speed
        rear_distance = min(self.usrleft_value, self.usrright_value)

        if rear_distance > 1.0:
            linear_speed = 0.2  # Move forward when rear distance is safe
        elif rear_distance < 0.3:
            linear_speed = 0.0  # Stop when getting closer to the rack
        else:
            linear_speed = -0.2  # Move back if too close to an obstacle

        return linear_speed
    
    def is_robot_aligned(self):

    # Implement alignment logic using ultrasonic sensor data
    # For example, check if both ultrasonic sensors provide similar readings
       return abs(self.usrleft_value - self.usrright_value) < 0.1  # Adjust the threshold as needed


    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):


        self.is_docking = True  # Start docking process
        self.dock_aligned = False  # Reset alignment flag
        self.linear_dock = request.linear_dock  # Set linear correction flag
        self.orientation_dock = request.orientation_dock  # Set angular correction flag
        self.dock_pose = [request.distance, request.orientation]  # Set desired docking pose

        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking (you may need to implement this logic)
        while not self.dock_aligned:
            self.get_logger().info("Waiting for alignment...")

            if self.is_robot_aligned():
                
                self.dock_aligned = True
            # Implement alignment logic here
            # You can use ultrasonic sensor data to determine alignment
            rate.sleep()

        # Set the service response indicating success
        response.success = True
        response.message = "Docking control initiated"
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()

    executor.add_node(my_robot_docking_controller)
    

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
