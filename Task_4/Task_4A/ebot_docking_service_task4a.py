#!/usr/bin/env python3



# Team ID:          CL#1868
# Author List:		Tahsin Khan 
# Filename:		    ebot_docking_service_task2b.py
# Functions:        odometery_callback,ultrasonic_rl_callback,ultrasonic_rr_callback,normalize_angle,controller_loop,calculate_linear_correction,
#                   dock_control_callback, main 
#			       
# Nodes:		   
#                
#			        Publishing Topics  - [ cmd_vel,odom]
#                   Subscribing Topics - [/ultrasonic_rl/scan ,/ultrasonic_rr/scan ]

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray,Float32
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()
        self.yaw = None

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        self.orientation_sub_for_docking = self.create_subscription(Float32,'orientation',self.orientation_callback,10)   # Topic for hardware !!!!!!!!!!!DONE
        

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultra_sub_for_docking = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultra_callback, 10)
        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize all flags and parameters here
        self.is_docking = False
        self.robot_pose = [0.0, 0.0, 0.0]
        self.dock_pose = [0.0,0.0]
        self.ultra_left = 0.0
        self.ultra_right = 0.0
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

    def ultra_callback(self,msg):

        self.ultra_left= msg.data[4]
        self.ultra_right = msg.data[5]
    
    def orientation_callback(self,msg):    

        self.yaw = msg.data 
        print(f"yaw is {self.yaw}")  # Use the yaw obtained in controller_loop function !!!!!!!!!!DONE

    # Utility function to normalize angles within the range of -π to π (OPTIONAL)
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # Main control loop for managing docking behavior
    def controller_loop(self):
        
        Controller_gain = 0.2     # Change this values accordingly in the 2nd slot !!!!!!!!!!!!!!!!!
        error = 0.2  
      
        angular_speed = 0.0
        linear_speed = 0.0 

        
        if self.is_docking :

            # Calculate angular correction to align the robot with the desired orientation
            target_angle = self.dock_pose[1] 

            angular_error = target_angle - self.yaw
            print(f"angular error is {angular_error}") 
              # Check the code also by removing the normalize angle function !!!!!!!!!!!!!!

            angular_speed = Controller_gain * angular_error  
            velocity_msg = Twist()
            velocity_msg.angular.z = angular_speed

            self.velocity_pub.publish(velocity_msg)
            
                # Check if the robot is aligned within a threshold
            if abs(angular_error) < error:
                self.dock_aligned = True
                self.get_logger().info("Robot is aligned for docking.")
            else:
                self.get_logger().info("Aligning robot...")

            if self.dock_aligned and self.linear_dock :
                
                linear_speed = self.calculate_linear_correction()

                if linear_speed == 0.0:

                # If no linear correction is needed, stop the robot
                    velocity_msg = Twist()
                    velocity_msg.linear.x = 0.0
                    self.velocity_pub.publish(velocity_msg)
                    self.is_docking = False  # Docking is complete
                    self.get_logger().info("Docking complete.")
                else:
                # Publish the calculated linear speed
                    velocity_msg = Twist()
                    velocity_msg.linear.x = linear_speed
                    self.velocity_pub.publish(velocity_msg)
                    self.get_logger().info(f"Linear correction: {linear_speed}")

    def calculate_linear_correction(self):
        # Implement linear correction based on ultrasonic sensor data
        # Use ultrasonic sensor data to find the rear distance and adjust linear_speed
        rear_distance = min(self.ultra_left, self.ultra_right)
        rear_distance = rear_distance/100
        print(f"distance is {rear_distance}")

        if rear_distance > 1.0:
            linear_speed = -0.2  # Move forward when rear distance is safe
        elif rear_distance < 0.25:
            linear_speed = 0.0  # Stop when getting closer to the rack
        else:
            linear_speed = -0.2  # Move back if too close to an obstacle

        return linear_speed

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):

        self.is_docking = True  # Start docking process
        self.dock_aligned = False  # Reset alignment flag
        self.linear_dock = request.linear_dock  # Set linear correction flag
        self.orientation_dock = request.orientation_dock  # Set angular correction flag
        self.dock_pose = [request.distance, request.orientation]  # Set desired docking pose

        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")

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