#! /usr/bin/env python3


from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray,Float32
from ebot_docking.srv import DockSw
from tf_transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from usb_relay.srv import RelaySw
import math
import yaml
import os

# Team ID:          CL#1868

# Author List:		Tahsin Khan 

# Filename:		    ebot_nav2_cmd_task2b.py

# Functions:        normalize_angle,left_sensor_callback,right_sensor_callback,odometry_callback_,navigate_to_home_pose,
#                   navigate_to_arm_pose,navigate_and_dock,trigger_docking_service_intial,trigger_docking_service_final,trigger_attachment_service,
#                   trigger_dettachment_service ,docking_error_control_loop ,main
#			        
# Nodes:		  
#			        Publishing Topics  - 
#                   Subscribing Topics - [ odom,/ultrasonic_rl/scan,/ultrasonic_rr/scan]

class NavigationAndDockingNode(Node):
    def __init__(self):
        super().__init__('navigation_and_docking_node')

        self.navigator = BasicNavigator()
        self.robot_pose = [0.0, 0.0, 0.0]
        self.docking_attempts = 0
        self.ultra_right = None
        self.ultra_left = None
        self.yaw = None
        self.rear_distance = 0.0
        self.rack_place_operation_complete = False
        self.docked = False                 # Flag to determine if docking is completed or not 
        self.target_angle_rack_hardware = 3.14
        self.target_angle_rack_1 = -3.14
        self.dock_service_error = 0.1   # Increase this factor in the 2nd Slot !!!!!!!!!!!
        self.pre_dock_correction_factors_rack1 = [-0.57,-0.86,0.26]
        self.pre_dock_correction_factors_rack3 = [-0.57,0.23,0.69]
        self.docking_service_client = self.create_client(DockSw, 'dock_control') 
        self.odom_sub_for_trigger = self.create_subscription(Odometry, 'odom', self.odometry_callback_, 10)
        self.ultra_sub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultra_callback, 10)
        self.orientation_sub = self.create_subscription(Float32,'orientation',self.orientation_callback,10)


        self.error_timer = self.create_timer(0.2,self.docking_error_control_loop)

    def docking_error_control_loop(self):

        dock_error =  self.target_angle_rack_hardware - self.yaw

        if abs(dock_error) < self.dock_service_error :
            self.docked = True 
        
        return self.docked  
    
    def ultra_callback(self,msg):

        self.ultra_left= msg.data[4]
        self.ultra_right = msg.data[5]

        self.rear_distance = min(self.ultra_left, self.ultra_right)
        self.rear_distance = self.rear_distance/100
    
    def orientation_callback(self,msg):     # Complete the callback function and replace the robot_pose by yaw !!!!!!!!!!!

        self.yaw = msg.data

    def navigate_to_home_pose(self):
        # Define the goal pose for the subsequent navigation
        pose_euler = [0.0,0.0,0.0]
        euler_rot = (R.from_euler('xyz',pose_euler,degrees=False))
        pose_quat = list(euler_rot.as_quat())

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.0 # Replace with your desired X coordinate
        goal_pose.pose.position.y = 0.0 # Replace with your desired Y coordinate

        goal_pose.pose.orientation.z = pose_quat[2]# Replace with your desired orientation
        goal_pose.pose.orientation.w = pose_quat[3]# Replace with your desired orientation

        # Navigate to the new goal pose
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation to the home pose succeeded.')
            self.rack_place_operation_complete = True
        else:
            self.get_logger().error('Navigation to the home pose failed.')

    def navigate_to_arm_pose(self):  
        X = 1.05      # Update this values according to the given values !!!!!!!!!!!!!
        Y = 2.04
        Yaw = 0.0
        pose_euler = [0,0,Yaw]
        euler_rot = (R.from_euler('xyz',pose_euler,degrees=False))
        pose_quat = list(euler_rot.as_quat())

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = X  # Co-ordinates of the arm pose 
        goal_pose.pose.position.y = Y

        goal_pose.pose.orientation.z = pose_quat[2]# Replace with your desired orientation
        goal_pose.pose.orientation.w = pose_quat[3]# Replace with your desired orientation

        # Navigate to the new goal pose
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation to the new pose succeeded.')
            self.trigger_docking_service_final()
        else:
            self.get_logger().error('Navigation to the new pose failed.')       
    
    def navigate_and_dock(self):

        X = 1.05      # Update this values according to the given values !!!!!!!!!!!!!
        Y = 2.04
        Yaw = 0.0
        pose_euler = [0,0,Yaw]
        euler_rot = (R.from_euler('xyz',pose_euler,degrees=False))
        pose_quat = list(euler_rot.as_quat())

        # Define the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = X
        goal_pose.pose.position.y = Y
        goal_pose.pose.orientation.z = pose_quat[2]
        goal_pose.pose.orientation.w = pose_quat[3]

        # Navigate to the goal pose
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation succeeded. Triggering docking service...')
            self.trigger_docking_service_intial()
        else:
            self.get_logger().error('Navigation failed.')

    def trigger_docking_service_intial(self):
        self.get_logger().info("Triggering the docking service ")
        dock_control_request = DockSw.Request()
        dock_control_request.linear_dock = True  # Enable linear correction
        dock_control_request.orientation_dock = True  # Enable angular correction
        dock_control_request.distance = 0.0 # Specify the desired distance
        dock_control_request.orientation = 0.0  # Specify the desired orientation
        dock_control_request.rack_no = "rack3"  # Specify the rack number

        future = self.docking_service_client.call_async(dock_control_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Docking service succeeded. Waiting for robot to come to rest.")
            while not self.rear_distance < 0.25:
                rclpy.spin_once(self)

            self.get_logger().info("Robot is near the rack . Triggering attachment service.")
            self.switch_eletromagent(True)        
        else:
            self.get_logger().error("Docking service failed.")

    def trigger_docking_service_final(self):

        self.get_logger().info("Triggering the docking service ")
        dock_control_request = DockSw.Request()
        dock_control_request.linear_dock = False# Enable linear correction
        dock_control_request.orientation_dock = True # Enable angular correction
        dock_control_request.distance = 0.0 # Specify the desired distance
        dock_control_request.orientation = 3.14  # Specify the desired orientation
        dock_control_request.rack_no = "rack3"  # Specify the rack number

        future = self.docking_service_client.call_async(dock_control_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Docking service succeeded. Waiting for robot to come to rest.")
            while not self.docked:
                rclpy.spin_once(self)
                self.get_logger().info('Second docking is not complete. Keep waiting.')

            self.switch_eletromagent(False)
        else:
            self.get_logger().error("Docking service failed.")
    

    def switch_eletromagent(self,relayState):
        
        self.get_logger().info('Changing state of the relay to '+str(relayState))
        self.trigger_usb_relay = self.create_client(RelaySw, 'usb_relay_sw')
        while not self.trigger_usb_relay.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('USB Trigger Service not available, waiting...')

        request_relay = RelaySw.Request()
        request_relay.relaychannel = True
        request_relay.relaystate = relayState
        self.usb_relay_service_resp=self.trigger_usb_relay.call_async(request_relay)
        rclpy.spin_until_future_complete(self, self.usb_relay_service_resp)
        if(self.usb_relay_service_resp.result().success== True):
            self.get_logger().info(self.usb_relay_service_resp.result().message)

            if relayState == True:
                self.navigate_to_arm_pose()
        else:
            self.get_logger().warn(self.usb_relay_service_resp.result().message)

def main(args=None):     

    rclpy.init(args=args)

    ebot_nav2_cmd_node = NavigationAndDockingNode()
    
    ebot_nav2_cmd_node.navigate_and_dock()
    
    while not ebot_nav2_cmd_node.rack_place_operation_complete:

        rclpy.spin(ebot_nav2_cmd_node)

    ebot_nav2_cmd_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':

    main()