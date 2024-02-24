#! /usr/bin/env python3


from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range 
from ebot_docking.srv import DockSw
from ebot_docking.srv import ArmBotSw
from tf_transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from linkattacher_msgs.srv import AttachLink, DetachLink  
import math
import yaml
import os
import threading

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

    def __init__(self, config_params, rack_idx ,navigator_service):

        super().__init__(f'navigation_and_docking_node{rack_idx}')
        self.navigator = navigator_service
        self.robot_pose = [0.0, 0.0, 0.0]
        self.docking_attempts = 0
        self.rack_place_operation_complete = False
        self.docked = False 
        self.docked_midway = False                # Flag to determine if docking is completed or not 
        self.target_angle_rack_3  = 3.14
        self.target_angle_rack_1 = - 1.57
        self.target_angle_rack_2 = -3.14
        self.dock_service_error = 0.07
        self.pre_dock_correction_factors_rack1 = [-1.2,-0.86,0.15]#-1.3
        self.pre_dock_correction_factors_rack3 = [-0.57,-0.3,0.6]#-0.57
        self.pre_dock_correction_factors_rack2 = [-0.5,-0.11,-0.9]#-0.5 , -0.05
        self.arm_pose_correction_factors_rack1 = []
        self.arm_pose_correction_factors_rack3 = []
        self.arm_pose_correction_factors_rack2 = []
        self.docking_service_client = self.create_client(DockSw, 'dock_control') 
        self.rack_arm_client = self.create_client(ArmBotSw , 'rack_place')
        self.link_attach_client = self.create_client(AttachLink, '/ATTACH_LINK')
        self.link_detach_client = self.create_client(DetachLink, '/DETACH_LINK')
        self.odom_sub_for_trigger = self.create_subscription(Odometry, 'odom', self.odometry_callback_, 10)
        self.left_sensor_subscription = self.create_subscription(Range,'/ultrasonic_rl/scan', self.left_sensor_callback,10)
        self.right_sensor_subscription = self.create_subscription( Range,'/ultrasonic_rr/scan', self.right_sensor_callback,10)
        self.cmd_vel_sub = self.create_publisher(Twist ,'cmd_vel',10)
        self.error_timer = self.create_timer(0.2,self.docking_error_control_loop)
        self.config_params = config_params
        self.package_id = self.config_params['package_id'][rack_idx]
        self.rack_info = self.config_params['position'][int(self.package_id)-1][f"rack{self.package_id}"]
        self.rack_pose_x = self.rack_info[0]
        self.rack_pose_y = self.rack_info[1]
        self.rack_orientation = self.rack_info[2]
        self.get_logger().info(f"The package id is {self.package_id}")
        self.get_logger().info(f"The required rack information is {self.rack_info}")
    
    def send_place_rack_request(self,package_id=None):
        # Send a service request to the navigation node
        request = ArmBotSw.Request()
        request.rack_id = self.package_id

        if package_id is not None:
            request.rack_id = package_id
        future = self.rack_arm_client.call_async(request)
        if future.result() is not None:
            self.get_logger().info("Arm Rack service suceeded ")
        else:
            self.get_logger().error("Service call failed!")

    def docking_error_control_loop(self):

        if self.package_id == 2 : 
            self.target_angle = self.target_angle_rack_2
        elif self.package_id == 1 :
            self.target_angle = self.target_angle_rack_1
        else :
            self.target_angle = self.target_angle_rack_3

        dock_error =  self.normalize_angle(self.target_angle - self.robot_pose[2])

        if abs(dock_error) < self.dock_service_error :
            self.docked = True 
            self.error_timer.cancel()
    
        return self.docked
    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi

        return angle  

    def left_sensor_callback(self, msg):
        self.left_sensor_distance = msg.range

    def right_sensor_callback(self, msg):
        self.right_sensor_distance = msg.range

    def odometry_callback_(self,msg): 

        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw 

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

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            pass
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation to the home pose succeeded.')
            self.rack_place_operation_complete = True
        else:
            self.get_logger().error('Navigation to the home pose failed.')
    
    def navigate_to_mid_dock_pose(self):
        # Define the goal pose for the subsequent navigation
        pose_euler = [0.0,0.0,3.0] #2.5
        euler_rot = (R.from_euler('xyz',pose_euler,degrees=False))
        pose_quat = list(euler_rot.as_quat())
# Correction factors for pre-dock poses
        self.mid_corrected_rack_pose_x = self.rack_pose_x+self.pre_dock_correction_factors_rack1[1]
        self.mid_corrected_rack_pose_y = self.rack_pose_y+self.pre_dock_correction_factors_rack1[2]

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 1.47# Replace with your desired X coordinate
        goal_pose.pose.position.y = -3.6# Replace with your desired Y coordinate

        goal_pose.pose.orientation.z = pose_quat[2]# Replace with your desired orientation
        goal_pose.pose.orientation.w = pose_quat[3]# Replace with your desired orientation

        # Navigate to the new goal pose
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation to the home pose succeeded.')
            self.send_place_rack_request(3)
            self.trigger_docking_service_midway()
        else:
            self.get_logger().error('Navigation to the home pose failed.')

    def navigate_to_arm_pose(self): 

        if self.package_id == 1 :         # Condition to check the packages ids and allot its respective post-docking angles 
            self.arm_orientation = 1.9
            self.arm_pose_x = 1.38
            self.arm_pose_y = -3.34

            
            # self.arm_pose_x = 1.258
            # self.arm_pose_y = -3.3

        elif self.package_id == 2:
            self.arm_orientation = 0.5
            self.arm_pose_x = 0.57
            self.arm_pose_y = -2.51

        pose_euler = [0,0,self.arm_orientation]
        euler_rot = (R.from_euler('xyz',pose_euler,degrees=False))
        pose_quat = list(euler_rot.as_quat())

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.arm_pose_x  # Co-ordinates of the arm pose 
        goal_pose.pose.position.y = self.arm_pose_y

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
        if self.package_id == 2: 
            self.corrected_rack_orientation = self.rack_orientation + self.pre_dock_correction_factors_rack2[0]    # Correction factors for pre-dock poses
            self.corrected_rack_pose_x = self.rack_pose_x+self.pre_dock_correction_factors_rack2[1]
            self.corrected_rack_pose_y = self.rack_pose_y+self.pre_dock_correction_factors_rack2[2]

            print(self.rack_orientation)

        elif self.package_id == 1:
            self.corrected_rack_orientation = self.rack_orientation + self.pre_dock_correction_factors_rack1[0]     # Correction factors for pre-dock poses
            self.corrected_rack_pose_x = self.rack_pose_x+self.pre_dock_correction_factors_rack1[1]
            self.corrected_rack_pose_y = self.rack_pose_y+self.pre_dock_correction_factors_rack1[2]
        
        elif self.package_id == 3 :

            self.corrected_rack_orientation = 0.0
            self.corrected_rack_pose_x = 0.0
            self.corrected_rack_pose_y = 0.0



        pose_euler = [0,0,self.corrected_rack_orientation]
        euler_rot = (R.from_euler('xyz',pose_euler,degrees=False))
        pose_quat = list(euler_rot.as_quat())

        # Define the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.corrected_rack_pose_x
        goal_pose.pose.position.y = self.corrected_rack_pose_y
        goal_pose.pose.orientation.z = pose_quat[2]
        goal_pose.pose.orientation.w = pose_quat[3]

        # Navigate to the goal pose
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:

            if self.package_id == 3 :
                self.rack_place_operation_complete = True
                # self.send_place_rack_request()
            else :
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
        dock_control_request.orientation = (self.rack_orientation)  # Specify the desired orientation
        dock_control_request.rack_no = f"rack{int(self.package_id)}"  # Specify the rack number

        future = self.docking_service_client.call_async(dock_control_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Docking service succeeded. Waiting for robot to come to rest.")
            while not ( self.left_sensor_distance < 0.15 and self.right_sensor_distance < 0.15):
                rclpy.spin_once(self)

            self.get_logger().info("Robot is near the rack . Triggering attachment service.")
            self.trigger_attachment_service()
        else:
            self.get_logger().error("Docking service failed.")

    def give_linear_velocity(self):

        velocity_cmd = Twist()


        if self.robot_pose[1] < -3.35:

            print(self.robot_pose[1])
            velocity_cmd.linear.x = -0.3

            self.cmd_vel_sub.publish(velocity_cmd)

        elif self.robot_pose[1] > -3.35:
            velocity_cmd.linear.x = 0.0
            self.cmd_vel_sub.publish(velocity_cmd)
            self.trigger_detachment_service()

    def trigger_docking_service_midway(self):
        self.get_logger().info("Triggering the docking service ")
        dock_control_request = DockSw.Request()
        dock_control_request.linear_dock = True # Enable linear correction
        dock_control_request.orientation_dock = True  # Enable angular correction
        dock_control_request.distance = 0.0 # Specify the desired distance
        dock_control_request.orientation = -1.57  # Specify the desired orientation
        dock_control_request.rack_no = f"rack{int(self.package_id)}"  # Specify the rack number

        future = self.docking_service_client.call_async(dock_control_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            while not self.docked:
                rclpy.spin_once(self)
                self.get_logger().info('Second docking is not complete. Keep waiting.')
            self.linear_timer = self.create_timer(0.2,self.give_linear_velocity)
            self.give_linear_velocity()
            self.get_logger().info("Docking service succeeded. Waiting for robot to come to rest.")
            self.get_logger().info("Robot is near the rack . Triggering attachment service.")
        else:
            self.get_logger().error("Docking service failed.")

    def trigger_docking_service_final(self):

        self.get_logger().info("Triggering the docking service ")
        dock_control_request = DockSw.Request()
        dock_control_request.linear_dock = True# Enable linear correction
        dock_control_request.orientation_dock = False # Enable angular correction
        dock_control_request.distance = 0.0 # Specify the desired distance
        dock_control_request.orientation = self.target_angle  # Specify the desired orientation
        dock_control_request.rack_no = f"rack{self.package_id}"  # Specify the rack number

        future = self.docking_service_client.call_async(dock_control_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Docking service succeeded. Waiting for robot to come to rest.")
            while not self.docked:
                rclpy.spin_once(self)
                self.get_logger().info('Second docking is not complete. Keep waiting.')
            # self.send_place_rack_request()
            self.trigger_detachment_service_new()
        else:
            self.get_logger().error("Docking service failed.")
    

            self.get_logger().error("Docking service failed.")

    def trigger_attachment_service(self):
        self.get_logger().info("Triggering the attachment service")
        attachment_request = AttachLink.Request()
        attachment_request.model1_name = 'ebot'
        attachment_request.link1_name = 'ebot_base_link'
        attachment_request.model2_name = f"rack{self.package_id}"  # Replace with the actual rack name
        attachment_request.link2_name = 'link'

        attachment_future = self.link_attach_client.call_async(attachment_request)
        rclpy.spin_until_future_complete(self, attachment_future)

        if attachment_future.result() is not None:
            self.get_logger().info("Attachment service succeeded.")
            self.get_logger().info("Navigating to arm pose after successful attachment...")
            if self.package_id == 1 :
                self.navigate_to_mid_dock_pose()
            elif self.package_id  == 2 :
                self.send_place_rack_request(1)
                self.navigate_to_arm_pose()
            else : 
                self.navigate_to_arm_pose()
        else:
            self.get_logger().error("Attachment service failed.")

    def trigger_detachment_service(self):
        self.linear_timer.cancel()
        self.get_logger().info("Triggering the detachment service")
        detachment_request = DetachLink.Request()
        detachment_request.model1_name = 'ebot'
        detachment_request.link1_name = 'ebot_base_link'
        detachment_request.model2_name = f"rack{self.package_id}"  # Replace with the actual rack name
        detachment_request.link2_name = 'link'

        detachment_future = self.link_detach_client.call_async(detachment_request)
        # rclpy.spin_until_future_complete(self, detachment_future)

        if detachment_future.result() is not None:
            self.get_logger().info("detachment service succeeded.")
        else:
            self.get_logger().error("Attachment service failed.")
            self.get_logger().info("Navigating to home after successful detachment...")
            self.rack_place_operation_complete = True  # Call the new navigation method
            # self.send_place_rack_request()


    def trigger_detachment_service_new(self):
        self.get_logger().info("Triggering the detachment service")
        detachment_request = DetachLink.Request()
        detachment_request.model1_name = 'ebot'
        detachment_request.link1_name = 'ebot_base_link'
        detachment_request.model2_name = f"rack{self.package_id}"  # Replace with the actual rack name
        detachment_request.link2_name = 'link'

        detachment_future = self.link_detach_client.call_async(detachment_request)
        rclpy.spin_until_future_complete(self, detachment_future)

        if detachment_future.result() is not None:
            self.send_place_rack_request()
            self.get_logger().info("detachment service succeeded.")
            self.rack_place_operation_complete = True  # Call the new navigation method
        else:
            self.get_logger().error("Attachment service failed.")
            self.get_logger().info("Navigating to home after successful detachment...")

def main(args=None):
    script_dir = os.path.dirname(os.path.abspath(__file__))

    yaml_file_path = os.path.join(script_dir, 'config.yaml')

    with open(yaml_file_path, 'r') as file:
        config_params = yaml.safe_load(file)        

    rclpy.init(args=args)

    navigator_service = BasicNavigator()

    for rack_idx in range(3):

        ebot_nav2_cmd_node = NavigationAndDockingNode(config_params,rack_idx,navigator_service)

        ebot_nav2_cmd_node.navigate_and_dock()
        
        while not ebot_nav2_cmd_node.rack_place_operation_complete:

            rclpy.spin_once(ebot_nav2_cmd_node,timeout_sec=0.01)

        print("lund")

        ebot_nav2_cmd_node.destroy_node()
    
    ebot_final_home_node = NavigationAndDockingNode(config_params,0,navigator_service)
    ebot_final_home_node.navigate_to_home_pose()
    ebot_final_home_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()


    