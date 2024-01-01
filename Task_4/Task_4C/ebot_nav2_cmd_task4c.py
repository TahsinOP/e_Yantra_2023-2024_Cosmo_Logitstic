#! /usr/bin/env python3


from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range 
from ebot_docking.srv import DockSw
from tf_transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from linkattacher_msgs.srv import AttachLink, DetachLink  
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
    def __init__(self, config_params):
        super().__init__('navigation_and_docking_node')

        self.navigator = BasicNavigator()
        self.robot_pose = [0.0, 0.0, 0.0]
        self.docking_attempts = 0
        self.rack_place_operation_complete = False
        self.docked = False                 # Flag to determine if docking is completed or not 
        self.target_angle_rack_3  = 3.14
        self.target_angle_rack_1 = -3.14
        self.dock_service_error = 0.02
        self.pre_dock_correction_factors_rack1 = [-0.57,-0.86,0.16]
        self.pre_dock_correction_factors_rack3 = [-0.57,0.23,0.69]
        self.pre_dock_correction_factors_rack2 = []
        self.arm_pose_correction_factors_rack1 = []
        self.arm_pose_correction_factors_rack3 = []
        self.arm_pose_correction_factors_rack2 = []
        self.docking_service_client = self.create_client(DockSw, 'dock_control') 
        self.link_attach_client = self.create_client(AttachLink, '/ATTACH_LINK')
        self.link_detach_client = self.create_client(DetachLink, '/DETACH_LINK')
        self.odom_sub_for_trigger = self.create_subscription(Odometry, 'odom', self.odometry_callback_, 10)
        self.left_sensor_subscription = self.create_subscription(Range,'/ultrasonic_rl/scan', self.left_sensor_callback,10)
        self.right_sensor_subscription = self.create_subscription( Range,'/ultrasonic_rr/scan', self.right_sensor_callback,10)

        self.error_timer = self.create_timer(0.2,self.docking_error_control_loop)

        # Retrieving package id and its poses from the config.yaml file 

        self.config_params = config_params
        self.package_id = self.config_params['package_id'][0]
        self.rack_info = self.config_params['position'][int(self.package_id)-1][f"rack{self.package_id}"]
        self.rack_pose_x = self.rack_info[0]
        self.rack_pose_y = self.rack_info[1]
        self.rack_orientation = self.rack_info[2]

        self.get_logger().info(f"The package id is {self.package_id}")
        self.get_logger().info(f"The required rack information is {self.rack_info}")

    def docking_error_control_loop(self):

        if self.package_id == 1 : 
            self.target_angle = self.target_angle_rack_1
        
        elif self.package_id == 3 :
            self.target_angle = self.target_angle_rack_3

        dock_error =  self.normalize_angle(self.target_angle - self.robot_pose[2])

        if abs(dock_error) < self.dock_service_error :
            self.docked = True 
        
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
        if self.package_id == 3 :         # Condition to check the packages ids and allot its respective post-docking angles 
            self.arm_orientation = 2.291  
            

        elif self.package_id == 1:
            self.arm_orientation = -1.4597
            self.arm_pose_x = 0.85
            self.arm_pose_y = -2.30

        pose_euler = [0,0,0]
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
        if self.package_id == 3 : 
            self.corrected_rack_orientation = self.rack_orientation +self.pre_dock_correction_factors_rack3[0]      # Correction factors for pre-dock poses
            self.corrected_rack_pose_x = self.rack_pose_x+self.pre_dock_correction_factors_rack3[1]
            self.corrected_rack_pose_y = self.rack_pose_y+self.pre_dock_correction_factors_rack3[2]

            print(self.rack_orientation)

        elif self.package_id == 1:
            self.corrected_rack_orientation = self.rack_orientation     # Correction factors for pre-dock poses
            self.corrected_rack_pose_x = self.rack_pose_x+self.pre_dock_correction_factors_rack1[1]
            self.corrected_rack_pose_y = self.rack_pose_y+self.pre_dock_correction_factors_rack1[2]

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
            self.trigger_detachment_service()
        else:
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
            self.navigate_to_arm_pose()  # Call the new navigation method
        else:
            self.get_logger().error("Attachment service failed.")

    def trigger_detachment_service(self):
        self.get_logger().info("Triggering the detachment service")
        detachment_request = DetachLink.Request()
        detachment_request.model1_name = 'ebot'
        detachment_request.link1_name = 'ebot_base_link'
        detachment_request.model2_name = f"rack{self.package_id}"  # Replace with the actual rack name
        detachment_request.link2_name = 'link'

        detachment_future = self.link_detach_client.call_async(detachment_request)
        rclpy.spin_until_future_complete(self, detachment_future)

        if detachment_future.result() is not None:
            self.get_logger().info("detachment service succeeded.")
            self.get_logger().info("Navigating to home after successful detachment...")
            self.navigate_to_home_pose()  # Call the new navigation method
        else:
            self.get_logger().error("Attachment service failed.")

def main(args=None):
    script_dir = os.path.dirname(os.path.abspath(__file__))

    yaml_file_path = os.path.join(script_dir, 'config.yaml')

    with open(yaml_file_path, 'r') as file:
        config_params = yaml.safe_load(file)        

    rclpy.init(args=args)

    ebot_nav2_cmd_node = NavigationAndDockingNode(config_params)
    
    ebot_nav2_cmd_node.navigate_and_dock()
    
    while not ebot_nav2_cmd_node.rack_place_operation_complete:

        rclpy.spin(ebot_nav2_cmd_node)

    ebot_nav2_cmd_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':

    main()


    