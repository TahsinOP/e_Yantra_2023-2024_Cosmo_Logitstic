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
from linkattacher_msgs.srv import AttachLink, DetachLink  
import math

# Team ID:          CL#1868
# Author List:		Tahsin Khan 
# Filename:		    ebot_nav2_cmd_task2b.py

# Functions:        normalize_angle,left_sensor_callback,right_sensor_callback,odometry_callback_for_trigger,vel_callback,navigate_to_home_pose,
#                   navigate_to_arm_pose,navigate_and_dock,trigger_docking_service_intial,trigger_docking_service_final,trigger_attachment_service,
#                   trigger_dettachment_service ,main
#			        
# Nodes:		  
#			        Publishing Topics  - 
#                   Subscribing Topics - [ cmd_vel,odom,/ultrasonic_rl/scan,/ultrasonic_rr/scan]

class NavigationAndDockingNode(Node):
    def __init__(self):
        super().__init__('navigation_and_docking_node')

        self.navigator = BasicNavigator()
        self.robot_pose = [0.0, 0.0, 0.0]
        self.docking_attempts = 0
        self.reached_2nd_dock_pose = False
        self.docked = False
        self.docking_service_client = self.create_client(DockSw, 'dock_control') 
        self.link_attach_client = self.create_client(AttachLink, '/ATTACH_LINK')
        self.link_detach_client = self.create_client(DetachLink, '/DETACH_LINK')
        self.velocity_subs = self.create_subscription(Twist,'cmd_vel',self.vel_callback ,10)
        self.odom_sub_for_trigger = self.create_subscription(Odometry, 'odom', self.odometry_callback_for_trigger, 10)
        self.left_sensor_subscription = self.create_subscription(
            Range,
            '/ultrasonic_rl/scan',  # Replace with the actual topic name
            self.left_sensor_callback,
            10)
        self.right_sensor_subscription = self.create_subscription(
            Range,
            '/ultrasonic_rr/scan',  # Replace with the actual topic name
            self.right_sensor_callback,
            10)

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
    
    def odometry_callback_for_trigger(self,msg): 

        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y


        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        
        self.robot_pose[2] = yaw 

    def vel_callback(self , msg ) : 
        
        self.vel_x = msg.linear.x 
        self.ang_z = msg.angular.z
             
        
    def navigate_to_home_pose(self):
        # Define the goal pose for the subsequent navigation
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.0 # Replace with your desired X coordinate
        goal_pose.pose.position.y = 0.0 # Replace with your desired Y coordinate

        # goal_pose.pose.orientation = Quaternion(z= 3.14, w=1.0)
        goal_pose.pose.orientation.z = 0.00# Replace with your desired orientation
        goal_pose.pose.orientation.w = 1.00 # Replace with your desired orientation

        # Navigate to the new goal pose
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation to the home pose succeeded.')
            
        else:
            self.get_logger().error('Navigation to the home pose failed.')

    def navigate_to_arm_pose(self):
        # Define the goal pose for the subsequent navigation
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.80 # Replace with your desired X coordinate
        goal_pose.pose.position.y = -2.455  # Replace with your desired Y coordinate

        # goal_pose.pose.orientation = Quaternion(z= 3.14, w=1.0)
        goal_pose.pose.orientation.z = -0.5774# Replace with your desired orientation
        goal_pose.pose.orientation.w = 0.8166 # Replace with your desired orientation

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
        # Define the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.40
        goal_pose.pose.position.y = 4.60
        goal_pose.pose.orientation.z = 0.90
        goal_pose.pose.orientation.w = 0.40

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
        dock_control_request.orientation = 3.14  # Specify the desired orientation
        dock_control_request.rack_no = 'rack1'  # Specify the rack number

        future = self.docking_service_client.call_async(dock_control_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Docking service succeeded. Waiting for robot to come to rest.")

            # Wait for the robot to come to rest
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
        dock_control_request.orientation = -3.14  # Specify the desired orientation
        dock_control_request.rack_no = 'rack1'  # Specify the rack number

        future = self.docking_service_client.call_async(dock_control_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Docking service succeeded. Waiting for robot to come to rest.")

            while not ((self.vel_x < 0.1) and (self.ang_z < 0.1) and (self.normalize_angle(self.robot_pose[2]) < - 3.12)):
               
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
        attachment_request.model2_name = 'rack1'  # Replace with the actual rack name
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
        detachment_request.model2_name = 'rack1'  # Replace with the actual rack name
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

    rclpy.init(args=args)

    ebot_nav2_cmd_node = NavigationAndDockingNode()
    
    ebot_nav2_cmd_node.navigate_and_dock()
     
    rclpy.spin(ebot_nav2_cmd_node)

    ebot_nav2_cmd_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':

    main()


    