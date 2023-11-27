#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from ebot_docking.srv import DockSw
from std_msgs.msg import String
from linkattacher_msgs.srv import AttachLink, DetachLink  
import math

class EbotNav2CmdNode(Node):
    def __init__(self):
        super().__init__('ebot_nav2_cmd_node')
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.docking_service_client = self.create_client(DockSw, 'dock_control')

        self.attach_link_cli = self.create_client(AttachLink, '/ATTACH_LINK')  # AttachLink service client
        self.detach_link_cli = self.create_client(DetachLink, '/DETACH_LINK')  # DetachLink service client

        self.reached_pre_dock = False
        
    

    def navigate_to_pre_dock_pose(self):

        self.pre_dock_pose = PoseStamped()
        self.pre_dock_pose.header.frame_id = 'map'  # Set the frame ID
        self.pre_dock_pose.pose.position.x = 0.4 # Set the X coordinate for pre-dock pose
        self.pre_dock_pose.pose.position.y = 4.60 # Set the Y coordinate for pre-dock pose
        self.pre_dock_pose.pose.orientation.z = 0.900
        self.pre_dock_pose.pose.orientation.w = 0.400 # Assuming no rotation
        self.pre_dock_pose_goal = NavigateToPose.Goal()
        self.pre_dock_pose_goal.pose = self.pre_dock_pose
        
        self.get_logger().info("Navigating to pre-dock pose...")
        self.nav_action_client.wait_for_server()
        future = self.nav_action_client.send_goal_async(self.pre_dock_pose_goal,feedback_callback= self.nav_feedback_callback)
        rclpy.spin_until_future_complete(self, future)

        # if future.result() is not None:
        #    result = future.result()

        #    if result.status == 3:
               
        #        self.reached_pre_dock = True
               
        #        self.get_logger().info("Reached the pre-dock pose.")
        #    else:
                
        #         self.get_logger().info("Navigation to pre-dock pose failed.")
        # else:
        #    self.get_logger().info("Navigation to pre-dock pose was not completed.")

        
    # def check_navigation_status(self):
    #     while rclpy.ok():
    #         result = self.nav_action_client.wait_for_server()
    #         if result:
    #             if result.status == 3:  # 3 indicates that the goal was reached
    #                 self.get_logger().info("Robot reached the pre-docking pose.")
    #                 return True
    #             else:
    #                 self.get_logger().error("Navigation to pre-docking pose failed.")
    #                 return False
        

        
       
    def trigger_docking_service(self):


        self.get_logger().info("Triggering the docking service ")
        dock_control_request = DockSw.Request()
        dock_control_request.linear_dock = True  # Enable linear correction
        dock_control_request.orientation_dock = True  # Enable angular correction
        dock_control_request.distance = 0.0 # Specify the desired distance
        dock_control_request.orientation = 3.14  # Specify the desired orientation
        dock_control_request.rack_no = 'rack1'  # Specify the rack number

        future = self.docking_service_client.call_async(dock_control_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.docking_complete = True
            self.get_logger().info("Docking complete.")

    def nav_feedback_callback(self, feedback_msg):
        # You can monitor the feedback from the navigation action here
        # For example, check if the robot has reached the goal pose
        if feedback_msg.navigation_result.code == 3:  # 3 indicates that the goal was reached
            self.get_logger().info("Robot reached the pre-docking pose.")
            self.trigger_docking_service()


    




    def attach_rack(self):

    
        self.get_logger().info("Attaching the rack")
        attach_link_request = AttachLink.Request()
        attach_link_request.model1_name = 'ebot'
        attach_link_request.link1_name = 'ebot_base_link'
        attach_link_request.model2_name = 'rack1'
        attach_link_request.link2_name = 'link'

        future = self.attach_link_cli.call_async(attach_link_request)
        rclpy.spin_until_future_complete(self, future)
            
        
        


    def detach_rack(self):

        self.get_logger().info("Detaching the rack")
        detach_link_request = DetachLink.Request()
        detach_link_request.model1_name = 'ebot'
        detach_link_request.link1_name = 'ebot_base_link'
        detach_link_request.model2_name = 'rack_1'  # Specify the rack model name
        detach_link_request.link2_name = 'link'  # Specify the link name on the rack

        future = self.detach_link_cli.call_async(detach_link_request)
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    ebot_nav2_cmd_node = EbotNav2CmdNode()
    
    ebot_nav2_cmd_node.navigate_to_pre_dock_pose()

  
    
    rclpy.spin(ebot_nav2_cmd_node)

    ebot_nav2_cmd_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
