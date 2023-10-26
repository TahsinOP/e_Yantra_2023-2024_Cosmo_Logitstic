#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from ebot_docking.srv import DockSw
from std_msgs.msg import String
import math

class EbotNav2CmdNode(Node):
    def __init__(self):
        super().__init__('ebot_nav2_cmd_node')
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.docking_service_client = self.create_client(DockSw, 'dock_control')
        self.pre_dock_pose = PoseStamped()
        self.pre_dock_pose.header.frame_id = 'map'  # Set the frame ID
        self.pre_dock_pose.pose.position.x = 0.5 # Set the X coordinate for pre-dock pose
        self.pre_dock_pose.pose.position.y = 4.55 # Set the Y coordinate for pre-dock pose
        self.pre_dock_pose.pose.orientation.z = 0.900
        self.pre_dock_pose.pose.orientation.w = 0.400 # Assuming no rotation
        self.pre_dock_pose_goal = NavigateToPose.Goal()
        self.pre_dock_pose_goal.pose = self.pre_dock_pose

    

    def navigate_to_pre_dock_pose(self):
        
        self.get_logger().info("Navigating to pre-dock pose...")
        self.nav_action_client.wait_for_server()
        future = self.nav_action_client.send_goal_async(self.pre_dock_pose_goal)
        rclpy.spin_until_future_complete(self, future)

        
        

        
       
    def trigger_docking_service(self):
        self.get_logger().info("Triggering the docking service ")
        dock_control_request = DockSw.Request()
        dock_control_request.linear_dock = True  # Enable linear correction
        dock_control_request.orientation_dock = True  # Enable angular correction
        dock_control_request.distance = 0.0 # Specify the desired distance
        dock_control_request.orientation = math.pi  # Specify the desired orientation
        dock_control_request.rack_no = 'rack_1'  # Specify the rack number

        future = self.docking_service_client.call_async(dock_control_request)
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    ebot_nav2_cmd_node = EbotNav2CmdNode()
    
    ebot_nav2_cmd_node.navigate_to_pre_dock_pose()
    
    ebot_nav2_cmd_node.trigger_docking_service()
    
    rclpy.spin(ebot_nav2_cmd_node)
    
    ebot_nav2_cmd_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
