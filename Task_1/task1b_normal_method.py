'''
# Team ID:          1868
# Theme:            Cosmo Logistic 
# Author List:      Tahsin Khan
# Filename:         task2b.py
# Functions:        
# Global variables: pose_1 , pose_2 , drop_pose , quat_1 , quat_2 , quat_3  , pose_box_1 , pose_box_2 , pose_box_3 , quat_box , collisoon_objects
'''
#!/usr/bin/env python3

#Importing required libraries for MoveIt 

from threading import Thread
from os import path
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

# Defining meshes for all the collison objects including a default one 

DEFAULT_EXAMPLE_MESH = path.join(
    path.dirname(path.realpath(__file__)), "assets", "suzzane.stl"
)

box_mesh = path.join(
    path.dirname(path.realpath(__file__)), "assets", "box.stl"
)

rack_mesh = path.join(
    path.dirname(path.realpath(__file__)), "assets", "rack.stl"
)
#Defining all the required poses and orientations 

pose_1 = [0.35, 0.10, 0.68]
quat_1 = [0.50,0.50,0.50,0.50]
quat_2 = [ 0.5, 0.5 ,-0.5 , -0.5 ]
quat_3 = [-0.5, 0.5, -0.5, -0.5]
drop_pose = [-0.37, 0.12, 0.397]
pose_2 =  [0.194, -0.43, 0.701]

pose_box_1 = [0.51, 0.00, 0.51]
pose_box_2 = [0.24 , -0.55 , 0.51]
pose_box_3 = [0.25, -0.55, 0.19]
quat_box = [0.0, 0.0, -0.707, 0.707]

pose_list = [
    [0.35, 0.10, 0.68],    # Pose 1
    [-0.37, 0.12, 0.397],  # Drop Pose
    [0.194, -0.43, 0.701], # Pose 2
    [-0.37, 0.12, 0.397]   # Another Drop Pose
]


collision_objects = [

    (box_mesh, "add", pose_box_1, quat_box,"box_1"),
    (box_mesh, "add", pose_box_2, quat_box ,"box_2"),
    (box_mesh ,"add", pose_box_3 , quat_box , "box_3")
    
]



def add_collision_objects(node, moveit2, collision_objects):
    for obj in collision_objects:
        filepath, action, position, quat_xyzw, obj_id = obj  # Extract obj_id

        # Use the default example mesh if invalid
        if not filepath:
            node.get_logger().info(f"Using the default example mesh file")
            filepath = DEFAULT_EXAMPLE_MESH

        # Make sure the mesh file exists
        if not path.exists(filepath):
            node.get_logger().error(f"File '{filepath}' does not exist")
            continue

        if "add" == action:
            # Add collision mesh with a unique ID
            node.get_logger().info(
                f"Adding collision mesh '{filepath}' with ID '{obj_id}' {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
            )
            moveit2.add_collision_mesh(
                filepath=filepath, id=obj_id, position=position, quat_xyzw=quat_xyzw, frame_id=ur5.base_link_name()
            )
        else:
            # Remove collision mesh by its unique ID
            node.get_logger().info(f"Removing collision mesh with ID '{obj_id}'")
            moveit2.remove_collision_mesh(id=obj_id)

def collison_object_move_pose(ur5, collision_objects):
    rclpy.init()

    # Create node for this example
    node = Node("ex_collision_object")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in a background thread
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

# Add collision objects

# Loop until collision objects are added successfully
    
    # Add collision objects using your function
    for i in range(1, 9):
           
        add_collision_objects(node,moveit2,collision_objects)

    # Declare parameters for position and orientation
    node.declare_parameter("position_1", pose_1)
    node.declare_parameter("position_drop",drop_pose)
    node.declare_parameter("position_2", pose_2)
    node.declare_parameter("quat_xyzw_1", quat_1)
    node.declare_parameter("quat_xyzw_2" , quat_2)
    node.declare_parameter("quat_xyzw_3",quat_3)
    node.declare_parameter("cartesian", False)

    # Get parameters
    position_1 = node.get_parameter("position_1").get_parameter_value().double_array_value
    position_drop = node.get_parameter("position_drop").get_parameter_value().double_array_value
    position_2 = node.get_parameter("position_2").get_parameter_value().double_array_value
    
    quat_xyzw_1 = node.get_parameter("quat_xyzw_1").get_parameter_value().double_array_value
    quat_xyzw_2 = node.get_parameter("quat_xyzw_2").get_parameter_value().double_array_value
    quat_xyzw_3 = node.get_parameter("quat_xyzw_3").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    
    # Move to pose_1
    
    node.get_logger().info(
            f"Moving to {{position: {list(position_1)}, quat_xyzw: {list(quat_xyzw_1)}}}"
        )
    moveit2.move_to_pose(position= position_1, quat_xyzw= quat_xyzw_1, cartesian= cartesian)

    moveit2.wait_until_executed()

    # Move to drop_pose 

    node.get_logger().info(
            f"Moving to {{position: {list(position_drop)}, quat_xyzw: {list(quat_xyzw_2)}}}"
        )

    moveit2.move_to_pose(position= position_drop, quat_xyzw= quat_xyzw_2, cartesian= cartesian)

    moveit2.wait_until_executed()

    # Move to pose_2

    node.get_logger().info(
            f"Moving to {{position: {list(position_2)}, quat_xyzw: {list(quat_xyzw_3)}}}"
        )

    moveit2.move_to_pose(position= position_2, quat_xyzw= quat_xyzw_3, cartesian= cartesian)

    moveit2.wait_until_executed()

    # Move to drop_pose 

    node.get_logger().info(
            f"Moving to {{position: {list(position_drop)}, quat_xyzw: {list(quat_xyzw_2)}}}"
        )

    moveit2.move_to_pose(position= position_drop, quat_xyzw= quat_xyzw_2, cartesian= cartesian)

    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":

    collison_object_move_pose(ur5,collision_objects)

   
    
    
    
  
