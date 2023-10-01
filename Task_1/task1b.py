'''
# Team ID:          1868
# Theme:            Cosmo Logistic 
# Author List:      Tahsin Khan
# Filename:         task2b.py
# Functions:        
# Global variables: 
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
quat_1 = [0.50,0.50,0.50,0.50,0.50]
'''pose_2 = 
quat_3 = 
drop_pose = 
quat_drop_pose = '''

quat_box = [0.0, 0.0, -0.707, 0.707]

def collison_object_move_pose(pose_list1,quat_list1,mesh_name,pose_list,quat_list):

    rclpy.init()

    # Create node for this example
    node = Node("ex_collision_object")

    # Declare parameter for joint positions
    node.declare_parameter("filepath", mesh_name)
    
    node.declare_parameter(
        "action",
        "add",
    )
    node.declare_parameter("position", pose_list1)
    node.declare_parameter("quat_xyzw", quat_list1)

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

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    filepath = node.get_parameter("filepath").get_parameter_value().string_value
    action = node.get_parameter("action").get_parameter_value().string_value
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value

    # Use the default example mesh if invalid
    if not filepath:
        node.get_logger().info(f"Using the default example mesh file")
        filepath = DEFAULT_EXAMPLE_MESH

    # Make sure the mesh file exists
    if not path.exists(filepath):
        node.get_logger().error(f"File '{filepath}' does not exist")
        rclpy.shutdown()
        exit(1)

    # Determine ID of the collision mesh
    mesh_id = path.basename(filepath).split(".")[0]

    if "add" == action:
        # Add collision mesh
        node.get_logger().info(
            f"Adding collision mesh '{filepath}' {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        # print(ur5.base_link_name())
        moveit2.add_collision_mesh(
            filepath=filepath, id=mesh_id, position=position, quat_xyzw=quat_xyzw, frame_id=ur5.base_link_name()
        )
    else:
        # Remove collision mesh
        node.get_logger().info(f"Removing collision mesh with ID '{mesh_id}'")
        moveit2.remove_collision_mesh(id=mesh_id)

    
    # Create node for this example
    node = Node("Move_arm_to_given_pose")

    # Declare parameters for position and orientation
    node.declare_parameter("position", pose_list)
    node.declare_parameter("quat_xyzw", quat_list)
    node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    '''moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    #)'''

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

    rclpy.shutdown()


    exit(0)


    


if __name__ == "__main__":

    collison_object_move_pose([0.5, 0.0, 0.5],[0.0, 0.0, -0.707, 0.707],box_mesh,[0.35, 0.10, 0.68],[0.5,0.5,0.5,0.5,0.5])
