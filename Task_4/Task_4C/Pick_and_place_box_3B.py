'''
# Team ID:          1868
# Theme:            Cosmo Logistic 
# Author List:      Tahsin Khan , Chinmaya Sahu
# Filename:        2task2b.py
# Functions:        
# Global variables: 
'''
#!/usr/bin/env python3

from scipy.spatial.transform import Rotation as R
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TransformException, Buffer, TransformListener
from linkattacher_msgs.srv import AttachLink, DetachLink

from std_srvs.srv import Trigger 
from ebot_docking.srv import ArmBotSw

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

import math
from threading import Thread
import yaml
import os



class RackDetachmentHandler(Node):
    def __init__(self):
        super().__init__('rack_detachment_handler')

        self.callback_group = ReentrantCallbackGroup()

        self.rack_arm_service = self.create_service(ArmBotSw , 'rack_place' , self.rack_place_callback,callback_group= self.callback_group)

        self.boxes_placed = 0

    def rack_place_callback(self,request,response):

        obj_no = request.rack_id

        tf_listener_node = TFListener(obj_no)

        while not tf_listener_node.first_transform_received:

            rclpy.spin_once(tf_listener_node, timeout_sec=1.0)
        
        tf_listener_node.get_logger().info("Tf Listener Node has received the transforms")

        obj_name = f"obj_{obj_no}"

        post_pick = list(tf_listener_node.get_transform(obj_name))

        rot = list(tf_listener_node.get_rotation(obj_name))
        target_rot_euler = list(R.from_quat(rot).as_euler('xyz'))

        
        yaw = target_rot_euler[2]

        #We have to bring the yaw in range
        while yaw < -math.pi:
            yaw += 2*math.pi

        error = 0.05

        #We have to account for the different orientations of the boxes
        if abs(yaw - math.pi/2) < error:
            post_pick[0] -= 0.187
        elif abs(yaw - 0) < error:
            post_pick[1] += 0.187
        elif abs(yaw - math.pi) < error:
            post_pick[1] -= 0.187   #Right hand side while facing the racks
            

        target_poses = [tf_listener_node.get_transform(obj_name),tf_listener_node.get_transform(obj_name)] #First the arm will attach to the box, and bring it back
        target_rotations = [tf_listener_node.get_rotation(obj_name) for _ in range(2)]

        servo_node = ServoNode(target_poses,target_rotations,obj_no)

        while not servo_node.box_done :

            rclpy.spin_once(servo_node,timeout_sec=0.02)

        servo_node.get_logger().info("Shutting down Servo Node")
        
        servo_node.destroy_node() 

        self.boxes_placed = self.boxes_placed + 1

        response.success = True  # Set to actual success status

        return response


class TFListener(Node):
    def __init__(self,obj_no):
        super().__init__('tf_listener')

        self.obj_no = obj_no

        # Initialize TF2 Buffer and TransformListener
        self.tf_buffer = Buffer(Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Lists to store translations and rotations for the 3 boxes
        self.translations = {f"obj_{self.obj_no}": None}
        self.rotations = {f"obj_{self.obj_no}": None}

        # Create a timer to periodically check for transforms
        self.timer = self.create_timer(1.0, self.lookup_transforms)
        self.first_transform_received = False  # Flag to track the first transform

    def lookup_transforms(self):
        try:
            # Lookup transforms and store them in the lists
            transform1 = self.tf_buffer.lookup_transform("base_link", f"1868_obj_{self.obj_no}", Time().to_msg())

            # Extract and store translations
            self.translations[f"obj_{self.obj_no}"] = (transform1.transform.translation.x, transform1.transform.translation.y, transform1.transform.translation.z)

            # # Extract and store rotations
            self.rotations[f"obj_{self.obj_no}"] = (transform1.transform.rotation.x, transform1.transform.rotation.y, transform1.transform.rotation.z, transform1.transform.rotation.w)
            

            # Print or process the transforms as needed
            self.print_transforms()

            # Stop the timer after receiving the first transform
            if not self.first_transform_received:

                self.first_transform_received = True
                
                self.timer.reset()                

        except TransformException as e:
            self.get_logger().warning(f"Failed to lookup transforms: {e}")

    def print_transforms(self):
        # Print or process the stored transforms
        for obj_id in self.translations:

            self.get_logger().info(f"Transform for {obj_id}:")
            self.get_logger().info(f"Translation: {self.translations[obj_id]}")
            self.get_logger().info(f"Rotation: {self.rotations[obj_id]}")

    def get_transform(self,obj_name):
        return self.translations[obj_name]
    
    def get_rotation(self,obj_name):
        return self.rotations[obj_name]

class ServoNode(Node):
    def __init__(self, target_poses, target_rotations, obj_no):
        super().__init__(f'servo_node{obj_no}')

        self.target_poses = target_poses
        self.target_rotations = target_rotations
        self.obj_no = obj_no
        self.ids = [obj_no]

        self.distance_threshold = 0.005
        self.error = 0.05
        self.current_target_index = 0
        self.box_done = False

        #Convert degree to radians
        cons = (math.pi)/180
        self.yaw_right_box_pose = [joint_angle*cons for joint_angle in [-90, -137 , 138 , -180 , -90 , 180 ]] 
        self.yaw_left_box_pose = [joint_angle*cons for joint_angle in [90, -137 , 138 , -180 , -90 , 180 ]] 
        self.drop_pose = [-0.027, -1.803, -1.3658, -3.039, -1.52, 3.15]
        self.home_pose = [joint_angle*cons for joint_angle in [0.0, -137 , 138 , -180 , -90 , 180 ]] 

        self.move_it_controller = MoveMultipleJointPositions(obj_no)                                 
        self.tf_buffer = Buffer(Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        callback_group = ReentrantCallbackGroup()

        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

        self.start_servo_service()

        self.timer = self.create_timer(0.02, self.servo_to_target,callback_group)

    def start_servo_service(self):

        # Create a client for the start_servo service
        start_servo_client = self.create_client(Trigger, '/servo_node/start_servo')

        # Wait for the service to be available
        if not start_servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service '/servo_node/start_servo' not available. Is Servo properly configured?")
        else:
            # Create a request for the start_servo service
            request = Trigger.Request()

            # Call the start_servo service
            future = start_servo_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                if future.result().success:                   

                    self.get_logger().info("Servo controller started successfully.")
                else:
                    self.get_logger().error("Failed to start Servo controller: %s", future.result().message)
            else:
                self.get_logger().error("Service call failed. Unable to start Servo controller.")


    def stop_servo_service(self): 

        stop_servo_client = self.create_client( Trigger, "/servo_node/stop_servo")

        if not stop_servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service '/servo_node/stop_servo' not available. Is Servo properly configured?")
        else:
            # Create a request for the start_servo service
            request = Trigger.Request()

            # Call the start_servo service
            future = stop_servo_client.call_async(request)

            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:

                if future.result().success:
                    self.get_logger().info("Servo controller stopped successfully.")               

                else:
                    self.get_logger().error("Failed to stop Servo controller: %s", future.result().message)
            else:
                self.get_logger().error("Service call failed. Unable to stop Servo controller.")


    def servo_to_target(self):
        
        if self.current_target_index < len(self.target_poses):
            try:
                trans = self.tf_buffer.lookup_transform(ur5.base_link_name(), 'tool0' , Time().to_msg())
                print(f"name is {ur5.end_effector_name}")

                target_pose = self.target_poses[self.current_target_index]
                target_rotation = self.target_rotations[self.current_target_index]

                current_orientation = [ trans.transform.rotation.x,
                                        trans.transform.rotation.y,
                                        trans.transform.rotation.z,
                                        trans.transform.rotation.w]
                
                current_translation = [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ]
                
                target_rot_euler = list(R.from_quat(target_rotation).as_euler('xyz'))
                current_orientation_euler = list(R.from_quat(current_orientation).as_euler('xyz'))

                yaw = target_rot_euler[2] - current_orientation_euler[2]     

                while yaw < -math.pi:
                    yaw += 2*math.pi

                #To position accordingly for left and right side boxes
                if abs(yaw - math.pi/2) < self.error:
                    self.timer.cancel()
                    self.move_it_controller.move_to_a_joint_config(self.yaw_left_box_pose)
                    self.timer.reset()
                elif abs(yaw - -math.pi/2) < self.error:
                    self.timer.cancel()
                    self.move_it_controller.move_to_a_joint_config(self.yaw_right_box_pose)
                    self.timer.reset()


                diff = [target_pose[i] - current_translation[i] for i in range(3)]

                distance = (sum([diff[i] ** 2 for i in range(3)])) ** 0.5

                if distance < self.distance_threshold:
                    self.attach_link_service()
                    
                    if (self.current_target_index%2) == 1 :
                        #To move to post pick position
                        # self.move_it_controller.move_to_home_and_drop_pose_after_servoing()          
                        self.move_it_controller.move_to_a_joint_config(self.home_pose)
                        self.move_it_controller.move_to_a_joint_config(self.drop_pose)         
                        self.detach_link_service()
                        self.move_it_controller.move_to_a_joint_config(self.home_pose)
                        self.box_done = True
                        if (self.box_done):
                            self.timer.reset()       

                    self.current_target_index += 1
                    
                else:
                    scaling_factor = 1.45
                    twist_msg = TwistStamped()
                    twist_msg.header.stamp = self.get_clock().now().to_msg()
                    twist_msg.twist.linear.x = diff[0] * scaling_factor
                    twist_msg.twist.linear.y = diff[1] * scaling_factor
                    twist_msg.twist.linear.z = diff[2] * scaling_factor
                    self.twist_pub.publish(twist_msg)

            except (LookupException,ConnectivityException,ExtrapolationException):
                self.get_logger().error("Failed to lookup transform from base_link to ee_link.")

    def attach_link_service(self):

        attach_link_client = self.create_client(AttachLink, '/GripperMagnetON')

        while not attach_link_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('AttachLink service not available, waiting...')

        req = AttachLink.Request()
        req.model1_name = f"box{self.obj_no}"  # Specify the box name
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'

        # Call the AttachLink service
        future = attach_link_client.call_async(req)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("Attachment successful.")
            else:
                self.get_logger().error("Attachment failed: %s", future.result().message)

    def detach_link_service(self):

        detach_link_client = self.create_client(DetachLink, '/GripperMagnetOFF')

        while not  detach_link_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('detachLink service not available, waiting...')

        req = DetachLink.Request()
        req.model1_name = f"box{self.obj_no}"  # Specify the box name
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'

        # Call the AttachLink service
        future = detach_link_client.call_async(req)    

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("detachment successful.")
                self.detached = True
            else:
                self.get_logger().error("detachment failed: %s", future.result().message)

class MoveMultipleJointPositions(Node):

    def __init__(self,obj_no):
        super().__init__(f'move_multiple_joint_positions{obj_no}')
        self.moveit2 = None
        self.detached = False
        self.movit_done = False

    def move_to_multiple_joint_positions(self, *joint_positions):

        for i, positions in enumerate(joint_positions, start=1):
            param_name = f"joint_positions_{i}"
            self.declare_parameter(param_name, positions)

            joint_positions = self.get_parameter(param_name).get_parameter_value().double_array_value

            self.get_logger().info(f"Moving to {param_name}: {list(joint_positions)}")
            self.moveit2.move_to_configuration(joint_positions)
            self.moveit2.wait_until_executed()
    
    def move_to_a_joint_config(self,joint_position):

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup()
        )

        executor = MultiThreadedExecutor(1)
        executor.add_node(self)
        executor = Thread(target= executor.spin, daemon=True, args=())
        executor.start()
        self.moveit2.move_to_configuration(joint_position)
        self.moveit2.wait_until_executed()
    

    def move_to_home_and_drop_pose_after_servoing(self):

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup()
        )

        executor = MultiThreadedExecutor(1)
        executor.add_node(self)
        executor = Thread(target= executor.spin, daemon=True, args=())
        executor.start()
        
        cons = (math.pi)/180
        joint_positions_1 = [joint_angle*cons for joint_angle in [0.0,-169,52,-241,-90,180]]
        joint_positions_2 = [joint_angle*cons for joint_angle in [0.0, -137 , 138 ,-180 , -90 , 180]]

        # Move to multiple joint configurations
        self.move_to_multiple_joint_positions(joint_positions_2, joint_positions_1)

def main(args=None):
    
    # script_dir = os.path.dirname(os.path.abspath(__file__))

    # yaml_file_path = os.path.join(script_dir, 'config.yaml')

    # with open(yaml_file_path, 'r') as file:
    #     config_params = yaml.safe_load(file)

    rclpy.init(args=args)

    rack_detachment_handler_node = RackDetachmentHandler()

    while rack_detachment_handler_node.boxes_placed < 3:
        rclpy.spin_once(rack_detachment_handler_node,timeout_sec= 0.01)

    rclpy.shutdown()

    # for obj_no in [int(package_id) for package_id in config_params['package_id']]:
    #     tf_listener_node = TFListener(obj_no)

    #     while not tf_listener_node.first_transform_received:

    #         rclpy.spin_once(tf_listener_node, timeout_sec=1.0)
        
    #     tf_listener_node.get_logger().info("Tf Listener Node has received the transforms")

    #     obj_name = f"obj_{obj_no}"

    #     post_pick = list(tf_listener_node.get_transform(obj_name))

    #     rot = list(tf_listener_node.get_rotation(obj_name))
    #     target_rot_euler = list(R.from_quat(rot).as_euler('xyz'))

        
    #     yaw = target_rot_euler[2]

    #     #We have to bring the yaw in range
    #     while yaw < -math.pi:
    #         yaw += 2*math.pi

    #     error = 0.05

    #     #We have to account for the different orientations of the boxes
    #     if abs(yaw - math.pi/2) < error:
    #         post_pick[0] -= 0.187
    #     elif abs(yaw - 0) < error:
    #         post_pick[1] += 0.187
    #     elif abs(yaw - math.pi) < error:
    #         post_pick[1] -= 0.187   #Right hand side while facing the racks
            

    #     target_poses = [tf_listener_node.get_transform(obj_name),post_pick] #First the arm will attach to the box, and bring it back
    #     target_rotations = [tf_listener_node.get_rotation(obj_name) for _ in range(2)]

    #     servo_node = ServoNode(target_poses,target_rotations,obj_no)

    #     while not servo_node.box_done :

    #         rclpy.spin_once(servo_node,timeout_sec=0.02)

    #     servo_node.get_logger().info("Shutting down Servo Node")
        
    #     servo_node.destroy_node() 

    # print('Pick and place operation completed')


    # rclpy.shutdown()

if __name__ == '__main__':
    main()
