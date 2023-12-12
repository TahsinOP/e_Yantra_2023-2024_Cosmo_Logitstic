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

class TFListener(Node):
    def __init__(self,obj_no):
        super().__init__('tf_listener')

        self.obj_no = obj_no

        # Initialize TF2 Buffer and TransformListener
        self.tf_buffer = Buffer(Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Lists to store translations and rotations for the 3 boxes
        self.translations = {f"obj_{self.obj_no}": None, "obj_3": None, "obj_49": None}
        self.rotations = {f"obj_{self.obj_no}": None, "obj_3": None, "obj_49": None}

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

class ServoNode(Node):

    def __init__(self, target_poses, target_rotations, target_obj_name, obj_no):
        super().__init__('servo_node')

        self.obj_no = obj_no

        self.move_it_controller = MoveMultipleJointPositions()                                 
        self.tf_buffer = Buffer(Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        callback_group = ReentrantCallbackGroup()

        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

        self.target_poses = target_poses
        self.target_rotations = target_rotations

        self.start_servo_service()

        print(f"target pose is {self.target_poses}")

        self.current_target_index = 0

        self.ids = [int(target_obj_name.split('_')[1])]
        
        self.distance_threshold = 0.01

        self.box_done = False

        self.cons = (math.pi)/180

        self.yaw_right_box_pose = [
            -90*(math.pi/180), -137 * (math.pi / 180), 138 * (math.pi / 180),
            -180 * (math.pi / 180), -90 * (math.pi / 180), 180 * (math.pi / 180)]

        self.yaw_left_box_pose = [
            90*(math.pi/180), -137 * (math.pi / 180), 138 * (math.pi / 180),
            -180 * (math.pi / 180), -90 * (math.pi / 180), 180 * (math.pi / 180)
        ]

        self.home_pose = [
            0.0, -137 * (math.pi / 180), 138* (math.pi / 180),
            -180 * (math.pi / 180), -90 * (math.pi / 180), 180 * (math.pi / 180)]
        
        
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
                      
                    self.move_it_controller.move_to_home_and_drop_pose_after_servoing()                   

                else:
                    self.get_logger().error("Failed to stop Servo controller: %s", future.result().message)
            else:
                self.get_logger().error("Service call failed. Unable to stop Servo controller.")

 
    def servo_to_target(self):
        
        if self.current_target_index < len(self.target_poses):
            try:
                trans = self.tf_buffer.lookup_transform(ur5.base_link_name(), ur5.end_effector_name(), Time().to_msg())

                target_pose = self.target_poses[self.current_target_index]
                target_rotation = self.target_rotations[self.current_target_index]

                current_orientation = [ trans.transform.rotation.x,
                        trans.transform.rotation.y,
                        trans.transform.rotation.z, trans.transform.rotation.w]
                
                target_rot_euler = list(R.from_quat(target_rotation).as_euler('xyz'))
                current_orientation_euler = list(R.from_quat(current_orientation).as_euler('xyz'))

                yaw = target_rot_euler[2] - current_orientation_euler[2]     

                while yaw < -math.pi:
                    yaw += 2*math.pi

                error = 0.05

                if abs(yaw - math.pi/2) < error:
                    
                    self.move_it_controller.move_to_a_joint_config(self.yaw_left_box_pose)

                elif abs(yaw - -math.pi/2) < error:

                    self.move_it_controller.move_to_a_joint_config(self.yaw_right_box_pose)
                               
                
                current_pose = [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ]
                   
                diff = [target_pose[i] - current_pose[i] for i in range(3)]

                distance = (sum([diff[i] ** 2 for i in range(3)])) ** 0.5

                if distance < self.distance_threshold:
                    self.attaching = True

                    self.attach_link_service()
                    
                    if (self.current_target_index%2) == 1 :
     
                        self.move_to_post_pick_pose()                     

                        self.detach_link_service()

                        self.move_it_controller.move_to_a_joint_config(self.home_pose)

                        self.box_done = True

                        print ( self.box_done) 

                        if (self.box_done):

                            self.timer.reset()

                            print("timer has been shut down")           
                       
                    self.current_target_index += 1

                    print(f"Current taget index increased{self.current_target_index}")
                    
                else:
                    scaling_factor = 0.8
                    twist_msg = TwistStamped()
                    twist_msg.header.stamp = self.get_clock().now().to_msg()
                    for i in range(3):
                        twist_msg.twist.linear.x = diff[0] * scaling_factor
                        twist_msg.twist.linear.y = diff[1] * scaling_factor
                        twist_msg.twist.linear.z = diff[2] * scaling_factor
                    self.twist_pub.publish(twist_msg)
            except (LookupException,ConnectivityException,ExtrapolationException):
                self.get_logger().error("Failed to lookup transform from base_link to ee_link.")

    def move_to_post_pick_pose(self):

        self.move_it_controller.move_to_home_and_drop_pose_after_servoing()


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
        else:
            self.get_logger().info("gg")

        
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
        else:
            self.get_logger().info("gg")
       
        
class MoveMultipleJointPositions(Node):

    def __init__(self):
        super().__init__('move_multiple_joint_positions')
        self.cons = math.pi / 180
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
        

        joint_positions_1 = [
            0.0,
            -169 * self.cons,
            52 * self.cons,
            -241 * self.cons,
            -90 * self.cons,
            180 * self.cons
        ]

        joint_positions_2 = [
            0.0, -137 * (math.pi / 180), 138 * (math.pi / 180),
            -180 * (math.pi / 180), -90 * (math.pi / 180), 180 * (math.pi / 180)
        ]

        # Move to multiple joint configurations
        self.move_to_multiple_joint_positions(joint_positions_2, joint_positions_1)



def main(args=None):

    f = open('config.yaml')
    config_params = yaml.load(f, Loader=yaml.FullLoader)        
    # obj_no = int(config_params['package_id'][0])

    rclpy.init(args=args)

    for obj_no in [int(package_id) for package_id in config_params['package_id']]:
        tf_listener_node = TFListener(obj_no)

        while not tf_listener_node.first_transform_received:

            rclpy.spin_once(tf_listener_node, timeout_sec=1.0)
        
        tf_listener_node.get_logger().info("Tf Listener Node has received the transforms")

        obj_name = f"obj_{obj_no}"
   

    # for obj_name in [f"obj_{obj_no}"]:

        test = list(tf_listener_node.translations[obj_name])
        rot = list(tf_listener_node.rotations[obj_name])

        target_rot_euler = list(R.from_quat(rot).as_euler('xyz'))

        yaw = target_rot_euler[2]

        print(yaw)

        while yaw < -math.pi:
            yaw += 2*math.pi

        error = 0.05

        if abs(yaw - math.pi/2) < error:
            test[0] -= 0.18

        elif abs(yaw - 0) < error:
            test[1] += 0.18
        elif abs(yaw - math.pi) < error:
            test[1] -= 0.18     #Right hand side while facing the racks
            

        target_poses = [tf_listener_node.translations[obj_name],test
                    ]
        target_rotations = [tf_listener_node.rotations[obj_name],tf_listener_node.rotations[obj_name]
                        ]

        servo_node = ServoNode(target_poses,target_rotations,obj_name,obj_no)

        
        while not servo_node.box_done :

            rclpy.spin_once(servo_node,timeout_sec=0.02)

        servo_node.get_logger().info("servo Node is bieng shut down gg")
        
        servo_node.destroy_node() 

    print('Pick and place operation completed')


    rclpy.shutdown()

if __name__ == '__main__':
    main()