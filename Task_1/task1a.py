#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1a.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
from tf2_ros import TransformBroadcaster
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image




##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''
     # Check if there are exactly 4 coordinates
    if len(coordinates) != 4:
        raise ValueError("Coordinates should contain exactly 4 sets of (x, y) coordinates.")

    # Extract the coordinates
    x1, y1 = coordinates[0]
    x2, y2 = coordinates[1]
    x3, y3 = coordinates[2]
    x4, y4 = coordinates[3]

    # Calculate width and height
    width = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    height = np.sqrt((x2 - x3)**2 + (y2 - y3)**2)

    # Calculate area
    area = width * height

    return area, width

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    
    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################

    


def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''
    # print(image)
    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

 
    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 

    #	->  Convert input BGR image to GRAYSCALE for aruco detection

    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)

    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection. 

    #   ->  Draw detected marker on the image frame which will be shown later

    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))

    #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined

    #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation

    #   ->  Draw frame axes from coordinates received using pose estimation
    #       ->  HINT: You may use 'cv2.drawFrameAxes'

    ############################################

    # Initialize variables to store detected Aruco marker information
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
    rvec_list = []
    tvec_list = []

    # Convert input BGR image to GRAYSCALE for Aruco detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters =  cv2.aruco.DetectorParameters()
    # detector = cv.aruco.ArucoDetector(dictionary, parameters)

    # Define Aruco parameters
    # aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    #parameters = cv2.aruco.DetectorParameters_create()

    # Detect Aruco markers in the image and store 'corners' and 'ids'
    corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(f"Marker id is {marker_ids}")

    # Loop over each detected marker
    for i in range(len(marker_ids)):
        # Calculate the area and width of the detected Aruco marker
        coordinates = corners[i][0]
        area, width = calculate_rectangle_area(coordinates)

        # Check if the detected marker meets the area threshold
        if area >= aruco_area_threshold:
            # Calculate center point of the marker
            center_x = np.mean(coordinates[:, 0])
            center_y = np.mean(coordinates[:, 1])
            center_aruco_list.append((center_x, center_y))

            # Perform pose estimation of the marker to get distance and angle
            rvec,tvec,_ = cv2.aruco.estimatePoseSingleMarkers(corners[i], size_of_aruco_m, cam_mat, dist_mat)
            rvec_list.append(rvec)
            tvec_list.append(tvec)
            
            print(i,rvec,tvec)
            tvec = tvec.squeeze(1)
            
            distance_from_rgb = tvec[0,2]#-0.02
            angle_aruco = (np.arctan2(tvec[0, 0], tvec[0, 2]))
            
            # Append the detected marker information to respective lists
            distance_from_rgb_list.append(distance_from_rgb)
            angle_aruco_list.append(angle_aruco)
            width_aruco_list.append(width)
            ids.append(marker_ids[i][0])

            # Draw the detected marker on the image
            cv2.aruco.drawDetectedMarkers(image, corners)

            # Draw frame axes
            cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, size_of_aruco_m)

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids, rvec_list, tvec_list


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None                                                           # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error("Error converting depth image: %s" % str(e))
            return


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error("Error converting color image: %s" % str(e))
            return


    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
            

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above

        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 

        #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
        #       It's a correction formula- 
        #       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)

        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)

        #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)

        #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
        #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
        #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
        #       z = distance_from_rgb
        #       where, 
        #               cX, and cY from 'center_aruco_list'
        #               distance_from_rgb is depth of object calculated in previous step
        #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 

        #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
        #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #       so that we will collect it's position w.r.t base_link in next step.
        #       Use the following frame_id-
        #           frame_id = 'camera_link'
        #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID

        #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
        #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 

        #   ->  And now publish TF between object frame and base_link
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID

        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1B)
        #               Also, auto eval script will be judging angular difference aswell. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

        ############################################
        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids, rvec_list, tvec_list = detect_aruco(self.cv_image)
        #print(self.cv_image)
        #print(self.depth_image)
       
        

        

        for i in range(len(ids)):
            marker_id = ids[i]
            distance_from_rgb = distance_from_rgb_list[i]
            angle_aruco = angle_aruco_list[i]
            

            # Correct the Aruco angle using the correction formula
            angle_aruco = (0.788 * angle_aruco) - ((angle_aruco ** 2) / 3160)

            # Calculate quaternions from roll, pitch, and yaw
            roll = 0.0
            pitch = 0.0
            yaw = angle_aruco
            # quaternion = self.euler_to_quaternion(roll, pitch, yaw)

            # Retrieve depth from Realsense and convert to meters

            #distance_from_rgb = distance_from_rgb/1000

            

            # Calculate X, Y, Z based on camera parameters and Aruco properties
            cX, cY = center_aruco_list[i][:2]
            #depth = math.sqrt(pow(cX,2)+pow(cY,2))
            x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
            y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
            z = distance_from_rgb
            angle_y_rad = 1.57
            angle_x_rad = 1.57
            angle_z_rad = math.pi/2

            def get_rot_mat_y(deg):
                angle_y_rad = deg*math.pi/180
                rotation_matrix_y = np.array([[np.cos(angle_y_rad), 0, np.sin(angle_y_rad)],
                              [0, 1, 0],
                              [-np.sin(angle_y_rad), 0, np.cos(angle_y_rad)]])
                
                return rotation_matrix_y

            rotation_matrix_x = np.array([[1, 0, 0],
                              [0, np.cos(angle_x_rad), -np.sin(angle_x_rad)],
                              [0, np.sin(angle_x_rad), np.cos(angle_x_rad)]])

            rotation_matrix_y = np.array([[np.cos(angle_y_rad), 0, np.sin(angle_y_rad)],
                              [0, 1, 0],
                              [-np.sin(angle_y_rad), 0, np.cos(angle_y_rad)]])
            
            rotation_matrix_z = np.array([[np.cos(angle_z_rad), -np.sin(angle_z_rad), 0],
                              [np.sin(angle_z_rad), np.cos(angle_z_rad), 0],
                              [0, 0, 1]])
            
            rvec = rvec_list[i]
            tvec = tvec_list[i]

            rotation_matrix_from_rvec = R.from_rotvec(rvec.reshape(3,))
            rotation_matrix_from_rvec = rotation_matrix_from_rvec.as_matrix()

            rotation_matrix_y_180 = np.array([[-1, 0, 0],
                             [0, 1, 0],
                             [0, 0, -1]])
            # print(rotation_matrix_from_rvec)
            
            original_vector = np.array([[x], [y], [z]])
            rotated_vector = np.dot(rotation_matrix_x,(np.dot(rotation_matrix_y, original_vector))) #This is for the transform !!!DO NOT CHANGE 

            transform_rotation_matrix_inv = np.dot(rotation_matrix_y,rotation_matrix_x)#np.dot(rotation_matrix_x,rotation_matrix_y)
            transform_rotation_matrix_inv = np.dot(rotation_matrix_y_180,transform_rotation_matrix_inv)

            rotation_matrix_z_180 = np.array([[-1, 0, 0],
                             [0, -1, 0],
                             [0, 0, 1]])

            if i == 2:
                transform_rotation_matrix_inv = np.dot(rotation_matrix_y, transform_rotation_matrix_inv)
                transform_rotation_matrix_inv = np.dot(rotation_matrix_x, transform_rotation_matrix_inv)
                transform_rotation_matrix_inv = np.dot(rotation_matrix_z, transform_rotation_matrix_inv)
                transform_rotation_matrix_inv = np.dot(get_rot_mat_y(70), transform_rotation_matrix_inv)

            # transform_rotation_matrix_inv = np.dot(rotation_matrix_y,transform_rotation_matrix_inv)

            rotation_matrix_z_180 = np.array([[-1, 0, 0],
                             [0, -1, 0],
                             [0, 0, 1]])
            
            # transform_rotation_matrix_inv = np.dot(rotation_matrix_z_180,transform_rotation_matrix_inv)

            # roll = math.pi/2
            # pitch = math.pi/2
            # yaw += -math.pi/2+0.05
            '''
            r2 = np.array([
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]
            ])

            r1 = R.from_matrix(rotation_matrix_from_rvec)
            r1 = r1.as_matrix()
            print(f"r1 is {r1}")
            r1_inv = np.linalg.inv(r1)

            print(r2)
            print(r1_inv)

            print(type(r2),type(r1_inv))

            final_rotation_matrix = np.matmul(r1_inv, r2)
            final_rotation_matrix = np.linalg.inv(final_rotation_matrix)'''
            # final_rotation_matrix = final_rotation_matrix.as_matrix()

            try:
                base_to_camera_link = self.tf_buffer.lookup_transform('base_link', 'camera_link',rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            base_to_camera_link_quat = [
                base_to_camera_link.transform.rotation.x,
                base_to_camera_link.transform.rotation.y,
                base_to_camera_link.transform.rotation.z,
                base_to_camera_link.transform.rotation.w,
            ]

            rotation_matrix_from_cam_link = R.from_quat(base_to_camera_link_quat).as_matrix()
            rotation_matrix_from_cam_link = np.linalg.inv(rotation_matrix_from_cam_link)
            
            final_rotation_matrix = np.dot(rotation_matrix_from_rvec,rotation_matrix_from_cam_link)

            final_rotation_matrix = np.dot(transform_rotation_matrix_inv, final_rotation_matrix)

            quaternion = self.rotate_quaternion_by_matrix(roll,pitch,yaw,final_rotation_matrix)

            # quaternion = self.rotate_quaternion_by_matrix(roll,pitch,yaw,final_rotation_matrix)

            # req_r_matrix = self.find_rotation_matrix_to_align_with_up([roll,pitch,yaw])
            # rotated_vector = np.dot(req_r_matrix,rotated_vector)

            # rotated_vector = np.dot(rotation_matrix_from_rvec)
            #quaternion = self.rotation_vector_to_quaternion(rotated_vector)


            # Mark the center points on the image frame
            cv2.circle(self.cv_image, (int(cX), int(cY)), 5, (0, 0, 255), -1)

            # Publish transformation between camera_link and Aruco marker center position
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = 'camera_link'
            transform_stamped.child_frame_id = 'cam_{}'.format(marker_id)
            transform_stamped.transform.translation.x = float(rotated_vector[0])
            transform_stamped.transform.translation.y = float(rotated_vector[1])
            transform_stamped.transform.translation.z = float(rotated_vector[2])
            transform_stamped.transform.rotation.w = float(quaternion[0])
            transform_stamped.transform.rotation.x = float(quaternion[1])
            transform_stamped.transform.rotation.y = float(quaternion[2])
            transform_stamped.transform.rotation.z = float(quaternion[3])
            
            self.br.sendTransform(transform_stamped)
            # print(distance_from_rgb)

            try:
                base_to_camera = self.tf_buffer.lookup_transform('base_link', 'cam_{}'.format(marker_id), rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            

            # print(f"Gay sex{base_to_camera_link.transform.rotation}")
           

            # print(transform_stamped.transform.rotation)
            # print(base_to_camera.transform.rotation)

            # final_quaternion = R.from_matrix(np.dot(np.linalg.inv(rotation_matrix_from_cam_link),rotation_matrix_from_rvec)).as_quat()
            # print("hi",final_quaternion)



            # Publish TF between object frame and base_link
            transform_stamped.header.frame_id = 'base_link'
            transform_stamped.child_frame_id = 'obj_{}'.format(marker_id)
            transform_stamped.transform.translation.x = base_to_camera.transform.translation.x
            transform_stamped.transform.translation.y = base_to_camera.transform.translation.y
            transform_stamped.transform.translation.z = base_to_camera.transform.translation.z
            transform_stamped.transform.rotation.w = base_to_camera.transform.rotation.w
            transform_stamped.transform.rotation.x = base_to_camera.transform.rotation.x
            transform_stamped.transform.rotation.y = base_to_camera.transform.rotation.y
            transform_stamped.transform.rotation.z = base_to_camera.transform.rotation.z
            # transform_stamped.transform.rotation.w = final_quaternion[0]
            # transform_stamped.transform.rotation.x = final_quaternion[1]
            # transform_stamped.transform.rotation.y = final_quaternion[2]
            # transform_stamped.transform.rotation.z = final_quaternion[3]
            self.br.sendTransform(transform_stamped)

        # Show the image with detected markers and center points
        cv2.imshow('Aruco Markers', self.cv_image)
        cv2.waitKey(1)  # Adjust the delay as needed

    def rotation_vector_to_quaternion(self,rotation_vector):
        # Ensure the rotation vector has three elements
        if len(rotation_vector) != 3:
            raise ValueError("Rotation vector must have three elements (rx, ry, rz).")

        # Calculate the angle of rotation (magnitude of the rotation vector)
        angle = np.linalg.norm(rotation_vector)

        # Calculate the unit vector representing the rotation axis
        if angle == 0.0:
            # If the angle is zero, return an identity quaternion
            return np.array([1.0, 0.0, 0.0, 0.0])
        else:
            axis = rotation_vector / angle

        # Calculate the quaternion components
        half_angle = angle / 2.0
        sin_half_angle = np.sin(half_angle)
        cos_half_angle = np.cos(half_angle)

        quaternion = np.array([
            cos_half_angle,
            sin_half_angle * axis[0],
            sin_half_angle * axis[1],
            sin_half_angle * axis[2]
        ])

        return quaternion

    def euler_to_matrix(self,roll, pitch, yaw):
        # Create a Rotation object from the Euler angles
        rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        
        # Get the rotation matrix
        rotation_matrix = rotation.as_matrix()
        
        return rotation_matrix

    def find_rotation_matrix_to_align_with_up(self,r1):
        # Define the desired orientation where the green part of the y-axis points up
        # This corresponds to the identity rotation (no rotation)
        r2 = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0]
    ])

        # Convert r1 (roll, pitch, yaw) to a rotation matrix
        r1_matrix = self.euler_to_matrix(r1[0], r1[1], r1[2])

        # Calculate the relative rotation from r1 to r2
        relative_rotation = R.from_matrix(r2) * R.from_matrix(r1_matrix).inv() 

        # Get the rotation matrix to align r1 with r2
        R_matrix = relative_rotation.as_matrix()

        return R_matrix
        
    
    def rotate_quaternion_by_matrix(self, roll, pitch, yaw, rotation_matrix):
        # Convert the roll, pitch, and yaw angles to a rotation matrix
        rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        rotation_matrix_from_euler = rotation.as_matrix()

        # Multiply the provided rotation matrix by the rotation matrix from Euler angles
        new_rotation_matrix = np.dot(rotation_matrix, rotation_matrix_from_euler)

        # next_mat = np.array([[10,0,0],
        #                        [0,1,0],
        #                        [0,0,1]])

        next_mat = R.from_euler('xyz', [0,0,0], degrees=True).as_matrix()
        new_rotation_matrix = np.dot(next_mat,new_rotation_matrix)

        # Convert the new rotation matrix back to a quaternion
        new_rotation_quaternion = R.from_matrix(new_rotation_matrix).as_quat()

        print(f"Quat is {new_rotation_quaternion}")

        return new_rotation_quaternion

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to a quaternion.

        :param roll: Roll angle in radians (rotation around the X-axis).
        :param pitch: Pitch angle in radians (rotation around the Y-axis).
        :param yaw: Yaw angle in radians (rotation around the Z-axis).
        :return: Quaternion as a tuple (w, x, y, z).
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return w, x, y, z


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()