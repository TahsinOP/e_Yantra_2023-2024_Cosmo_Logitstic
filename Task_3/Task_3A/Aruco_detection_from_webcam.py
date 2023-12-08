

import rclpy
import sys
import cv2
import math
import tf2_ros
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
import time




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
    # width_aruco_list = []
    ids = []
    rvec_list = []
    tvec_list = []


  
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    print(gray)

    
  

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters =  cv2.aruco.DetectorParameters()
    
    # Detect Aruco markers in the image and store 'corners' and 'ids'
    corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if marker_ids is not None:
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
            
            tvec = tvec.squeeze(1)
            
            distance_from_rgb = tvec[0,2]
            
            
            # Append the detected marker information to respective lists
            distance_from_rgb_list.append(distance_from_rgb)
            angle_aruco_list.append(rvec)
            # width_aruco_list.append(width)
            ids.append(marker_ids[i][0])

            # Draw the detected marker on the image
            cv2.aruco.drawDetectedMarkers(image, corners)

            # Draw frame axes
            cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, size_of_aruco_m)

    
    cv2.imshow('Ankit is GAY',image)
            # detect_aruco(gray)
    cv2.waitKey(10)  # Adjust the delay as needed
    if len(marker_ids) == 0:
        return

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list,  ids, rvec_list, tvec_list

camera = cv2.VideoCapture(0)

while True:
    try:
        
        frame = camera.read()[1]
        # print(frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters =  cv2.aruco.DetectorParameters()

        # corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # print(marker_ids)
        detect_aruco(frame)
        # cv2.imshow('Aruco Markers', gray)
        # cv2.waitKey(10)
        
        
        # time.sleep(1)
    except:
        pass