# Cosmo Logistic - eYantra 2023-24
### Team Members : Tahsin Khan,Chinmaya Sahu 
### Team ID : 1868
# Stage - 1 : ( Completed ) 
## Task 1: - ( Completed - 100/100)
1. Write proper and clean code for Task 1A (correct the hard code part of one box) - ( Completed )
2. Implement and learn the servo method for Task 1B (optional) - ( Completed )
3. Use the `nav_to_pose` method for Task 1C (current code waypoint_nav method) - ( Completed)

## Task 2: - (Completed - 100/100)

1. Task 2A (Completed) : Correct the aruco box estimation code in task1A , use the TF recieved of the boxes w.r.t to base_link now need to use move it servo to pick the boxes , use the gripper service to grab it and place at drop pose .

https://github.com/TahsinOP/e_Yantra_2023-2024_Cosmo_Logitstic/assets/117567813/f4076960-b6ef-44c4-b1c1-9b0bf0d41e27

2. Task 2B (Completed) : Docking , attaching the rack to ebot and navigation ( using nav2 ) to the given poses
   
   Docking : Use the given custom service , use ultrasonic sensors for linear allignment and use the odometery data for orientation allignment 
   
   Racking : Attach and detach the rack to box using the custom services given
   
   Navigation : Combine all the three in one code , first use nav2,go_to_pose to go the Rack1 ( map the warehouse again ) , call the docking service and allign the ebot with the rack ,attach 
   the rack go to drop pose and detach the rack

https://github.com/TahsinOP/e_Yantra_2023-2024_Cosmo_Logitstic/assets/117567813/c13fb30d-3859-417b-b385-ca64f45a9aaa
   
# Stage - 2 : (Selected) 
## Task 3 : (Completed - 97.26/100) 

1. Task 3A : (Completed)
   
   Issues resolved :
   1. launching of the 3A script (solved : Increased the image processing rate ) 
   2. flickering of rightmost tf marker in hardware camera ( solved : used alpha and beta factors )

![Screenshot from 2023-12-08 14-22-55](https://github.com/TahsinOP/e_Yantra_2023-2024_Cosmo_Logitstic/assets/117567813/e557d9ea-713e-436e-85cb-b394cb4cbd4a)

   
2. Task 3B : (Completed - Time = 128 seconds ) 

    Created a custom ROS2 python launch file to launch all the python scripts in the below order :
   
     i. Launch 'ebot_docking_service_task3b.py' and 'ebot_nav2_cmd_task3b.py' simultaneously

     ii. Launch 'aruco_detection_3B' as soon as 'ebot_nav2_cmd_task3b.py' exits
   
     iii. Launch the 'Pick_and_place_box_3B.py' with a delay timer (5secs) after the 3rd script starts


https://github.com/TahsinOP/e_Yantra_2023-2024_Cosmo_Logitstic/assets/117567813/a50750c0-593b-4c77-90c6-3d2d47225154

## Task 4 : ( Deadline : 19th January )  -----  Finished at Rank # 16 - 59.31/100 
1. Task 4A : 2nd slot completed ( Marks : 10/35)

   Status : Navigation and docking completed ( Need to attach and undock)
   Issues resolved : Docking error and ultrasonic sensor unit callibiration

   Commands to be executed in the terminal during the slot before running the scripts : 
   ```bash
   ros2 run ebot_control duplicate_imu
   ```
   ```bash
   ros2 run ebot_control ebot_dock
   ```
   ```bash
   ros2 run ebot_control ebot_docking_service
   
3. Task 4B : 2nd slot completed ( Marks : 20/35)
    
   Status : Picked and placed front box succefully , need to take care of the side box
   Issues reloved : Changed the mapping of axis in hardware ("tool0" link ),Tuned the scaling factor and distance threshold to reach the front box 

4. Task 4C : Completed ( Marks : 29.31/30 )

https://github.com/TahsinOP/e_Yantra_2023-2024_Cosmo_Logitstic/assets/117567813/aa9b9b31-4ead-4218-9104-c058acd71ea4

### Reasons due to which we couldn't complete hardware tasks 
#### Task 4B 
i .The main issue was with servoing ( not axis mapping ) we were directly using the difference calculated and scaling it to the twist messages 

ii . The above logic worked well in simualation performed very poorly in hardware 

Solution : Should have used the unit vector logic suggested in 2A ( Find the unit vector using diff_vector / distance and scale it accordingly ) . This will handle the velocities of the end-effector more dynamically 

#### Task 4A



