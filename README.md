# Cosmo Logistic - eYantra 2023-24

## Task 1: ( Completed )
1. Write proper and clean code for Task 1A (correct the hard code part of one box).
2. Implement and learn the servo method for Task 1B (optional).
3. Use the `nav_to_pose` method for Task 1C (current code waypoint_nav method).

## Task 2: ( Deadline - 10th Nov) - 2B Completed

1. Task 2A : Correct the aruco box estimation code in task1A , use the TF recieved of the boxes w.r.t to base_link now need to use move it servo to pick the boxes , use the gripper service to grab it and place at drop pose . 

2. Task 2B (Completed) : Docking , attaching the rack to ebot and navigation ( using nav2 ) to the given poses
   
   Docking : Use the given custom service , use ultrasonic sensors for linear allignment and use the odometery data for orientation allignment .
   
   Racking : Attach and detach the rack to box using the custom services given
   
   Navigation : Combine all the three in one code , first use nav2,go_to_pose to go the Rack1 ( map the warehouse again ) , call the docking service and allign the ebot with the rack , attach the 
   rack go to drop pose and detach the rack  
