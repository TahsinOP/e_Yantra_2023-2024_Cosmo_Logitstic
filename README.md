# Cosmo Logistic - eYantra 2023-24
### Team Members : Tahsin Khan,Chinmaya Sahu 
### Team ID : 1868
# Stage - 1 : ( Completed ) 
## Task 1: - ( Completed )
1. Write proper and clean code for Task 1A (correct the hard code part of one box) - ( Completed )
2. Implement and learn the servo method for Task 1B (optional) - ( Completed )
3. Use the `nav_to_pose` method for Task 1C (current code waypoint_nav method) - ( Completed)

## Task 2: - (Completed)

1. Task 2A (Completed) : Correct the aruco box estimation code in task1A , use the TF recieved of the boxes w.r.t to base_link now need to use move it servo to pick the boxes , use the gripper service to grab it and place at drop pose .

https://github.com/TahsinOP/e_Yantra_2023-2024_Cosmo_Logitstic/assets/117567813/f4076960-b6ef-44c4-b1c1-9b0bf0d41e27

2. Task 2B (Completed) : Docking , attaching the rack to ebot and navigation ( using nav2 ) to the given poses
   
   Docking : Use the given custom service , use ultrasonic sensors for linear allignment and use the odometery data for orientation allignment 
   
   Racking : Attach and detach the rack to box using the custom services given
   
   Navigation : Combine all the three in one code , first use nav2,go_to_pose to go the Rack1 ( map the warehouse again ) , call the docking service and allign the ebot with the rack ,attach 
   the rack go to drop pose and detach the rack

https://github.com/TahsinOP/e_Yantra_2023-2024_Cosmo_Logitstic/assets/117567813/c13fb30d-3859-417b-b385-ca64f45a9aaa
   
# Stage - 2 : (Selected) 
## Task 3 : ( 14th December : Deadline ) 

1. Task 3A : (Completed)
   
   Issues to resolve : launching of the 3A script (solved) , flickering of rightmost tf marker in hardware camera

![Screenshot from 2023-12-08 14-22-55](https://github.com/TahsinOP/e_Yantra_2023-2024_Cosmo_Logitstic/assets/117567813/e557d9ea-713e-436e-85cb-b394cb4cbd4a)

   
3. Task 3B : 

