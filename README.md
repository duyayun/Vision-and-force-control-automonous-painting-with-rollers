# Vision-and-force-control-automonous-painting-with-rollers

We proposed an autonomous painting framework at low cost that can realize the human-level performance.

The roller handle design is inlcuded in the folder "Roller_design". As our paper "Vision and force based autonomous coating with rollers" includes our work in two aspects: 3D autonomous painting and 2D painting quality evaluation. The code for 2D painting is included in the folder "2D painting codes". 

The code is in the folder "3D painting files" including all steps for 3D coating such as, hand/eye calibration, clock synchronization problem, build obstacle in MoveIt for obstacle avoidance and path planning. 

Since our workspace is named as 'ros_ws', the following command is frequently used when using Sawyer for experiments:
>> cd ros_ws/
>> ./intera.sh
>> rosrun intera_interface enable_robot.py -e 
