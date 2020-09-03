# Vision-and-force-control-automonous-painting-with-rollers

We proposed an autonomous painting framework at low cost that can realize the human-level performance.

The roller handle design is inlcuded in the folder "Roller_design". You can 3D-print the components and then assemble them with the spring-damper system and the robotic arm by looking at the following close-up of the roller designed by me, i.e. Figure 4 in our paper. The spring-damper system, the microcontroller, and ultrasonic sensor we used were bought from here: 
https://www.amazon.com/BQLZR-Aluminum-Absorbers-Upgrade-Suspension/dp/B00S4SG6KI
https://www.adafruit.com/product/2000
https://www.digikey.com/catalog/en/partgroup/hc-sr04-ultrasonic-sonar-distance-sensors/82205?utm_adgroup=Sensors%20%26%20Transducers&utm_source=google&utm_medium=cpc&utm_campaign=Dynamic%20Search_RLSA_Cart&utm_term=&utm_content=Sensors%20%26%20Transducers&gclid=CjwKCAjwqML6BRAHEiwAdquMnXmd7aK8EVTLs4LGN8ujgTQU1mxTO-4nRDXvDEWne5HxvXRcT0hGQRoCa6AQAvD_BwE

As our paper "Vision and force based autonomous coating with rollers" includes our work in two aspects: 3D autonomous painting and 2D painting quality evaluation. The code for 2D painting is included in the folder "2D painting codes". 

The code is in the folder "3D painting files" including all steps for 3D coating such as, hand/eye calibration, clock synchronization problem, build obstacle in MoveIt for obstacle avoidance and path planning. 

Since our workspace is named as 'ros_ws', the following command is frequently used when using Sawyer for experiments:
>> cd ros_ws/
>> ./intera.sh
>> rosrun intera_interface enable_robot.py -e 

If you use our tutorial, our design or part of our code, please remember to cite our paper:
Link to be added.
