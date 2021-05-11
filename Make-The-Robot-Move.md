# Instructions on how to get the Arlo Robot moving:


Following the instructions using the link below, log into the robot: https://github.com/smartsystemslab-uf/ZynqRobotController. 2 robots in the lab already has a flashed sd card and is ready to go. The third fully assembled robot only requires a sd card.

Ensure the ethernet cable is connected to the robot since the wifi is not setup. 

Once a SSH connection is established follow the steps below:
        
        $ cd ~/catkin_ws
        $ catkin_make
        $ . ~/catkin_ws/devel/setup.bash
        $ cd 

Open 3 separate terminals, in the first terminal run: 

        $ roscore

Roscore is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a roscore running in order for ROS nodes to communicate.

In the second terminal run the following commands:

        $ cd ~/catkin_ws
        $ rosrun dhb10-controller motor-controller-interface.py

In the third terminal run the command: 

        $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Follow instructions in the third terminal to move robot around. Sidenote: Decrease speed before moving the robot for better control.

A demonstration video can be found at this link: https://drive.google.com/file/d/1m09CBd7YE4mIdfX33Fac-yUXeAr02Pfi/view?usp=sharing

To troubleshoot please test the relay by following the steps below:

        $ cd ~/UIO
        $ g++ -o relay_on GPIO_0_High.cpp //To compile the executable to turn on the relay
        $ ./relay_on //Run the executable to turn on the relay
        $ g++ -o relay_off GPIO_0_Low.cpp //To compile the executable to turn off the relay
        $ ./relay_off //Run the executable to turn off the relay

After turning on the relay the 'Featherwing Power Relay' on the robot should power on, you should hear a click and the red power light should come on.
