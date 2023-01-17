# CrazyFile_Drone

Before downloading the file, please make sure that you should have the right version of Ubuntu(20.04) and ROS1 version(Noetic). 

1. Refering this link and download the file on the computer.
  https://github.com/gsilano/CrazyS
  
  Make sure your computer have enough room for this file.
  
 2. Using the google link to download the SRC file and replace the SRC file with the origin one.
  
 3. Under the Catkin_ws file, open the command window. In order to execute the C++ file, you need to type following command
     
     catkin_make_isolated
 
 4. The launch file that we used is "crazyfile2_with_joy". If you want to check with your gazebo model, you will need to put this in the command window.
   
   roslaunch rotors_gazebo crazyflie2_with_joy.launch
 
 5. The python file that we used to control the robot in this stage is test.py (CrazyS/rotors_gazebo/scripts). 
    When you want to execute the file and control the drone, open another command window and type the following
    
    python3 test.py
    
 
 Test file note:
 Control list: the Pwm number you need to put in for control the angular velocity of motor.
 Self.r : this is the frequency that the file tell robot to do
 omega: this is the motor speed in rad/s
 
 
 
 
 
 
 
 
 reference:
 https://www.bitcraze.io/2018/11/demystifying-drone-dynamics/
    

 
