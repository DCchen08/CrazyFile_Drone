#!/usr/bin/python3

import rospy
import math
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu



KF = 1.28192e-08
KN = 8.06428e-05  
GRAVITY = 9.81                                   #rotors_description/urdf/crazyfile2.xacro                      
MASS = 0.03                                     #kg  #rotors_description/urdf/crazyfile2.xacro
l = 0.046 

angle_roll_pitch_yaw = [0,0,0]
angle_roll_pitch_yaw_gazebo = [0,0,0]

distance_x_y_z = [0.5,0.5, 0.015]
x_theory = 0
y_theory = 0
z_theory = 0
velocity_x_y_z = [0,0,0]
vx_theory = 0
vy_theory = 0
vz_theory = 0
ts = 1/500
init_z = distance_x_y_z[2]



def publish_motor_speeds(motor_speeds):
    global x_theory,y_theory,z_theory,vx_theory,vy_theory,vz_theory
    global motor_speed_pub
    motor_speed_msg = Actuators()
    motor_speed_msg.angular_velocities = motor_speeds
    motor_speed_pub.publish(motor_speed_msg)
    #print("publish_motor_speed")
    motor_1_theory,motor_2_theory,motor_3_theory,motor_4_theory = motor_speeds


    Force_1_theory = motor_1_theory**2 * KF
    Force_2_theory = motor_2_theory**2 * KF 
    Force_3_theory = motor_3_theory**2 * KF 
    Force_4_theory = motor_4_theory**2 * KF 

    Force_sum_theory =  Force_1_theory + Force_2_theory + Force_3_theory + Force_4_theory
    #print("")
    #print("Force_sum_theory: {:.5f}".format(Force_sum_theory))
    #print("")

    Moment_1_theory = KN * motor_1_theory**2
    Moment_2_theory = KN * motor_2_theory**2
    Moment_3_theory = KN * motor_3_theory**2
    Moment_4_theory = KN * motor_4_theory**2

    Moment_z_gazebo = Moment_1_theory - Moment_2_theory + Moment_3_theory - Moment_4_theory
    Moment_x_gazebo = (Force_2_theory - Force_4_theory) * l
    Moment_y_gazebo = (Force_3_theory - Force_1_theory) * l   

    roll_gazebo, pitch_gazebo, yaw_gazebo = angle_roll_pitch_yaw_gazebo  

    a_x_theory = Force_sum_theory * ( math.cos(yaw_gazebo) * math.sin(pitch_gazebo) + math.cos(pitch_gazebo) * math.sin(roll_gazebo) * math.sin(yaw_gazebo) ) / MASS 
    a_y_theory = Force_sum_theory * ( math.sin(yaw_gazebo) * math.sin(pitch_gazebo) - math.cos(yaw_gazebo) * math.cos(pitch_gazebo) * math.sin(roll_gazebo) ) / MASS
    a_z_theory = (Force_sum_theory * math.cos(pitch_gazebo) * math.cos(roll_gazebo)) / MASS - GRAVITY

    x_theory += (vx_theory * ts + 0.5 * a_x_theory * ts**2)
    y_theory += (vy_theory * ts + 0.5 * a_y_theory * ts**2)
    z_theory += (vz_theory * ts + 0.5 * a_z_theory * ts**2)

    vx_theory += a_x_theory * ts
    vy_theory += a_y_theory * ts
    vz_theory += a_z_theory * ts

    #print("ax_theory:{:.5f}, ay_theory:{:.5f}, az_theory:{:.5f}".format(a_x_theory,a_y_theory,a_z_theory))
    print("x_theory:{:.5f}, y_theory:{:.5f}, z_theory:{:.5f}".format(x_theory,y_theory,z_theory))

#def theory_show(motor_speeds):

    



def callback_motor_speed(msg):
    motor_1_sub,motor_2_sub,motor_3_sub,motor_4_sub = msg.angular_velocities

    print(" ")
    #rospy.loginfo("angular_velocities: {:.5f},{:.5f},{:.5f},{:.5f}".format(motor_1_sub,motor_2_sub,motor_3_sub,motor_4_sub))
    print(" ")

def callback_odometry(msg):
    x = msg.twist.twist.linear.x   
    y = msg.twist.twist.linear.y
    z = msg.twist.twist.linear.z
    roll_x  = msg.twist.twist.angular.x
    pitch_y = msg.twist.twist.angular.y
    yaw_z   = msg.twist.twist.angular.z
    velocity_x = msg.twist.twist.linear.x   
    velocity_y = msg.twist.twist.linear.y
    velocity_z = msg.twist.twist.linear.z
    angular_velocity_roll_x  = msg.twist.twist.angular.x
    augular_veloicty_pitch_y = msg.twist.twist.angular.y
    gngular_velocity_yaw_z   = msg.twist.twist.angular.z

    print("odom location: x: {:.5f}, y: {:.5f}, z: {:.5f}".format(x,y,z))
    #rospy.loginfo("odom angular velocity: roll: {:.5f}, pitch: {:.5f}, yaw: {:.5f}".format(roll,pitch,yaw))
    #Moment_1 = KN * omega_1**2
    #Moment_2 = KN * omega_2**2
    #Moment_3 = KN * omega_3**2
    #Moment_4 = KN * omega_4**2

    #Moment_z = Moment_1 - Moment_2 + Moment_3 - Moment_4
    #Moment_x = (Force_2 - Force_4) * l
    #Moment_y = (Force_3 - Force_1) * l 
    #return roll

def callback_angular_acceleration(msg):
    ax_subscriber = msg.linear_acceleration.x
    ay_subscriber = msg.linear_acceleration.y
    az_subscriber = msg.linear_acceleration.z
    #print("ax_subscriber:{:.5f}, ay_subscriber:{:.5f}, az_subscriber:{:.5f}".format(ax_subscriber,ay_subscriber,az_subscriber))


if __name__=='__main__':
    # Initialize the ROS node
    rospy.init_node('test_6', anonymous=False)
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    reset_world()
    #rospy.sleep(1)
    r = rospy.Rate(500)
    motor_speed_pub = rospy.Publisher('/crazyflie2/command/motor_speed',Actuators, queue_size=1)
    rospy.Subscriber("/crazyflie2/motor_speed", Actuators,callback_motor_speed)
    rospy.Subscriber("/crazyflie2/odometry", Odometry,callback_odometry)
    rospy.Subscriber("/crazyflie2/ground_truth/imu",Imu,callback_angular_acceleration)
    #print("sb is ",sub)

    while not rospy.is_shutdown():
        motor_speeds = [2500,2500,2500, 2500]
        #theory_show(motor_speeds)
        publish_motor_speeds(motor_speeds)
        r.sleep()
    
    #motor_speeds_shut_down = [0, 0, 0, 0]
    #publish_motor_speeds(motor_speeds_shut_down)
    #rospy.spin()
