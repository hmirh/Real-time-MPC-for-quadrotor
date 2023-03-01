#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
from simple_pid import PID
import tf
import threading
import matplotlib.pyplot as plt
import math
import csv

#global cmd

cmd = Twist()
cmd.linear.x = 0.0
cmd.linear.y = 0.0
cmd.linear.z = 0.0
cmd.angular.x = 0.0
cmd.angular.y = 0.0
cmd.angular.z = 0.0

positionx = []
positiony = []
velocityx = []
velocityy = []
refx = []
refy = []

pidx = PID(0.35, 0.006, 0.4)
pidy = PID(0.35, 0.006, 0.4)
pidyaw = PID(1, 0, 0, setpoint=0)
pidz = PID(0.6, 0, 0.2, setpoint=1)
t = 0

csvfile = open('experiment_rose_pid.csv','w',encoding="utf-8")
dronewriter = csv.writer(csvfile, quotechar = '\'', delimiter=',',quoting = csv.QUOTE_NONE,escapechar = ' ')
# write the header
dronewriter.writerow(["position_x","position_y","position_z","roll","pitch","yaw","u_x","u_y","u_z","u_yaw"]) 


#def pose_callback2(data1):
    #pidx.setpoint = data1.pose.position.x
    #pidy.setpoint = data1.pose.position.y
    #print(pidx.setpoint)

def pose_callback(data):
    global cmd, t
    #pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
    #x = data.pose.position.x
    #y = data.pose.position.y
    #if (x == 0 and y == 0):
        #pub_land.publish(Empty())
    angle = math.pi / 15
    pidx.setpoint = 1 * math.cos(2 * angle * t) * math.cos(angle * t)
    pidy.setpoint = 1 * math.cos(2 * angle * t) * math.sin(angle * t)
    refx.append(pidx.setpoint)
    refy.append(pidy.setpoint)
    #control x
    xn = data.pose.position.x
    positionx.append(xn)
    #pidx = PID(0.35, 0.006, 0.4, setpoint=1) #not good
    #pidx = PID(0.15, 0.007, 0.4, setpoint=1) # good
    #pidx = PID(0.15, 0.02, 0.1, setpoint=0.5) # good
    #pidx = PID(0.25, 0.007, 0.3, setpoint=1) #image1 good
    #pidx = PID(0.15, 0.006, 0.4, setpoint=0) image2
    #pidx = PID(0.25, 0.004, 0.3, setpoint=0) image3
    #pidx = PID(0.25, 0.01, 0.3, setpoint=0) image4
    #pidx = PID(0.25, 0.007, 0.008, setpoint=0) image5
    ux_control = pidx(xn)
    #print(pidx.setpoint)
    velocityx.append(ux_control)
    #control y
    yn = data.pose.position.y
    positiony.append(yn)
    #pidy = PID(0.25, 0.007, 0.3, setpoint=0)
    #pidy = PID(0.15, 0.007, 0.4, setpoint=0)
    #pidy = PID(0.15, 0.02, 0.1, setpoint=0.5)
    uy_control = pidy(yn)
    #print(pidy.setpoint)
    velocityy.append(uy_control)
    zn = data.pose.position.z
    #control yaw/heading
    wo = data.pose.orientation.w
    xo = data.pose.orientation.x
    yo = data.pose.orientation.y
    zo = data.pose.orientation.z
   
    quaternion = [xo, yo, zo, wo]

    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2] - (3.1415926535/2)
   
    #pidyaw = PID(1, 0, 0, setpoint=0)
    uw_control = pidyaw(yaw)
    uz_control = pidz(zn)
    #pub_bebop_vel = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
   
    #cmd = Twist()
    cmd.linear.x = ux_control
    cmd.linear.y = uy_control
    cmd.linear.z = uz_control
    #cmd.angular.x = 0.0
    #cmd.angular.y = 0.0
    cmd.angular.z = uw_control
    #pub.publish(cmd)
    #pub_bebop_vel.publish(cmd)
    t += 0.01
    dronewriter.writerow([xn,yn,zn,roll,pitch,yaw,ux_control,uy_control,uz_control,uw_control])


def publisher_thread():
    global cmd
    pub_bebop_vel = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(100) # ROS Rate at 5Hz
    while not rospy.is_shutdown():
        pub_bebop_vel.publish(cmd)
        rate.sleep()     

def landing():
    pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
    print("Landing")
    pub_land.publish(Empty())    


def main():
     rospy.init_node('pid_rate', anonymous=True)
     #sub2 = rospy.Subscriber('/rectangular_trajectory', PoseStamped, callback=pose_callback2)
     sub = rospy.Subscriber('/mocap_node/bebop_01/pose', PoseStamped, callback=pose_callback)
     
     worker = threading.Thread(target=publisher_thread)
     worker.start()
     pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
     pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 10)
     
     #try:
        #rospy.spin()
     #except KeyboardInterrupt:
        #print("Landing")
        #pub_land.publish(Empty())
        
     rospy.spin()
         
       
if __name__ == '__main__':
     starting = input("Press s to start: ")
     if starting == "s":
        main()
        landing()
        csvfile.close()
        fig, axs = plt.subplots(4)
        axs[0].plot(positionx)
        axs[0].set(ylabel = 'position_x')
        axs[1].plot(positiony)
        axs[1].set(ylabel = 'position_y')
        axs[2].plot(velocityx)
        axs[2].set(ylabel = 'input_x')
        axs[3].plot(velocityy)
        axs[3].set(ylabel = 'input_y')
        plt.figure(2)
        plt. plot(positionx,positiony, label="PID")
        plt. plot(refx,refy, label="Path")
        plt.show()


     




