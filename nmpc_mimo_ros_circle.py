#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
from simple_pid import PID
import tf
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
import sys
import do_mpc
from casadi import *
import threading
import csv

#____________________________________________________________________________________
# class controller:
#     def __init__(self, type:str):
#         if type.upper() == 'mpc':
#             self.type = 'mpc'
#             if setPoints != None:
#                 self.set_points(setPoints)
#____________________________________________________________________________________

#global cmd, position_x, position_y

position_x = []
position_y = []

cmd = Twist()
cmd.linear.x = 0.0
cmd.linear.y = 0.0
cmd.linear.z = 0.0
cmd.angular.x = 0.0
cmd.angular.y = 0.0
cmd.angular.z = 0.0

pidyaw = PID(1, 0, 0, setpoint=0)
pidz = PID(0.6, 0, 0.2, setpoint=1)

csvfile = open('experiment_circle_mpc.csv','w',encoding="utf-8")
dronewriter = csv.writer(csvfile, quotechar = '\'', delimiter=',',quoting = csv.QUOTE_NONE,escapechar = ' ')
# write the header
dronewriter.writerow(["position_x","position_y","position_z","roll","pitch","yaw","u_x","u_y","u_z","u_yaw","velocity_x","velocity_y"]) 

#global dx, dy
#dx = 0
#dy = 0
#________________________________________________________________________
#________________________________________________________________________

# def pose_callback2(data1):
#     global dx, dy
#     dx = data1.pose.position.x
#     dy = data1.pose.position.y
    #print(dx, dy)

#________________________________________________________________________
#________________________________________________________________________

def template_model():

   
    model_type = 'discrete'  # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    _x = model.set_variable(var_type='_x', var_name='x', shape=(4, 1))
    _u = model.set_variable(var_type='_u', var_name='u', shape=(2, 1))
    tvp_x = model.set_variable(var_type='_tvp', var_name='tvp_x')
    tvp_y = model.set_variable(var_type='_tvp', var_name='tvp_y')
    #set_point_y = model.set_variable(var_type='_tvp', var_name='set_point_y')



    a = -0.69837 
    b = 0.23428 

    A = np.array([[1, 1, 0, 0],
                  [0, a, 0, 0],
                  [0, 0, 1, 1],
                  [0, 0, 0, a]])

    B = np.array([[0, 0],
                  [b, 0],
                  [0, 0],
                  [0, b]])

    x_next = A @ _x + B  @ _u #10
    model.set_rhs('x', x_next)
    #model.set_expression(expr_name='cost', expr=(0.01*(_x[0] - 1) ** 2 + 0.01*(_x[2] - 0)** 2))
    #model.set_expression(expr_name='cost', expr=(0.01*((_x[0] - 0) ** 2) + 0.01*((_x[2] - 0)** 2)+ 0.001* (_x[1]) + 0.001*(_x[3])))
    # Setup model
    model.setup()

    return model

#________________________________________________________________________
#________________________________________________________________________

def template_mpc(model):
#def template_mpc(model, dx, dy):
    mpc = do_mpc.controller.MPC(model)

    # Set parameters:
    setup_mpc = {
        'n_horizon': 20,
        #'control_horizon':5,
        'n_robust': 0,
        't_step': 0.01,
        'state_discretization': 'discrete',
        'store_full_solution': True,
    }
    mpc.set_param(**setup_mpc)
    _x = model.x
    x = _x['x']

    _tvp = model.tvp
    tvp_x = _tvp['tvp_x']
    tvp_y = _tvp['tvp_y']
    #set_point_y = model.tvp

    #sub2 = rospy.Subscriber('/rectangular_trajectory', PoseStamped, callback=pose_callback2)
    #cost = 0.01*((_x[0] - 0) ** 2) + 0.01*((_x[2] - 0)** 2)+ 0.001* _x[1] + 0.001*_x[3]
    mterm = 0.09*((x[0] - tvp_x) ** 2) + 0.09*((x[2] - tvp_y)** 2) + 0.01* (x[1]**2) + 0.01*(x[3]**2)  # terminal cost
    lterm = 0.09*((x[0] - tvp_x) ** 2) + 0.09*((x[2] - tvp_y)** 2) + 0.01* (x[1]**2) + 0.01*(x[3]**2)  # terminal cost
   # model.set_expression(expr_name='cost', expr=(lterm+mterm))
    #print(dx, dy)
    #mterm = model.aux['cost']  # terminal cost
    #lterm = model.aux['cost']  # terminal cost

    # stage cost
    mpc.set_objective(mterm=mterm, lterm=lterm)      
    ru = np.array([[100], [100]]) #100
    mpc.set_rterm(u=ru) # Scaling for quad. cost.

    # Lower bounds on states:
    mpc.bounds['lower', '_u', 'u'] = -0.15

    # Upper bounds on states
    mpc.bounds['upper', '_u', 'u'] = 0.15
    
    #mpc.scaling['_x', 'x'] = 1
    mpc.scaling['_u', 'u'] = 10#10

#----------------------------------------------------------------
    # tvp_template_1 = mpc.get_tvp_template()
    # tvp_template_1['_tvp'] = 0
    #
    # tvp_template_2 = mpc.get_tvp_template()
    # tvp_template_2['_tvp'] = 1
    # def tvp_fun(t_now):
    #     if t_now < 10:
    #         return  tvp_template_1
    #     else:
    #         return  tvp_template_2
#----------------------------------------------------------------

    tvp_template = mpc.get_tvp_template()


    # tvp_template_2 = mpc.get_tvp_template()
    # tvp_template_2['_tvp'] = np.array([1,1])
    #
    # tvp_template_3 = mpc.get_tvp_template()
    # tvp_template_3['_tvp'] = np.array([-1,1])
    #
    # tvp_template_4 = mpc.get_tvp_template()
    # tvp_template_4['_tvp'] = np.array([-1,0])


    def tvp_fun(t_now):
        angle = math.pi / 5
        sx = 1 * math.cos(angle * t_now)
        sy = 1 * math.sin(angle * t_now)
        tvp_template['_tvp'] = np.array([sx,sy])
        #tvp_template['_tvp', 'tvp_y'] = sy
        return tvp_template
        # if t_now < 20:
        #     return tvp_template_1
        # if (t_now > 20 and t_now < 40):
        #     return tvp_template_2
        # if (t_now > 40 and t_now < 60):
        #     return tvp_template_3
        # if t_now > 60:
        #     return tvp_template_4
#-------------------------------------------------------------------
    mpc.set_tvp_fun(tvp_fun)

    mpc.setup()

    return mpc
#____________________________________________________
#________________________________________________________________________
def diff(xn, yn):
    global position_x, position_y
    current_time = rospy.Time.now()
    if position_x:
         prev_time, prev_position_x = position_x[-1]
         time_diff = (current_time - prev_time).to_sec()
         position_diff_x = xn - prev_position_x
         velocity_x = position_diff_x / time_diff
    position_x.append((current_time, xn))
    if position_y:
         prev_time, prev_position_y = position_y[-1]
         time_diff = (current_time - prev_time).to_sec()
         position_diff_y = yn - prev_position_y
         velocity_y = position_diff_y / time_diff
    position_y.append((current_time, yn))
    return velocity_x, velocity_y

def pose_callback(data):
    global cmd
    #control x
    xn = data.pose.position.x
    yn = data.pose.position.y
    zn = data.pose.position.z
    velocity_x, velocity_y = diff(xn, yn)
    x1 = np.array([xn, velocity_x, yn, velocity_y]).reshape(-1,1)  
    u_control = mpc.make_step(x1)
    #print(dx, dy)
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
    #print(yaw)
    uw_control = pidyaw(yaw)
    uz_control = pidz(zn)
    #pub_bebop_vel = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.linear.x = u_control[0,0]
    cmd.linear.y = u_control[1,0]
    cmd.linear.z = uz_control
    #cmd.angular.x = 0.0
    #cmd.angular.y = 0.0
    cmd.angular.z = uw_control
    #pub_bebop_vel.publish(cmd)
      
    # write the data
    dronewriter.writerow([xn,yn,zn,roll,pitch,yaw,u_control[0,0],u_control[1,0],uz_control,uw_control,velocity_x, velocity_y])

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

    rospy.init_node('nmpc_mimo_ros_vel_rate', anonymous=True)
    #sub2 = rospy.Subscriber('/rectangular_trajectory', PoseStamped, callback=pose_callback2)
    sub = rospy.Subscriber('/mocap_node/bebop_01/pose', PoseStamped, callback=pose_callback)
    worker = threading.Thread(target= publisher_thread)
    worker.start()
    pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
    pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 10)
     #try:
        #rospy.spin()
     #except KeyboardInterrupt:
        #print("Landing")
        #pub_land.publish(Empty())   
    rospy.spin()

x00 = np.array([0, 0, 0, 0]).reshape(-1,1)
u0 = np.array([0, 0]).reshape(-1,1)
model = template_model()
#mpc = template_mpc(model,dx,dy)
mpc = template_mpc(model)
mpc.reset_history()
mpc.x0 = x00
mpc.u0 = u0
mpc.set_initial_guess()
       
if __name__ == '__main__':
     #starting = input("Press s to start: ")
     #if starting == "s":

     starting = input("Press s to start: ")
     if starting == "s":

          main() 
          landing()
          csvfile.close()
          
          rcParams['axes.grid'] = True
          rcParams['font.size'] = 18
          
          

          ref_x = mpc.data['_tvp', 'tvp_x']
          ref_y = mpc.data['_tvp', 'tvp_y']

          pose_x = mpc.data['_x', 'x'][:,0]
          pose_y = mpc.data['_x', 'x'][:,2]
          
          #print(len(ref_x) , len(ref_x) , len(pose_x), len(pose_y))
          
          plt.plot(pose_x,pose_y, label="MPC")
          plt.plot(ref_x,ref_y, label="Path")
          plt.legend(loc="upper left")


          fig, ax, graphics = do_mpc.graphics.default_plot(mpc.data, figsize=(16,9))
          graphics.plot_results()
          graphics.reset_axes()

          ax[0].legend(graphics.result_lines['_x', 'x'], 'xvyv', title='States', loc='center right')
          ax[1].legend(graphics.result_lines['_u', 'u'], 'xy', title='States', loc='center right')
          plt.show()
          
     
