#!/usr/bin/env python3
# Software License Agreement (BSD License)


import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
import tf
import numpy as np
import matplotlib.pyplot as plt
import os
#from nav_msgs import Odometry

cmd_vel = Twist()

global x,y,theta,v,w

# Robot's initial position and velocity
# TODO: actually get from supervisor or marvelmind
x = -2.0
y = 4.0
theta = 0.0

v = 0.0
w = 0.0

# Target path nodes list
path = [[-2.0,-0.3], [0,-0.3]]


# Target position (TODO: read from master)
global target_x , target_y, tot_error, target_id, vel_errors, pos_error, times
target_id = 0
target_x = path[target_id][0]
target_y = path[target_id][1]

tot_error=0

# Control gains
Ka = 0.2
Kv = 0.1

ki = 0.0
kd = 0.001

# Set the desired constant velocity
v_desired = 0.1#3

error_plots = True
vel_errors = []
pos_errors = []
times = []
current_dir = os.getcwd()



def compute_vel():
    global v, w, x, y, target_x, target_y, tot_error, target_id

    # Loop until the robot reaches the target position If or while ?
    if math.sqrt((target_x - x)**2 + (target_y - y)**2) > 0.1:
        # Calculate the heading to the target
        heading = math.atan2(target_y - y, target_x - x)
        rospy.loginfo('heading')
        rospy.loginfo(heading)
        rospy.loginfo('theta')
        rospy.loginfo(theta)
        
        # Calculate the angular error
        alpha = heading - theta
        
        # Normalize the angular error to the range [-pi, pi]
        if alpha > math.pi:
            alpha -= 2*math.pi
        elif alpha < -math.pi:
            alpha += 2*math.pi

        tot_error += alpha

        rospy.loginfo('alpha')
        rospy.loginfo(alpha)

        # Move backwards if the goal is behind the robot
        if abs(alpha)>math.pi/2:
            v_des= -v_desired
        else:
            v_des= v_desired
        
        # Calculate the linear error
        distance = math.sqrt((target_x - x)**2 + (target_y - y)**2)
        # Calculate the control inputs
        v_error = v_des - v #TODO: get true vel from robot data instead of vel command
        if error_plots:
            vel_errors.append(v_error)
            arr = np.array([x-target_x, y-target_y])
            min_idx = np.argmin(abs(arr))
            pos_errors.append(arr[min_idx])
            

        change_dir = (abs(alpha)>0.05*math.pi/2)
        v = (v_des + Kv*v_error)*(not change_dir) #only apply linear vel if aligned enough with target to avoid avershoot of trajectory at changes in direction
        w = -Ka*(alpha+2*change_dir) + ki*tot_error #angular correction, turn more if change in direction
        
        # Adjust the angular velocity to maintain approximately constant linear velocity
        if distance < 0.5:
            w = w*(distance/0.5)
        
    else:
        rospy.loginfo('Goal reached')
        target_id +=1
        v = 0
        w = 0
        tot_error = 0
        vel_fig = plt.figure()
        plt.plot(vel_errors)
        #plt.show()
        vel_fig.savefig(current_dir+'/catkin_ws/src/edymobile/edy_controller/figures/vel_error_'+str(target_id)+'.png')
        pos_fig = plt.figure()
        plt.plot(pos_errors)
        #plt.show()
        pos_fig.savefig(current_dir+'/catkin_ws/src/edymobile/edy_controller/figures/pos_error_'+str(target_id)+'.png')
        if target_id < len(path):
            rospy.loginfo('target id')
            rospy.loginfo(target_id)
            target_x = path[target_id][0]
            target_y = path[target_id][1]
        else:
            rospy.loginfo('Final goal reached')
     
        

def poseCallBack(odom_msg):
    global x, y, theta
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])
    theta = yaw

    compute_vel()
    cmd_vel.linear.x = v
    #msg.linear.y = 0 #doesn't make sense for a differential drive robot
    cmd_vel.angular.z = w
    rospy.loginfo(w)



def talker():

    rospy.init_node('edy_controller_node', anonymous=True)

    rospy.Subscriber('/edymobile/odom', Odometry, poseCallBack)

    vel_pub = rospy.Publisher('edymobile/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz

    rospy.loginfo('sending velocity target')
    while not rospy.is_shutdown():
        vel_pub.publish(cmd_vel)
        if target_id >= len(path): #>
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo('fail')
        pass

