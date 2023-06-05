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

# TODO: get from fleet manager
# Target path nodes list
path = [[-2.0,-0.3], [0,-0.3]]

global tot_error
tot_error = 0.0


error_plots = True
vel_errors = []
pos_errors = []
times = []
current_dir = os.getcwd()

class DiffDriveRobot:

    def __init__(self, init_pos = [-2.0,4.0], init_heading = 0.0, init_vel_cmd = [0.0, 0.0], v_cst = 0.1):
        # Pose
        self.x = init_pos[0]
        self.y = init_pos[1]
        self.theta = init_heading
        # Velocity command
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = init_vel_cmd[0]
        self.cmd_vel.angular.z = init_vel_cmd[1]
        self.v_desired = v_cst
        # Position goal
        self.target_x = None
        self.target_y = None
        self.goal_reached = False
        # Control gains
        self.Ka = 0.2
        self.Kv = 0.1
        self.ki = 0.0
        self.kd = 0.001

    def set_target(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
        self.goal_reached = False

    def set_cmd_vel(self, v, w):
        self.cmd_vel.linear.x = v
        self.cmd_vel.angular.z = w

    def compute_vel(self):

        global tot_error

        # Calculate the linear error
        distance = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)

        if distance > 0.1:
            # Calculate the heading to the target
            heading = math.atan2(self.target_y - self.y, self.target_x - self.x)

            # Calculate the angular error
            alpha = heading - self.theta
            
            # Normalize the angular error to the range [-pi, pi]
            if alpha > math.pi:
                alpha -= 2*math.pi
            elif alpha < -math.pi:
                alpha += 2*math.pi

            tot_error += alpha

            # Move backwards if the goal is behind the robot
            if abs(alpha)>math.pi/2:
                v_des = -self.v_desired
            else:
                v_des = self.v_desired
            
            # Calculate the control inputs
            v_error = v_des - self.cmd_vel.linear.x #TODO: get true vel from robot data instead of vel command
            
            '''
            if error_plots:
                vel_errors.append(v_error)
                arr = np.array([x-target_x, y-target_y])
                min_idx = np.argmin(abs(arr))
                pos_errors.append(arr[min_idx])
            '''
                
            change_dir = (abs(alpha)>0.05*math.pi/2)
            self.cmd_vel.linear.x = (v_des + self.Kv*v_error)*(not change_dir) #only apply linear vel if aligned enough with target to avoid avershoot of trajectory at changes in direction
            self.cmd_vel.angular.z = -self.Ka*(alpha+2*change_dir) + self.ki*tot_error #angular correction, turn more if change in direction
            
            # Adjust the angular velocity to maintain approximately constant linear velocity
            if distance < 0.5:
                self.cmd_vel.angular.z = self.cmd_vel.angular.z*(distance/0.5)

            
        else:
            rospy.loginfo('Goal reached')
            self.goal_reached = True
            self.set_cmd_vel(0,0)
            tot_error = 0
            #vel_fig = plt.figure()
            #plt.plot(vel_errors)
            #plt.show()
            #vel_fig.savefig(current_dir+'/catkin_ws/src/edymobile/edy_controller/figures/vel_error_'+str(target_id)+'.png')
            #pos_fig = plt.figure()
            #plt.plot(pos_errors)
            #plt.show()
            #pos_fig.savefig(current_dir+'/catkin_ws/src/edymobile/edy_controller/figures/pos_error_'+str(target_id)+'.png')

    def poseCallBack(self, odom_msg):
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        (roll, pitch, self.theta) = tf.transformations.euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])




 



def talker():

    global target_id, path

    # Initialize robot
    Edymobile = DiffDriveRobot()  

    rospy.init_node('edy_controller_node', anonymous=True)

    # Subscribe to robot odometry information
    rospy.Subscriber('/edymobile/odom', Odometry, Edymobile.poseCallBack)

    # Publish Diff Drive Twist message velocity command
    vel_pub = rospy.Publisher('edymobile/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz


    # TODO: 
    Edymobile.set_target(path[0][0], path[0][1])

    while not rospy.is_shutdown() and Edymobile.target_x is not None and Edymobile.target_y is not None:
        Edymobile.compute_vel()
        vel_pub.publish(Edymobile.cmd_vel)
        rospy.loginfo(Edymobile.cmd_vel)
        if Edymobile.goal_reached == True:
            rospy.loginfo('Goal reached')
            target_id +=1
            if target_id < len(path):
                Edymobile.set_target(path[target_id][0], path[target_id][1])
                Edymobile.goal_reached = False
            else:
                rospy.loginfo('Final goal reached')
                break
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo('fail')
        pass



