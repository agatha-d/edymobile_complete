#!/usr/bin/env python3
'''
Simple controller node for a differential drive robot to be used on a circuit with only straight paths
----------------------------------------------------------------------------------------------------------

Description: 
The controller encourages a constant linear velocity since the paths are computed by a fleet manager thanks to 
the Hungarian algorithm which requires a constant velocity for all agents. Since the algorithm ignores the time 
required for turning, the paths will be recomputed every time a robot has to change direction, and the controller
encourages the robots to turn in place.

Subscribed topics:
- edymobile/move_base_goal (edymobile can be replaced yith the desired robot name): Position of the next target
- edymobile/odom: Current robot odometry information

Published topics:
- edymobile/cmd_vel: Velocity command for the differential drive robot

Author: Agatha Duranceau
Contact: agatha.duranceau@yahoo.fr

'''


import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseGoal
import tf
import numpy as np
import matplotlib.pyplot as plt
import os



VERBOSE = False
DIST_THRESH = 0.3



class DiffDriveRobot:

    def __init__(self, robot_name = 'edymobile', init_pos = [-3.18,-3.595], init_heading = 0.0, init_vel_cmd = [0.0, 0.0], v_cst = 0.1):
        self.robot_name = robot_name
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
        self.goal = MoveBaseGoal()
        # TODO: modify to only use the movebase goal format
        self.target_x = None
        self.target_y = None
        self.goal_reached = False
        self.dist_to_goal = None
        # Control gains
        self.Ka = 0.1
        self.Kv = 0.2
        self.ki = 0.00008
        self.kd = 0.001
        self.tot_error = 0.0

    def set_target(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
        self.goal_reached = False

    def set_cmd_vel(self, v, w):
        self.cmd_vel.linear.x = v
        self.cmd_vel.angular.z = w

    def compute_vel(self):

        # Calculate the linear error
        self.dist_to_goal = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)

        if VERBOSE:
            rospy.loginfo(self.robot_name+': Current x: '+str(self.x))
            rospy.loginfo(self.robot_name+': New x target: '+ str(self.target_x))
            rospy.loginfo(self.robot_name+': Current y: '+str(self.y))
            rospy.loginfo(self.robot_name+': New y target: '+str(self.target_y))

        # must use same condition as fleet manager, otherwise might remain stuck
        if abs(self.target_x - self.x)>DIST_THRESH or abs(self.target_y - self.y)>DIST_THRESH or min(abs(self.target_y - self.y), abs(self.target_x - self.x))>0.1: #self.dist_to_goal > 0.4:

            # Calculate the heading to the target
            heading = math.atan2(self.target_y - self.y, self.target_x - self.x)

            # Calculate the angular error
            alpha = heading - self.theta

            if VERBOSE:
                rospy.loginfo(self.robot_name+': heading to goal '+ str(heading))
                rospy.loginfo(self.robot_name+': theta robot in map '+ str(self.theta))

            
            # Normalize the angular error to the range [-pi, pi]
            if abs(alpha) > math.pi:
                alpha -= 2*math.pi
            elif alpha < -math.pi:
                alpha += 2*math.pi
            

            if VERBOSE:
                rospy.loginfo(self.robot_name+': angular error: '+ str(alpha))

            self.tot_error += alpha

            rospy.loginfo(self.robot_name+': heading to goal '+ str(heading))
            rospy.loginfo(self.robot_name+': theta robot in map '+ str(self.theta))
            rospy.loginfo(self.robot_name+': angular error: '+ str(alpha))

            # Move backwards if the goal is behind the robot
            if abs(alpha)>0.5*math.pi:
                v_des = -self.v_desired
                rospy.loginfo(self.robot_name+' Should move backwards')
            else:
                v_des = self.v_desired
            
            # Calculate the control inputs
            # TODO: get true vel from robot data instead of vel command
            v_error = v_des - self.cmd_vel.linear.x 

                
            change_dir = (abs(alpha)>0.05*math.pi and abs(alpha)<0.8*math.pi)

            if VERBOSE:
                rospy.loginfo(self.robot_name+': change dir ? '+str(change_dir))

            self.cmd_vel.linear.x = (v_des + self.Kv*v_error)*(not change_dir) #only apply linear vel if aligned enough with target to avoid avershoot of trajectory at changes in direction
            self.cmd_vel.angular.z = -self.Ka*(alpha+4*np.sign(alpha)*change_dir)*np.sign(v_des) #- self.ki*self.tot_error #angular correction, turn more if change in direction
            

            # Adjust the angular velocity to maintain approximately constant linear velocity
            #if self.dist_to_goal < 0.5 and not change_dir:
            #    self.cmd_vel.angular.z = self.cmd_vel.angular.z*(self.dist_to_goal/0.5)

            
        else:
            # stop is the goal has been reached
            if VERBOSE:
                rospy.loginfo(self.robot_name+': Goal reached')
            rospy.loginfo(self.robot_name+': Goal reached')
            self.goal_reached = True
            self.set_cmd_vel(0,0)
            self.tot_error = 0

    def poseCallBack(self, odom_msg):
        '''
        Get pose information from robot odometry topic
        '''
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        (roll, pitch, self.theta) = tf.transformations.euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])

    def goalCallBack(self, moveBaseGoal_msg):
        '''
        Get next goal sent by fleet manager
        '''
        self.goal = moveBaseGoal_msg
        #TODO: directly use goal in functions
        self.target_x = self.goal.target_pose.pose.position.x
        self.target_y = self.goal.target_pose.pose.position.y



 



def controller_node():

    # Initialize robot
    Edymobile = DiffDriveRobot()  
    Edymobile.robot_name = rospy.get_namespace()

    # Initialize ROS node
    rospy.init_node('edy_controller_node', anonymous=True)

    # ################# ROS Subscribers ################# 
    # Subscribe to robot odometry information and update pose
    rospy.Subscriber(Edymobile.robot_name+'odom', Odometry, Edymobile.poseCallBack)
    # Subscribe to goal information from fleet manager: takes next checkpoint in path as goal and update robot goal
    rospy.Subscriber(Edymobile.robot_name+'move_base/goal', MoveBaseGoal, Edymobile.goalCallBack)


    # ################# ROS Publishers ###################
    # Publish Diff Drive Twist message velocity command
    vel_pub = rospy.Publisher(Edymobile.robot_name+'cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10) # 10Hz

    while not rospy.is_shutdown():
        # If goals have not beed received yet, do not move
        if  Edymobile.target_x is None or Edymobile.target_y is None: 
            Edymobile.set_cmd_vel(0, 0)
        else:
            Edymobile.compute_vel()
            if VERBOSE:
                rospy.loginfo(Edymobile.robot_name+' Forward Velocity command: '+ str(Edymobile.cmd_vel.linear.x) + ' angular: '+ str(Edymobile.cmd_vel.angular.z))
            rospy.loginfo(Edymobile.robot_name+' Forward Velocity command: '+ str(Edymobile.cmd_vel.linear.x) + ' angular: '+ str(Edymobile.cmd_vel.angular.z))

        vel_pub.publish(Edymobile.cmd_vel)

        rate.sleep()

if __name__ == '__main__':
    try:
        controller_node()
    except rospy.ROSInterruptException:
        rospy.loginfo('fail')
        pass



