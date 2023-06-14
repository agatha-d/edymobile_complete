#!/usr/bin/env python3
# Software License Agreement (BSD License)


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


#TODO: remove
global tot_error
tot_error = 0.0


# for saving error plots and check if trajectory diverges from a straight line
error_plots = True
vel_errors = []
pos_errors = []
times = []
current_dir = os.getcwd()

class DiffDriveRobot:

    def __init__(self, robot_name = 'edymobile', init_pos = [-3.18,-3.595], init_heading = 0.0, init_vel_cmd = [0.0, 0.0], v_cst = 0.2):
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
        #goal.target_pose.header.frame_id = "map"
        #goal.target_pose.pose.orientation.w = 1.0
        #goal.target_pose.pose.position.x = self.path_gazebo[0][0]
        #goal.target_pose.pose.position.y = self.path_gazebo[0][1]
        # TODO: modify to only use the movebase goal format
        self.target_x = None
        self.target_y = None
        self.goal_reached = False
        self.dist_to_goal = None
        # Control gains
        self.Ka = 0.1
        self.Kv = 0.2
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
        self.dist_to_goal = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)

        rospy.loginfo(self.robot_name+': New x target: '+ str(self.target_x))
        rospy.loginfo(self.robot_name+': New y target: '+str(self.target_y))
        rospy.loginfo(self.robot_name+': Current x: '+str(self.x))
        rospy.loginfo(self.robot_name+': Current y: '+str(self.y))
        rospy.loginfo(self.robot_name+': distance to goal: '+str(self.dist_to_goal))

        #use same condition as fleet manager
        if abs(self.target_x - self.x)>0.5 or abs(self.target_y - self.y)>0.5 or min(abs(self.target_y - self.y), abs(self.target_x - self.x))>0.1: #self.dist_to_goal > 0.4:

            # Calculate the heading to the target
            heading = math.atan2(self.target_y - self.y, self.target_x - self.x)

            # Calculate the angular error
            alpha = heading - self.theta

            rospy.loginfo(self.robot_name+': heading to goal '+ str(heading))
            rospy.loginfo(self.robot_name+': theta robot in map '+ str(self.theta))
            
            # Normalize the angular error to the range [-pi, pi]
            if alpha > math.pi:
                alpha -= 2*math.pi
            elif alpha < -math.pi:
                alpha += 2*math.pi


            rospy.loginfo(self.robot_name+': angular error: '+ str(alpha))

            tot_error += alpha

            # Move backwards if the goal is behind the robot
            if abs(alpha)>math.pi/2:
                v_des = -self.v_desired
            else:
                v_des = self.v_desired
            
            # Calculate the control inputs
            # TODO: get true vel from robot data instead of vel command
            v_error = v_des - self.cmd_vel.linear.x 

            #rospy.loginfo('vel error')
            #rospy.loginfo(v_error)
            '''
            if error_plots:
                vel_errors.append(v_error)
                arr = np.array([x-target_x, y-target_y])
                min_idx = np.argmin(abs(arr))
                pos_errors.append(arr[min_idx])
            '''
                
            change_dir = (abs(alpha)>0.05*math.pi/2 and abs(alpha)<0.8*math.pi)
            #if not change_dir and abs(alpha)>math.pi/2: #move backwards if goal is behind
            #    alpha = math.pi-alpha

            #rospy.loginfo('change dir?')
            rospy.loginfo(self.robot_name+': cond1 ? '+str(abs(alpha)>0.05*math.pi/2))
            rospy.loginfo(self.robot_name+': cond2 ? '+str(abs(alpha)<0.8*math.pi/2))
            rospy.loginfo(self.robot_name+': change dir ? '+str(change_dir))

            self.cmd_vel.linear.x = (v_des + self.Kv*v_error)*(not change_dir) #+ change_dir*np.sign(v_des)*self.Kv/100 #only apply linear vel if aligned enough with target to avoid avershoot of trajectory at changes in direction
            self.cmd_vel.angular.z = -self.Ka*(alpha+4*np.sign(alpha)*change_dir) + self.ki*tot_error #angular correction, turn more if change in direction
            
            #rospy.loginfo('in compute vel function')
            #rospy.loginfo(self.cmd_vel.linear.x)

            # Adjust the angular velocity to maintain approximately constant linear velocity
            if self.dist_to_goal < 0.5:
            #    rospy.loginfo('Angular vel adjustment')
                self.cmd_vel.angular.z = self.cmd_vel.angular.z*(self.dist_to_goal/0.5)

            
        else:
            rospy.loginfo(self.robot_name+': Goal reached')
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

    def goalCallBack(self, moveBaseGoal_msg):
        self.goal = moveBaseGoal_msg

        #test:#TODO: directly use goal in functions
        self.target_x = self.goal.target_pose.pose.position.x
        self.target_y = self.goal.target_pose.pose.position.y



 



def talker():

    # Initialize robot
    Edymobile = DiffDriveRobot()  
    Edymobile.robot_name = rospy.get_namespace()

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

        vel_pub.publish(Edymobile.cmd_vel)

        rospy.loginfo(Edymobile.robot_name+' Velocity command')
        rospy.loginfo(Edymobile.cmd_vel)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo('fail')
        pass



