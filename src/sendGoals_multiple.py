#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goal(x, y):
    # Create a move_base client
    client = actionlib.SimpleActionClient('/robot2/move_base', MoveBaseAction)
    # Wait for the action server to start up
    client.wait_for_server()

    # Set up the goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0


    # Send the goal and wait for completion
    client.send_goal(goal)
    client.wait_for_result()

if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('send_goal', anonymous=True)

        # Send some goals
        send_goal(-2, 1)  # send goal to (0, 0)
        #send_goal(1, 1)  # send goal to (1, 1)
    except rospy.ROSInterruptException:
        pass