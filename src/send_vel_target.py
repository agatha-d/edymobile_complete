#!/usr/bin/env python
# Software License Agreement (BSD License)


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
import math

msg = Twist()



# Initialize the robot's position and velocity
x = 0
y = 0
theta = 0
v = 0
w = 0

# Set the target position
target_x = 5
target_y = 0

# Set the control gains
Kp = 1
Ka = 1
Kv = 0.1

# Set the desired constant velocity
v_desired = 0.5



def compute_vel(current_pose, goal):

    # Loop until the robot reaches the target position
    while math.sqrt((target_x - x)**2 + (target_y - y)**2) > 0.1:
        # Calculate the heading to the target
        heading = math.atan2(target_y - y, target_x - x)
        
        # Calculate the angular error
        alpha = heading - theta
        
        # Normalize the angular error to the range [-pi, pi]
        if alpha > math.pi:
            alpha -= 2*math.pi
        elif alpha < -math.pi:
            alpha += 2*math.pi
        
        # Calculate the linear error
        distance = math.sqrt((target_x - x)**2 + (target_y - y)**2)
        
        # Calculate the control inputs
        v_error = v_desired - v
        v = v + Kv*v_error
        w = Ka*alpha
        
        # Adjust the angular velocity to maintain approximately constant linear velocity
        if distance < 0.5:
            w = w*(distance/0.5)
        
        # Send the control inputs to the robot
        # (Here you would implement code to actually control the robot's motors)
        
        # Update the robot's position and heading based on the control inputs
        x += v*math.cos(theta)
        y += v*math.sin(theta)
        theta += w
        
        # Normalize the heading to the range [-pi, pi]
        if theta > math.pi:
            theta -= 2*math.pi
        elif theta < -math.pi:
            theta += 2*math.pi



def talker():
    pub = rospy.Publisher('edymobile/cmd_vel', Twist, queue_size=10)
    rospy.init_node('send_vel_target', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg.linear.x = 0
    #msg.linear.y = o #doesn't make sense for a differential drive robot
    msg.angular.z = 0
    rospy.loginfo('sending velocity target')
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

