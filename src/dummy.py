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
#from nav_msgs import Odometry


# Target path nodes list
path = [[-2.0,-0.3], [0,-0.3]]

#importing the os module
import os

#to get the current working directory
current_dir = os.getcwd()


plt.figure()
plt.plot(path)
plt.savefig(current_dir+"/catkin_ws/src/edymobile/edy_controller/figures/vel_error_.png")



