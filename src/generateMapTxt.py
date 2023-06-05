# Generate map.txt file from a png image

#!/usr/bin/env python
#import roslib; roslib.load_manifest('Phoebe')
import rospy
#import irobot_create_2_1

from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal




from PIL import Image

from numpy import asarray

# load the image

image = Image.open('circuit.png')

# convert image to numpy array

data = asarray(image)

print(all(data[0][0]-[255, 255, 255, 255]))
print(all(data[100][100]-[254, 254, 254, 254]))

# summarize shape

print(data.shape)


# if pixel gray or black print @ else print . for each pixel of the image
# and save the data in a text file:
file = open("circuit-test.txt","r+")
for i in range (0, data.shape[0]):
    for j in range (0, data.shape[1]):
        #print(data[i][j]-[254, 254, 254, 254],all((data[i][j]-[254, 254, 254, 254])))
        
        if sum(data[i][j]-[255, 255, 255, 255]) == 0:
            print('. ', end = '')
            file.write('. ')
        else:
            print('@ ', end = '')
            file.write('@ ')
        
    print('')
    file.write('\n')

file.close()


print(data[0][0]-[254, 254, 254, 254], all(data[0][0]-[254, 254, 254, 254]))
print(data[100][100]-[255, 255, 255, 255], sum(data[100][100]-[255, 255, 255, 255]))

'''
# create Pillow image

image2 = Image.fromarray(data)

print(type(image2))


# summarize image details

print(image2.mode)

print(image2.size)





counter = 0

class Robot:
    x = 0;
    y = 0;
    z = 0;

    def Position(self, odom_data):
        curr_time = odom_data.header.stamp
        self.x = odom_data.pose.pose.position.x
        self.y = odom_data.pose.pose.position.y
        self.z = odom_data.pose.pose.position.z
        #print(self.x)



^
def Position(odom_data, robot):

    #global counter
    #rospy.sleep(1)
    curr_time = odom_data.header.stamp
    pose = odom_data.pose.pose.position.x #  the x,y,z pose and quaternion orientation
    #counter= counter+1
    robot.x = odom_data.pose.pose.position.x
    robot.y = odom_data.pose.pose.position.y
    robot.z = odom_data.pose.pose.position.z
   
    print(pose)






def transformation(tf_data):
    global counter
    rospy.sleep(1)
    transform = tf_data.transform
    print('transform')
    print(transform)


def begin():
    while not rospy.is_shutdown():
        rospy.init_node('odometry', anonymous=True) #make node 
        rospy.Subscriber('/robot1/odom',Odometry,Position)



if __name__ == "__main__":
    robot1 = Robot()
    robot2 = Robot()

    rospy.init_node('odometry', anonymous=True) #make node 


    while True:
        rospy.Subscriber('/robot1/odom',Odometry,robot1.Position)
        rospy.Subscriber('/robot2/odom',Odometry,robot2.Position)

        print(robot1.x)
        print(robot2.y)
    #print(robot1.x)
    #rospy.loginfo(robot1.x)


    rospy.spin() # not really necessary because we have while not rospy.is_shutdown()




def send_goal(robot):
    # Create a move_base client
    name = '/robot1/move_base'
    client = actionlib.SimpleActionClient('/robot1/move_base', MoveBaseAction)
    # Wait for the action server to start up
    client.wait_for_server()

    #print('Goal send for robot', robot )

    # Set up the goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    
    goal.target_pose.pose.position.x = 1
    goal.target_pose.pose.position.y = 1
    goal.target_pose.pose.orientation.w = 1.0



    # Send the goal and wait for completion
    client.send_goal(goal)
    client.wait_for_result()

if __name__ == "__main__":
    try:
        rospy.init_node('send_goal', anonymous=True)

        send_goal('robot1')
    except rospy.ROSInterruptException:
        pass
'''
'''
import random

stations = { 'discovery' : (5, 2), 'omni' : (5, 10), 'synt' : (9, 10), 'sfc' : (0, 10) }
robots = { '0' : (17, 1), '1' : (19, 1), '2' : (3, 5), '3' : (17, 10),'4' : (19, 10)}
number_robot = 5
number_station = 4




def task_generator(number_station, stations):
    # input: number of station and name of stations
    # output: list containing the tasks to perform
    number_task = random.randint(1, number_station)
    tasks_to_do = random.sample(list(stations), k=number_task)
    print('Tasks to do', tasks_to_do)
    next_tasks_to_do = random.sample(list(stations), k=number_task)
    print('Next tasks to do', next_tasks_to_do)

    #tasks_to_do = ['discovery', 'omni', 'synt', 'sfc']
    #next_tasks_to_do = ['discovery', 'synt', 'sfc', 'omni']

    # check element wise if next_tasks_to_do is identical to tasks_to_do and change next_tasks_to_do if its the case 
    for i in range(0, number_task):
        if tasks_to_do[i] == next_tasks_to_do[i]:
            print('Same task ')
            station_list =  list(stations)
            station_list.remove(tasks_to_do[i])
            print('Station list', station_list)
            next_tasks_to_do[i] = random.choice(station_list)
   
    return tasks_to_do, next_tasks_to_do

# generate the tasks
tasks_to_do, next_tasks_to_do = task_generator(number_station, stations)
print('task generated')
print(tasks_to_do, next_tasks_to_do)
'''


