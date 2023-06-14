# Fleet Manager script: it generates the goals for the robots, compute non-collinding paths and send them to the robots


# Librairies
from pathlib import Path
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
import random
import numpy as np
#from hungarian_algorithm import hungarian_algorithm, ans_calculation
from munkres import Munkres # hungarian algorithm
from cbs import CBSSolver
from prioritized import PrioritizedPlanningSolver

from nav_msgs.msg import Odometry # to get the position of the robot from Gazebo
from geometry_msgs.msg import *
from tf.msg import *

import configparser


import rospy
#import time 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # to send goals to the robots


# VARIABLES
time_begin = 0.0
checkpoint_thres = 0.4 # threshold to consider that the robot reached the checkpoint
scale_var = 2.1 # scale to convert the coordinates from the MAPF frame to the GAZEBO frame

class Robot():

    def __init__(self, nameRobot, initX, initY, number):
        self.name = nameRobot
        self.id = number
        # in MAPF frame
        self.posX = initX
        self.posY = initY
        self.goalX = [initX] # the goals are stored in descending order of priority: the first goal to reach (the collecting station) is the last in the list
        self.goalY = [initY]
        self.path = []
        # in GAZEBO frame
        self.posX_gazebo = 0 
        self.posY_gazebo = 0 
        self.goal_tempX = 0 # contains the checkpoint to reach
        self.goal_tempY = 0
        self.path_gazebo = []
        # States
        self.state_checkpoint = 0 # Arrived to the checkpoint? 0: arrived, 1: on the way
        self.state_mission = 0 # All the tasks are achieved? 0: free, 1: still is performing a mission
        self.state_goal = 0 # Is the current task achieved? 0: finished the task, 1: is performing the task
        self.timeGoal=[] # we store the times to achieve tasks


    def Position(self, odom_data):
        # get the real time position of the robot from gazebo simulation
        # input: odom data from the robot
        # output: position of the robot in gazebo frame
        self.posX_gazebo = round(odom_data.pose.pose.position.x, 2)
        self.posY_gazebo = round(odom_data.pose.pose.position.y, 2)


   
    def base_MAPF2Gazebo(self):
        # Change of base from MAPF frame to GAZEBO
        # input: paths in the MAPF frame
        # output: paths in the GAZEBO frame
        self.path_gazebo.clear()
        matrix_chg_base = np.array([[0, 1, -3.6],[-1, 0, 4.5], [0, 0, 1]])
        for i in range(0, len(self.path)): 
            path_temp = (self.path[i][0]/scale_var, self.path[i][1]/scale_var, 1) # we scale it by 2.1
            produit = matrix_chg_base @ path_temp
            self.path_gazebo.append((round(produit[0], 2), round(produit[1], 2)))
        print('Path in converted Gazebo coordinate')


    def base_Gazebo2MAPF(self):
        # Change of base from GAZEBO frame to MAPF frame
        # input: paths in the GAZEBO frame
        # output: paths in the MAPF frame
        matrix_chg_base = np.array([[0, -1, 4.5],[1, 0, 3.6], [0, 0, 1]])
        path_temp = (self.posX_gazebo, self.posY_gazebo, 1)
        produit = matrix_chg_base @ path_temp
        self.posX = int(round(produit[0]*scale_var, 0)) # we scale it by 2.1
        self.posY = int(round(produit[1]*scale_var, 0))
        
    # Envoyer position au robot
    def send_goal(self):
        # Send the goal to the robot with move_base

        # Create a move_base client
        name = '/'+ self.name + '/move_base'
        client = actionlib.SimpleActionClient(name, MoveBaseAction)
        # Wait for the action server to start up
        client.wait_for_server()

        # Set up the goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.orientation.w = 1.0 # quaternion defining the orientation of the robot
        
        if len(self.path_gazebo)> 1: # if there is a path to follow
            self.state_checkpoint = 1 # the robot is on the way to the checkpoint
            self.path_gazebo.pop(0)  # we remove the previous checkpoint   
            goal.target_pose.pose.position.x = self.path_gazebo[0][0] # we send the new checkpoint
            goal.target_pose.pose.position.y = self.path_gazebo[0][1]
            self.goal_tempX = self.path_gazebo[0][0] # we store the checkpoint we are trying to reach
            self.goal_tempY = self.path_gazebo[0][1]
            print('new checkpoint or goal for ', self.name, self.path_gazebo[0][0], self.path_gazebo[0][1])
            
        else: # if there is no path to follow or that the robot reach the goal
            duration = rospy.Time.now()-time_begin # we store the time taken to reach the goal
            self.timeGoal.append(duration.to_sec())
            print(self.name, 'arrived at the goal', self.timeGoal[-1])
            goal.target_pose.pose.position.x = self.path_gazebo[0][0] 
            goal.target_pose.pose.position.y = self.path_gazebo[0][1]
            self.state_goal = 0 # the robot finished its tasks

        # Send the goal and wait for completion
        client.send_goal(goal)
        #client.wait_for_result()
    
    '''
    # find where the direction change from self.path  
    def change_direction(self):
        self.checkpoint.clear()
        print('path', self.path, len(self.path))
        for i in range(0, len(self.path)-1):
            if len(self.path) == 1:
                self.checkpoint.append(self.path[0])
            else:
                if self.path[i][0] != self.path[i+1][0] or self.path[i][1] != self.path[i+1][1]:
                    print('here')
                    self.checkpoint.append(self.path[i])    
        print('checkpoint', self.checkpoint)
    '''
    
class Station:
    def __init__(self, name, x, y):
        self.name = name
        # position in the MAPF frame
        self.x = x 
        self.y = y

# MAP GENERATION (FOR A*)
def import_mapf_instance(filename):
    # input: text file representing the map
    # output: table containing free and occupied cells

    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    print('Map generated')
    return my_map


#TASK GENERATION
def task_generator(stations):
    # input: lists of station element 
    # output: list containing the tasks to perform called tasks_to_do
    # tasks_to_do is a list containing: [[Collection station][Delivery station]]
    # tasks_to_do[0][0],tasks_to_do[1][0] : contains the collection and delivery station of the first task
    # the tasks are represented as number where 0 is the first element of stations (discovery)
    
    #stations=discovery,5,2;omni,5,10;synt,9,10;sfc,0,10
    
    tasks = [[[0,3],[1,2]], [[1,2],[0,3]], [[1,0],[3,2]], [[0,3, 1],[1,2, 0]], [[3, 1,2],[1, 0,3]], [[2, 1,0],[0,3,2]], [[3, 2, 1,0],[1,0,3,2]]]
    #tasks_to_do = random.choice(tasks)
    tasks_to_do = [[1,0],[2,1]]  #-- Scenario 1
    #tasks_to_do = [[0,2,3],[2,3,0]]  #-- Scenario 2
    #tasks_to_do = [[0,2,3,1],[1,3,0,2]]  #-- Scenario 3
    

    return tasks_to_do[0], tasks_to_do[1]

# TASK ALLOCATION
def task_allocation(stations, tasks_to_do, next_tasks_to_do):
    #input: list of station, list of tasks: collecting stations in task_to_do, delivery station in next_tasks_to_do
    #output: list goals for each robot; the robot that don't have a task stay at their current position (the goal is their current position)
    # Function that allocate the tasks to the robots by minimizing the total distance travelled by the robots
    # it updates state_goal and state_mission if robots were assigned tasks

    starts_robots = [(robot.posX, robot.posY) for robot in robots] # list of the current position of the robots (in MAPF frame)
    goals_allocation = []
    cost_matrix = np.zeros((len(tasks_to_do), len(robots))) # initialization of the cost matrix

    # 1. OPTIMIZATION - Hungarian algorithm

    # update goals_allocation with the collection stations to reach
    for i in range(0, len(tasks_to_do)):
        goals_allocation.append((stations[tasks_to_do[i]].x, stations[tasks_to_do[i]].y))
    heuristics = []
    for goal in goals_allocation:
        # compute the heuristics for each goal using  compute_heuristic function from single_agent_planner.py
        heuristics.append(compute_heuristics(my_map, goal)) 
    
    # compute the cost matrix
    for i in range(0, len(tasks_to_do)):
        for j in range(0, len(robots)):
            # compute the path for each robot performing each goal using a_star function from single_agent_planner.py
            path = a_star(my_map, starts_robots[j], goals_allocation[i], heuristics[i], 0, []) 
            cost_matrix[i][j]=len(path)

    # apply the Hungarian algorithm
    m = Munkres()
    indexes = m.compute(cost_matrix) # it outputs a list of optimized assignments indexes = [(task, robot)]
    
    # 2. Update goals for the robots performing a task
    for j in range(0, len(indexes)):
        robot_index = indexes[j][1]
        # update the delivery station
        robots[robot_index].goalX.append(stations[next_tasks_to_do[indexes[j][0]]].x) 
        robots[robot_index].goalY.append(stations[next_tasks_to_do[indexes[j][0]]].y)
        # update the collection station
        robots[robot_index].goalX.append(stations[tasks_to_do[indexes[j][0]]].x) 
        robots[robot_index].goalY.append(stations[tasks_to_do[indexes[j][0]]].y)
        robots[robot_index].state_goal = 1 # update the goal state
        robots[robot_index].state_mission = 1 # update the mission state
        print(robots[robot_index].name, 'is taking the task', j)

    # 3. Update goals_allocation list containing the goals for each robot
    goals_allocation = [(robot.goalX[-1], robot.goalY[-1]) for robot in robots]

    return goals_allocation


# Goal reached
# Check if goal is reached, if it's the case, remove from goal list
def update_goal(robot, goals):
    # input: robot, list of goals
    # update the list of goal when all the goals are reached.
    
   
    if len(robot.goalX) > 1: # if the robot didn't finish his mission: it has more than one goal
        # remove the goal reached from the robot element 
        robot.goalX.pop(-1)
        robot.goalY.pop(-1)
        # update the goal state
        robot.state_goal = 1
    else:
        robot.state_mission = 0
    # update the new goal in goals_allocation
    goals[robot.id]=(robot.goalX[-1], robot.goalY[-1])
    print('new goal for', robot.name, 'is', robot.goalX[-1], robot.goalY[-1])

def check_checkpoint(robots):
    # check checkpoint is reached for the robots performing a task
    # if the robot close from the checkpoint, we update its state_checkpoint 
    for robot in robots:
        if abs(robot.posX_gazebo-robot.goal_tempX)<checkpoint_thres and abs(robot.posY_gazebo-robot.goal_tempY)<checkpoint_thres and robot.state_goal == 1:
            print(robot.name)
            print('distance to goal X', abs(robot.posX_gazebo-robot.goal_tempX))
            print('distance to goal Y', abs(robot.posY_gazebo-robot.goal_tempY))
            print('next checkpoint')
            robot.state_checkpoint = 0
        
def send_checkpoint(robots):
    # check if all the robots reached their checkpoint
    # if it's the case, we send the next checkpoint calling send.goal
    
    for robot in robots:
        if robot.state_checkpoint == 1: # if the robot didn't reach the checkpoint, we don't send the next goal
            return True
        
    for robot in robots: # all the robots are at the checkpoint
        if robot.state_goal == 1: # we send the goal only to the robots performing a task 
            robot.send_goal()
    return True

def state_task(robots):
    # Check if all robots achieved their task
    # if it's the case, we return 0, else we return 1
    for robot in robots:
         if robot.state_goal == 1:
                return 1
    return 0


def state_mission(robots):
    # Check if all robots achieved their mission (collecting, delivering and returning to the base) 
    # if it's the case, we return 0, else we return 1
    for robot in robots:
         if robot.state_mission == 1:
                return 1
    return 0

'''def checkpoint_generator(robot):
    # Create checkpoint from the path of the robot where the direction changes
    checkpoint = []
    checkpoint.append(robot.path[0])
    for i in range(len(robot.path)-2):
        if  (robot.path[i+1][0]-robot.path[i][0])*(robot.path[i+2][0]-robot.path[i+1][0])+(robot.path[i+1][1]-robot.path[i][1])*(robot.path[i+2][1]-robot.path[i+1][1]) == 0:
            checkpoint.append(robot.path[i+1])
    checkpoint.append(robot.path[-1])
    robot.path.clear()
    robot.path = checkpoint.copy()
'''
    

def create_robots_from_ini_file(file_path):
    # Creates the list robots containing robot element from the ini file
    config = configparser.ConfigParser()
    config.read(file_path)
    global_section = config['Global']
    number_robot = int(global_section['number_robot'])
    robots = []
    for i in range(1, number_robot + 1):
        robot_config = config['Robots'][f'robot{i}']
        x, y, station_id = map(int, robot_config.split(','))
        robot = Robot("robot" + str(i) , x, y, station_id)
        robots.append(robot)
    return robots


def create_stations_from_ini_file(file_path):
    # Creates the list stations containing station element from the ini file

    config = configparser.ConfigParser()
    config.read(file_path)
    stations_list = []
    stations_config = config['Stations']['stations']
    station_configs = stations_config.split(';')
    for station_config in station_configs:
        name, x, y = station_config.split(',')
        station = Station(name.strip(), int(x), int(y))
        stations_list.append(station)
    return stations_list
          
def create_all_from_ini_file(file_path):
    # Creates the list robots and stations containing robot and station element from the ini file
    robots = create_robots_from_ini_file(file_path)
    stations = create_stations_from_ini_file(file_path)
    return robots, stations

if __name__ == '__main__':
    try:
        goals = []
        starts = []

        rospy.init_node('odometry', anonymous=True) #make node 

        # init robots and stations with  initial position and name
        robots, stations = create_all_from_ini_file('configrobots.ini')

        # Robots and stations positions:
        for robot in robots:
            print(f"Robot {robot.name}: ({robot.posX}, {robot.posY})")

        for station in stations:
            print(f"Stations {station.name}: ({station.x}, {station.y})")

        # init map
        my_map = import_mapf_instance('circuit_MAPF.txt')
        
        # generate the tasks
        tasks_to_do, next_tasks_to_do = task_generator(stations)
        print('task generated')
        
        # task allocation
        goals = task_allocation(stations, tasks_to_do, next_tasks_to_do)
        print('Goals allocated', goals)

        # Find paths using MAPF solver: from cbs.py
        starts = [(robot.posX, robot.posY) for robot in robots]
        print('Current position: ', starts)
        '''solver = PrioritizedPlanningSolver(my_map, starts, goals)
        paths = solver.find_solution()'''
        cbs = CBSSolver(my_map, starts, goals)
        paths = cbs.find_solution('--disjoint')
                      
        # Update paths
        for robot in robots:
            robot.path = paths[robots.index(robot)]
            print(robot.name, robot.path)
    
        print('Paths updated') 

        # Change frame MAPF 2 Gazebo (for path)
        robots[0].base_MAPF2Gazebo()
        robots[1].base_MAPF2Gazebo()
        robots[2].base_MAPF2Gazebo()
        robots[3].base_MAPF2Gazebo()
        robots[4].base_MAPF2Gazebo()


        # Get position (in the Gazebo frame)
        rospy.Subscriber('/robot1/odom',Odometry,robots[0].Position)
        rospy.Subscriber('/robot2/odom',Odometry,robots[1].Position)
        rospy.Subscriber('/robot3/odom',Odometry,robots[2].Position)
        rospy.Subscriber('/robot4/odom',Odometry,robots[3].Position)
        rospy.Subscriber('/robot5/odom',Odometry,robots[4].Position)
    

        # Initialize the fist temporary goal as initial position in the gazebo frame 
        robots[0].goal_tempX = robots[0].posX_gazebo
        robots[0].goal_tempY = robots[0].posY_gazebo
        robots[1].goal_tempX = robots[1].posX_gazebo
        robots[1].goal_tempY = robots[1].posY_gazebo
        robots[2].goal_tempX = robots[2].posX_gazebo
        robots[2].goal_tempY = robots[2].posY_gazebo
        robots[3].goal_tempX = robots[3].posX_gazebo
        robots[3].goal_tempY = robots[3].posY_gazebo
        robots[4].goal_tempX = robots[4].posX_gazebo
        robots[4].goal_tempY = robots[4].posY_gazebo

        time_begin = rospy.Time.now() # set the time to 0
        while True:
        
            print(' ')
            print('..................................................................')

            state_task_var = state_task(robots) #
            state_miss = state_mission(robots)

            if state_task_var == 0: # all the robots finished their current task
                
                for robot in robots:
                    print('state_goal', robot.name, robot.state_goal)
                
                # Update goals
                update_goal(robots[0], goals)
                update_goal(robots[1], goals)
                update_goal(robots[2], goals)
                update_goal(robots[3], goals)
                update_goal(robots[4], goals)

                # Compute the new paths
                # 1. Update MAPF position
                robots[0].base_Gazebo2MAPF()
                robots[1].base_Gazebo2MAPF()
                robots[2].base_Gazebo2MAPF()
                robots[3].base_Gazebo2MAPF()
                robots[4].base_Gazebo2MAPF()
    
                # 2. Get the new starts
                starts = [(robot.posX, robot.posY) for robot in robots]
                print('Current position: ', starts)
                # 3. Compute the new paths
                '''solver = PrioritizedPlanningSolver(my_map, starts, goals)
                paths = solver.find_solution()'''
                cbs = CBSSolver(my_map, starts, goals)
                paths = cbs.find_solution('--disjoint')
                # 4. Update the paths
                robots[0].path = paths[0] 
                robots[1].path = paths[1]
                robots[2].path = paths[2]
                robots[3].path = paths[3]
                robots[4].path = paths[4]
     
                # 5. Change paths frame to Gazebo coordinates
                robots[0].base_MAPF2Gazebo()
                robots[1].base_MAPF2Gazebo()
                robots[2].base_MAPF2Gazebo()
                robots[3].base_MAPF2Gazebo()
                robots[4].base_MAPF2Gazebo()
                print('Paths updated')
            
            if state_miss == 0:
                print('Mission completed')
                for robot in robots:
                    print(robot.name, robot.timeGoal)
                break

            # check if checkpoint is reached
            check_checkpoint(robots)
            # if all the robots reached their checkpoint, send a new one
            send_checkpoint(robots)     
      
            # Wait 1 sec
            rospy.sleep(1)
            
            
        rospy.spin()
       
            

        
    except rospy.ROSInterruptException:
        pass



