#!/usr/bin/env python3
'''
Simple controller node for a differential drive robot to be used on a circuit with only straight paths
----------------------------------------------------------------------------------------------------------

Description: 
This fleet manager generates the goals for the robots, compute non-collinding paths and send them to the robots
The algorithm used for task allocation is Hungarian algorithm which requires a constant velocity for all agents. 
Since the algorithm ignores the time required for turning, the paths will be recomputed every time a robot has 
to change direction. Tho goals sent to the robot controllers are the initial and final checkpoints of the path,
as well as all the intermediate changes in direction along the way. 

The current implementation supports the fontrol of a fleet of 5 robots that must transport samples from and 
to 4 stations, namely { 'discovery' : (5, 2), 'omni' : (5, 10), 'synt' : (9, 10), 'sfc' : (0, 10) } given by their 
position in a hardcoded map.

Subscribed topics:
- robot/odom: (robot can be replaced yith the desired robot name) Current robot odometry information to compute desired paths

Published topics:
- robot/move_base_goal: Position of the next target


Author: Yasmine El Goumi
modified by: Agatha Duranceau, contact: agatha.duranceau@yahoo.fr
'''


from pathlib import Path
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
import random
import numpy as np
from hungarian_algorithm import hungarian_algorithm, ans_calculation
from munkres import Munkres # hungarian algorithm
from cbs import CBSSolver
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *
import configparser
import rospy
from move_base_msgs.msg import MoveBaseGoal



VERBOSE = False
DIST_THRESH = 0.3

class Robot():

    def __init__(self, nameRobot, initX, initY, number):
        self.name = nameRobot
        self.id = number
        # robot initial position
        self.posX = initX
        self.posY = initY
        # the goals are stored in descending order of priority: the first goal to reach (the collecting station) is the last in the list
        self.goalX = [initX]
        self.goalY = [initY]
        self.path = [] # path in MAPF frame
        self.produit = np.array([[0, 1, -3.6],[-1, 0, 4.5], [0, 0, 1]]) @ (self.posX/2.1, self.posY/2.1, 1)
        self.posX_gazebo = round(self.produit[0], 2 )
        self.posY_gazebo = round(self.produit[1], 2 )
        self.goal_tempX = 0
        self.goal_tempY = 0
        self.path_gazebo = []     # path in Gazebo reference frame
        self.change_dir_idx = []  # store indices in path at which there is a change in direction
        # sytem state variables
        self.state_checkpoint = 0 # Arrived to the checkpoint? 0: arrived, 1: on the way
        self.state_mission = 0    # All the tasks are achieved? 0: free, 1: still is performing a mission (one mission = a set of tasks)
        self.state = 0            # Is the current task achieved? 0: finished the task, 1: is performing the task
        self.checkpoint = []

        # ROS Publisher for sending goal to controller
        self.pub = rospy.Publisher('/' +self.name+ '/move_base/goal',MoveBaseGoal, queue_size=10)


    def Position(self, odom_data):
        '''
        Get data from odometry topic
        '''
        self.posX_gazebo = round(odom_data.pose.pose.position.x, 2)
        self.posY_gazebo = round(odom_data.pose.pose.position.y, 2)


    def base_MAPF2Gazebo(self):
        '''
        Change of base MAPF 2 GAZEBO
        - input : paths in the MAPF frame
        - output: paths in the GAZEBO frame
        '''
        self.path_gazebo.clear()
        matrix_chg_base = np.array([[0, 1, -3.6],[-1, 0, 4.5], [0, 0, 1]])
        for i in range(0, len(self.path)): 
            path_temp = (self.path[i][0]/2.1, self.path[i][1]/2.1, 1)
            produit = matrix_chg_base @ path_temp
            self.path_gazebo.append((round(produit[0], 2), round(produit[1], 2)))
        if VERBOSE:
            print('Path in converted Gazebo coordinate')


    def base_Gazebo2MAPF(self):
        '''
        Change of base GAZEBO 2 MAPF
        - input : paths in the MAPF frame
        - output: paths in the GAZEBO frame
        '''
        matrix_chg_base = np.array([[0, -1, 4.5],[1, 0, 3.6], [0, 0, 1]])
        path_temp = (self.posX_gazebo, self.posY_gazebo, 1)
        produit = matrix_chg_base @ path_temp
        self.posX = int(round(produit[0]*2.1, 0))
        self.posY = int(round(produit[1]*2.1, 0))
        if self.posX > 19:
            self.posX = 19
        if self.posY > 12:
            self.posX = 12


    def send_goal(self):
        '''
        Publish robot next goal or checkpoint
        '''
        if VERBOSE:
            print('----SEND GOAL----')

        # Set up the goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.orientation.w = 1.0 # (not used)
        self.state_checkpoint = 1 # the robot is on the way to the checkpoint
        goal.target_pose.pose.position.x = self.path_gazebo[0][0]
        goal.target_pose.pose.position.y = self.path_gazebo[0][1]
        self.goal_tempX = self.path_gazebo[0][0]
        self.goal_tempY = self.path_gazebo[0][1]
        if VERBOSE:
            print('new checkpoint or goal for ', self.name, self.path_gazebo[0][0], self.path_gazebo[0][1])

        # If the robot has reached its gooal, make it free again
        if len(self.path_gazebo)< 1:
            if VERBOSE:
                print('same pos', self.path_gazebo[0][0], self.path_gazebo[0][1])
            self.state = 0 # the robot is free again

        # Publish the goal 
        self.pub.publish(goal)

        return(goal)
    
    
class Station:
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y


def import_mapf_instance(filename):
    '''
    MAP GENERATION (FOR A*)
    - input : text file representing the map
    - output: table containing free and occupied cells
    '''
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    if VERBOSE:
        print('Map generated')
    return my_map


def task_generator(stations):
    '''
    TASK GENERATION: generate random tasks for the robots

    - input:  stations: 0: discovery (5,2) ; 1: omni (5,10) ; 2: synt (9,10) ; 3: sfc (0,10)
    - output: tasks_to_do is a list containing: [[Collection station(s)][Delivery station(s)]]

    example: [[0,3],[1,2]] consists in sending one robot to collect samples in station 0 and deliver them to station 1
    and another robot to collect samples  in station 3 and deliver them to station 2
    '''
    #TODO: get tasks from a task generation manager at higher level

    # possible tasks:
    #tasks = [[[0,3],[1,2]], [[1,2],[0,3]], [[1,0],[3,2]], [[0,3, 1],[1,2, 0]], [[3, 1,2],[1, 0,3]], [[2, 1,0],[0,3,2]], [[3, 2, 1,0],[1,0,3,2]]]
    #tasks_to_do = random.choice(tasks) # select a random set of tasks

    tasks_to_do = [[0],[3]] #[[2],[1]]#[[2,0],[1, 3]] # values for testing
   
    return tasks_to_do[0], tasks_to_do[1]


def task_allocation(stations, tasks_to_do, next_tasks_to_do):
    '''
    Allocate the tasks to the robots by minimizing the total distance travelled by the robots
    - input : list of station, list of tasks: collecting stations in task_to_do, delivery station in next_tasks_to_do
    - output: list of goals allocated to each robot, updates state_goal and state_mission if robots were assigned tasks
    '''

    starts_robots = [(robot.posX, robot.posY) for robot in robots]
    goals_allocation = []
    cost_matrix = np.zeros((len(tasks_to_do), len(robots)))

    # 1. OPTIMIZATION - Hungarian algorithm

    # update goals_allocation with the collection stations to reach
    for i in range(0, len(tasks_to_do)):
        goals_allocation.append((stations[tasks_to_do[i]].x, stations[tasks_to_do[i]].y))
    
    heuristics = []
    for goal in goals_allocation:
        heuristics.append(compute_heuristics(my_map, goal))

    # compute the cost matrix
    for i in range(0, len(tasks_to_do)):
        for j in range(0, len(robots)):
            path = a_star(my_map, starts_robots[j], goals_allocation[i], heuristics[i], 0, [])
            cost_matrix[i][j]=len(path)

    # Hungarian algorithm
    m = Munkres()
    indexes = m.compute(cost_matrix) # it outputs a list of assignments (task, robot)

    if VERBOSE:
        print(indexes)
    
    # 2. Update goals for the robots performing a task
    for j in range(0, len(indexes)):
        robot_index = indexes[j][1]
        robots[robot_index].goalX.append(stations[next_tasks_to_do[indexes[j][0]]].x)
        robots[robot_index].goalY.append(stations[next_tasks_to_do[indexes[j][0]]].y)
        robots[robot_index].goalX.append(stations[tasks_to_do[indexes[j][0]]].x)
        robots[robot_index].goalY.append(stations[tasks_to_do[indexes[j][0]]].y)

        robots[robot_index].state = 1
        robots[robot_index].state_mission = 1 # update the mission state
        if VERBOSE:
            print(robots[robot_index].name, 'is taking the task', j)

    # 3. Update goals_allocation list containing the goals for each robot
    goals_allocation = [(robot.goalX[-1], robot.goalY[-1]) for robot in robots]
    return goals_allocation



def update_goal(robot, goals):
    '''
    Check if goal is reached, if it's the case, remove from goal list
    '''
    if VERBOSE:
        print(robot.name, 'reached its goal')

    # remove the goal from the robot
    if len(robot.goalX) > 1:
        robot.goalX.pop(-1)
        robot.goalY.pop(-1)
        robot.state = 1
    else: 
        robot.state_mission = 0
    # add the new goal in goals_allocation
    goals[robot.id]=(robot.goalX[-1], robot.goalY[-1])
    if VERBOSE:
        print('new goal for', robot.name, 'is', robot.goalX[-1], robot.goalY[-1])


def check_checkpoint(current_robot, robots):
    '''
    Check if checkpoint is reached, i.e. if the robot is at a certain distance radius from the checkpoint
    '''
    
    for robot in robots:
        if robot.state == 1:
            robot.send_goal()
        if VERBOSE:
            print(robot.name)
            print('distance to goal X', abs(robot.posX_gazebo-robot.goal_tempX))
            print('distance to goal Y', abs(robot.posY_gazebo-robot.goal_tempY))
        if (abs(robot.posX_gazebo-robot.goal_tempX)<DIST_THRESH and abs(robot.posY_gazebo-robot.goal_tempY)<DIST_THRESH) and min(abs(robot.posX_gazebo-robot.goal_tempX), abs(robot.posY_gazebo-robot.goal_tempY))<0.1:
            if VERBOSE:
                print('next checkpoint')
            robot.state_checkpoint = 0
            if len(robot.path_gazebo)<=1:
                robot.state = 0 


def send_checkpoint(robots):
    '''
    Send next point to reach to each robot if all robots have reached their previous checkpoint
    '''
    print(robots)
    for robot in robots:
        if robot.state_checkpoint == 1:
            if VERBOSE:
                print('not there yet')
            return True 
        
    for robot in robots:
        if len(robot.path_gazebo)>1:
            robot.path_gazebo.pop(0)
    return True 

def state_mission(robots):
    '''
    Check if all robots achieved their mission (collecting, delivering and returning to the base) 
    if it's the case, we return 0, else we return 1
    '''
    for robot in robots:
         if robot.state_mission == 1: 
                return 1
    return 0

def state_system(robots):
    '''
    Move to next task if all robot are free
    '''
    for robot in robots:
         if robot.state == 1:
                return 1
    return 0


def find_change_dir_idx(robot):
    '''
    Will store the first and last nodes in paths as well as all nodes at which there is a change in orientation
    The goal of this function is to avoid having to send all nodes to the robot
    - input: path in MAPF frame
    '''
    for i in range(len(robot.path)-2):
        # if two consecutive segments are perpendicular or if the robot has to move back
        if np.dot( np.array([robot.path[i+1][0]-robot.path[i][0], robot.path[i+1][1]-robot.path[i][1]]), np.array([robot.path[i+2][0]-robot.path[i+1][0], robot.path[i+2][1]-robot.path[i+1][1]]))<=0:
            # Store the indices of the changes of direction along the path
            robot.change_dir_idx.append(i+1)


def check_boundaries(X, Y, my_map):
    '''
    Check if a point is inside  the map
    '''
    if X < len(my_map) and Y < len(my_map)[0] and X > 0 and Y > 0:
        return True
    else:
        return False
    

def create_robots_from_ini_file(file_path):
    '''
    Load robot configuration parameters from ini file 
    '''
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
    '''
    Load station configuration parameters from ini file 
    '''
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
    '''
    Initialize robots and stations
    '''
    robots = create_robots_from_ini_file(file_path)
    stations = create_stations_from_ini_file(file_path)
    return robots, stations







if __name__ == '__main__':
    try:
        goals = []
        starts = []

        rospy.init_node('agatha_generate_goals', anonymous=True) #make node 

        # init robots and stations with  initial position and name
        robots, stations = create_all_from_ini_file('configrobots.ini')

        if VERBOSE:
            # Accessing robot and station properties:
            for robot in robots:
                print(f"Robot {robot.name}: ({robot.posX}, {robot.posY})")

            for station in stations:
                print(f"Stations {station.name}: ({station.x}, {station.y})")

        # init map
        my_map = import_mapf_instance('circuit_MAPF.txt')
        
        # generate the tasks
        tasks_to_do, next_tasks_to_do = task_generator(stations)
        if VERBOSE:
            print('task generated')
            print(tasks_to_do, next_tasks_to_do)
        
        # task allocation
        goals = task_allocation(stations, tasks_to_do, next_tasks_to_do )
        if VERBOSE:
            print('Goals allocated', goals)
        
        # Find paths
        starts = [(robot.posX, robot.posY) for robot in robots]
        if VERBOSE:
            print('Current position: ', starts)
        cbs = CBSSolver(my_map, starts, goals)
        paths = cbs.find_solution('--disjoint')
        
        # Update paths
        for robot in robots:
            robot.path = paths[robots.index(robot)]

        # Filter the checkpoints to only keep changes in direction, initial and final positions in path
        # This will allow to recompute the paths only if one of the robot changes direction in order to avoid collisions
        # while still maintaining a consant velocity since there is no delay at every node but only when one of the robots have to turn
        all_change_dir = []
        all_change_dir.append(0)
        for robot in robots:
            find_change_dir_idx(robot)
            all_change_dir.extend(robot.change_dir_idx)
            all_change_dir.append(len(robot.path)-1)
        # get all indices for which AT LEAST one robot is turning
        all_change_dir = np.unique(all_change_dir)
        for robot in robots:
            robot.path = [robot.path[i] for i in list(filter(lambda item: item < len(robot.path), all_change_dir))] 


        if VERBOSE:
            print('Paths updated') 

        # Change frame MAPF 2 Gazebo (for path)
        for robot in robots:
            robots[robots.index(robot)].base_MAPF2Gazebo()

        # Get position (Gazebo frame)
        rospy.Subscriber('/robot1/odom',Odometry,robots[0].Position)
        rospy.Subscriber('/robot2/odom',Odometry,robots[1].Position)
        rospy.Subscriber('/robot3/odom',Odometry,robots[2].Position)
        rospy.Subscriber('/robot4/odom',Odometry,robots[3].Position)
        rospy.Subscriber('/robot5/odom',Odometry,robots[4].Position)
    
        # a checker 
        for robot in robots:
            robots[robots.index(robot)].goal_tempX = robots[robots.index(robot)].posX_gazebo
            robots[robots.index(robot)].goal_tempY = robots[robots.index(robot)].posY_gazebo

        
        while True:
            if VERBOSE:
                print(' ')
                print('..................................................................')

            state_var = state_system(robots)
            state_miss = state_mission(robots)

            if state_var == 0:
                
                # Update goals in MAPF frame
                for robot in robots:
                    update_goal(robots[robots.index(robot)], goals)

                # Compute the new paths in MAPF frame
                # 1. Update MAPF position
                for robot in robots:
                    robots[robots.index(robot)].base_Gazebo2MAPF()
    
                # 2. Get the new starts
                starts = [(robot.posX, robot.posY) for robot in robots]
                if VERBOSE:
                    print('Current position: ', starts)

                # 3. Compute the new paths
                cbs = CBSSolver(my_map, starts, goals)
                paths = cbs.find_solution('--disjoint')

                # 4. Update the paths
                for robot in robots:
                    robots[robots.index(robot)].path = paths[robots.index(robot)] 

                # Filter the checkpoints to only keep changes in direction, initial and final positions in path
                # This will allow to recompute the paths only if one of the robot changes direction in order to avoid collisions
                # while still maintaining a consant velocity since there is no delay at every node but only when one of the robots have to turn
                all_change_dir = []
                all_change_dir.append(0) # add initial position index
                for robot in robots:
                    find_change_dir_idx(robot)
                    # store indices of direction changes for ALL robots
                    all_change_dir.extend(robot.change_dir_idx) 
                    # add last position index
                    all_change_dir.append(len(robot.path)-1)    
                # get all indices for which AT LEAST one robot is turning and keep only those for each robot
                all_change_dir = np.unique(all_change_dir)
                for robot in robots:
                    robot.path = [robot.path[i] for i in list(filter(lambda item: item < len(robot.path), all_change_dir))] 
                
                # 5. Change paths from MAPF frame to Gazebo coordinates
                for robot in robots:
                    robots[robots.index(robot)].base_MAPF2Gazebo()

                if VERBOSE:
                    print('Paths updated')

            if state_miss == 0:
                if VERBOSE:
                    print('Mission completed')
                break

            # check if checkpoint is reached
            check_checkpoint(robots[0], robots)

            # send target to controller
            send_checkpoint(robots)
      
            # Wait 1 sec
            rospy.sleep(1)
            
            
        rospy.spin()
       
            

        
    except rospy.ROSInterruptException:
        pass



