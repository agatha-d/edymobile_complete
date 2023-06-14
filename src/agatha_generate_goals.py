# Difference dans la generation des taches, on ne peut pas avoir deux fois le meme goal
# On envoie les goals aux robots
# Structure finale
# On effectue les missions de A à Z 
# on envoie des checkpoints aux robots - changement de direction
# on calcul une seule fois le chemin

# python3 run_experiments.py --disjoint --instance instances/circuit.txt --solver Prioritized


# Librairie
from pathlib import Path
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
import random
import numpy as np
from hungarian_algorithm import hungarian_algorithm, ans_calculation
from munkres import Munkres # hungarian algorithm
from cbs import CBSSolver
#from prioritized import PrioritizedPlanningSolver

from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *
import configparser


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#stations = { 'discovery' : (5, 2), 'omni' : (5, 10), 'synt' : (9, 10), 'sfc' : (0, 10) }
#robots = { '0' : (17, 1), '1' : (19, 1), '2' : (3, 5), '3' : (17, 10),'4' : (19, 10)}
#number_robot = 5
#number_station = 4
#state_var = 0 # 0: free, 1: has a task

EPSILON = 0.0001

class Robot():

    def __init__(self, nameRobot, initX, initY, number):
        self.name = nameRobot
        self.id = number
        self.posX = initX
        self.posY = initY
        self.goalX = [initX]
        self.goalY = [initY]
        self.path = []
        self.produit = np.array([[0, 1, -3.6],[-1, 0, 4.5], [0, 0, 1]]) @ (self.posX/2.1, self.posY/2.1, 1)
        self.posX_gazebo = round(self.produit[0], 2 )
        self.posY_gazebo = round(self.produit[1], 2 )
        self.goal_tempX = 0
        self.goal_tempY = 0
        self.path_gazebo = []
        self.state_checkpoint = 0 # 0: arrived, 1: on the way
        self.state = 0 # 0: free, 1: has a task
        self.checkpoint = []

        # ROS Publisher for sending goal to controller
        self.pub = rospy.Publisher('/' +self.name+ '/move_base/goal',MoveBaseGoal, queue_size=10)


    def Position(self, odom_data):
        curr_time = odom_data.header.stamp
        self.posX_gazebo = round(odom_data.pose.pose.position.x, 2)
        self.posY_gazebo = round(odom_data.pose.pose.position.y, 2)


    def base_MAPF2Gazebo(self):
        '''
        Change of base MAPF 2 GAZEBO
        - input: paths in the MAPF frame
        - output: paths in the GAZEBO frame
        '''
        self.path_gazebo.clear()
        matrix_chg_base = np.array([[0, 1, -3.6],[-1, 0, 4.5], [0, 0, 1]])
        #print(self.name, len(self.path), self.path)
        for i in range(0, len(self.path)): #We start at 1 so we don't take into account the initial position NOT the case anymore
            path_temp = (self.path[i][0]/2.1, self.path[i][1]/2.1, 1)
            produit = matrix_chg_base @ path_temp
            self.path_gazebo.append((round(produit[0], 2), round(produit[1], 2)))
        print('Path in converted Gazebo coordinate')


    def base_Gazebo2MAPF(self):
        '''
        Change of base GAZEBO 2 MAPF
        - input: paths in the MAPF frame
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
        print('----SEND GOAL----')

        # Set up the goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.orientation.w = 1.0 # not used
        self.state_checkpoint = 1
        
        if len(self.path_gazebo)> 1:
            self.path_gazebo.pop(0)
            goal.target_pose.pose.position.x = self.path_gazebo[0][0]
            goal.target_pose.pose.position.y = self.path_gazebo[0][1]
            self.goal_tempX = self.path_gazebo[0][0]
            self.goal_tempY = self.path_gazebo[0][1]
            print('new checkpoint or goal for ', self.name, self.path_gazebo[0][0], self.path_gazebo[0][1])
            
        else:
            goal.target_pose.pose.position.x = self.path_gazebo[0][0]
            goal.target_pose.pose.position.y = self.path_gazebo[0][1]
            print('same pos', self.path_gazebo[0][0], self.path_gazebo[0][1])
            self.state = 0 # the robot is free again

        # Publish the goal and 
        self.pub.publish(goal)

        #TODO: wait for completion of the turn in change dir ??

        rospy.loginfo('Goal'+str(goal))

        return(goal)
    
    
    # TODO: remove, probably unnecessary
    def change_direction(self):
        '''
        Only keep changes of direction in the path
        '''
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
    
    
class Station:
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y


def import_mapf_instance(filename):
    '''
    MAP GENERATION (FOR A*)
    - input: text file representing the map
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


def task_generator(stations):
    '''
    TASK GENERATION: generate random tasks for the robots
    TODO: get tasks from a task generation manager at higher level
    - input: stations: discovery,5,2;omni,5,10;synt,9,10;sfc,0,10
    '''
    #tasks = [[[0,3],[1,2]], [[1,2],[0,3]], [[1,0],[3,2]], [[0,3, 1],[1,2, 0]], [[3, 1,2],[1, 0,3]], [[2, 1,0],[0,3,2]], [[3, 2, 1,0],[1,0,3,2]]]
    #tasks_to_do = random.choice(tasks)
    tasks_to_do = [[2],[1]]
   
    return tasks_to_do[0], tasks_to_do[1]


def task_allocation(stations, tasks_to_do, next_tasks_to_do):
    '''
    TASK ALLOCATION
    - input: robots
    - output: goals allocated to each robot
    '''

    starts_robots = [(robot.posX, robot.posY) for robot in robots]
    goals_allocation = []
    cost_matrix = np.zeros((len(tasks_to_do), len(robots)))
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

    print(indexes)
    #goals_allocation = [(robot.goalX, robot.goalY) for robot in robots]
    
    # Update goals
    for j in range(0, len(indexes)):
        robot_index = indexes[j][1]
        robots[robot_index].goalX.append(stations[next_tasks_to_do[indexes[j][0]]].x)
        robots[robot_index].goalY.append(stations[next_tasks_to_do[indexes[j][0]]].y)
        robots[robot_index].goalX.append(stations[tasks_to_do[indexes[j][0]]].x)
        robots[robot_index].goalY.append(stations[tasks_to_do[indexes[j][0]]].y)

        robots[robot_index].state = 1
        print(robots[robot_index].name, 'is taking the task', j)

    goals_allocation = [(robot.goalX[-1], robot.goalY[-1]) for robot in robots]
    #print('goals', goals_allocation)
    return goals_allocation





def get_smallest_path_len_without_change_dir(robots):
    '''
    Among all robots, the the length of the shorterst path without a change in direction
    '''

    idx = max([len(robot.path) for robot in robots])
    print(idx)

    for robot in robots:
        for i in range(0, len(robot.path)-1):
            # If there is a change of direction in the path
            if abs(robot.path[i][0]- robot.path[i+1][0]) <  EPSILON or abs(robot.path[i][1] -robot.path[i+1][1]) < EPSILON:
                # keep the index of the first change in direction among all paths
                idx = min(i,idx)
                print('Smaller Change of dir at index:')
                print(idx)
                break # check path of next robot

    print('final idx')
    print(idx)
                 
    return idx



def check_goal(robot, goals):
    '''
    Check if goal is reached, if it's the case, remove from goal list
    '''

    print(robot.name, 'reached its goal')

    # remove the goal from the robot
    if len(robot.goalX) > 1:
        robot.goalX.pop(-1)
        robot.goalY.pop(-1)
        robot.state = 1
    # add the new goal in goals_allocation
    goals[robot.id]=(robot.goalX[-1], robot.goalY[-1])
    print('new goal for', robot.name, 'is', robot.goalX[-1], robot.goalY[-1])


def check_checkpoint(current_robot, robots):
    '''
    Check if checkpoint is reached, i.e. if the robot is at a certain distance radius from the checkpoint
    '''
    
    for robot in robots:
        print(robot.name)
        print('distance to goal X', abs(robot.posX_gazebo-robot.goal_tempX))
        print('distance to goal Y', abs(robot.posY_gazebo-robot.goal_tempY))
        if (abs(robot.posX_gazebo-robot.goal_tempX)<0.4 and abs(robot.posY_gazebo-robot.goal_tempY)<0.4) and min(abs(robot.posX_gazebo-robot.goal_tempX), abs(robot.posY_gazebo-robot.goal_tempY))<0.2:
            print('next checkpoint')
            robot.state_checkpoint = 0
        

def send_checkpoint(robots):
    '''
    Send next point to reach to each robot if all robots have reached their previous checkpoint
    '''

    for robot in robots:
        if robot.state_checkpoint == 1:
            print('not there yet')
            return True 
        
    for robot in robots:
        robot.send_goal()
    return True 


def state_system(robots):
    '''
    Move to next task if all robot are free
    '''
    for robot in robots:
         if robot.state == 1:
                return 1
    return 0


def checkpoint_generator(robot):
    '''
    Create checkpoint from the path of the robot where the direction changes
    '''
    change_dir = []
    change_dir.append(robot.path[0])
    for i in range(len(robot.path)-2):
        if  (robot.path[i+1][0]-robot.path[i][0])*(robot.path[i+2][0]-robot.path[i+1][0])+(robot.path[i+1][1]-robot.path[i][1])*(robot.path[i+2][1]-robot.path[i+1][1]) == 0:
            change_dir.append(robot.path[i+1])
    change_dir.append(robot.path[-1])
    #robot.path.clear()
    #robot.path = checkpoint.copy()
    return change_dir


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

        #rospy.init_node('odometry', anonymous=True) #make node 
        rospy.init_node('agatha_generate_goals', anonymous=True) #make node 

        # init robots and stations with  initial position and name
        robots, stations = create_all_from_ini_file('configrobots.ini')

        # Accessing robot and station properties:
        for robot in robots:
            print(f"Robot {robot.name}: ({robot.posX}, {robot.posY})")

        for station in stations:
            print(f"Stations {station.name}: ({station.x}, {station.y})")

        # init map
        my_map = import_mapf_instance('circuit_MAPF.txt')
        
        # generate the tasks
        tasks_to_do, next_tasks_to_do = task_generator(stations)
        print('task generated')
        print(tasks_to_do, next_tasks_to_do)
        
        # task allocation
        goals = task_allocation(stations, tasks_to_do, next_tasks_to_do )
        print('Goals allocated', goals)
        #print([(robot.name, robot.goalX, robot.goalY, '................') for robot in robots])
        
        # Find paths
        starts = [(robot.posX, robot.posY) for robot in robots]
        print('Current position: ', starts)
        cbs = CBSSolver(my_map, starts, goals)
        paths = cbs.find_solution('--disjoint')

        
        # Update paths

        for robot in robots:
            robot.path = paths[robots.index(robot)]
            #print(robot.name, robot.path)
    
        print('Paths updated') 

        # Change frame MAPF 2 Gazebo (for path)
        robots[0].base_MAPF2Gazebo()
        robots[1].base_MAPF2Gazebo()
        robots[2].base_MAPF2Gazebo()
        robots[3].base_MAPF2Gazebo()
        robots[4].base_MAPF2Gazebo()


        # Get position (Gazebo frame)
        rospy.Subscriber('/robot1/odom',Odometry,robots[0].Position)
        rospy.Subscriber('/robot2/odom',Odometry,robots[1].Position)
        rospy.Subscriber('/robot3/odom',Odometry,robots[2].Position)
        rospy.Subscriber('/robot4/odom',Odometry,robots[3].Position)
        rospy.Subscriber('/robot5/odom',Odometry,robots[4].Position)

    
        # a checker 
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



        while True:
        
            print(' ')
            print('..................................................................')

            state_var = state_system(robots)

            if state_var == 0:
                
                # Update goals
                check_goal(robots[0], goals)
                check_goal(robots[1], goals)
                check_goal(robots[2], goals)
                check_goal(robots[3], goals)
                check_goal(robots[4], goals)

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


            # check if checkpoint is reached
            
            check_checkpoint(robots[0], robots)

            send_checkpoint(robots)

            get_smallest_path_len_without_change_dir(robots)
      
      
            # Wait 1 sec
            rospy.sleep(1)
            
            
        rospy.spin()
       
            

        
    except rospy.ROSInterruptException:
        pass



