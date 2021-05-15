#!/usr/bin/env python
import rospy
import time
import math
import sys
import os
import rospkg
import random
sys.path.append(os.path.join(rospkg.RosPack().get_path('me_cs301_robots'), 'scripts'))
from robot_control import RobotControl
from map import *

'''
MotorIDstrings for Hexapod follow the convention legN_jM, where N can be 1,2,3,4,5,6 (for each of the 6 legs) and M can 1,2,3. M=1 is the joint closest to the body and M=3 is the joint farthest from the body.

Available Robot API commands:

1. setMotorTargetJointPosition(motor_id_string, target_joint_angle) - send the target position for motor_id_string to CoppeliaSim
2. getSensorValue(sensor_type), where sensor_type can be from ['front', 'left', 'right'] - retrieves the current reading of the sensor sensor_type
3. getMotorCurrentJointPosition(motor_id_string) - retrieves the current angle for motor motor_id_string
4. getRobotWorldLocation() - returns (position, orientation) of the robot with respect to the world frame. Note, that orientation is represented as a quaternion.
5. getCurrentSimTime() - returns the current simulation time

Helper functions

degToRad() - Converts degrees to radians

Note that, this list of API functions could potentially grow. You will be notified via Canvas if anything is updated

self.joint_states.name

'''

j1_limits = [-0.85, 0.85]
j2_limits = [-1.30, 0.52]
j3_limits = [-3.14, 3.14]

step = 0.55

class Cell():
    def __init__(self, position):
        self.position = position
        self.f = 0
        self.parent = []
        self.children = []

    def __eq__(self, other):
        return self.position == other.position

class HexapodControl(RobotControl):
    i = 0
    j = 0
    orientation = 'south'
    visited = []

    def __init__(self):
        super(HexapodControl, self).__init__(robot_type='hexapod')
        rospy.loginfo("hexapod setup complete")
        self.hold_neutral()
        time.sleep(2.0)

        map = CSMEMap()
        map.clearObstacleMap()
        map.printObstacleMap()
        
        current = Cell((self.i, self.j))
        self.initialize_first_cell(map, current)
        
        #main control loop
        while not rospy.is_shutdown(): 
            # self.hold_neutral() #remove if not necessary
            # ---- add your code for a particular behavior here ----- #


            # THIS IS WHERE YOU INPUT THE START AND GOAL CELLS
            # start = Cell((self.i, self.j))
            # print('starting location is at (', self.i, ', ', self.j, ')')
            # goal = Cell((3, 5))

            # # Generates a path / sequence of grid cells to visit
            # path = self.a_star_search(map, start, goal)
            # # Generates a command sequence to achieve path
            # sequence = self.command_sequence(path)
            # print(sequence)
            # # Walks the path by following the command sequence
            # for direction in sequence:
            #     self.move_cell(direction)

            # print('ending location is at (', self.i, ', ', self.j, ')')

            next_cell = self.wander(map, current)

            print('current location is (', self.i, ', ', self.j, ')')
            print('current cell is: (', next_cell.position[0], ', ', next_cell.position[1], ')')
            print('orientation is: ', self.orientation)

            current = next_cell

            time.sleep(0.1) # change the sleep time to whatever is the appropriate control rate for simulation

    def hold_neutral(self):
        # --- simple example of a behavior ---- #
        self.setMotorTargetJointPosition('leg1_j1', 0.0)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg1_j3', 2.09)

        self.setMotorTargetJointPosition('leg2_j1', 0.0)
        self.setMotorTargetJointPosition('leg2_j2', -0.5)
        self.setMotorTargetJointPosition('leg2_j3', 2.09)

        self.setMotorTargetJointPosition('leg3_j1', 0.0)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j3', 2.09)

        self.setMotorTargetJointPosition('leg4_j1', 0.0)
        self.setMotorTargetJointPosition('leg4_j2', -0.5)
        self.setMotorTargetJointPosition('leg4_j3', 2.09)

        self.setMotorTargetJointPosition('leg5_j1', 0.0)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j3', 2.09)

        self.setMotorTargetJointPosition('leg6_j1', 0.0)
        self.setMotorTargetJointPosition('leg6_j2', -0.5)
        self.setMotorTargetJointPosition('leg6_j3', 2.09)

    def next_move(self, joint_ids, target_angles):

        flag = False
        while (not flag):
            flag = True
            for i in range(len(joint_ids)):
                flag *= (abs(self.getMotorCurrentJointPosition("hexa_" + joint_ids[i]) - target_angles[i]) < 1e-4)

    def pid_control(self, curr_error, prev_error, tot_error):
        p = 0.05
        i = 0.0
        d = 0.0
        dt = 0.05

        u = (p * curr_error) + (i * tot_error * dt) + (d * (curr_error - prev_error) / dt)
        return u

    # function that makes the hexapod take one step forward
    def step(self, control):

        # home position joint 1, 2, 3
        home = [0.0, -0.50, 2.09]
        raise_j2 = -0.7

        # raise legs 1, 3, 5 (joint 2)
        list1 = ['leg1_j2', 'leg3_j2', 'leg5_j2']
    
        for i in range(len(list1)):
            self.setMotorTargetJointPosition(list1[i], raise_j2)
        self.next_move(list1, [raise_j2]*len(list1))

        # rotate legs 1, 3, 5 forward (joint 1)
        # rotate legs 2, 4, 6 backward (joint 1)
        list2 = ['leg1_j1', 'leg2_j1', 'leg3_j1', 'leg4_j1', 'leg5_j1', 'leg6_j1', 'leg4_j3']
        target2 = [control, step, -step+control, 0.0, step+control, -step, 1.79]
        
        for i in range(len(list2)):
            self.setMotorTargetJointPosition(list2[i], target2[i])
        self.next_move(list2, target2)

        self.setMotorTargetJointPosition('leg4_j3', home[2])
        self.next_move(['leg4_j3'], [home[2]])

        # lower legs 1, 3, 5 (joint 2)
        # raise legs 2, 4, 6 (joint 2)
        list3 = ['leg1_j2', 'leg2_j2', 'leg3_j2', 'leg4_j2', 'leg5_j2', 'leg6_j2']
        target3 = [home[1], raise_j2, home[1], raise_j2, home[1], raise_j2]

        for i in range(len(list3)):
            self.setMotorTargetJointPosition(list3[i], target3[i])
        self.next_move(list3, target3)

        # rotate legs 2, 4, 6 forward (joint 1)
        # rotate legs 1, 3, 5 backward (joint 1)
        list4 = ['leg1_j1', 'leg2_j1', 'leg3_j1', 'leg4_j1', 'leg5_j1', 'leg6_j1', 'leg1_j3']
        target4 = [home[0], home[0], home[0], home[0], home[0], home[0], 1.79]

        for i in range(len(list4)):
            self.setMotorTargetJointPosition(list4[i], target4[i])
        self.next_move(list4, target4)

        self.setMotorTargetJointPosition('leg1_j3', home[2])
        self.next_move(['leg1_j3'], [home[2]])

        # lower legs 2, 4, 6 (joint 2)
        list5 = ['leg2_j2', 'leg4_j2', 'leg6_j2']

        for i in range(len(list5)):
            self.setMotorTargetJointPosition(list5[i], home[1])
        self.next_move(list5, [home[1]]*len(list5))

    # function that rotates the hexapod by one step size (pi/6 radians) in a given direction
    def turn(self, cw):

        if (cw == True):
            # rotate_j1 = -math.pi/6
            rotate_j1 = -0.54
        else:
            # rotate_j1 = math.pi/6
            rotate_j1 = 0.54

        raise_j2 = -0.7

        home = [0.0, -0.5, 2.09]

        tripod1_ids = ['leg1_j2', 'leg3_j2', 'leg5_j2']
        rotate_ids = ['leg1_j1', 'leg2_j1', 'leg3_j1', 'leg4_j1', 'leg5_j1', 'leg6_j1']
        tripod2_ids = ['leg2_j2', 'leg4_j2', 'leg6_j2']

        # raise legs 1, 3, 5
        for i in range(len(tripod1_ids)):
            self.setMotorTargetJointPosition(tripod1_ids[i], raise_j2)
        self.next_move(tripod1_ids, [raise_j2]*len(tripod1_ids))

        # rotate legs 1, 3, 5
        rotate_t1_angles = [rotate_j1, -rotate_j1, rotate_j1, -rotate_j1, rotate_j1, -rotate_j1]

        for i in range(len(rotate_ids)):
            self.setMotorTargetJointPosition(rotate_ids[i], rotate_t1_angles[i])

        self.next_move(rotate_ids, rotate_t1_angles)

        # lower legs 1, 3, 5
        for i in range(len(tripod1_ids)):
            self.setMotorTargetJointPosition(tripod1_ids[i], home[1])
        self.next_move(tripod1_ids, [home[1]]*len(tripod1_ids))

        # raise legs 2, 4, 6
        for i in range(len(tripod2_ids)):
            self.setMotorTargetJointPosition(tripod2_ids[i], raise_j2)
        self.next_move(tripod2_ids, [raise_j2]*len(tripod2_ids))
       
        # rotate legs 2, 4, 6
        # rotate_t2_angles = [-rotate_j1, rotate_j1, -rotate_j1, rotate_j1, -rotate_j1, rotate_j1]

        for i in range(len(rotate_ids)):
            # self.setMotorTargetJointPosition(rotate_ids[i], rotate_t2_angles[i])
            self.setMotorTargetJointPosition(rotate_ids[i], home[0])
        # self.next_move(rotate_ids, rotate_t2_angles)
        self.next_move(rotate_ids, [home[0]]*len(rotate_ids))

        # lower legs 2, 4, 6
        for i in range(len(tripod2_ids)):
            self.setMotorTargetJointPosition(tripod2_ids[i], home[1])
        self.setMotorTargetJointPosition(tripod2_ids, [home[1]]*len(tripod2_ids))

    def turn_left90(self):
        i = 0
        while (i < 3):
            self.turn(False)
            i += 1

    def turn_right90(self):
        i = 0
        while (i < 3):
            self.turn(True)
            i += 1

    def turn_around(self):
        i = 0
        while (i < 6):
            self.turn(False)
            i += 1

    # function that moves the robot south
    def move_cell(self, direction):

        # orients the robot in the direction it's supposed to move
        if self.orientation == 'north':
            if direction == 'east':
                self.turn_right90()
                self.orientation = 'east'
                self.j += 1

            elif direction == 'south':
                self.turn_around()
                self.orientation = 'south'
                self.i += 1

            elif direction == 'west':
                self.turn_left90()
                self.orientation = 'west'
                self.j -= 1
            
            elif direction == 'north':
                self.i -= 1

            
        if self.orientation == 'east':
            if direction == 'south':
                self.turn_right90()
                self.orientation = 'south'
                self.i += 1

            elif direction == 'west':
                self.turn_around()
                self.orientation = 'west'
                self.j -= 1

            elif direction == 'north':
                self.turn_left90()
                self.orientation = 'north'
                self.i -= 1
            
            elif direction == 'east':
                self.j += 1
            
        if self.orientation == 'south':
            if direction == 'west':
                self.turn_right90()
                self.orientation = 'west'
                self.j -= 1

            elif direction == 'north':
                self.turn_around()
                self.orientation = 'north'
                self.i -= 1

            elif direction == 'east':
                self.turn_left90()
                self.orientation = 'east'
                self.j += 1
            
            elif direction == 'south':
                self.i += 1

        if self.orientation == 'west':
            if direction == 'north':
                self.turn_right90()
                self.orientation = 'north'
                self.i -= 1

            elif direction == 'east':
                self.turn_around()
                self.orientation = 'east'
                self.j += 1

            elif direction == 'south':
                self.turn_left90()
                self.orientation = 'south'
                self.i += 1
            
            elif direction == 'west':
                self.j -= 1

        i = 0

        curr_error = 0
        prev_error = 0
        tot_error = 0
        dist = 0.40

        while (i < 11):

            # following left-sided wall
            if self.getSensorValue('left') > 0 and self.getSensorValue('right') < 0:
                curr_error = self.getSensorValue('left') - dist
                u = self.pid_control(curr_error, prev_error, tot_error)

            elif self.getSensorValue('right') > 0 and self.getSensorValue('left') < 0:
                curr_error = self.getSensorValue('right') - dist
                u = -self.pid_control(curr_error, prev_error, tot_error)

            elif self.getSensorValue('left') > 0 and self.getSensorValue('right') > 0:
                curr_error = self.getSensorValue('left') - dist
                u = self.pid_control(curr_error, prev_error, tot_error)

            else:
                u = 0

            # check for joint limits!!
            if u > 0.1:
                u = 0.1
            elif u < -0.1:
                u = -0.1

            self.step(u)

            i += 1
        
        print('moved one cell ', direction)
        return

    # function that calculates the manhatten distance between cells       
    def manhatten_distance(self, current, goal):
        return abs(current.position[0] - goal.position[0]) + abs(current.position[1] - goal.position[1])

    # function that calculates the straight line distance between cells
    # Straight line distance is used to calculate the f cost of each grid cell
    def straight_line_distance(self, current, goal):
        return math.sqrt((current.position[0] - goal.position[0])**2 + (current.position[1] - goal.position[1])**2)

    # function that looks for node's successors
    def successors(self, map, node):

        successors = []
        # check if north cell is not blocked
        if map.getNeighborObstacle(node.position[0], node.position[1], 1) == 0:
            successor = Cell((node.position[0]-1, node.position[1]))
            successor.parent.append(node)
            successors.append(successor)
        # check if east cell is not blocked
        if map.getNeighborObstacle(node.position[0], node.position[1], 2) == 0:
            successor = Cell((node.position[0], node.position[1]+1))
            successor.parent.append(node)
            successors.append(successor)
        # check if south cell is not blocked
        if map.getNeighborObstacle(node.position[0], node.position[1], 3) == 0:
            successor = Cell((node.position[0]+1, node.position[1]))
            successor.parent.append(node)
            successors.append(successor)
        # check if west cell is not blocked
        if map.getNeighborObstacle(node.position[0], node.position[1], 4) == 0:
            successor = Cell((node.position[0], node.position[1]-1))
            successor.parent.append(node)
            successors.append(successor)

        return successors

    # function that implements a* search algorithm
    def a_star_search(self, map, start, goal):
        # open: list of nodes that may need to be expanded
        queue = [start]

        # closed: list of nodes that represent the best path
        closed = []

        iterations = 0

        # loop through until no more cells in the queue
        while len(queue) > 0:

            # if iterations is too large, return no path found
            iterations += 1
            if iterations > 200:
                print('Took too long to find path, none found.')
                return []

            # find the cell with the least f cost on the queue (open list)
            f_dict = {}
            for cell in queue:
                cell.f = self.manhatten_distance(start, cell) + self.straight_line_distance(cell, goal)
                f_dict[queue.index(cell)] = cell.f
            
            q = queue[min(f_dict, key=f_dict.get)]
            
            # remove q from queue and add to path
            queue.remove(q)
            closed.append(q)

            # not quite sure if I can just clear the queue, but everywhere I see on the internet
            # looks like older items in the queue never get visited... it also caused me problems
            # to keep old items in the queue
            # queue = []

            if q == goal:
                print('Found the goal cell!')
                path = [start]
                cell = q
                while len(cell.parent) > 0:
                    path.insert(1, cell)
                    cell = cell.parent[0]
                return path

            # generate q's successors and set parents to q

            successors = self.successors(map, q)

            for s in successors:
                q.children.append(s)

            # loop through each successor
            for cell in q.children:

                # calculate f score
                cell.f = self.manhatten_distance(start, q) + self.straight_line_distance(cell, goal)

                # if the successor has already been visited, move to next successor
                if len([visited for visited in closed if visited == cell]) > 0:
                    continue
            
                # if the successor is in the queue and the f cost is already lower, move to next successor
                if len([item for item in queue if item == cell and cell.f > item.f]) > 0:
                    continue
                
                # add the successor to the queue
                queue.append(cell)

    # *******************************************************
    # Function Name     :   command_sequence
    # Description       :   takes a path (list of cells) and generates a sequence of commands to move the hexapod
    # Input             :   path : a list of cells to visit
    # Ouptut            :   sequence : a list of directions to move in
    # *******************************************************
    def command_sequence(self, path):

        sequence = []

        for i in range(len(path)-1):
            curr_cell = path[i].position
            next_cell = path[i+1].position

            # if the next cell is to the left
            if curr_cell[0] > next_cell[0]:
                sequence.append('north')
            elif curr_cell[0] < next_cell[0]:
                sequence.append('south')
            elif curr_cell[1] > next_cell[1]:
                sequence.append('west')
            elif curr_cell[1] < next_cell[1]:
                sequence.append('east')

        return sequence

    # *******************************************************
    # Function Name     : find_obstacles
    # Description       : function that checks front, left and right sensors for obstacles
    # Input             : map : the current map
    # Input             : current : the current cell that the hexapod is in
    # *******************************************************
    def find_obstacles(self, map, current):
        # checks for obstacle in front
        if self.getSensorValue('front') > 0:
            if self.orientation == 'north':
                map.setObstacle(current.position[0], current.position[1], True, 1)
            elif self.orientation == 'east':
                map.setObstacle(current.position[0], current.position[1], True, 2)
            elif self.orientation == 'south':
                map.setObstacle(current.position[0], current.position[1], True, 3)
            elif self.orientation == 'west':
                map.setObstacle(current.position[0], current.position[1], True, 4)
        # checks for obstacle to the left
        if self.getSensorValue('left') > 0:
            if self.orientation == 'north':
                map.setObstacle(current.position[0], current.position[1], True, 4)
            elif self.orientation == 'east':
                map.setObstacle(current.position[0], current.position[1], True, 1)
            elif self.orientation == 'south':
                map.setObstacle(current.position[0], current.position[1], True, 2)
            elif self.orientation == 'west':
                map.setObstacle(current.position[0], current.position[1], True, 3)
        # checks for obstacle to the right
        if self.getSensorValue('right') > 0:
            if self.orientation == 'north':
                map.setObstacle(current.position[0], current.position[1], True, 2)
            elif self.orientation == 'east':
                map.setObstacle(current.position[0], current.position[1], True, 3)
            elif self.orientation == 'south':
                map.setObstacle(current.position[0], current.position[1], True, 4)
            elif self.orientation == 'west':
                map.setObstacle(current.position[0], current.position[1], True, 1)
        
        map.printObstacleMap()

    # *******************************************************
    # Function Name     : wander
    # Description       : explores map B by walking to unvisited nodes and looking for obstacles
    # Input             : map : the current map
    #                   : current : the current cell that the hexapod is in
    # Output            : next : the cell that the hexapod has moved to
    # *******************************************************
    def wander(self, map, current):
        # add the current node to the list of visited nodes
        self.visited.append(current)

        print('wandering!')

        # updating obstacles
        self.find_obstacles(map, current)

        # find possible successors of the current cell
        successors = self.successors(map, current)

        # if a successor has not been visited, walk in that direction
        options = []
        for successor in successors:
            if successor not in self.visited:
                options.append(successor)

        print('the possible directions are: ')
        for option in options:
            print('(', option.position[0], ', ', option.position[1], ')')

        if len(options) > 0:
            random_index = random.randint(0, len(options)-1)
            next = options[random_index]
        else:
            random_index = random.randint(0, len(successors)-1)
            next = successors[random_index]

        # chooses a random cell out of unvisited cells to move

        if next.position[0] > current.position[0]:
            self.move_cell('south')
        elif next.position[0] < current.position[0]:
            self.move_cell('north')
        elif next.position[1] > current.position[1]:
            self.move_cell('east')
        elif next.position[1] < current.position[1]:
            self.move_cell('west')

        return next

    # *******************************************************
    # Function Name     : initialize_first_cell
    # Description       : implemented only when in cell (0, 0), checks for walls in every direction
    # Input             : map : the current map
    def initialize_first_cell(self, map, current):
        self.find_obstacles(map, current)
        self.turn_around()
        self.orientation = 'north'
        self.find_obstacles(map, current)
        self.turn_around()
        self.orientation = 'south'


if __name__ == "__main__":
    q = HexapodControl()
    rospy.spin()