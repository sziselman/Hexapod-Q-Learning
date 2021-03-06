#!/usr/bin/env python
import rospy
import csv
import time
import math
import sys
import os
import rospkg
import random
from enum import Enum
import numpy as np
sys.path.append(os.path.join(rospkg.RosPack().get_path('me_cs301_robots'), 'scripts'))
from robot_control import RobotControl
from map import *
import matplotlib.pyplot as plt

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

j1_limits = [-0.75, 0.75]
j2_limits = [-1.30, 0.52]
j3_limits = [-3.14, 3.14]

uniform_step = 0.50

# function that takes a sensor reading (distance from the wall) and fits it to the nearest state
def fit2states(states, distance):
    array = np.asarray(states)
    index = (np.abs(array - distance)).argmin()
    return index

# function that takes roll, pitch, yaw and returns a quaternion
def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


class HexapodControl(RobotControl):
    N = 10 # the number of actions/states
    # Q_mat = np.zeros((N, N))
    Q_mat = np.genfromtxt("/home/sarah/Documents/ME301/me_cs301_coppeliasim_robots/10x10qmatrix.csv", delimiter=",")
    epsilon = 0.25

    def __init__(self):
        super(HexapodControl, self).__init__(robot_type='hexapod')
        rospy.loginfo("hexapod setup complete")
        self.hold_neutral()
        time.sleep(2.0)

        actions = np.linspace(0.35, 0.65, self.N)
        states = np.linspace(0.10, 0.40, self.N)

        train = input('Are you training, implementing or evaluating? ')
        while train != 'training' and train != 'implementing' and train != 'evaluating':
            train = input('Are you training, implementing or evaluating? ')

        if train == 'training':
            # checks for what policy will be used to decide robot's actions
            policy = input('Enter policy to build Q-Table (random or epsilon greedy): ')
            while policy != 'random' and policy != 'epsilon greedy':
                policy = input('Enter policy to build Q-Table: ')

        runs = 0
        
        curr_error = 0
        prev_error = 0
        tot_error = 0
        desired_dist = 0.325

        while not rospy.is_shutdown():
            
            ###############################
            # TRAINING: Used for building the Q-table using a random policy and epsilon greedy policy
            ###############################
            if train == 'training':

                prev_position, prev_orientation = self.getRobotWorldLocation()
                prev_x = prev_position.x
                prev_y = prev_position.y

                # have robot walk until it detects an obstacle in front, at each step update the q matrix
                while self.getSensorValue('front') < 0:

                    # get the distance from the wall depending on the sensor that is being triggered, calculate current error for PID controller
                    if self.getSensorValue('left') > 0 and self.getSensorValue('right') < 0:
                        distance = self.getSensorValue('left')
                        curr_error = distance - desired_dist
                    elif self.getSensorValue('right') > 0 and self.getSensorValue('left') < 0:
                        distance = self.getSensorValue('right')
                        curr_error = distance - desired_dist
                    elif self.getSensorValue('left') > 0 and self.getSensorValue('right') > 0:
                        distance = self.getSensorValue('left')
                        curr_error = distance - desired_dist
                    else:
                        break
                    
                    # transition function (distance from the wall)
                    state_index = fit2states(states, distance)
                    state = states[state_index]

                    # # randomly select an action for training
                    # random_action_index = random.randint(0, self.N-1)
                    # random_action = actions[random_action_index]

                    if policy == 'random':
                        action_index = random.randint(0, self.N-1)
                        action = actions[action_index]
                    elif policy == 'epsilon greedy':
                        probability = random.random()
                        if probability > self.epsilon:
                            action_index = (self.Q_mat[state_index]).argmax()
                            action = actions[action_index]
                        else:
                            action_index = random.randint(0, self.N-1)
                            action = actions[action_index]

                    # determine the control using PID
                    tot_error += curr_error
                    control = self.pid_control(curr_error, prev_error, tot_error)
                    prev_error = curr_error

                    # check to ensure robot has not exceeded joint limits
                    if action + control > j1_limits[1]:
                        control = j1_limits[1] - action
                    elif -action + control < j1_limits[0]:
                        control = j1_limits[0] + action

                    # have robot take one step forward
                    self.step(control, action)

                    curr_position, curr_orientation = self.getRobotWorldLocation()

                    print('current position of robot is: ', curr_position.x, curr_position.y)
                    
                    # reward function is distance traveled by each step
                    distance_traveled = math.sqrt((curr_position.x - prev_x)**2 + (curr_position.y - prev_y)**2)

                    prev_x = curr_position.x
                    prev_y = curr_position.y

                    print('the distance traveled is: ', distance_traveled)

                    # increase cell in Q matrix corresponding to state/action by reward function
                    self.Q_mat[state_index][action_index] += distance_traveled

                    # save the .csv file, however this file will save in the me_cs301_coppeliasim_robots directory which is not uploaded to the github!!
                    np.savetxt("10x10qmatrix.csv", self.Q_mat, delimiter=",")

                    runs += 1
                    print('The amount of runs executed: ', runs, '\n')

                # reset the location and orientation of the robot to starting
                quaternion = euler_to_quaternion(0.0, -math.pi/2, math.pi)
                position, orientation = self.getRobotWorldLocation()
                position.x = 0
                position.y = 0
                orientation.x = quaternion[0]
                orientation.y = quaternion[1]
                orientation.z = quaternion[2]
                orientation.w = quaternion[3]

                self.setRobotPose(position, orientation)

                # reset the PID controller errors
                curr_error = 0
                prev_error = 0
                tot_error = 0
                desired_dist = 0.325

            ###############################
            # IMPLEMENTING: Implementing the robot's walking gait using the Q-Learning model through the provided obstacle course
            ###############################
            elif train == 'implementing':
                
                # if front sensor is within turning range
                if self.getSensorValue('front') > 0 and self.getSensorValue('front') < 0.275:
                    # readings only from left sensor
                    if self.getSensorValue('left') > 0 and self.getSensorValue('right') < 0:
                        self.turn_right90()
                    
                    # readings only from right sensor
                    elif self.getSensorValue('right') > 0 and self.getSensorValue('left') < 0:
                        self.turn_left90()
                    
                    # readings from both left and right sensor
                    elif self.getSensorValue('left') > 0 and self.getSensorValue('right') > 0:
                        self.turn_around()

                
                # get the distance from the wall depending on the sensor that is being triggered, calculate current error for PID controller
                # readings only from left sensor
                if self.getSensorValue('left') > 0 and self.getSensorValue('right') < 0:
                    distance = self.getSensorValue('left')
                    curr_error = distance - desired_dist
                
                # readings only from right sensor
                elif self.getSensorValue('right') > 0 and self.getSensorValue('left') < 0:
                    distance = self.getSensorValue('right')
                    curr_error = distance - desired_dist

                # readings from both left and right sensors
                elif self.getSensorValue('left') > 0 and self.getSensorValue('right') > 0:
                    distance = self.getSensorValue('left')
                    curr_error = distance - desired_dist
                
                # readings from neither sensors, stop the robot
                else:
                    break

                # transition function (distance from the wall)
                state_index = fit2states(states, distance)
                state = states[state_index]

                # find action with maximum reward
                action_index = (self.Q_mat[state_index]).argmax()
                action = actions[action_index]

                # determine the control using PID
                tot_error += curr_error

                if self.getSensorValue('left') > 0:
                    control = self.pid_control(curr_error, prev_error, tot_error)
                elif self.getSensorValue('right') > 0:
                    control = -self.pid_control(curr_error, prev_error, tot_error)

                prev_error = curr_error

                # check to ensure robot has not exceeded joint limits
                if action + control > j1_limits[1]:
                    control = j1_limits[1] - action
                elif - action + control < j1_limits[0]:
                    control = j1_limits[0] + action

                # have robot take one step forward
                # if you want to have the robot step using the Q-Learning model, uncomment the following:
                self.step(control, action)

                # if you want to have the robot step with a uniform step size of 0.5, uncomment the following:
                # self.step(control, uniform_step)

                time.sleep(0.1)         

            ###############################
            # EVALUATING: Have the robot walk a distance of approximately 1.5 m, measure the travel time, generate velocity
            ###############################
            elif train == 'evaluating':

                start_position, start_orientation = self.getRobotWorldLocation()
                start_x = start_position.x
                start_y = start_position.y

                start_time = rospy.get_time()

                distance_traveled = 0
                
                # have robot walk until it detects an obstacle in front, at each step update the q matrix
                while distance_traveled < 1.5:

                    # get the distance from the wall depending on the sensor that is being triggered, calculate current error for PID controller
                    if self.getSensorValue('left') > 0 and self.getSensorValue('right') < 0:
                        distance = self.getSensorValue('left')
                        curr_error = distance - desired_dist
                    elif self.getSensorValue('right') > 0 and self.getSensorValue('left') < 0:
                        distance = self.getSensorValue('right')
                        curr_error = distance - desired_dist
                    elif self.getSensorValue('left') > 0 and self.getSensorValue('right') > 0:
                        distance = self.getSensorValue('left')
                        curr_error = distance - desired_dist
                    else:
                        break
                    
                    # transition function (distance from the wall)
                    state_index = fit2states(states, distance)
                    state = states[state_index]

                    # find action with maximum reward
                    action_index = (self.Q_mat[state_index]).argmax()
                    action = actions[action_index]

                    # determine the control using PID
                    tot_error += curr_error
                    control = self.pid_control(curr_error, prev_error, tot_error)
                    prev_error = curr_error

                    # check to ensure robot has not exceeded joint limits
                    if action + control > j1_limits[1]:
                        control = j1_limits[1] - action
                    elif -action + control < j1_limits[0]:
                        control = j1_limits[0] + action

                    # have robot take one step forward
                    # if you want to have the robot step using the Q-Learning model, uncomment the following:
                    self.step(control, action)

                    # if you want to have the robot step with a uniform step size of 0.5, uncomment the following:
                    # self.step(control, uniform_step)

                    curr_position, curr_orientation = self.getRobotWorldLocation()
                    
                    # calculate total distance traveled
                    distance_traveled = math.sqrt((curr_position.x - start_x)**2 + (curr_position.y - start_y)**2)

                end_time = rospy.get_time()

                total_time = end_time - start_time

                velocity = distance_traveled / total_time

                print('It took ', total_time, 'seconds to travel a distance of ', distance_traveled, 'meters.')
                print('This results in a velocity of ', velocity, 'm/s.\n')

                # reset the location and orientation of the robot to starting
                quaternion = euler_to_quaternion(0.0, -math.pi/2, math.pi)
                position, orientation = self.getRobotWorldLocation()
                position.x = 0
                position.y = 0
                orientation.x = quaternion[0]
                orientation.y = quaternion[1]
                orientation.z = quaternion[2]
                orientation.w = quaternion[3]

                self.setRobotPose(position, orientation)

                # reset the PID controller errors
                curr_error = 0
                prev_error = 0
                tot_error = 0
                desired_dist = 0.325

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
        p = 0.1
        i = 0.1
        d = 0.1
        dt = 0.05

        u = (p * curr_error) + (i * tot_error * dt) + (d * (curr_error - prev_error) / dt)
        return u

    # function that makes the hexapod take one step forward
    def step(self, control, step):

        # home position joint 1, 2, 3
        home = [0.0, -0.50, 2.09]
        raise_j2 = -0.7

        # raise legs 1, 3, 5 (joint 2)
        list1 = ['leg1_j2', 'leg3_j2', 'leg5_j2']
    
        for i in range(len(list1)):
            self.setMotorTargetJointPosition(list1[i], raise_j2)
        # self.next_move(list1, [raise_j2]*len(list1))
        time.sleep(0.10)

        # rotate legs 1, 3, 5 forward (joint 1)
        # rotate legs 2, 4, 6 backward (joint 1)
        list2 = ['leg1_j1', 'leg2_j1', 'leg3_j1', 'leg4_j1', 'leg5_j1', 'leg6_j1', 'leg4_j3']
        target2 = [control, step, -step+control, 0.0, step+control, -step, 1.79]
        
        for i in range(len(list2)):
            self.setMotorTargetJointPosition(list2[i], target2[i])
        self.next_move(list2, target2)

        self.setMotorTargetJointPosition('leg4_j3', home[2])
        # self.next_move(['leg4_j3'], [home[2]])

        # lower legs 1, 3, 5 (joint 2)
        # raise legs 2, 4, 6 (joint 2)
        list3 = ['leg1_j2', 'leg2_j2', 'leg3_j2', 'leg4_j2', 'leg5_j2', 'leg6_j2']
        target3 = [home[1], raise_j2, home[1], raise_j2, home[1], raise_j2]

        for i in range(len(list3)):
            self.setMotorTargetJointPosition(list3[i], target3[i])
        # self.next_move(list3, target3)
        time.sleep(0.10)

        # rotate legs 2, 4, 6 forward (joint 1)
        # rotate legs 1, 3, 5 backward (joint 1)
        list4 = ['leg1_j1', 'leg2_j1', 'leg3_j1', 'leg4_j1', 'leg5_j1', 'leg6_j1', 'leg1_j3']
        target4 = [home[0], home[0], home[0], home[0], home[0], home[0], 1.79]

        for i in range(len(list4)):
            self.setMotorTargetJointPosition(list4[i], target4[i])
        self.next_move(list4, target4)

        self.setMotorTargetJointPosition('leg1_j3', home[2])
        # self.next_move(['leg1_j3'], [home[2]])

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
        for i in range(len(rotate_ids)):
            self.setMotorTargetJointPosition(rotate_ids[i], home[0])
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
    
if __name__ == "__main__":
    q = HexapodControl()
    rospy.spin()