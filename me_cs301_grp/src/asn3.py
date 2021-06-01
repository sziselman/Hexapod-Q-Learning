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

def fit2states(states, distance):
    array = np.asarray(states)
    index = (np.abs(array - distance)).argmin()
    return index

def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


class HexapodControl(RobotControl):
    N = 10 # the number of actions/states
    Q_mat = np.genfromtxt("/home/sarah/Documents/ME301/me_cs301_coppeliasim_robots/results.csv", delimiter=",")

    # Q_mat = np.zeros((N, N))

    def __init__(self):
        super(HexapodControl, self).__init__(robot_type='hexapod')
        rospy.loginfo("hexapod setup complete")
        self.hold_neutral()
        time.sleep(2.0)

        actions = np.linspace(0.35, 0.65, self.N)
        states = np.linspace(0.10, 0.40, self.N)

        # ##################################
        # # Uncomment section to build dataset
        # ##################################

        # runs = 0
        
        # while not rospy.is_shutdown():

        #     i = 0

        #     curr_error = 0
        #     prev_error = 0
        #     tot_error = 0
        #     desired_dist = 0.325

        #     prev_position, prev_orientation = self.getRobotWorldLocation()
        #     prev_x = prev_position.x
        #     prev_y = prev_position.y

        #     # have robot walk until it detects an obstacle in front, at each step update the q matrix
        #     while self.getSensorValue('front') < 0:

        #         # get the distance from the wall depending on the sensor that is being triggered, calculate current error for PID controller
        #         if self.getSensorValue('left') > 0 and self.getSensorValue('right') < 0:
        #             distance = self.getSensorValue('left')
        #             curr_error = distance - desired_dist
        #         elif self.getSensorValue('right') > 0 and self.getSensorValue('left') < 0:
        #             distance = self.getSensorValue('right')
        #             curr_error = distance - desired_dist
        #         elif self.getSensorValue('left') > 0 and self.getSensorValue('right') > 0:
        #             distance = self.getSensorValue('left')
        #             curr_error = distance - desired_dist
        #         else:
        #             break
                
        #         # transition function (distance from the wall)
        #         state_index = fit2states(states, distance)
        #         state = states[state_index]

        #         # randomly select an action for training
        #         random_action_index = random.randint(0, self.N-1)
        #         random_action = actions[random_action_index]

        #         # determine the control using PID
        #         control = self.pid_control(curr_error, prev_error, tot_error)

        #         if random_action + control > j1_limits[1]:
        #             control = j1_limits[1] - random_action
        #         elif -random_action + control < j1_limits[0]:
        #             control = j1_limits[0] + random_action

        #         # have robot take one step forward
        #         self.step(control, random_action)

        #         # reward function is distance traveled by each step
        #         curr_position, curr_orientation = self.getRobotWorldLocation()

        #         print('current position of robot is: ', curr_position.x, curr_position.y)

        #         distance_traveled = math.sqrt((curr_position.x - prev_x)**2 + (curr_position.y - prev_y)**2)

        #         prev_x = curr_position.x
        #         prev_y = curr_position.y

        #         print('the distance traveled is: ', distance_traveled)

        #         # increase cell in Q matrix corresponding to state/action by reward function
        #         self.Q_mat[state_index][random_action_index] += distance_traveled

        #         np.savetxt("results.csv", self.Q_mat, delimiter=",")
                
        #         i += 1
        #         runs += 1
        #         print('The amount of runs executed: ', runs, '\n')

        #     # reset the location and orientation of the robot to starting
        #     quaternion = euler_to_quaternion(0.0, -math.pi/2, math.pi)
        #     position, orientation = self.getRobotWorldLocation()
        #     position.x = 0
        #     position.y = 0
        #     orientation.x = quaternion[0]
        #     orientation.y = quaternion[1]
        #     orientation.z = quaternion[2]
        #     orientation.w = quaternion[3]

        #     self.setRobotPose(position, orientation)

        ##################################
        # Uncomment section to build model using Least Squares regression
        ##################################

        while not rospy.is_shutdown():
            
            curr_error = 0
            prev_error = 0
            tot_error = 0
            desired_dist = 0.325

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

            action_index = (self.Q_mat[state_index]).argmax()
            action = actions[action_index]

            # determine the control using PID
            control = self.pid_control(curr_error, prev_error, tot_error)

            if action + control > j1_limits[1]:
                control = j1_limits[1] - action
            elif - action + control < j1_limits[0]:
                control = j1_limits[0] + action

            # have robot take one step forward
            self.step(control, action)



            time.sleep(0.1)
            

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
        # time.sleep(0.1)

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
        # time.sleep(0.1)
       
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

    # function that moves the robot south
    def walk_forward(self, step):

        start_seconds = rospy.get_time()

        i = 0

        curr_error = 0
        prev_error = 0
        tot_error = 0
        dist = 0.325

        position, orientation = self.getRobotWorldLocation()
        x_distance_traveled = position.x

        while (x_distance_traveled > -2.0):

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

            self.step(u, step)

            i += 1

            position, orientation = self.getRobotWorldLocation()
            x_distance_traveled = position.x

        end_seconds = rospy.get_time()

        distance_traveled = math.sqrt(position.x**2 + position.y**2)

        velocity = distance_traveled / (end_seconds - start_seconds)

        return velocity

    # newton's method function
    def gradient_descent(self, w):
        weight_history = [w]
        gradient

    # compute linear combination of input point
    def model(self, x_p, w):
        a = w[0] + np.dot(x_p.T, w[1:])
        return a.T

    def least_squares(self, w, x, y):
        return

    
if __name__ == "__main__":
    q = HexapodControl()
    rospy.spin()