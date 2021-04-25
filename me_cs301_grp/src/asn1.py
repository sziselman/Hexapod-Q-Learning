#!/usr/bin/env python
import rospy
import time
import math
import sys
import os
import rospkg
sys.path.append(os.path.join(rospkg.RosPack().get_path('me_cs301_robots'), 'scripts'))
from robot_control import RobotControl

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

j1_limits = [-1.36, 0.415]
j2_limits = [-1.30, 0.52]
j3_limits = [-3.14, 3.14]

# global rotate_j1 = math.pi/3
rotate_j1 = 0.2

def quat_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians

class HexapodControl(RobotControl):
    def __init__(self):
        super(HexapodControl, self).__init__(robot_type='hexapod')
        rospy.loginfo("hexapod setup complete")
        self.hold_neutral()
        time.sleep(2.0)

        curr_error = 0
        prev_error = 0
        tot_error = 0
        dist = 0.20

        #main control loop
        while not rospy.is_shutdown(): 
            # self.hold_neutral() #remove if not necessary
            # ---- add your code for a particular behavior here ----- #

            # if left sensor reading (wall to the left)
            if self.getSensorValue('left') > 0:
                print(self.getSensorValue('left'))
                curr_error = self.getSensorValue('left') - dist
                tot_error += curr_error
                u = self.pid_control(curr_error, prev_error, tot_error)
            elif self.getSensorValue('right') > 0:
                curr_error = self.getSensorValue('right') - dist
                tot_error += curr_error
                u = -self.pid_control(curr_error, prev_error, tot_error)
            else:
                curr_error = 0
                u = 0
            
            if self.getMotorCurrentJointPosition('hexa_leg5_j1') + rotate_j1 + u > j1_limits[1]:
                u = j1_limits[1] - self.getMotorCurrentJointPosition('hexa_leg5_j1') - rotate_j1


            self.step(u)
            prev_error = curr_error

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
        p = 5.0
        i = 0.0
        d = 0.1
        dt = 0.05

        u = (p * curr_error) + (i * tot_error * dt) + (d * (curr_error - prev_error) / dt)
        return u

    # function that makes the hexapod take one step forward
    def step(self, control):
        print("the control is")
        print(control)

        # home position joint 1, 2, 3
        home = [0.0, -0.50, 2.09]

        # extended position joint 1, 2, 3 (only used for legs 1 and 4)
        extend = [0.0, 0.18, 0.55]

        raise_j2 = -1.0
        # rotate_j1 = math.pi/3
        # rotate_j1 = 0.2
        
        tripod1_j1_ids = ['leg1_j1', 'leg3_j1', 'leg5_j1']
        tripod1_j2_ids = ['leg1_j2', 'leg3_j2', 'leg5_j2']

        tripod2_j1_ids = ['leg2_j1', 'leg4_j1', 'leg6_j1']
        tripod2_j2_ids = ['leg2_j2', 'leg4_j2', 'leg6_j2']

        tripod1_step_ids = ['leg1_j3', 'leg3_j1', 'leg5_j1']
        tripod2_step_ids = ['leg2_j1', 'leg4_j3', 'leg6_j1']
        transition_ids = ['leg1_j2', 'leg1_j3', 'leg2_j1', 'leg3_j1', 'leg4_j2', 'leg4_j3', 'leg5_j1', 'leg6_j1', 'leg1_j1']

        # raise legs 1, 3, 5 (joint 2)
        for i in range(len(tripod1_j2_ids)):
            self.setMotorTargetJointPosition(tripod1_j2_ids[i], raise_j2)
        self.next_move(tripod1_j2_ids, [raise_j2]*len(tripod1_j2_ids))

        # rotate legs 1, 3, 5 (joint 1)
        rotate_angles = [self.getMotorCurrentJointPosition('hexa_leg1_j1') + control,
                         self.getMotorCurrentJointPosition('hexa_leg3_j1') -rotate_j1 + control,
                         self.getMotorCurrentJointPosition('hexa_leg5_j1') +rotate_j1 + control]
        for i in range(len(tripod1_j1_ids)):
            self.setMotorTargetJointPosition(tripod1_j1_ids[i], rotate_angles[i])
        self.next_move(tripod1_j1_ids, rotate_angles)

        self.setMotorTargetJointPosition('leg1_j3', extend[2])
        self.next_move(['leg1_j3'], [extend[2]])

        # lower 3 and 5
        # lowering to extended position leg 1
        lower_t1_angles = [extend[1], home[1], home[1]]
        for i in range(len(tripod1_j2_ids)):
            self.setMotorTargetJointPosition(tripod1_j2_ids[i], lower_t1_angles[i])
        self.next_move(tripod1_j2_ids, lower_t1_angles)

        # rotate legs 2, 3, 5, 6
        # return leg 1 to home
        # extend leg 4
        transition_angles = [home[1],
                             home[2],
                             rotate_j1 - control,
                             home[0],
                             extend[1],
                             extend[2],
                             home[0],
                             -rotate_j1 - control,
                             home[0]]
        for i in range(len(transition_ids)):
            self.setMotorTargetJointPosition(transition_ids[i], transition_angles[i])
        self.next_move(transition_ids, transition_angles)

        self.setMotorTargetJointPosition('leg1_j1', home[0])
        self.next_move(['leg1_j1'], [home[0]])

        # raise legs 2,4 and 6
        for i in range(len(tripod2_j2_ids)):
            self.setMotorTargetJointPosition(tripod2_j2_ids[i], raise_j2)
        self.next_move(tripod2_j2_ids, [raise_j2]*len(tripod2_j2_ids))

        # move legs 2, 4 and 6 back to home position
        rotate_fold_angles = [home[0], home[2], home[0]]
        for i in range(len(tripod2_step_ids)):
            self.setMotorTargetJointPosition(tripod2_step_ids[i], rotate_fold_angles[i])
        self.next_move(tripod2_step_ids, rotate_fold_angles)

        # lower legs 2 and 6 back to home position
        for i in range(len(tripod2_j2_ids)):
            self.setMotorTargetJointPosition(tripod2_j2_ids[i], home[1])
        self.next_move(tripod2_j2_ids, [home[1]]*len(tripod2_j2_ids))
        

    # function that rotates the hexapod by one step size (pi/6 radians) in a given direction
    def turn(self, cw):

        if (cw == True):
            rotate_j1 = -math.pi/6
        else:
            rotate_j1 = math.pi/6

        raise_j2 = -1.0

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
        rotate_t2_angles = [-rotate_j1, rotate_j1, -rotate_j1, rotate_j1, -rotate_j1, rotate_j1]

        for i in range(len(rotate_ids)):
            self.setMotorTargetJointPosition(rotate_ids[i], rotate_t2_angles[i])
        self.next_move(rotate_ids, rotate_t2_angles)

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