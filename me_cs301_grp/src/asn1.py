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

        #main control loop
        while not rospy.is_shutdown(): 
            # self.hold_neutral() #remove if not necessary
            # ---- add your code for a particular behavior here ----- #

            self.step()

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

    def next_move(self, joint_names, target_angles):

        flag = False
        while (not flag):
            flag = True
            for i in range(len(joint_names)):
                flag *= (abs(self.getMotorCurrentJointPosition(joint_names[i]) - target_angles[i]) < 1e-4)


    # def walk_forward(self):

    # function that makes the hexapod take one step forward
    def step(self):

        home_j1 = 0.0
        home_j2 = -0.50
        home_j3 = 2.09

        extend_j3 = 0.55
        extend_j2 = 0.18
        angle_change = math.pi/3

        raise_j2 = -1.0
        target_rai_j2 = -1.0
        

        # raise legs 1, 3, 5 (joint 2)
        # rotate legs 3 and 5 (joint 1)
        # extend leg 1 (joint 3)

        joint_names = ['hexa_leg1_j2', 'hexa_leg1_j3', 'hexa_leg3_j1', 'hexa_leg3_j2', 'hexa_leg5_j1', 'hexa_leg5_j2']
        target_angles = [raise_j2, extend_j3, -angle_change, raise_j2, angle_change, raise_j2]

        self.setMotorTargetJointPosition('leg1_j2', target_angles[0])
        self.setMotorTargetJointPosition('leg1_j3', target_angles[1])
        self.setMotorTargetJointPosition('leg3_j2', target_angles[3])
        self.setMotorTargetJointPosition('leg3_j1', target_angles[2])
        self.setMotorTargetJointPosition('leg5_j2', target_angles[5])
        self.setMotorTargetJointPosition('leg5_j1', target_angles[4])

        self.next_move(joint_names, target_angles)

        #####################################################################

        # lower legs 3 and 5
        joint_names = ['hexa_leg1_j2', 'hexa_leg3_j2', 'hexa_leg5_j2']
        target_angles = [home_j2]*len(joint_names)
        target_angles[0] = extend_j2

        self.setMotorTargetJointPosition('leg1_j2', target_angles[0])
        self.setMotorTargetJointPosition('leg3_j2', target_angles[1])
        self.setMotorTargetJointPosition('leg5_j2', target_angles[2])

        self.next_move(joint_names, target_angles)

        # rotate legs 2, 3, 5, 6
        # fold leg 1
        joint_names = ['hexa_leg1_j2', 'hexa_leg1_j3', 'hexa_leg2_j1', 'hexa_leg3_j1', 'hexa_leg4_j2', 'hexa_leg4_j3','hexa_leg5_j1', 'hexa_leg6_j1']
        target_angles = [home_j2, home_j3, angle_change, home_j1, extend_j2, extend_j3, home_j1, -angle_change]

        self.setMotorTargetJointPosition('leg1_j2', target_angles[0])
        self.setMotorTargetJointPosition('leg1_j3', target_angles[1])
        self.setMotorTargetJointPosition('leg2_j1', target_angles[2])
        self.setMotorTargetJointPosition('leg3_j1', target_angles[3])
        self.setMotorTargetJointPosition('leg4_j2', target_angles[4])
        self.setMotorTargetJointPosition('leg4_j3', target_angles[5])
        self.setMotorTargetJointPosition('leg5_j1', target_angles[6])
        self.setMotorTargetJointPosition('leg6_j1', target_angles[7])

        self.next_move(joint_names, target_angles)

        ########################################################################

        # raise legs 2,4 and 6
        # move legs 2, 4 and 6 back to home position

        joint_names = ['hexa_leg2_j1', 'hexa_leg2_j2', 'hexa_leg4_j2', 'hexa_leg4_j3', 'hexa_leg6_j1', 'hexa_leg6_j2']
        target_angles = [home_j1, raise_j2, raise_j2, home_j3, home_j1, raise_j2]

        self.setMotorTargetJointPosition('leg2_j1', target_angles[0])
        self.setMotorTargetJointPosition('leg2_j2', target_angles[1])
        self.setMotorTargetJointPosition('leg4_j2', target_angles[2])
        self.setMotorTargetJointPosition('leg4_j3', target_angles[3])
        self.setMotorTargetJointPosition('leg6_j1', target_angles[4])
        self.setMotorTargetJointPosition('leg6_j2', target_angles[5])

        self.next_move(joint_names, target_angles)

        ##########################################################################

        # lower legs 2 and 6 back to home position
        joint_names = ['hexa_leg2_j2', 'hexa_leg4_j2', 'hexa_leg6_j2']
        target_angles = [home_j2]*len(joint_names)

        self.setMotorTargetJointPosition('leg2_j2', target_angles[0])
        self.setMotorTargetJointPosition('leg4_j2', target_angles[1])
        self.setMotorTargetJointPosition('leg6_j2', target_angles[2])

        self.next_move(joint_names, target_angles)
        

    # function that rotates the hexapod by one step size (pi/6 radians) in a given direction
    def turn(self, cw):

        target_rai_j2 = -1
        target_res_j2 = -0.50

        if (cw == True):
            angle_change = -math.pi/6
        else:
            angle_change = math.pi/6

        # raise legs 1, 3, 5

        joint_names = ['hexa_leg1_j2', 'hexa_leg3_j2', 'hexa_leg5_j2']
        target_angles = [target_rai_j2]*len(joint_names)

        self.setMotorTargetJointPosition('leg1_j2', target_rai_j2)
        self.setMotorTargetJointPosition('leg3_j2', target_rai_j2)
        self.setMotorTargetJointPosition('leg5_j2', target_rai_j2)

        self.next_move(joint_names, target_angles)

        # rotate legs 1, 3, 5
        joint_names = ['hexa_leg1_j1', 'hexa_leg2_j1', 'hexa_leg3_j1', 'hexa_leg4_j1', 'hexa_leg5_j1', 'hexa_leg6_j1']
        target_angles = [0]*len(joint_names)
        target_angles[0] = self.getMotorCurrentJointPosition(joint_names[0]) + angle_change
        target_angles[1] = self.getMotorCurrentJointPosition(joint_names[1]) - angle_change
        target_angles[2] = self.getMotorCurrentJointPosition(joint_names[2]) + angle_change
        target_angles[3] = self.getMotorCurrentJointPosition(joint_names[3]) - angle_change
        target_angles[4] = self.getMotorCurrentJointPosition(joint_names[4]) + angle_change
        target_angles[5] = self.getMotorCurrentJointPosition(joint_names[5]) - angle_change

        self.setMotorTargetJointPosition('leg1_j1', target_angles[0])
        self.setMotorTargetJointPosition('leg2_j1', target_angles[1])
        self.setMotorTargetJointPosition('leg3_j1', target_angles[2])
        self.setMotorTargetJointPosition('leg4_j1', target_angles[3])
        self.setMotorTargetJointPosition('leg5_j1', target_angles[4])
        self.setMotorTargetJointPosition('leg6_j1', target_angles[5])

        self.next_move(joint_names, target_angles)

        # lower legs 1, 3, 5
        joint_names = ['hexa_leg1_j2', 'hexa_leg3_j2', 'hexa_leg5_j2']
        target_angles = [target_res_j2]*len(joint_names)

        self.setMotorTargetJointPosition('leg1_j2', target_res_j2)
        self.setMotorTargetJointPosition('leg3_j2', target_res_j2)
        self.setMotorTargetJointPosition('leg5_j2', target_res_j2)

        self.next_move(joint_names, target_angles)

        # raise legs 2, 4, 6

        joint_names = ['hexa_leg2_j2', 'hexa_leg4_j2', 'hexa_leg6_j2']
        target_angles = [target_rai_j2]*len(joint_names)

        self.setMotorTargetJointPosition('leg2_j2', target_rai_j2)
        self.setMotorTargetJointPosition('leg4_j2', target_rai_j2)
        self.setMotorTargetJointPosition('leg6_j2', target_rai_j2)

        self.next_move(joint_names, target_angles)

        # rotate legs 2, 4, 6
        joint_names = ['hexa_leg1_j1', 'hexa_leg2_j1', 'hexa_leg3_j1', 'hexa_leg4_j1', 'hexa_leg5_j1', 'hexa_leg6_j1']
        target_angles = [0]*len(joint_names)
        target_angles[0] = self.getMotorCurrentJointPosition(joint_names[0]) - angle_change
        target_angles[1] = self.getMotorCurrentJointPosition(joint_names[1]) + angle_change
        target_angles[2] = self.getMotorCurrentJointPosition(joint_names[2]) - angle_change
        target_angles[3] = self.getMotorCurrentJointPosition(joint_names[3]) + angle_change
        target_angles[4] = self.getMotorCurrentJointPosition(joint_names[4]) - angle_change
        target_angles[5] = self.getMotorCurrentJointPosition(joint_names[5]) + angle_change

        self.setMotorTargetJointPosition('leg1_j1', target_angles[0])
        self.setMotorTargetJointPosition('leg2_j1', target_angles[1])
        self.setMotorTargetJointPosition('leg3_j1', target_angles[2])
        self.setMotorTargetJointPosition('leg4_j1', target_angles[3])
        self.setMotorTargetJointPosition('leg5_j1', target_angles[4])
        self.setMotorTargetJointPosition('leg6_j1', target_angles[5])

        self.next_move(joint_names, target_angles)

        # lower legs 2, 4, 6
        joint_names = ['hexa_leg2_j2', 'hexa_leg4_j2', 'hexa_leg6_j2']
        target_angles = [target_res_j2]*len(joint_names)

        self.setMotorTargetJointPosition('leg2_j2', target_res_j2)
        self.setMotorTargetJointPosition('leg4_j2', target_res_j2)
        self.setMotorTargetJointPosition('leg6_j2', target_res_j2)

        self.next_move(joint_names, target_angles)

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