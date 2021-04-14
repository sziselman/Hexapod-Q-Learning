#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range, JointState
import tf2_ros
from tf2_msgs.msg import TFMessage
import geometry_msgs.msg
import time
import numpy as np
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from me_cs301_robots.cfg import HexapodJointControlConfig, RWJointControlConfig
import math


class RobotControl(object):
    def __init__(self, robot_type='rollerwalker'):
        self.robot_type = robot_type
        if self.robot_type == 'hexapod':
            rospy.init_node('hexapod_control', anonymous=True)
        elif self.robot_type == 'rollerwalker':
            rospy.init_node('roller_walker_control', anonymous=True)
            
        print('INIT NODE')
        
        self.initialize_publishers()
        self.initialize_subscribers()
        self.sim_time = 0.0
        
        self.robot_pose = Pose()
        self.front_sensor_val = Float32()
        self.left_sensor_val = Float32()
        self.right_sensor_val = Float32()

        self.joint_states = JointState()
        self.SENSOR_TYPES = ['front', 'left', 'right']
        # self.is_go_down = False

        self.initialize_publisher_msgs()

        if self.robot_type == 'hexapod':
            self.paradigm_reconfigure_srv = DynamicReconfigureServer(HexapodJointControlConfig, self.reconfig_paradigm_cb)
        else:
            self.paradigm_reconfigure_srv = DynamicReconfigureServer(RWJointControlConfig, self.reconfig_paradigm_cb)

    def degToRad(self, deg):
        return deg*math.pi/180.0
        
    def initialize_publisher_msgs(self):
        if self.robot_type == 'hexapod':
            self.leg1_j1 = Float32()
            self.leg1_j2 = Float32()
            self.leg1_j3 = Float32()

            self.leg2_j1 = Float32()
            self.leg2_j2 = Float32()
            self.leg2_j3 = Float32()
            
            self.leg3_j1 = Float32()
            self.leg3_j2 = Float32()
            self.leg3_j3 = Float32()

            self.leg4_j1 = Float32()
            self.leg4_j2 = Float32()
            self.leg4_j3 = Float32()
            
            self.leg5_j1 = Float32()
            self.leg5_j2 = Float32()
            self.leg5_j3 = Float32()
            
            self.leg6_j1 = Float32()
            self.leg6_j2 = Float32()
            self.leg6_j3 = Float32()

        elif self.robot_type == 'rollerwalker':
            self.leg1_j1 = Float32()
            self.leg1_j2 = Float32()
            self.leg1_j3 = Float32()

            self.leg2_j1 = Float32()
            self.leg2_j2 = Float32()
            self.leg2_j3 = Float32()

            self.leg3_j1 = Float32()
            self.leg3_j2 = Float32()
            self.leg3_j3 = Float32()

            self.leg4_j1 = Float32()
            self.leg4_j2 = Float32()
            self.leg4_j3 = Float32()

    def initialize_publishers(self):
        # initialized latched publishers
        if self.robot_type == 'hexapod':
            self.leg1_j1_pub = rospy.Publisher('/leg1_j1', Float32, queue_size=1, latch=True)
            self.leg1_j2_pub = rospy.Publisher('/leg1_j2', Float32, queue_size=1, latch=True)
            self.leg1_j3_pub = rospy.Publisher('/leg1_j3', Float32, queue_size=1, latch=True)
            
            self.leg2_j1_pub = rospy.Publisher('/leg2_j1', Float32, queue_size=1, latch=True)
            self.leg2_j2_pub = rospy.Publisher('/leg2_j2', Float32, queue_size=1, latch=True)
            self.leg2_j3_pub = rospy.Publisher('/leg2_j3', Float32, queue_size=1, latch=True)

            self.leg3_j1_pub = rospy.Publisher('/leg3_j1', Float32, queue_size=1, latch=True)
            self.leg3_j2_pub = rospy.Publisher('/leg3_j2', Float32, queue_size=1, latch=True)
            self.leg3_j3_pub = rospy.Publisher('/leg3_j3', Float32, queue_size=1, latch=True)

            self.leg4_j1_pub = rospy.Publisher('/leg4_j1', Float32, queue_size=1, latch=True)
            self.leg4_j2_pub = rospy.Publisher('/leg4_j2', Float32, queue_size=1, latch=True)
            self.leg4_j3_pub = rospy.Publisher('/leg4_j3', Float32, queue_size=1, latch=True)

            self.leg5_j1_pub = rospy.Publisher('/leg5_j1', Float32, queue_size=1, latch=True)
            self.leg5_j2_pub = rospy.Publisher('/leg5_j2', Float32, queue_size=1, latch=True)
            self.leg5_j3_pub = rospy.Publisher('/leg5_j3', Float32, queue_size=1, latch=True)

            self.leg6_j1_pub = rospy.Publisher('/leg6_j1', Float32, queue_size=1, latch=True)
            self.leg6_j2_pub = rospy.Publisher('/leg6_j2', Float32, queue_size=1, latch=True)
            self.leg6_j3_pub = rospy.Publisher('/leg6_j3', Float32, queue_size=1, latch=True)

        elif self.robot_type == 'rollerwalker':
            self.leg1_j1_pub = rospy.Publisher('/leg1_j1', Float32, queue_size=1, latch=True)
            self.leg1_j2_pub = rospy.Publisher('/leg1_j2', Float32, queue_size=1, latch=True)
            self.leg1_j3_pub = rospy.Publisher('/leg1_j3', Float32, queue_size=1, latch=True)
            
            self.leg2_j1_pub = rospy.Publisher('/leg2_j1', Float32, queue_size=1, latch=True)
            self.leg2_j2_pub = rospy.Publisher('/leg2_j2', Float32, queue_size=1, latch=True)
            self.leg2_j3_pub = rospy.Publisher('/leg2_j3', Float32, queue_size=1, latch=True)

            self.leg3_j1_pub = rospy.Publisher('/leg3_j1', Float32, queue_size=1, latch=True)
            self.leg3_j2_pub = rospy.Publisher('/leg3_j2', Float32, queue_size=1, latch=True)
            self.leg3_j3_pub = rospy.Publisher('/leg3_j3', Float32, queue_size=1, latch=True)

            self.leg4_j1_pub = rospy.Publisher('/leg4_j1', Float32, queue_size=1, latch=True)
            self.leg4_j2_pub = rospy.Publisher('/leg4_j2', Float32, queue_size=1, latch=True)
            self.leg4_j3_pub = rospy.Publisher('/leg4_j3', Float32, queue_size=1, latch=True)
    
    def initialize_subscribers(self):
        rospy.Subscriber('/simTime', Float32, self.simTime_cb)
        rospy.Subscriber('/tf', TFMessage, self.tf_cb)

        rospy.Subscriber('/frontSensorDistance', Range, self.distance_front_cb)
        rospy.Subscriber('/leftSensorDistance',  Range, self.distance_left_cb)
        rospy.Subscriber('/rightSensorDistance', Range, self.distance_right_cb)

        if self.robot_type == 'hexapod':
            rospy.Subscriber('/hexapod/joint_states', JointState, self.joint_states_cb)
        elif self.robot_type == 'rollerwalker':
            rospy.Subscriber('/rollerwalker/joint_states', JointState, self.joint_states_cb)
    
    # subscriber callbacks
    def simTime_cb(self, msg):
        self.sim_time = msg.data
    
    def tf_cb(self, msg):
        self.robot_pose.position.x = msg.transforms[0].transform.translation.x
        self.robot_pose.position.y = msg.transforms[0].transform.translation.y
        self.robot_pose.position.z = msg.transforms[0].transform.translation.z
        self.robot_pose.orientation.x = msg.transforms[0].transform.rotation.x
        self.robot_pose.orientation.y = msg.transforms[0].transform.rotation.y
        self.robot_pose.orientation.z = msg.transforms[0].transform.rotation.z
        self.robot_pose.orientation.w = msg.transforms[0].transform.rotation.w

    #Sensor callbacks Same for both robots. 
    def distance_front_cb(self, msg):
        self.front_sensor_val.data = msg.range
    
    def distance_left_cb(self, msg):
        self.left_sensor_val.data = msg.range    

    def distance_right_cb(self, msg):
        self.right_sensor_val.data = msg.range
        
    def joint_states_cb(self, msg):
        self.joint_states = msg

    def publish_joint_values(self): 
        if self.robot_type == 'hexapod':
            self.leg1_j1_pub.publish(self.leg1_j1)
            self.leg1_j2_pub.publish(self.leg1_j2)
            self.leg1_j3_pub.publish(self.leg1_j3)

            self.leg2_j1_pub.publish(self.leg2_j1)
            self.leg2_j2_pub.publish(self.leg2_j2)
            self.leg2_j3_pub.publish(self.leg2_j3)
            
            self.leg3_j1_pub.publish(self.leg3_j1)
            self.leg3_j2_pub.publish(self.leg3_j2)
            self.leg3_j3_pub.publish(self.leg3_j3)

            self.leg4_j1_pub.publish(self.leg4_j1)
            self.leg4_j2_pub.publish(self.leg4_j2)
            self.leg4_j3_pub.publish(self.leg4_j3)

            self.leg5_j1_pub.publish(self.leg5_j1)
            self.leg5_j2_pub.publish(self.leg5_j2)
            self.leg5_j3_pub.publish(self.leg5_j3)

            self.leg6_j1_pub.publish(self.leg6_j1)
            self.leg6_j2_pub.publish(self.leg6_j2)
            self.leg6_j3_pub.publish(self.leg6_j3)

        elif self.robot_type == 'rollerwalker':
            self.leg1_j1_pub.publish(self.leg1_j1)
            self.leg1_j2_pub.publish(self.leg1_j2)
            self.leg1_j3_pub.publish(self.leg1_j3)

            self.leg2_j1_pub.publish(self.leg2_j1)
            self.leg2_j2_pub.publish(self.leg2_j2)
            self.leg2_j3_pub.publish(self.leg2_j3)
            
            self.leg3_j1_pub.publish(self.leg3_j1)
            self.leg3_j2_pub.publish(self.leg3_j2)
            self.leg3_j3_pub.publish(self.leg3_j3)

            self.leg4_j1_pub.publish(self.leg4_j1)
            self.leg4_j2_pub.publish(self.leg4_j2)
            self.leg4_j3_pub.publish(self.leg4_j3)
    
    def reconfig_paradigm_cb(self, config, level):
        if self.robot_type == 'hexapod':
            self.leg1_j1.data = config.leg1_j1
            self.leg1_j2.data = config.leg1_j2
            self.leg1_j3.data = config.leg1_j3

            self.leg2_j1.data = config.leg2_j1
            self.leg2_j2.data = config.leg2_j2
            self.leg2_j3.data = config.leg2_j3

            self.leg3_j1.data = config.leg3_j1
            self.leg3_j2.data = config.leg3_j2
            self.leg3_j3.data = config.leg3_j3

            self.leg4_j1.data = config.leg4_j1
            self.leg4_j2.data = config.leg4_j2
            self.leg4_j3.data = config.leg4_j3

            self.leg5_j1.data = config.leg5_j1
            self.leg5_j2.data = config.leg5_j2
            self.leg5_j3.data = config.leg5_j3

            self.leg6_j1.data = config.leg6_j1
            self.leg6_j2.data = config.leg6_j2
            self.leg6_j3.data = config.leg6_j3
            
        elif self.robot_type == 'rollerwalker':
            self.leg1_j1.data = config.leg1_j1
            self.leg1_j2.data = config.leg1_j2
            self.leg1_j3.data = config.leg1_j3

            self.leg2_j1.data = config.leg2_j1
            self.leg2_j2.data = config.leg2_j2
            self.leg2_j3.data = config.leg2_j3

            self.leg3_j1.data = config.leg3_j1
            self.leg3_j2.data = config.leg3_j2
            self.leg3_j3.data = config.leg3_j3

            self.leg4_j1.data = config.leg4_j1
            self.leg4_j2.data = config.leg4_j2
            self.leg4_j3.data = config.leg4_j3


        self.publish_joint_values()

        return config
    
    #getters 
    def getSensorValue(self, sensor_type='front'):
        assert sensor_type in self.SENSOR_TYPES
        if sensor_type=='front':
            return self.front_sensor_val.data
        elif sensor_type == 'left':
            return self.left_sensor_val.data
        elif sensor_type == 'right':
            return self.right_sensor_val.data
    
    def getMotorCurrentJointPosition(self, motor_id_string='leg1_j1'):
        assert motor_id_string in self.joint_states.name
        motor_id = self.joint_states.name.index(motor_id_string)
        return self.joint_states.position[motor_id]
    
    def getRobotWorldLocation(self):
        return self.robot_pose.position, self.robot_pose.orientation
    
    def getCurrentSimTime(self):
        return self.sim_time
    
    #setters
    
    def setMotorTargetJointPosition(self, motor_id_string='leg1_j1', target_joint_angle=0.0):
        if motor_id_string == 'leg1_j1':
            self.leg1_j1.data = target_joint_angle
        elif motor_id_string == 'leg1_j2':
            self.leg1_j2.data = target_joint_angle
        elif motor_id_string == 'leg1_j3':
            self.leg1_j3.data = target_joint_angle
        elif motor_id_string == 'leg2_j1':
            self.leg2_j1.data = target_joint_angle
        elif motor_id_string == 'leg2_j2':
            self.leg2_j2.data = target_joint_angle
        elif motor_id_string == 'leg2_j3':
            self.leg2_j3.data = target_joint_angle
        elif motor_id_string == 'leg3_j1':
            self.leg3_j1.data = target_joint_angle
        elif motor_id_string == 'leg3_j2':
            self.leg3_j2.data = target_joint_angle
        elif motor_id_string == 'leg3_j3':
            self.leg3_j3.data = target_joint_angle
        elif motor_id_string == 'leg4_j1':
            self.leg4_j1.data = target_joint_angle
        elif motor_id_string == 'leg4_j2':
            self.leg4_j2.data = target_joint_angle
        elif motor_id_string == 'leg4_j3':
            self.leg4_j3.data = target_joint_angle
        elif motor_id_string == 'leg5_j1':
            self.leg5_j1.data = target_joint_angle
        elif motor_id_string == 'leg5_j2':
            self.leg5_j2.data = target_joint_angle
        elif motor_id_string == 'leg5_j3':
            self.leg5_j3.data = target_joint_angle
        elif motor_id_string == 'leg6_j1':
            self.leg6_j1.data = target_joint_angle
        elif motor_id_string == 'leg6_j2':
            self.leg6_j2.data = target_joint_angle
        elif motor_id_string == 'leg6_j3':
            self.leg6_j3.data = target_joint_angle
        
        self.publish_joint_values()
    