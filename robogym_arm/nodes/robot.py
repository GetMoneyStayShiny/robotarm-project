#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import time
import tf
import select
import math
import sys, termios, tty, os, time
# import Tkinter as tk
import threading
# import matplotlib.pyplot as plt
# import matplotlib.figure
# import matplotlib.animation as animation

from math import cos, sin
from numpy.linalg import inv
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, WrenchStamped,Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Wrench


class Robot():
    def __init__(self,freq=125):
        rospy.init_node('RoboGYM', anonymous=True)

        # Publisher and Subscribers
        self.listener = tf.TransformListener()
        # self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
        self.pub = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)
        # Start-pose for rowing exercise
        position = np.array([0.15756176236973007, 0.3590548461660456, 0.41006310959573894])
        quaternion = np.array([-0.70700723, -0.00474353, 0.00277345, 0.7071849])

        temp_pose = Pose()
        temp_pose.orientation.x = -0.70700723
        temp_pose.orientation.y = -0.00474353
        temp_pose.orientation.z = 0.00277345
        temp_pose.orientation.w = 0.7071849

        print type(temp_pose)

        temp_quaternion = (
            temp_pose.orientation.x,
            temp_pose.orientation.y,
            temp_pose.orientation.z,
            temp_pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(temp_quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        rospy.logwarn("ROLL %f",roll)
        rospy.logwarn("PITCH %f", pitch)
        rospy.logwarn("YAW %f", yaw)

        pose = [0.2,0.3,0.5,0,0,3.14]


        init = True
        self.rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():

            if init:
                self.free_drive_mode()
                init = False
                tmpcmd = 'speedj([0.2,0.3,0.1,0.05,0,0], 0.5, 5)'
                self.pub.publish(tmpcmd)

            rospy.logwarn("POSE %d",pose[0])
            # self.temp_pose(pose,1.2,0.25)
            rospy.logwarn("Velocity")
            control_law = self.pose_control()
            self.velocity_cmd(control_law)
            self.rate.sleep()

    def free_drive_mode(self):
        self.mode = 'free drive mode'
        command = "freedrive_mode()"
        # self.command_mode(command)

    def command_mode(self, mode):
        command = "def command_mode():\n\n\t" + mode + "\n\twhile (True):\n\t\tsync()\n\tend\nend\n"
        rospy.loginfo(command)
        self.pub.publish(command)

    def velocity_cmd(self, control_law, acceleration=5, time=5):
        command = "speedl(" + np.array2string(control_law, precision=3, separator=',') + "," + \
                  str(acceleration) + "," + str(time) + ")"  # 0.3,0.2
        # rospy.loginfo(control_law)
        # print(command)
        tmpcmd = 'speedj([0.2,0.3,0.1,0.05,0,0], 0.5, 5)'
        self.pub.publish(tmpcmd)

    def temp_pose(self,pose,a,v,t=0.05,r=0):
        command = "movel([0.6,3,0.1,1,0,4.14]" + "," + str(a)  + "," + str(t) + ")"
        self.pub.publish(command)

    def pose_control(self,kp=5):
        self.mode = 'position controller'
        quaternion = self.getTaskQuaternion()
        actual_position = self.getTaskPosi()
        desired_position_base = np.array([0.15756176236973007, 0.3590548461660456, 0.41006310959573894])
        desired_quaternion = np.array([-0.70700723, -0.00474353, 0.00277345, 0.7071849])

        error_position = desired_position_base - actual_position
        error_angle = tf.transformations.quaternion_multiply(desired_quaternion,
                                                             tf.transformations.quaternion_inverse(quaternion))
        error = np.append(error_position, error_angle[0:3])

        # V = kp * A(phi_e) * (xd - x)
        control_law = kp * error
        return control_law

    def transfMat(self, frame_1, frame_2):
        try:
            self.listener.waitForTransform('/' + frame_1, '/' + frame_2, rospy.Time(0), rospy.Duration(1))
            (trans, rot) = self.listener.lookupTransform('/' + frame_1, '/' + frame_2, rospy.Time(0))
            transrotM = self.listener.fromTranslationRotation(trans, rot)

            # Comment on the rotation sequences
            # 'sxyz' is the sequence for getting the euler-angles RPY(Roll,Pitch,Yaw)in radians
            # it means static rotation around x-y-z
            euler = tf.transformations.euler_from_matrix(transrotM, 'sxyz')
            quaternion = tf.transformations.quaternion_from_matrix(transrotM)
            return trans, euler, transrotM, quaternion
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "EXCEPTION"
            pass


    # Returns x-y and z pos in meters as a vector [x, y, z]
    def getTaskPosi(self):
        result = self.transfMat('base', 'tool0_controller')
        self.position = result[0]
        return self.position

    # Returns current rotation in quaternions
    def getTaskQuaternion(self):
        result = self.transfMat('base', 'tool0_controller')
        self.quaternion = result[3]
        return self.quaternion


if __name__ == '__main__':
    Robot()
