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

class tempClass():
    def __init__(self):
        print "test"

        rospy.init_node('TestController', anonymous=True)
        rospy.Subscriber("/joint_states", JointState, self.jointStateCallback)
        rospy.Subscriber("/tool_velocity", TwistStamped, self.toolVelocityCallback)
        rospy.Subscriber("/wrench", WrenchStamped, self.wrenchCallback)





if __name__ == '__main__':
    tempClass()




