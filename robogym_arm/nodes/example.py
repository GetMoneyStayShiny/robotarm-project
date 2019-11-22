#!/usr/bin/env python

import rospy
import time
import tf
from numpy import matrix
from numpy import linalg
import math

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# --------------------------------------------------------------------
#  Define the node as a class
# --------------------------------------------------------------------

class example_node():
    # Initialize the parameters and topics
    def __init__(self):

        # Start listener:
        self.listener = tf.TransformListener()

        # Register the subscribtion to the topic
        rospy.Subscriber("/joint_states", JointState, self.cb1)

        # Register the Publisher to the topic
        self.urScriptPub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)

        # Go into spin, with rateOption!
        self.rate = rospy.Rate(10)  # 10hz
        rospy.loginfo(rospy.get_caller_id() + "Example node started...")

        # To keep the node alive!
        self.spin()

    # --------------------------------------------------------------------
    #  CBs
    # --------------------------------------------------------------------
    def cb1(self, data):
        # Get update from the manipulator:
        self.joints = data.position;
        # print self.joints;
        # print type(self.joints)

    # --------------------------------------------------------------------
    # spin
    # --------------------------------------------------------------------
    def spin(self):
        while (not rospy.is_shutdown()):
            trans = self.FTSframe()

            # Sending Command to the manipulator:
            # command = "speedl([" +str(V_ref[0,0]) +","+  str(-V_ref[1,0]) +",0,0,0,"+ str(-V_ref[2,0]) + "],0.01, 0.1)";
            command = "speedl([0,0,0.1,0,0,0],0.01, 0.1)";
            print command
            self.urScriptPub.publish(command)

            # Go into spin, with rateOption!
            self.rate.sleep()

        # --------------------------------------------------------------------

    def FTSframe(self):
        try:
            self.listener.waitForTransform('/base', '/ee_link', rospy.Time(0), rospy.Duration(1))
            (trans, rot) = self.listener.lookupTransform('/base', '/ee_link', rospy.Time(0))
            transrotM = self.listener.fromTranslationRotation(trans, rot)
            rotationMat = transrotM[0:3, 0:3]
            print transrotM
            return transrotM
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "EXCEPTION"


#             pass

# --------------------------------------------------------------------
# --------------------------------------------------------------------
# --------------------------------------------------------------------
# Here is the main entry point
# --------------------------------------------------------------------
if __name__ == '__main__':
    try:
        # Init the node:
        rospy.init_node('example_node')

        # Initializing and continue running the class example_node:
        example_node()

    except rospy.ROSInterruptException:
        pass