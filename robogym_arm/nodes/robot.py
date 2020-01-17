#!/usr/bin/env python
import rospy
import numpy as np
import tf
import math
from numpy import matrix
from math import cos, sin
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, WrenchStamped,Pose
from robogym_arm.msg import InterfaceStamped,RobotStamped
import enum

class Robot:
    def __init__(self):
        rospy.init_node('RoboGYM', anonymous=True)

        # Robot States
        self.joints_order = []
        self.q = []
        self.dq = []
        self.position = []
        self.orientation = []
        self.tool_velocity_linear = []
        self.tool_velocity_angular = []
        self.force = []
        self.torque = []
        self.RotationM = []
        self.mode = 'Stop'
        self.atHome = False

        # Set ros-launch params
        isSimulation = rospy.get_param('~is_simulation', default=False)
        self.rate = rospy.Rate(rospy.get_param('~rate', default=125))
        self.lamda = rospy.get_param('~lambda', default=0.1)
        self.isPsudoInverse = rospy.get_param('~is_psudo_inverse', default=False)

        # Default exercise
        self.exerciseType = ExerciseType.Horizontal

        # Sensor measurements
        self.force_sensor = []
        self.torque_sensor = []
        self.sensor_header = WrenchStamped().header

        # Publisher and Subscribers
        self.listener = tf.TransformListener()

        if isSimulation:
            self.pub = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)
        else:
            self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)

        # rospy.Subscriber("/optoforce_node/wrench_HEXHA094", WrenchStamped, self.wrenchSensorCallback)
        self.pub_reset = rospy.Publisher('/ethdaq_zero', Bool, queue_size=1)
        # Publishers for the position
        self.pub_x = rospy.Publisher('/position_x', Float32, queue_size=1)
        self.pub_y = rospy.Publisher('/position_y', Float32, queue_size=1)
        self.pub_z = rospy.Publisher('/position_z', Float32, queue_size=1)
        # Publishers for the forces
        self.pub_force_x = rospy.Publisher('/force_x', Float32, queue_size=1)
        self.pub_force_y = rospy.Publisher('/force_y', Float32, queue_size=1)
        self.pub_force_z = rospy.Publisher('/force_z', Float32, queue_size=1)

        # Publisher for the interface ()
        self.interface_pub = rospy.Publisher('/robogym/robot_interface_status',RobotStamped,queue_size=1)

        rospy.Subscriber("/joint_states", JointState, self.jointStateCallback)
        rospy.Subscriber("/tool_velocity", TwistStamped, self.toolVelocityCallback)
        rospy.Subscriber("/wrench", WrenchStamped, self.wrenchCallback)
        rospy.Subscriber("/ethdaq_data_raw", WrenchStamped, self.wrenchSensorCallback)
        rospy.Subscriber("/ethdaq_data", WrenchStamped, self.wrenchSensorCallback)
        rospy.Subscriber("/robogym/interface_cmd",InterfaceStamped,self.interfaceCallback)
        # Sensor publishers and subscribers
        # self.pub_reset = rospy.Publisher('/optoforce_node/reset', Bool, queue_size=1)
        self.hasExerciseChanged = False
        init = True
        rospy.loginfo("STARTING ROBOT NODE")
        while not rospy.is_shutdown():
            if init or self.hasExerciseChanged:


                if self.hasExerciseChanged:
                    rospy.logwarn("Changing the exercise")
                    rospy.sleep(1)
                    rospy.logwarn("Moving to new position")
                else:
                    rospy.sleep(1)

                position, quaternion = self.getGoalPosition()
                self.start_pose = np.array([position, quaternion])

                print self.start_pose

                init = False
                self.hasExerciseChanged = False

            self.getCurrentTrasformationData()
            self.pubPosition()

            q1 = self.q[0]
            q2 = self.q[1]
            q3 = self.q[2]
            q4 = self.q[3]
            q5 = self.q[4]
            q6 = self.q[5]
            controller = self.impedance_control(self.start_pose, 50)

            print self.start_pose
            print controller
            Jac_psudo = self.jac_function(q1, q2, q3, q4, q5, q6)
            V_ref = np.transpose(controller)

            dq = np.matmul(Jac_psudo, V_ref)
            dq_value = np.asarray(dq).reshape(-1)

            self.q_dot(dq_value)

            # self.getRobotState()
            self.rate.sleep()

    # region Callbacks
    def jointStateCallback(self, data):
        self.q = data.position
        self.dq = data.velocity
        self.joints_order = data.name

    def toolVelocityCallback(self, data):
        self.tool_velocity_linear = [data.twist.linear.x, data.twist.linear.y, data.twist.linear.z]
        self.tool_velocity_angular = [data.twist.angular.x, data.twist.angular.y, data.twist.angular.z]

    def wrenchCallback(self, data):
        self.force = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]
        self.torque = [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]

    def interfaceCallback(self,data):
        if(data.interface.exercise_type != 0):
            self.hasExerciseChanged = True
            if(data.interface.exercise_type == 1):
                self.exerciseType = ExerciseType.Horizontal
            elif(data.interface.exercise_type == 2):
                self.exerciseType = ExerciseType.Vertical
            elif(data.interface.exercise_type == 3):
                self.exerciseType = ExerciseType.BicepCurl

    def wrenchSensorCallback(self, data):
        self.force_sensor = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]
        self.torque_sensor = [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]
        self.sensor_header = data.header
    # endregion

    # region Publishers for interface and for the system
    def pubPosition(self):
        self.pub_x.publish(self.position[0] - self.start_pose[0][0])
        self.pub_y.publish(self.position[1] - self.start_pose[0][1])
        self.pub_z.publish(self.position[2] - self.start_pose[0][2])

    def pubForce(self,x,y,z):
        self.pub_force_x.publish(x)
        self.pub_force_y.publish(y)
        self.pub_force_z.publish(z)

    def publishToInterface(self):
        robotMsg = RobotStamped()
        robotMsg.robot.at_home = self.atHome
        # robotMsg.robot.force_str =

    def getRobotState(self):
        state = {}
        state['joints_order'] = self.joints_order
        state['joint_angles'] = self.q
        state['joint_velocity'] = self.dq
        state['end_effector_position'] = self.position
        state['end_effector_orientation'] = self.orientation
        state['end_effector_linear_velocity'] = self.tool_velocity_linear
        state['end_effector_angular_velocity'] = self.tool_velocity_angular
        state['force'] = self.force
        state['torque'] = self.torque
        state['Transf_matrix'] = self.RotationM
        state['mode'] = self.mode
        return state
    # endregion

    # region UR10 function
    def stop(self):
        self.mode = 'stop'
        command1 = "stopj(1) \n"
        command2 = "stopl(1) \n"
        self.pub.publish(command1)
        self.pub.publish(command2)

    def command_mode(self, mode):
        command = "def command_mode():\n\n\t" + mode + "\n\twhile (True):\n\t\tsync()\n\tend\nend\n"
        rospy.loginfo(command)
        self.pub.publish(command)

    def end_force_mode(self):
        self.mode = 'Stop'
        command = "end_force_mode()"
        self.pub.publish(command)

    def end_free_drive_mode(self):
        self.mode = 'Stop'
        command = "end_freedrive_mode()"
        self.pub.publish(command)

    def force_mode(self, task_frame, selection_vector, wrench, type_, limits):
        self.mode = 'force mode'
        command = "force_mode(" + task_frame + "," + selection_vector + "," + wrench + "," \
                  + type_ + "," + limits + ")"
        self.command_mode(command)

    def free_drive_mode(self):
        self.mode = 'free drive mode'
        command = "freedrive_mode()"
        self.command_mode(command)

    # Used to command linear speed command to the robot
    # control_law is a 6x1 vector with tool speed in x,y and z-direction and rotational speed around x-y and z
    # def velocity_cmd(self, control_law, acceleration=5, time=0.05):
    #     command = "speedl(" + np.array2string(control_law, precision=3, separator=',') + "," + \
    #               str(acceleration) + "," + str(time) + ")"  # 0.3,0.2
    #     # rospy.loginfo(control_law)
    #     # print(command)
    #     self.pub.publish(command)

    def q_dot(self, dq_value, acceleration=5, time=0.05):
        command = "speedj(" + np.array2string(dq_value, precision=3, separator=',') + "," + \
                  str(acceleration) + "," + str(time) + ")"  # 0.3,0.2
        self.pub.publish(command)

    # endregion

    # region Get functions
    def getGoalPosition(self):
        position = np.array([0, 0, 0])
        quaternion = np.array([0, 0, 0, 0])
        if(self.exerciseType == ExerciseType.Horizontal):
            position = np.array([-0.15792, -0.35975, 0.4525])
            quaternion = np.array([ 0.6757655,  -0.00147379,  0.00949065,  0.7370541])
        elif(self.exerciseType == ExerciseType.Vertical):
            position = np.array([-0.15651, -0.69471, 0.0553])
            quaternion = np.array([0.67570833, -0.01028819, 0.01689194, 0.7369037])
        elif(self.exerciseType == ExerciseType.BicepCurl):
            position = np.array([-0.07609, -0.47965, -0.01873])
            quaternion = np.array([0.67242794, 0.06324287, 0.06463054, 0.7346182])

        return position, quaternion

    def getWrench(self):
        wrench = np.concatenate((np.array(self.force_sensor), np.array(self.torque_sensor))) - self.force_offset
        if len(wrench) == 0:
            wrench = np.array([-1, 0, 0, 0, 0, 0])
        return wrench

    def getCurrentTrasformationData(self):
        result = self.transfMat('base', 'tool0_controller')
        self.position  = result[0]
        self.orientation = result[1]
        self.quaternion = result[2]
        self.RotationM = result[3]
    # endregion

    # region ROS functions
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
            return trans, euler, quaternion, transrotM

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "EXCEPTION"
            pass

    # endregion

    # region Rotational matrices
    def Matrix_T(self, phi, theta):  # mapping matrix from euler ZYZ velocities and angular velocities
        Matrix = np.matrix(
            [[0, -sin(phi), cos(phi) * sin(theta)], [0, cos(phi), sin(phi) * sin(theta)], [1.0, 0, cos(theta)]])
        # Matrix = np.matrix( [ ])
        return Matrix

    def rotateToolFrame(self,axis,angle,measuredVec):
        rotMat = []
        if axis == 'x':
            rotMat = np.matrix([[1, 0, 0],
                                [0, cos(math.radians(angle)), -sin(math.radians(angle))],
                                [0, sin(math.radians(angle)), cos(math.radians(angle))]])
        elif axis == 'y':
            rotMat = np.matrix([[cos(math.radians(angle)), 0, sin(math.radians(angle))],
                                [0, 1, 0],
                                [-sin(math.radians(angle)), 0, cos(math.radians(angle))]])
        elif axis == 'z':
            rotMat = np.matrix([[cos(math.radians(angle)), -sin(math.radians(angle)), 0],
                                [sin(math.radians(angle)), cos(math.radians(angle)), 0],
                                [0, 0, 1]])

        return np.matmul(rotMat,np.transpose(measuredVec))
    # Transforms control from tool frame to base frame if frame is set to 'tool'
    # Used to tranform the fwrench witch is measured in tool frame and calculations are made in base frame
    def reference_frame(self, frame, control_law):
        rot_mat = self.RotationM
        if frame == 'base':
            control_law = control_law
        elif frame == 'tool':
            linear_velocity = np.matmul(rot_mat[0:3, 0:3], control_law[0:3])
            angular_velocity = np.matmul(rot_mat[0:3, 0:3], control_law[3:6])
            control_law = np.concatenate((linear_velocity, angular_velocity))
        else:
            raise NameError('Specify base or tool as a frame!')
        return control_law

    # endregion

    def impedance_control(self, desired_pose, k, force_scaling=0.00005):
        c = 160
        self.mode = 'impedance controller'
        desired_position_base = desired_pose[0]
        desired_quaternion = desired_pose[1]
        error_position = desired_position_base - self.position
        print self.position
        # multiplication of two quaternions gives the rotations of doing the two rotations consecutive,
        # could be seen as "adding" two rotations
        # multiplying the desired quaternion with the inverse of the current quaternion gives the error,
        # could be seen as the "difference" between them
        error_angle = tf.transformations.quaternion_multiply(desired_quaternion,
                                                             tf.transformations.quaternion_inverse(self.quaternion))
        error = np.append(error_position, error_angle[0:3])
        SelectionVector = np.array([1, 2, 2, 2, 2, 2])
        error = error * SelectionVector
        # print "error"
        # print error
        # print "end error    "
        # measured_force is in tool frame and must be transformed to base frame
        # velocity = np.append(np.array(self.tool_velocity_linear), np.array([0, 0, 0]))
        # measured_force = self.reference_frame('tool', self.getWrench())
        empty_array = np.array([0, 0, 0])
        force_array = np.array(self.force_sensor)
        # force_array = np.array([0, 0, 0])
        kp = 2
        kd = 1
        # In the current steup, all the tool positions for the three exercises are
        # oriented in the pose, hence the traformation matrix in all cases are same
        if (self.exerciseType == ExerciseType.Horizontal):
            force_array = self.rotateToolFrame(axis='x', angle=90, measuredVec=force_array)
        elif (self.exerciseType == ExerciseType.Vertical):
            force_array = self.rotateToolFrame(axis='x', angle=90, measuredVec=force_array)
        elif (self.exerciseType == ExerciseType.BicepCurl):
            force_array = self.rotateToolFrame(axis='x', angle=90, measuredVec=force_array)

        tmp = np.asarray(force_array).reshape(-1)
        measured = np.append(tmp, empty_array)

        val = 0.3
        corr = 0.48

        if (abs(measured[2]) < 55000 and abs(error[0]) < 0.03 and abs(error[1]) < 0.03):
            measured[1] = 0
            measured[0] = 0
            measured[2] = 0
            self.isHome = True
            rospy.logwarn("HOME POSITION")

        if (self.exerciseType == ExerciseType.Horizontal):
            measured[1] = measured[1] * 0.5
            measured[2] = measured[2] * 2
            measured[0] = measured[0] * 0.1
            c=160
            control_law = (k *error)/ c + (force_scaling *measured)/ c
            return control_law
        elif (self.exerciseType == ExerciseType.Vertical):
            measured[1] = measured[1] * 0.5
            measured[0] = measured[0] * 0.1
            measured[2] = measured[2] * 2
            c = 160
            control_law = (k *error)/ c + (force_scaling *measured)/ c
            return control_law
        elif (self.exerciseType == ExerciseType.BicepCurl):
            measured[1] = measured[1] * 0.5
            measured[2] = measured[2] * 2
            measured[0] = measured[0] * 0.1
            c = 160
            control_law = val * error + (force_scaling / c) * measured
            return control_law

    def jac_function(self, q1, q2, q3, q4, q5, q6):
        d1 = 0.1273
        a2 = -0.612
        a3 = -0.5723
        d4 = 0.1639
        d5 = 0.0157
        d6 = 0.0922

        Jac = matrix([[(((math.cos(q4) * math.sin(q5) * d6 - math.sin(q4) * d5 - a3) * math.cos(q3) + math.sin(q3) * (
                    -math.sin(q4) * math.sin(q5) * d6 - math.cos(q4) * d5) - a2) * math.cos(q2) - math.sin(q2) * (
                                    (math.sin(q4) * math.sin(q5) * d6 + math.cos(q4) * d5) * math.cos(q3) + math.sin(
                                q3) * (math.cos(q4) * math.sin(q5) * d6 - math.sin(q4) * d5 - a3))) * math.sin(
            q1) + math.cos(q1) * (math.cos(q5) * d6 + d4), math.cos(q1) * (((math.cos(q4) * math.sin(
            q5) * d6 - math.sin(q4) * d5 - a3) * math.cos(q3) + math.sin(q3) * (-math.sin(q4) * math.sin(
            q5) * d6 - math.cos(q4) * d5) - a2) * math.sin(q2) + ((math.sin(q4) * math.sin(q5) * d6 + math.cos(
            q4) * d5) * math.cos(q3) + math.sin(q3) * (math.cos(q4) * math.sin(q5) * d6 - math.sin(
            q4) * d5 - a3)) * math.cos(q2)), (((math.sin(q4) * math.sin(q5) * d6 + math.cos(q4) * d5) * math.cos(
            q3) + math.sin(q3) * (math.cos(q4) * math.sin(q5) * d6 - math.sin(q4) * d5 - a3)) * math.cos(q2) + math.sin(
            q2) * ((math.cos(q4) * math.sin(q5) * d6 - math.sin(q4) * d5 - a3) * math.cos(q3) - math.sin(q3) * (
                    math.sin(q4) * math.sin(q5) * d6 + math.cos(q4) * d5))) * math.cos(q1), math.cos(q1) * (((math.sin(
            q4) * math.sin(q5) * d6 + math.cos(q4) * d5) * math.cos(q3) + math.sin(q3) * (math.cos(q4) * math.sin(
            q5) * d6 - math.sin(q4) * d5)) * math.cos(q2) + math.sin(q2) * ((math.cos(q4) * math.sin(
            q5) * d6 - math.sin(q4) * d5) * math.cos(q3) - math.sin(q3) * (math.sin(q4) * math.sin(q5) * d6 + math.cos(
            q4) * d5))), -(((math.cos(q3) * math.cos(q4) - math.sin(q3) * math.sin(q4)) * math.cos(q2) - math.sin(
            q2) * (math.cos(q3) * math.sin(q4) + math.sin(q3) * math.cos(q4))) * math.cos(q5) * math.cos(q1) + math.sin(
            q1) * math.sin(q5)) * d6, 0], [(((-math.cos(q4) * math.sin(q5) * d6 + math.sin(q4) * d5 + a3) * math.cos(
            q3) + math.sin(q3) * (math.sin(q4) * math.sin(q5) * d6 + math.cos(q4) * d5) + a2) * math.cos(q2) + math.sin(
            q2) * ((math.sin(q4) * math.sin(q5) * d6 + math.cos(q4) * d5) * math.cos(q3) + math.sin(q3) * (
                    math.cos(q4) * math.sin(q5) * d6 - math.sin(q4) * d5 - a3))) * math.cos(q1) + math.sin(q1) * (
                                                       math.cos(q5) * d6 + d4), math.sin(q1) * (((math.cos(
            q4) * math.sin(q5) * d6 - math.sin(q4) * d5 - a3) * math.cos(q3) + math.sin(q3) * (-math.sin(q4) * math.sin(
            q5) * d6 - math.cos(q4) * d5) - a2) * math.sin(q2) + ((math.sin(q4) * math.sin(q5) * d6 + math.cos(
            q4) * d5) * math.cos(q3) + math.sin(q3) * (math.cos(q4) * math.sin(q5) * d6 - math.sin(
            q4) * d5 - a3)) * math.cos(q2)), math.sin(q1) * (((math.sin(q4) * math.sin(q5) * d6 + math.cos(
            q4) * d5) * math.cos(q3) + math.sin(q3) * (math.cos(q4) * math.sin(q5) * d6 - math.sin(
            q4) * d5 - a3)) * math.cos(q2) + math.sin(q2) * ((math.cos(q4) * math.sin(q5) * d6 - math.sin(
            q4) * d5 - a3) * math.cos(q3) - math.sin(q3) * (math.sin(q4) * math.sin(q5) * d6 + math.cos(q4) * d5))), (((math.sin(q4) * math.sin(
                                                                                                                               q5) * d6 + math.cos(
                                                                                                                               q4) * d5) * math.cos(
            q3) + math.sin(q3) * (math.cos(q4) * math.sin(q5) * d6 - math.sin(q4) * d5)) * math.cos(q2) + math.sin(
            q2) * ((math.cos(q4) * math.sin(q5) * d6 - math.sin(q4) * d5) * math.cos(q3) - math.sin(q3) * (
                    math.sin(q4) * math.sin(q5) * d6 + math.cos(q4) * d5))) * math.sin(q1), -d6 * (math.sin(q1) * (
                    (math.cos(q3) * math.cos(q4) - math.sin(q3) * math.sin(q4)) * math.cos(q2) - math.sin(q2) * (
                        math.cos(q3) * math.sin(q4) + math.sin(q3) * math.cos(q4))) * math.cos(q5) - math.cos(
            q1) * math.sin(q5)), 0], [0, (
                    (-math.cos(q4) * math.sin(q5) * d6 + math.sin(q4) * d5 + a3) * math.cos(q3) + math.sin(q3) * (
                        math.sin(q4) * math.sin(q5) * d6 + math.cos(q4) * d5) + a2) * math.cos(q2) + math.sin(q2) * (
                                                  (math.sin(q4) * math.sin(q5) * d6 + math.cos(q4) * d5) * math.cos(
                                              q3) + math.sin(q3) * (math.cos(q4) * math.sin(q5) * d6 - math.sin(
                                              q4) * d5 - a3)), ((-math.cos(q4) * math.sin(q5) * d6 + math.sin(
            q4) * d5 + a3) * math.cos(q3) + math.sin(q3) * (math.sin(q4) * math.sin(q5) * d6 + math.cos(
            q4) * d5)) * math.cos(q2) + math.sin(q2) * (
                                                  (math.sin(q4) * math.sin(q5) * d6 + math.cos(q4) * d5) * math.cos(
                                              q3) + math.sin(q3) * (math.cos(q4) * math.sin(q5) * d6 - math.sin(
                                              q4) * d5 - a3)), (math.cos(q3) * (
                    -math.cos(q4) * math.sin(q5) * d6 + math.sin(q4) * d5) + math.sin(q3) * (
                                                                            math.sin(q4) * math.sin(q5) * d6 + math.cos(
                                                                        q4) * d5)) * math.cos(q2) + (
                                                  (math.sin(q4) * math.sin(q5) * d6 + math.cos(q4) * d5) * math.cos(
                                              q3) + math.sin(q3) * (math.cos(q4) * math.sin(q5) * d6 - math.sin(
                                              q4) * d5)) * math.sin(q2), ((-math.sin(q3) * math.cos(q4) - math.cos(
            q3) * math.sin(q4)) * math.cos(q2) + math.sin(q2) * (math.sin(q3) * math.sin(q4) - math.cos(q3) * math.cos(
            q4))) * d6 * math.cos(q5), 0], [0, math.sin(q1), math.sin(q1), math.sin(q1), -math.cos(q1) * (
                    math.sin(q4) * (math.sin(q2) * math.sin(q3) - math.cos(q2) * math.cos(q3)) - math.cos(q4) * (
                        math.sin(q3) * math.cos(q2) + math.sin(q2) * math.cos(q3))), (math.sin(q2) * (
                    math.cos(q3) * math.sin(q4) + math.sin(q3) * math.cos(q4)) + math.cos(q2) * (math.sin(
            q3) * math.sin(q4) - math.cos(q3) * math.cos(q4))) * math.sin(q5) * math.cos(q1) + math.sin(q1) * math.cos(
            q5)], [0, -math.cos(q1), -math.cos(q1), -math.cos(q1), -math.sin(q1) * (
                    math.sin(q4) * (math.sin(q2) * math.sin(q3) - math.cos(q2) * math.cos(q3)) - math.cos(q4) * (
                        math.sin(q3) * math.cos(q2) + math.sin(q2) * math.cos(q3))), (
                               math.sin(q2) * (math.cos(q3) * math.sin(q4) + math.sin(q3) * math.cos(q4)) + math.cos(
                           q2) * (math.sin(q3) * math.sin(q4) - math.cos(q3) * math.cos(q4))) * math.sin(q5) * math.sin(
            q1) - math.cos(q1) * math.cos(q5)], [1, 0, 0, 0, math.sin(q4) * (
                    math.sin(q3) * math.cos(q2) + math.sin(q2) * math.cos(q3)) - math.cos(q4) * (
                                                             math.cos(q2) * math.cos(q3) - math.sin(q2) * math.sin(q3)),
                                                 (math.sin(q4) * (math.sin(q2) * math.sin(q3) - math.cos(q2) * math.cos(
                                                     q3)) - math.cos(q4) * (
                                                              math.sin(q3) * math.cos(q2) + math.sin(q2) * math.cos(
                                                          q3))) * math.sin(q5)]])

        if(self.isPsudoInverse):
            Jac_psudo = np.matmul(np.transpose(Jac), np.linalg.inv(
                np.matmul(Jac, np.transpose(Jac)) + np.multiply(self.lamda, np.identity(6))))
        else:
            Jac_psudo = np.linalg.pinv(Jac)

        return Jac_psudo

    def findPosition(self):
        roll = 1.483
        pitch = 0.006
        yaw = 0.181

        rospy.logwarn("ROLL %f", roll)
        rospy.logwarn("PITCH %f", pitch)
        rospy.logwarn("YAW %f", yaw)

        newQut = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        rospy.loginfo(newQut)

class ExerciseType(enum.Enum):
    Horizontal = 1
    Vertical = 2
    BicepCurl = 3

if __name__ == '__main__':
    Robot()
