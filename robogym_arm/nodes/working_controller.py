#!/usr/bin/env python2.7
import roslib
import rospy 
import numpy as np 
import time
import tf2_ros as tf2
import tf
import select
import math
import sys, termios, tty, os, time


import Tkinter as tk
import threading
#import matplotlib.pyplot as plt
#import matplotlib.figure
#import matplotlib.animation as animation

from math import cos, sin
from numpy.linalg import inv
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Wrench
from Tkinter import * 
from threading import *
from PIL import ImageTk, Image
from numpy import matrix
import imp
import re


sys.path.append('/usr/local/lib/python3.5/dist-packages/atracsys-4.4.1.6.dev2+g58fe057-py3.5-linux-x86_64.egg/atracsys/ftk/')

#print(sys.path)
#import atracsys.ftk as tracker_sdk

root = Tk()
#popup = Toplevel()

offset = 0
resistance = 50
str_res = StringVar()
atHome = True
run_robot = True
force_plot = np.zeros(10)
force_now = 0
guide_z = False
forceVar = StringVar() 
red  = 30
green = 35
blue = 191


pointsCam = np.array([[0,0,0, 0]])
pointsRobo = np.array([[0,0,0]])
rotGen = np.zeros((3,3))
trajCount = 0


onoff_light = StringVar()
onoff_light.set("red")

#at_home_light = StringVar()
#at_home_light.set("red")

start_string = """
Unlocks the robot. The robot is now movable.
"""
set_pos_string = """
Press the button "Set height", then move the robot vertically 
to find your prefered position and press "Done". This is now 
the new start position. The robot needs to be unlocked and in 
start position to perform this.
"""
set_res_string = """
Move the slidebar to a preferred value and then press "Set 
resistance" to select resistance. The robot needs to be in 
start position to perform this.   
"""
stop_string = """
Locks the robot, no motion is possible. The robot needs to be 
in start position to perform this. 
"""
s_down_string = """
Terminates the whole program. The interface will close and the 
robot will no longer receive any force input.
"""

exercise_string = """
Set yourself in a stable position with your feet facing towards the robot. 
The vertical height of the handle should be between your shoulder and elbow. 
Pull the handle with one hand in a controlled rowing motion back and forth
and do not release the handle until you are finished with the exercise. 
This exercise will mainly stimulate the back, biceps and core musculature. 
"""
        
class Interface(threading.Thread):
    def __init__(self, tk_root):
        self.root = tk_root
        #self.popup = popup
        threading.Thread.__init__(self)
        self.start()

        self.root.title("RobotGym")
        frame=Frame(self.root)
        Grid.rowconfigure(self.root, 0, weight=1)
        Grid.columnconfigure(self.root, 0, weight=1)
        frame.grid(row=0, column=0, sticky=N+S+E+W)
        grid=Frame(frame)
        grid.grid(sticky=N+S+E+W, column=0, row=0, columnspan=2)
        Grid.rowconfigure(frame, 0, weight=1)
        Grid.columnconfigure(frame, 0, weight=1)
        
        for x in range(80):
            Grid.columnconfigure(frame, x, weight=1)

        for y in range(40):
            Grid.rowconfigure(frame, y, weight=1)


        #Unlock button
        self.start_button = Button(frame, text="Unlock",command=self.starter)
        self.start_button.grid(row=10, column=30,padx = 2, pady = 2, sticky=N+S+E+W)        

        #Z-Position button
        self.set_position_button = Button(frame, text="Set height", command = self.poition_set)
        self.set_position_button.grid(row=12,column = 30, padx = 2, pady = 2, sticky= N+S+E+W)

        #Resistance button
        self.get_weight = Button(frame, text = "Set resistance", command = self.getWeight)
        self.get_weight.grid(row=14,column = 30, padx = 2, pady = 2, sticky= N+S+E+W)

        #Lock button
        self.stop_button = Button(frame, text="Lock",command=self.stop)
        self.stop_button.grid(row=16, column=30,padx = 2, pady = 2, sticky=N+S+E+W)

        #Unit label
        self.slide_label = Label(frame, text= "Resistance (N/m)", font=("Helvetica", 12))
        self.slide_label.grid(row=39, column = 30, sticky = E)   

        #Slidebar
        self.slideBar = Scale(frame, from_=125, to=25, length=400, width= 50, tickinterval=25)
        self.slideBar.grid(row = 30, rowspan = 9, column = 30, sticky = W)
        self.slideBar.set(50)     
        
        #Current resistance label
        self.current_resistans = Label(frame, textvariable = str_res, font=("Helvetica",12))
        self.current_resistans.grid(row = 26, rowspan=4, column = 30)

        #Text widget explaning interface
        text_button = Text(frame, height=23, width=64)
        text_button.tag_configure('bold_italics', font=('Arial', 12, 'bold', 'italic'))
        text_button.tag_configure('big', font=('Verdana', 12, 'bold'))
        text_button.tag_configure('color', foreground='#476042', font=('Tempus Sans ITC', 12, 'bold'))

        text_button.insert(END,'Unlock', 'big')
        text_button.insert(END, start_string)

        text_button.insert(END,'\nSet height', 'big')
        text_button.insert(END, set_pos_string)

        text_button.insert(END,'\nSet resistance', 'big')
        text_button.insert(END, set_res_string)

        text_button.insert(END,'\nLock', 'big')
        text_button.insert(END, stop_string)

        text_button.insert(END,'\nShutdown', 'big')
        text_button.insert(END, s_down_string)

        text_button.config(state=DISABLED)
        text_button.grid(row=10, rowspan=22, column=50, columnspan = 10, padx=6, pady=6,sticky=N)

        #Test widget explaning exercise
        text_exercise = Text(frame, height=10, width=80)
        text_exercise.tag_configure('bold_italics', font=('Arial', 12, 'bold', 'italic'))
        text_exercise.tag_configure('big', font=('Verdana', 12, 'bold'))
        text_exercise.tag_configure('color', foreground='#476042', font=('Tempus Sans ITC', 12, 'bold'))

        text_exercise.tag_bind('follow', '<1>', lambda e, t=text_exercise: t.insert(END, "Not now, maybe later!"))

        text_exercise.insert(END,exercise_string)   
        text_exercise.config(state = DISABLED)
        text_exercise.grid(row=10, rowspan=10, column=66, columnspan = 14,padx=6, pady = 6 ,sticky=N)
        
        #Exercise image
        text_image = Text(frame, height=25, width=80)   
	# "gym_instructions.png"
        org_img = Image.open("test.png")
        res_img = org_img.resize((555,320),Image.ANTIALIAS)

        self.photo = ImageTk.PhotoImage(res_img)

        text_image.insert(END,'\n')
        text_image.image_create(END, image=self.photo)
        text_image.config(state = DISABLED)
        text_image.grid(row=18,rowspan=25, column=66,columnspan =18,sticky = N)
        
        #Force display label
        self.force = Label(frame, textvariable = forceVar, font=("Helvetica", 100), fg = "#%02x%02x%02x" % (red, green,blue), borderwidth=3, relief = "ridge")
        self.force.config(height=1, width=5)
        self.force.grid(row = 33, rowspan = 4, column=52, columnspan = 4, pady=4 , padx=4)

        #Force display unit
        self.slide_label = Label(frame, text= "Force (N)", font=("Helvetica", 12))
        self.slide_label.grid(row=32, column = 53, sticky = S+E)

        #Shutdown button
        self.end_button = Button(frame, text="Shutdown", command=self.end)
        self.end_button.grid(row=39, column=79, padx = 8, pady = 2, sticky=N+S+E+W)

        #ON/Off label 
        self.onoff = Label(frame, bg=onoff_light.get())
        self.onoff.config(height = 3, width=5)
        self.onoff.grid(row=18, column=30,pady = 2,sticky = E)

        self.onoff_status = Label(frame, text = "Robot active:")
        self.onoff.config(height = 3, width=5)
        self.onoff_status.grid(row = 18, column = 30, pady = 2, sticky = W)

        '''
        #at home label 
        self.at_home = Label(frame, bg=at_home_light.get())
        self.at_home.config(height = 3, width=5)
        self.at_home.grid(row=21, column=30,pady = 2, columnspan= 2, sticky = E)
        self.at_home_status = Label(frame, text = "Home position:")
        self.at_home_status.config(height = 3, width=5)
        self.at_home_status.grid(row = 21, column = 30, pady = 2, columnspan=2, sticky = W)
        '''

        global w,h
        #Till for samus dator
        #w,h = self.root.winfo_screenwidth(), self.root.winfo_screenwidth()
        w= 1366
        h= 1366
        
        #root.minsize(width=w/2, height=h/2)
        #root.maxsize(width=w, height=h)
        self.root.geometry( str(w) + 'x' + str(h))
        #self.root.geometry( '500x300')      
        
        
    def guide_z_false(self):
        global guide_z
        guide_z = False

    def guide_z_true(self):
        global guide_z
        guide_z = True

    def getWeight(self):
        if(atHome):
            global resistance
            resistance = self.slideBar.get()
        else:
            popup = Toplevel()
            #self.root.withdraw()
            popup.grab_set()
            popup.title("Error, robot must be in home position")
            popup.geometry(str(w/2) + 'x' + str(h/10))
            popup.geometry("+300+400")
            #popup.geometry("+d%")
            explanation = """Wait for the robot to return to its home position and try again."""
            popw2 = Label(popup, justify=LEFT, padx=200, pady=25, height=3,width=15, text=explanation).pack()
        
        def popup_done():
            popup.destroy()
            #self.root.deiconify()
            popup.grab_release()
        
        self.B1 = Button(popup, text="Done", command=popup_done).pack()
        
        
        
    
    def starter(self):
        global run_robot
        run_robot=True
        onoff_light.set("green")
        self.onoff.config(bg=onoff_light.get())

    def stop(self):
       
        if(atHome):
            global run_robot
            run_robot=False
            onoff_light.set("red")
            self.onoff.config(bg=onoff_light.get())
            
    def end(self):
        self.root.destroy()
        run_robot = False
        rospy.is_shutdown()

    def get(self):
        global offset 
        offset = float(z.get())
        #print(offset)
    
    def rgb_to_hex(rgb):
        return "#%02x%02x%02x" % rgb 

    def poition_set(self): 
        if(atHome):
            popup = Toplevel()
            popup.grab_set()
            global guide_z 
            guide_z = True
            #self.root.withdraw()
            
            popup.title("Set you start position")
            popup.geometry(str(w/2) + 'x' + str(h/10))
            popup.geometry("+300+400")
            explanation = """Move the robot vertically to find your prefered position"""
            popw2 = Label(popup, justify=LEFT, padx=200, pady=25, height=3,width=15, text=explanation).pack()
        else:
            popup = Toplevel()
            popup.grab_set()
            #self.root.withdraw()
            popup.title("Error, robot must be in home position")
            popup.geometry(str(w/2) + 'x' + str(h/10))
            popup.geometry("+300+400")
            explanation = """Wait for the robot to return to its home position and try again."""
            popw2 = Label(popup, justify=LEFT, padx = 200, pady=25, height=3,width=15, text=explanation).pack()
        
        def popup_done(): 
            global guide_z 
            guide_z = False
            popup.destroy()
            popup.grab_release()

   
        
        self.B1 = Button(popup, text="Done", command=popup_done).pack()

      

class Robot(threading.Thread): 

    def __init__(self, freq= 120):
        threading.Thread.__init__(self)
    
        rospy.init_node('Controller', anonymous=True)
        self.rate = rospy.Rate(freq) # Default frequency 125Hz
       
        #Robot States
        self.joints_order = []
        self.q = []
        self.dq = [] 
        self.position = []
        self.orientation = [] 
        self.tool_velocity_linear__ = []
        self.tool_velocity_angular = []
        self.force = []
        self.torque = []
        self.RotationM = [] 
        self.mode = 'Stop'

        #Sensor measurements
        self.force_sensor = []
        self.torque_sensor = []
        self.sensor_header = WrenchStamped().header
 
        #Publisher and Subscribers 
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
        #self.pub = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self.jointStateCallback)
        rospy.Subscriber("/tool_velocity", TwistStamped, self.toolVelocityCallback)
        rospy.Subscriber("/wrench", WrenchStamped, self.wrenchCallback)

        #Sensor publishers and subscribers
        #self.pub_reset = rospy.Publisher('/optoforce_node/reset', Bool, queue_size=1)
        #rospy.Subscriber("/optoforce_node/wrench_HEXHA094", WrenchStamped, self.wrenchSensorCallback)
        self.pub_reset = rospy.Publisher('/ethdaq_zero', Bool, queue_size=1)
        #Publishers for the position
        self.pub_x = rospy.Publisher('/positionx', Float32, queue_size=1)
        self.pub_y = rospy.Publisher('/positiony', Float32, queue_size=1)
        self.pub_z = rospy.Publisher('/positionz', Float32, queue_size=1)
        self.pub_xd = rospy.Publisher('/positionxd', Float32, queue_size=1)
        self.pub_yd = rospy.Publisher('/positionyd', Float32, queue_size=1)
        self.pub_zd = rospy.Publisher('/positionzd', Float32, queue_size=1)
        rospy.Subscriber("/ethdaq_data_raw", WrenchStamped, self.wrenchSensorCallback)
        rospy.Subscriber("/ethdaq_data", WrenchStamped, self.wrenchSensorCallback)
        rospy.Subscriber("/geom2", String,  self.cameraCallback2)
        rospy.Subscriber("/geom4", String,  self.cameraCallback4)
        rospy.Subscriber("/geom9", String,  self.cameraCallback9)
        self.pub_x = rospy.Publisher('/positionx', Float32, queue_size=1) 
        
        rospy.sleep(0.1) #time needed for initialization 

        self.force_sensor_reset()
        self.force_offset = self.getWrenchNoOffset()
        #self.spin()


#######################################
###### USER DEFINED CONTROLLERS #######
######  WITH VELOCITY COMMAND   #######
#######################################

    ##### Impedance Control ####
    # Force control of the robot modelled as a spring-damper system
    # Kp - Spring constant, values between 0.5 and 4 works fine
    # force_scaling is used to scale the forces to appropiate magnitude, 0.000001
    # desired_pose is a two dimensional numpy-array consisting of the position coordinates wrt the base 
    # and the rotation in terms of quaternion
    def guide_z_control(self, force_scaling = 0.000001):
        self.mode = 'guide z controller'
        measured_force = self.reference_frame('tool', self.getWrench())
        selectionVector = np.array([0,0,1,0,0,0])
        control_law = measured_force*force_scaling * selectionVector
        return control_law

    def calibration(self, pointsCamLista, pointsRoboLista, desired_pose):
        global pointsCam
        global pointsRobo
        actual_position = self.getTaskPosi()
        desired_position_base = desired_pose[0]
        desired_quaternion = desired_pose[1]
        Tmatrix2 = self.cameraData2
        Tmatrix4 = self.cameraData4
        #Mmat = np.array()
        switchonoff = True
        desired_pos_camera = Tmatrix4[0:3,3:4].flatten()/1000
        actual_pos_camera = Tmatrix2[0:4,3:4].flatten()/1000
        
        global rotGen
        actual_pos_camera[len(actual_pos_camera) - 1] = actual_pos_camera[len(actual_pos_camera)-1]*1000
        actual_pos_camera_tmp = np.copy(actual_pos_camera)
        for i in range(len(actual_pos_camera)):
            actual_pos_camera_tmp[i] = round(actual_pos_camera[i], 2)
        



        if(len(pointsCam) > 400):
            switchonoff = False
            pointsCamReal = np.delete(pointsCam,[0,0,0,0],0)
            pointsRoboReal = np.delete(pointsRobo,[0,0,0],0)
            
            dCam = np.copy(pointsCamReal)
            dRobo = np.copy(pointsRoboReal)
            meanCam = np.mean(pointsCamReal,axis=0)
            meanRobo = np.mean(pointsRoboReal,axis=0)
            for i in range(len(pointsCamReal)):
                for j in range(len(pointsCamReal[0])-1):
                    dCam[i][j] = pointsCamReal[i][j] - meanCam[j] 
                    dRobo[i][j] = pointsRoboReal[i][j] - meanRobo[j]
                
            s = 12
            l = 0
            b = 0
            Mmat = np.zeros((len(pointsCamReal)*3, len(pointsCamReal) + 12))
            
            for i in range(len(pointsCamReal)):
                for j in range(len(pointsCamReal[0])):     
                    Mmat[b][j] = dCam[i][j]
                    Mmat[b+1][j+4] = dCam[i][j]
                    Mmat[b+2][j+8] = dCam[i][j]   
                b = b+3

            
            for i in range(len(pointsCamReal)):
                for v in range(len(pointsRoboReal[0])):
                    Mmat[l+v][s] = dRobo[i][v]
                l=l+3
                s=s+1
             
            U1, S1, V1 = np.linalg.svd(Mmat)
            V1_end = V1[len(V1)-1][0:12]
            Pmat = np.reshape(V1_end,(3,4))
            rotGen = Pmat[0:3,0:3]
            
        #print(round(pointsCam[len(pointsCam) - 1][2], 3))
        #print(actual_pos_camera)
        if(switchonoff == True):
            if(actual_pos_camera_tmp[0] != round(pointsCam[len(pointsCam) - 1][0],2) or actual_pos_camera_tmp[1] != round(pointsCam[len(pointsCam) - 1][1],2) or actual_pos_camera_tmp[2] != round(pointsCam[len(pointsCam) - 1][2],2)):
                pointsCam = np.vstack([pointsCam, actual_pos_camera])
                pointsRobo = np.vstack([pointsRobo, actual_position])

        
            
        

        print("cam", len(pointsCam))
        rotGen = rotGen*-1
        return rotGen

    def impedance_control(self, desired_pose, rotGen):
        k = 3
        global pointsCam
        global pointsRobo
        self.mode = 'impedance controller'
        quaternion = self.getTaskQuaternion()
        actual_position = self.getTaskPosi()
        desired_position_base = desired_pose[0]
        desired_quaternion = desired_pose[1]
        
        
        Tmatrix2 = self.cameraData2
        Tmatrix4 = self.cameraData4



        desired_pos_camera = Tmatrix4[0:3,3:4].flatten()/1000
        actual_pos_camera = Tmatrix2[0:3,3:4].flatten()/1000

        
        err = (desired_pos_camera  - actual_pos_camera)
        #error_pos_camera = np.matmul(rotBC, np.matmul(rotY , err))
        error_pos_camera = np.matmul(rotGen , err)

        #error_position = desired_position_base - actual_position
        # multiplication of two quaternions gives the rotations of doing the two rotations consecutive, 
        # could be seen as "adding" two rotations
        # multiplying the desired quaternion with the inverse of the current quaternion gives the error, 
        # could be seen as the "difference" between them
        #sameCoord = np.matmul(rotBC4x4,Tmatrix4)

        quaternions_goal = tf.transformations.quaternion_from_matrix(Tmatrix4)
        quaternions_endeffector = tf.transformations.quaternion_from_matrix(Tmatrix2)

        #error_angle = tf.transformations.quaternion_multiply(desired_quaternion, tf.transformations.quaternion_inverse(quaternion))
        error_angle = tf.transformations.quaternion_multiply(quaternions_goal, tf.transformations.quaternion_inverse(quaternions_endeffector))


        error_ang = np.matmul(rotGen , error_angle[0:3])
        error = np.append(error_pos_camera, error_ang)

    
        #error = np.append(error_position, error_angle[0:3]) #Why can we use only error_angle[0:3]?
        #print(error)
       
        #print(error)
        #control_law = kp*error + measured_force - kd*velocity
        #control_law = kp*error - kd*velocity
        #control_law = kp*error
        #control_law = k/c*error
	#print measured_force
        control_law = k*error 
        #control_law = control_law *testSelectionVector
        
        
        return [control_law, Tmatrix2, Tmatrix4]
        



    def trajGenPoints(self, desired_pose, count):
        c = 1
        k = 0.5
        x = 0
        y = 1
        z = 2
        threshhold = 0.1
        global trajCount
        

        self.mode = 'impedance controller'
        quaternion = self.getTaskQuaternion()
        actual_position = self.getTaskPosi()
        positions = np.array([[0.82486, -0.51018, 0.21066], [0.82738, -0.54933, 0.65417], [0.94009, 0.29343, 0.66002], [1.02991, 0.23619, 0.25965], [ 0.62244, 0.63766, 0.33269], [0.59919, 0.70732, 0.76191], [-0.31424, 0.83937, 0.7834], [-0.34466, 0.87406, 0.26547], [-0.8256, 0.45144, 0.29477], [-0.87887, 0.41303, 0.78414], [-0.95871, -0.22587, 0.75526], [-0.86459, -0.41394, 0.33856], [-0.76049, -0.74522, 0.50626], [-0.53952, -0.83571, 0.22054], [-0.5795, -0.7196, 0.64038], [0.12494, -0.92563, 0.82685], [0.13152, -0.47089, 0.28201], [-0.23391, -0.43449, 0.52768], [-0.38107, 0.31347, 0.38621], [0.33293, 0.3641, 0.38627], [0.35508, -0.34246, 0.38626]])


        quats = np.array([[-0.38292308,  0.45933626, -0.59887538,  0.53266161], [-0.40914157,  0.51389096, -0.54640928,  0.51957305],  [-0.4826565,   0.41776057, -0.39745305,  0.65920397], [-0.49307207,  0.50383661, -0.44756228,  0.55019688],  [-0.6718397,   0.10352994, -0.05911462,  0.73103928],  [-0.68133928,  0.09291008, -0.03919293,  0.72498856], [-0.63853426, -0.05921477,  0.20528296,  0.73934195], [-0.72144138, -0.10371669,  0.15037521,  0.66794647], [-0.53357887, -0.34190591,  0.49526981,  0.59422366], [-0.55541995, -0.4360819,   0.49133063,  0.50983867], [-0.46665292, -0.51137324,  0.56836963,  0.44462167], [-0.33972754, -0.59958032,  0.65153409,  0.31716237], [-0.18740591, -0.67400559,  0.69192061,  0.17844149],  [-0.10934784, -0.70206126,  0.68978216,  0.13911727], [-0.14582507, -0.61320037,  0.77455579,  0.05276061],[0.5681533,  -0.40237072,  0.33911757,  0.6326918 ], [-0.67774699,  0.01613773,  0.48899118,  0.54889545], [-0.00365793,  0.67665133,  0.7345087,   0.05124996], [0.32931651, 0.62052238, 0.40866113, 0.58266516], [ 0.67676412,  0.18838271, -0.14129164,  0.69752344], [ 0.60708547, -0.35347507, -0.5982922,   0.38542062]])
        
        

        desired_position_base = positions[count]
        desired_quaternion = quats[count]
        error_position = desired_position_base - actual_position
        # multiplication of two quaternions gives the rotations of doing the two rotations consecutive, 
        # could be seen as "adding" two rotations
        # multiplying the desired quaternion with the inverse of the current quaternion gives the error, 
        # could be seen as the "difference" between them

        error_angle = tf.transformations.quaternion_multiply(desired_quaternion, tf.transformations.quaternion_inverse(quaternion))

        
        error = np.append(error_position, error_angle[0:3]) #Why can we use only error_angle[0:3]?

        #print(error)
        if(trajCount > 15):
            x = 3
            y = 4
            z = 5
        if(abs(error[x]) < threshhold and abs(error[y]) < threshhold and abs(error[z]) < threshhold):
            trajCount = trajCount + 1
        if(trajCount == len(positions)-1):
            trajCount = 0
        #control_law = kp*error + measured_force - kd*velocity
        #control_law = kp*error - kd*velocity
        #control_law = kp*error
        #control_law = k/c*error
	#print measured_force
        control_law = k*error 
        #control_law = control_law *testSelectionVector
        
    
        return control_law
    # Uses only the position to control and does not involve forces. 
    # The robot returns to a fixed pose and can't be moved away by applying forces
    def pose_control(self, desired_pose, kp = 5): 
        self.mode = 'position controller'
        quaternion = self.getTaskQuaternion()
        actual_position = self.getTaskPosi()
        desired_position_base = desired_pose[0]
        desired_quaternion = desired_pose[1]

        error_position = desired_position_base - actual_position
        error_angle =  tf.transformations.quaternion_multiply(desired_quaternion, tf.transformations.quaternion_inverse(quaternion))
        error = np.append(error_position, error_angle[0:3])

        # V = kp * A(phi_e) * (xd - x)
        control_law = kp*error
        return control_law


    def velocity_control(self, frame,  velocity_d): 
        self.mode = 'velocity controller'

        # V = Vd 
        control_law = velocity_d 
        control_law = self.reference_frame(frame,  control_law)
        #rospy.loginfo(control_law)
        return control_law

     
    def force_control(self, frame, selection_vector,  desired_force, kd_inv= 0.001): # Kd = 1000 by default 
        self.mode = 'force controller'
        measured_force  = self.getWrench() # measured force is in the base frame (not true anymore) 

        # V = Kd^{-1} * (fd - f) =>  Kd V  = f  - fd 
        control_law = kd_inv * (desired_force - measured_force )  
        control_law = selection_vector * control_law
        control_law = self.reference_frame(frame,  control_law)
        return control_law


    def compliant_control(self, frame, desired_pose , desire_velocity = [] ,kd_inv = 1,  kp = 0.001):
        self.mode = 'compliant controller'
        measured_force  = self.refrence_frame('tool',self.getWrench())
        
        position_term = self.pose_control(desired_pose) 

        # V =  Vd + Kd^{-1} ( A(phi_e) * Kp (Xd - X) + f) =>  Kd (V - Vd) +  A(phi_e)* Kp (X - Xd) = f   
        control_law = desire_velocity + kd_inv * ( position_term + (  measured_force ) ) #force  is measured wrt the base frame(not true anymore) 
        return self.reference_frame(frame,control_law)
 

    def force_sensor_reset(self):      
        self.pub_reset.publish(True)




#######################################
############ JACOBIAN #################
#######################################

    def jac_function(self):   
	q1 = self.q[0]
	q2 = self.q[1]
	q3 = self.q[2]
	q4 = self.q[3]
	q5 = self.q[4]
	q6 = self.q[5]

	#print q6

	#print self.q
	#print q4
	#print 
	"""
        q1 = -1.96
        q2 = -1.57
        q3 = -0.75
        q4 = -0.741
        q5 = -1.54
        q6 = -0.005
	"""

	d1 =  0.1273
        a2 = -0.612
        a3 = -0.5723
        d4 =  0.1639
        d5 =  0.0157
        d6 =  0.0922
   
        Jac = matrix( [[(((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)*math.cos(q3)+math.sin(q3)*(-math.sin(q4)*math.sin(q5)*d6-math.cos(q4)*d5)-a2)*math.cos(q2)-math.sin(q2)*((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)))*math.sin(q1)+math.cos(q1)*(math.cos(q5)*d6+d4), math.cos(q1)*(((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)*math.cos(q3)+math.sin(q3)*(-math.sin(q4)*math.sin(q5)*d6-math.cos(q4)*d5)-a2)*math.sin(q2)+((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3))*math.cos(q2)), (((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3))*math.cos(q2)+math.sin(q2)*((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)*math.cos(q3)-math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)))*math.cos(q1), math.cos(q1)*(((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5))*math.cos(q2)+math.sin(q2)*((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5)*math.cos(q3)-math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5))), -(((math.cos(q3)*math.cos(q4)-math.sin(q3)*math.sin(q4))*math.cos(q2)-math.sin(q2)*(math.cos(q3)*math.sin(q4)+math.sin(q3)*math.cos(q4)))*math.cos(q5)*math.cos(q1)+math.sin(q1)*math.sin(q5))*d6, 0],[(((-math.cos(q4)*math.sin(q5)*d6+math.sin(q4)*d5+a3)*math.cos(q3)+math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)+a2)*math.cos(q2)+math.sin(q2)*((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)))*math.cos(q1)+math.sin(q1)*(math.cos(q5)*d6+d4), math.sin(q1)*(((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)*math.cos(q3)+math.sin(q3)*(-math.sin(q4)*math.sin(q5)*d6-math.cos(q4)*d5)-a2)*math.sin(q2)+((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3))*math.cos(q2)), math.sin(q1)*(((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3))*math.cos(q2)+math.sin(q2)*((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)*math.cos(q3)-math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5))), (((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5))*math.cos(q2)+math.sin(q2)*((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5)*math.cos(q3)-math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)))*math.sin(q1), -d6*(math.sin(q1)*((math.cos(q3)*math.cos(q4)-math.sin(q3)*math.sin(q4))*math.cos(q2)-math.sin(q2)*(math.cos(q3)*math.sin(q4)+math.sin(q3)*math.cos(q4)))*math.cos(q5)-math.cos(q1)*math.sin(q5)), 0],[0, ((-math.cos(q4)*math.sin(q5)*d6+math.sin(q4)*d5+a3)*math.cos(q3)+math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)+a2)*math.cos(q2)+math.sin(q2)*((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)), ((-math.cos(q4)*math.sin(q5)*d6+math.sin(q4)*d5+a3)*math.cos(q3)+math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5))*math.cos(q2)+math.sin(q2)*((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)), (math.cos(q3)*(-math.cos(q4)*math.sin(q5)*d6+math.sin(q4)*d5)+math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5))*math.cos(q2)+((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5))*math.sin(q2), ((-math.sin(q3)*math.cos(q4)-math.cos(q3)*math.sin(q4))*math.cos(q2)+math.sin(q2)*(math.sin(q3)*math.sin(q4)-math.cos(q3)*math.cos(q4)))*d6*math.cos(q5), 0],[0, math.sin(q1), math.sin(q1), math.sin(q1), -math.cos(q1)*(math.sin(q4)*(math.sin(q2)*math.sin(q3)-math.cos(q2)*math.cos(q3))-math.cos(q4)*(math.sin(q3)*math.cos(q2)+math.sin(q2)*math.cos(q3))), (math.sin(q2)*(math.cos(q3)*math.sin(q4)+math.sin(q3)*math.cos(q4))+math.cos(q2)*(math.sin(q3)*math.sin(q4)-math.cos(q3)*math.cos(q4)))*math.sin(q5)*math.cos(q1)+math.sin(q1)*math.cos(q5)],[0, -math.cos(q1), -math.cos(q1), -math.cos(q1), -math.sin(q1)*(math.sin(q4)*(math.sin(q2)*math.sin(q3)-math.cos(q2)*math.cos(q3))-math.cos(q4)*(math.sin(q3)*math.cos(q2)+math.sin(q2)*math.cos(q3))), (math.sin(q2)*(math.cos(q3)*math.sin(q4)+math.sin(q3)*math.cos(q4))+math.cos(q2)*(math.sin(q3)*math.sin(q4)-math.cos(q3)*math.cos(q4)))*math.sin(q5)*math.sin(q1)-math.cos(q1)*math.cos(q5)],[1, 0, 0, 0, math.sin(q4)*(math.sin(q3)*math.cos(q2)+math.sin(q2)*math.cos(q3))-math.cos(q4)*(math.cos(q2)*math.cos(q3)-math.sin(q2)*math.sin(q3)), (math.sin(q4)*(math.sin(q2)*math.sin(q3)-math.cos(q2)*math.cos(q3))-math.cos(q4)*(math.sin(q3)*math.cos(q2)+math.sin(q2)*math.cos(q3)))*math.sin(q5)]])

        Jac_psudo = np.linalg.pinv(Jac)
        return Jac_psudo

#######################################
############ NEW FUNCTION #############
#######################################

    def findPosition(self):
	roll = 0.1158
        pitch = 0.266
        yaw = -2.296

        rospy.logwarn("ROLL %f",roll)
        rospy.logwarn("PITCH %f", pitch)
        rospy.logwarn("YAW %f", yaw)

        newQut = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        rospy.loginfo(newQut)
	global V_ref
	V_ref = matrix([[0],[0],[0],[0],[0],[0]])
	#return newQut

    def goalPosition(self, position, quaternion):
        ############ 1st point ###################
	position = np.array([0.5, -1.05908, 0.76039])
        quaternion = np.array([ 0.05793482,  0.64728254, -0.64398721,  0.40366984])
        
	return position, quaternion
#######################################
############ UR FUNCTIONS #############
#######################################

    def stop(self):
        self.mode = 'stop'
        command1 = "stopj(1) \n"
        command2 = "stopl(1) \n"
        self.pub.publish(command1)
        self.pub.publish(command2)


    def command_mode(self, mode):
        command = "def command_mode():\n\n\t" + mode + "\n\twhile (True):\n\t\tsync()\n\tend\nend\n"
        #rospy.loginfo(command)
        self.pub.publish(command)
    

    def end_force_mode(self):
        self.mode = 'Stop'
        command = "end_force_mode()"
        self.pub.publish(command)

    
    def end_free_drive_mode(self):
        self.mode = 'Stop'
        command = "end_freedrive_mode()"
        self.pub.publish(command)


    def force_mode(self, task_frame, selection_vector, wrench, type_, limits ):
        self.mode = 'force mode'
        command = "force_mode("+ task_frame + ","+  selection_vector + "," + wrench + "," \
        + type_ + "," + limits + ")" 
        self.command_mode(command)


    def free_drive_mode(self):
        self.mode = 'free drive mode'
        command = "freedrive_mode()"
        self.command_mode(command)

    # Used to command linear speed command to the robot
    # control_law is a 6x1 vector with tool speed in x,y and z-direction and rotational speed around x-y and z
    def velocity_cmd(self, control_law, acceleration = 5, time = 0.05):
        command = "speedl(" + np.array2string(control_law, precision= 3, separator=',') +","+ \
        str(acceleration) + "," + str(time) + ")" #0.3,0.2
        #rospy.loginfo(control_law)
        #print(command)
        self.pub.publish(command)

    def q_dot(self, dq_value, acceleration = 3, time = 0.05):
        command = "speedj(" + np.array2string(dq_value, precision= 3, separator=',') +","+ \
        str(acceleration) + "," + str(time) + ")" #0.3,0.2
        #rospy.loginfo(control_law)
        #print(command)
        self.pub.publish(command)


#######################################
######### ROTATION MATRICES ###########
#######################################
        
    def Matrix_T(self, phi,theta): #mapping matrix from euler ZYZ velocities and angular velocities 
        Matrix = np.matrix( [ [0 , -sin(phi) , cos(phi)*sin(theta)] , [0, cos(phi), sin(phi)*sin(theta)] , [1.0, 0, cos(theta)]]) 
        #Matrix = np.matrix( [ ])
        return Matrix

    # Transforms control from tool frame to base frame if frame is set to 'tool'
    # Used to tranform the fwrench witch is measured in tool frame and calculations are made in base frame
    def reference_frame(self, frame, control_law):
        rot_mat = self.getTransMatrix()
        if frame == 'base': 
            control_law = control_law
        elif frame == 'tool': 
            linear_velocity = np.matmul(rot_mat[0:3,0:3], control_law[0:3]) 
            angular_velocity = np.matmul(rot_mat[0:3,0:3], control_law[3:6])  
            control_law = np.concatenate((linear_velocity, angular_velocity))
	    #print(linear_velocity)
	    #print(angular_velocity) 
        else: 
            raise NameError('Specify base or tool as a frame!')
        return control_law

#######################################
############# CALLBACKS ###############
#######################################

    def wrenchSensorCallback(self, data):
        self.force_sensor = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]
        self.torque_sensor = [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]
        self.sensor_header = data.header
	#return self.force_sensor 

    def cameraCallback2(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        stringform = data.data.replace("[", "")
        stringform = stringform.replace("]", "")
        stringform = stringform.replace(",", " ")
        Tmatrix2 = np.fromstring(stringform, dtype=float, sep=' ')
        Tmatrix2 = np.reshape(Tmatrix2, (4,4))
        self.cameraData2 = Tmatrix2

    def cameraCallback4(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        stringform = data.data.replace("[", "")
        stringform = stringform.replace("]", "")
        stringform = stringform.replace(",", " ")
        Tmatrix4 = np.fromstring(stringform, dtype=float, sep=' ')
        Tmatrix4 = np.reshape(Tmatrix4, (4,4))
        self.cameraData4 = Tmatrix4

    def cameraCallback9(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        stringform = data.data.replace("[", "")
        stringform = stringform.replace("]", "")
        stringform = stringform.replace(",", " ")
        Tmatrix9 = np.fromstring(stringform, dtype=float, sep=' ')
        Tmatrix9 = np.reshape(Tmatrix9, (4,4))
        self.cameraData9 = Tmatrix9

    def jointStateCallback(self, data):
        self.q = data.position
        self.dq = data.velocity
        self.joints_order = data.name

    def toolVelocityCallback(self, data):
        self.tool_velocity_linear__ = [data.twist.linear.x, data.twist.linear.y, data.twist.linear.z]
        self.tool_velocity_angular = [data.twist.angular.x, data.twist.angular.y, data.twist.angular.z]

    def wrenchCallback(self, data): 
        self.force = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]
        self.torque = [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]

    def transfMat(self, frame_1,  frame_2):
        try:
            self.listener.waitForTransform( '/'+frame_1, '/'+frame_2, rospy.Time(0),rospy.Duration(1))
            (trans,rot) = self.listener.lookupTransform('/'+frame_1, '/'+frame_2, rospy.Time(0))
            transrotM = self.listener.fromTranslationRotation(trans, rot)
            
            # Comment on the rotation sequences 
            # 'sxyz' is the sequence for getting the euler-angles RPY(Roll,Pitch,Yaw)in radians
            # it means static rotation around x-y-z
            euler = tf.transformations.euler_from_matrix(transrotM, 'sxyz'); 
            quaternion = tf.transformations.quaternion_from_matrix(transrotM)
            return trans, euler,transrotM, quaternion
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2.TransformException):
            print("EXCEPTION")
            pass 


    #######################################
    ############## FUNCTIONS ##############
    #######################################



    #Returns false if TCP distance to homePose is greater than distance, else returns true.
    def proximityCheck(self,homePose,distance):
        homePos = homePose[0]
        currentPosistion = self.getTaskPosi()
        error = abs(np.linalg.norm(homePos-currentPosistion))
        #print('Distance to home: ' + str(error))
        if(error > distance):
            return False
        else: 
            return True


    # Zeroes all values in the array that is between max and min, could be seen as a "filter"
    def filterArray(self, array, min, max):
        result = np.zeros(array.size)
        counter = 0
        for i in array:
            if i > max or i < min:
                result[counter] = array[counter] 
            counter = counter + 1      
        return result

    # Returns an average of an array with arbitrary length
    def averageOfArray(self, array):
        sizeOfArray = np.shape(array)
        result = np.zeros(6)
        for i in array[0,:]:
            result[i] = np.mean(array[:,i])
        return result

    # Returns an median of an matrix with arbitrary length
    def medianOfArray(self, array):
        result = np.zeros(6)
        result = np.median(array, axis=0)
        #for i in array[0,:]:
            #result[i] = np.median(array[:,i])
        return result 

    
        
                
    #######################################
    ########## GET FUNCTIONS ##############
    #######################################

    def getToolLinearVelocity(self): 
        return self.tool_velocity_linear__

    #Returns x-y and z pos in meters as a vector [x, y, z]
    def getTaskPosi(self): 
        result = self.transfMat('base','tool0_controller')
        if result == None:
            result = np.array([[0,0,0],[0,0,0]])
        self.position = result[0]
        return self.position

    #Returns current rotation in euler-angles (roll,pitch,yaw)
    def getTaskEuler(self): 
        result = self.transfMat('base','tool0_controller')
        self.orientation = result[1]
        return self.orientation

    #Returns current rotation in quaternions
    def getTaskQuaternion(self): 
        result = self.transfMat('base','tool0_controller')
        self.quaternion = result[3]
        return self.quaternion

    def getTransMatrix(self): 
        transrotM = self.transfMat( 'base','tool0_controller')
        self.RotationM = transrotM[2]
        return self.RotationM

    # Returns actual wrench from F/T-sensor
    def getWrenchNoOffset(self):
        return np.concatenate((np.array(self.force_sensor), np.array(self.torque_sensor)))    

    # Returns wrench from the force sensor with respect taken to initial values
    def getWrench(self):
        wrench = np.concatenate((np.array(self.force_sensor), np.array(self.torque_sensor)))-self.force_offset 
	if len(wrench) == 0:
		wrench = np.array([-1,0,0,0,0,0])     
	return wrench
        #return np.random.rand(6)*100000*2
        #return np.zeros(6)
        #return np.array([-1,0,0,0,0,0])


    # Returns the current forces values from the F/T-sensor
    def getSensorForce(self):
        return self.force_sensor

    def getSensorHeader(self):
        return self.sensor_header

    # Returns the current torque values from the F/T-sensor
    def getSensorTorque(self):
        return self.torque_sensor

    # Returns the current state of the robot
    def getRobotState(self):
        state = {}
        state['joints_order'] = self.joints_order  
        state['joint_angles'] = self.q 
        state['joint_velocity'] = self.dq  
        state['end_effector_position'] = self.position 
        state['end_effector_orientation'] = self.orientation 
        state['end_effector_linear_velocity'] = self.tool_velocity_linear__ 
        state['end_effector_angular_velocity'] = self.tool_velocity_angular
        state['force'] = self.force 
        state['torque'] =self.torque 
        state['Transf_matrix']=self.RotationM 
        state['mode'] = self.mode 
        return state
		
		
		
#--------------------------------------------------------------------
# spin 
#--------------------------------------------------------------------
    def run(self):
        #--Simulation--

        #position = np.array([-0.12, -0.43, 0.26])
	#position = np.array([-0.17239, -0.38559, 0.46098])
        #quaternion = np.array([-2.91e-04, 9.998e-01, 1.245e-02, 1.25e-02]) 
	
        #quaternion = np.array([3.49848602e-06,  9.99961671e-01, -3.99984690e-04,  8.74621458e-03])
        #quaternion = np.array([0.68450141, -0.07245186,  0.15660954,  0.70829514])
	#--Real robot--  
        #position = np.array([0.20864163268953798, 0.2901168672870089, 0.7255072413646502])
        #quaternion = np.array([0.95663979,-0.29068509,-0.01717201,-0.00689979])

        #Start-pose for rowing exercise
        #position = np.array([0.15756176236973007, 0.3590548461660456, 0.41006310959573894])
        #quaternion = np.array([-0.70700723, -0.00474353, 0.00277345, 0.7071849])
    
        #New start-pose(start-pose 2), only testing, not compatile with safety limits (yet)
        #position = np.array([0.3988074665985663, -0.2106094069148962, 0.39359153019942167])
        #quaternion = np.array([0.4993599, 0.50327672, 0.50019774, 0.49714627])
	

        #print Jac_psudo



	isCalibrated = False
	position = np.array([0, 0, 0])
        quaternion = np.array([0, 0,  0,  0])
        position, quaternion = self.goalPosition(position,quaternion)
	prev_error_0 = 1000
        prev_error_1 = 1000
        prev_error_2 = 1000
        start_pose = np.array([position, quaternion])
        while (not rospy.is_shutdown()): 
	
            # Publishing, for being able to plot later
            
            #self.pub_x.publish(self.cameraData2[0][3])
            #self.pub_y.publish(self.getTaskPosi()[1]-start_pose[0][1])
            #self.pub_z.publish(self.getTaskPosi()[2]-start_pose[0][2])
            

            forceVar.set(str(int(np.linalg.norm(self.getWrench()*np.array([1,1,1,0,0,0]))/10000)))            
            force_now = int(np.linalg.norm(self.getWrench()*np.array([1,1,1,0,0,0]))/10000)
            global red
            red = force_now*(239-30)/250 + 30 
            global green
            green = force_now*(23-35)/250 + 35
            global blue
            blue = force_now*(23-191)/250 + 191

            #global force_plot
            #force_plot = np.roll(force_plot,1)
            #force_plot[0]= force_now
            #print(force_now)
            

            res = resistance
            global str_res 
            str_res.set("Current resistance: "+str(res))

            pose = start_pose
            global atHome
            #atHome = self.proximityCheck(start_pose, 0.03)
            #print(run_robot)
            #print(atHome)
            global rotGen

	    #######################################
            ############ MAIN  ####################
            #######################################
            #self.findPosition()
            
            if(np.mean(rotGen[0]) == 0 or np.mean(rotGen[1]) == 0 or np.mean(rotGen[2]) == 0 ):
	        controller = self.trajGenPoints(pose, trajCount)
                rotGen = self.calibration(pointsCam, pointsRobo, pose)
            
            
            if(np.mean(rotGen[0]) != 0 or np.mean(rotGen[1]) != 0 or np.mean(rotGen[2]) != 0 ):
                controller, Tmatrix2, Tmatrix4 = self.impedance_control(pose, rotGen)
            
                if(abs(round(controller[0],4)) > prev_error_0 and abs(round(controller[1],4)) > prev_error_1 and abs(round(controller[2],4)) > prev_error_2 and abs(round(controller[3],4)) > prev_error_3 and abs(round(controller[4],4)) > prev_error_4 and abs(round(controller[5],4)) > prev_error_5 ):
                    if(isCalibrated == False):
                        rotGen = rotGen*-1
                        isCalibrated = True
                        print("##################################################")
                prev_error_0 = abs(round(controller[0],4))
                prev_error_1 = abs(round(controller[1],4))
                prev_error_2 = abs(round(controller[2],4))
                prev_error_3 = abs(round(controller[3],4))
                prev_error_4 = abs(round(controller[4],4))
                prev_error_5 = abs(round(controller[5],4))
               
            print(controller)
            Jac_psudo = self.jac_function()
	    V_ref = np.transpose(controller)
	    



            # Future recom.     False/True key, "have you calibrated" if true then self.q_dot(dq_value)
            
	    dq = np.matmul(Jac_psudo, V_ref)
	    dq_value = np.asarray(dq).reshape(-1)
	    
            self.q_dot(dq_value)



            #######################################
            ############ Plots  ###################
            ####################################### 
	    #self.pub_x.publish(Tmatrix2[0][3])
            #self.pub_y.publish(Tmatrix2[1][3])
            #self.pub_z.publish(Tmatrix2[2][3])
            #self.pub_xd.publish(Tmatrix4[0][3])
            #self.pub_yd.publish(Tmatrix4[1][3])
            #self.pub_zd.publish(Tmatrix4[2][3])
            
             













	    
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot_thread = Robot()
        robot_thread.setName("Robot Thread")
        robot_thread.daemon = True
        robot_thread.start()

        #gui_thread = Interface(root)
        #gui_thread.setName("GUI Thread")
       
        root.mainloop()
         
    except rospy.ROSInterruptException:
        pass
