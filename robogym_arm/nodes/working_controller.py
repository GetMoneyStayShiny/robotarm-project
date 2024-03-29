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
clockCount = 0
collectedPoints = 0

testMarker = False
prePlannedReady = False
readyToCollect3D = False
readyToGo = False
isTipToolCalibrated = False
isReversed = False

calib_pointerTmatrix = np.array([[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]])
prePlanMatrix = np.array([[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]])
checkrow1 = 0.25
checkrow2 = 0.25
checkrow3 = 0.25
flipTarget = False

checkrowPrePlan1 = 0.25
checkrowPrePlan2 = 0.25
checkrowPrePlan3 = 0.25


pointsCam = np.array([[0,0,0, 0]])
pointsRobo = np.array([[0,0,0]])
rotGen = np.zeros((3,3))
trajCount = 0
speedK = 0
lambdaK = 0
joint1 = 0
onoff_light = StringVar()
onoff_light.set("red")

#at_home_light = StringVar()
#at_home_light.set("red")

confirm_string = """
Please verify that the controller follows your marker.
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

explination_string = """
This is the calibration verification interface. \n 
Firt, calibrate the tip of the the tool.
Second, find a goal position and save it thorugh the interface.
Lastly, collect 3D points. Use the free-drive function to manually move around the robot \n end-effector until 100 points have been collected. 
Remember to hold the end-effector so that it faces the camera, and collect points all \n around the robot, in diferent directions and depths. The images presented to the right \n illustrates how to calibrate the system.
Note: Use the stop button in the interface to stop or pause the movement of the robot. But be ready with the emergency break on the teach pendant in case of unexpected collsions. 
"""
#Please move the robot in the vision field of the camera in order to collect 3D points. We have estimated 100 points should be enough. After the calibration is complete and the robot calculates the position of the camera, you should verify that this works. Pick up the verification marker and move it around very close to the robots end effector, see if the marker follows you as inteded. IF so the calibration is complete! If for some reason the robot does not work as expected. Check the correct checkbox to redo the calibration. Each time the collected points counter will increase with 30 extra points!  



        
class Interface(threading.Thread):
    def __init__(self, tk_root):
        self.root = tk_root
        #self.popup = popup
        threading.Thread.__init__(self)
        self.start()

        self.root.title("Robo-assistant surgery: Calibration")
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



        self.PP_done = Button(frame, text="Save goal position",command=self.prePlannedFunc)
        self.PP_done.grid(row=10, column=2,padx = 2, pady = 2, sticky=N+S+E+W)

        self.test_calibration = Button(frame, text="Verify the calibration",command=self.testCalibration)
        self.test_calibration.grid(row=26, column=1,padx = 2, pady = 2, sticky=N+S+E+W)

        self.save_unit = Button(frame, text="Save tool tip calibration",command=self.saveUnit)
        self.save_unit.grid(row=10, column=1,padx = 2, pady = 2, sticky=N+S+E+W)


        self.collect_3Dp = Button(frame, text="Collect 3D points",command=self.collect3Dp)
        self.collect_3Dp.grid(row=10, column=3,padx = 2, pady = 2, sticky=N+S+E+W)

        self.go_button = Button(frame, text="Go",command=self.goButton)
        self.go_button.grid(row=15, column=1,padx = 2, pady = 2, sticky=N+S+E+W)


        self.stop_button = Button(frame, text="Stop",command=self.stopButton)
        self.stop_button.grid(row=15, column=2,padx = 2, pady = 2, sticky=N+S+E+W)

        self.reset = Button(frame, text = "Reset Calibration", command = self.reset)
        self.reset.grid(row=15,column = 3, padx = 2, pady = 2, sticky= N+S+E+W)

        self.cancel_button = Button(frame, text="Cancel",command=self.cancelButton)
        self.cancel_button.grid(row=40, column=10,padx = 2, pady = 2, sticky=N+S+E+W)

        self.set_gain = Button(frame, text = "Set K", command = self.setGain)
        self.set_gain.grid(row=26,column = 3, padx = 2, pady = 2, sticky= N+S+E+W)
        
        #Slidebar1
        self.slideBar = Scale(frame, from_=1, to=10, length=200, width= 20, tickinterval=1)
        self.slideBar.grid(row = 22, rowspan = 9, column = 2, sticky = S)
        self.slideBar.set(1)
        speedK = self.slideBar.get()
       

        #Explain text
        text_exercise = Text(frame, height=13, width=90)
        text_exercise.tag_configure('bold_italics', font=('Arial', 12, 'bold', 'italic'))
        text_exercise.tag_configure('big', font=('Verdana', 12, 'bold'))
        text_exercise.tag_configure('color', foreground='#476042', font=('Tempus Sans ITC', 12, 'bold'))

        text_exercise.tag_bind('follow', '<1>', lambda e, t=text_exercise: t.insert(END, "Not now, maybe later!"))

        text_exercise.insert(END,explination_string)   
        text_exercise.config(state = DISABLED)
        text_exercise.grid(row=1, rowspan=10, column=1, columnspan = 14,padx=2, pady = 2 ,sticky=N)
        





        #Display numbers
        self.txt_points = Label(frame, text = "Number of collected 3D points: ", font = ('Avenir Next', 13))
        self.txt_points.grid(row=23, rowspan=1, column=0, columnspan = 2, padx=15, pady=2 ,sticky=N+W)
        self.txt_pointsCollected = Label(frame, text = len(pointsCam) , font = ('Avenir Next', 13))
        self.txt_pointsCollected.grid(row=23, rowspan=1, column=2, columnspan = 2, padx=2, pady=2 ,sticky=N+W)
        self.txt_pointsCollected2 = Label(frame, text = 100 , font = ('Avenir Next', 13))
        self.txt_pointsCollected2.grid(row=23, rowspan=1, column=4, columnspan = 1, padx=2, pady=2 ,sticky=N+W)
        self.txt_pointsCollected3 = Label(frame, text = "/" , font = ('Avenir Next', 13))
        self.txt_pointsCollected3.grid(row=23, rowspan=1, column=3, columnspan = 1, padx=2, pady=2 ,sticky=N+W)
        self.update_pointsCollected()
        self.update_pointsCollected2()


        
        #Exercise image
        text_image = Text(frame, height=35, width=40)   

        
        org_img = Image.open("unitCali.jpg") # IMAGE OF MARKER

        self.photo = ImageTk.PhotoImage(org_img)
        
        text_image.insert(END,'\n')
        text_image.image_create(END, image=self.photo)
        text_image.config(state = DISABLED)
        text_image.grid(row=5,rowspan=25, column=20,columnspan =18, sticky = N)
        


        #image2
        text_image = Text(frame, height=20, width=50)   

        org_img = Image.open("collectPoints.jpg") # IMAGE OF MARKER


        self.photo2 = ImageTk.PhotoImage(org_img)
        
        text_image.insert(END,'\n')
        text_image.image_create(END, image=self.photo2)
        text_image.config(state = DISABLED)
        text_image.grid(row=25,rowspan=25, column=20,columnspan =18, sticky = N)
         
      

        global w,h

        w= 1366
        h= 1366
        
        # Buttons
        self.root.geometry( str(w) + 'x' + str(h))
      
        
    def update_pointsCollected(self):
        self.txt_pointsCollected.configure(text = len(pointsCam))
        self.txt_pointsCollected.after(100, self.update_pointsCollected)

    def update_pointsCollected2(self):
        global collectedPoints
        self.txt_pointsCollected2.configure(text = 101+collectedPoints)
        self.txt_pointsCollected2.after(100, self.update_pointsCollected2)


    
    def slideBar(self):
        global k_var
        k_var = self.slideBar.get()

       
    def slideBar2(self):
        global lambda_var
        lambda_var = self.slideBar2.get()

    
    def slideBarConfirm(self):										
        global run_robot
        run_robot=True

   
    def setGain(self):
        global speedK
        global lambdaK
        speedK = self.slideBar.get()
        lambdaK = self.slideBar2.get()


    def prePlannedFunc(self):
        global prePlannedReady
        prePlannedReady = True
        popup = Toplevel()

        popup.grab_set()
        popup.title("Target has been saved")
        popup.geometry(str(w/2) + 'x' + str(h/10))
        popup.geometry("+300+400")

        explanation = "Target is saved, please attach the tool to the end-effector"
        popw2 = Label(popup, justify=LEFT, padx=200, pady=25, height=3,width=10, text=explanation).pack()
        def popup_done():
            popup.destroy()
            popup.grab_release()
        
        self.B1 = Button(popup, text="OK", command=popup_done).pack()


    def saveUnit(self):
        global isTipToolCalibrated
        isTipToolCalibrated = True
        popup = Toplevel()
        popup.grab_set()
        popup.title("Tool tip has been calibrated")
        popup.geometry(str(w/2) + 'x' + str(h/10))
        popup.geometry("+300+400")
        explanation = "Tool tip calibrated, please attach the tool to the end-effector"
        popw2 = Label(popup, justify=LEFT, padx=200, pady=25, height=3,width=10, text=explanation).pack()
        def popup_done():
            popup.destroy()
            popup.grab_release()
        
        self.B1 = Button(popup, text="OK", command=popup_done).pack()


    def collect3Dp(self):
        global readyToCollect3D
        readyToCollect3D = True
        popup = Toplevel()
        popup.grab_set()
        popup.title("Ready to collect 3D points")
        popup.geometry(str(w/2) + 'x' + str(h/10))
        popup.geometry("+300+400")
        explanation = "Move around the robot's end-effector in order to collect 3D points"
        popw2 = Label(popup, justify=LEFT, padx=200, pady=25, height=3,width=10, text=explanation).pack()
        def popup_done():
            popup.destroy()
            popup.grab_release()
        
        self.B1 = Button(popup, text="OK", command=popup_done).pack()


    def testCalibration(self):
        global testMarker
        testMarker = True
        popup = Toplevel()
        popup.grab_set()
        popup.title("Test your calibration")
        popup.geometry(str(w/2) + 'x' + str(h/10))
        popup.geometry("+300+400")
        explanation = "Pick up marker 9 and try to let the end effector to follow your marker, if not reset calibration"
        popw2 = Label(popup, justify=LEFT, padx=200, pady=25, height=3,width=10, text=explanation).pack()
        def popup_done():
            global testMarker
            global flipTarget
            popup.destroy()
            testMarker = False
            flipTarget = False 
            popup.grab_release()
        
        self.B1 = Button(popup, text="OK", command=popup_done).pack()


    def cali_done(self):
        global readyToGo
        readyToGo=True

      
    def goButton(self):
        global readyToGo
        #global isTipToolCalibrated
        #isTipToolCalibrated = True
        readyToGo=True
   
    def stopButton(self):
        global readyToGo
        global testMarker
        testMarker = False
        readyToGo=False

    def cancelButton(self):
        global readyToGo
        global testMarker


    def reset(self):
        global rotGen, collectedPoints, readyToGo, prePlannedReady, isTipToolCalibrated, pointsCam, pointsRobo, readyToCollect3D, testMarker
        
        isTipToolCalibrated = False
        readyToGo=False
        prePlannedReady = False
        readyToCollect3D = False
        testMarker = False
        pointsCam = np.array([[0,0,0,0]])
        pointsRobo = np.array([[0,0,0]])
        rotGen = np.zeros((3,3))
        collectedPoints += 20 
        
        popup = Toplevel()
        popup.grab_set()
        popup.title("The system has been reset")
        popup.geometry(str(w/2) + 'x' + str(h/10))
        popup.geometry("+300+400")
        explanation = "Please recalibrate the tool tip and target, and recollect 3D points with an additional 20 points"
        popw2 = Label(popup, justify=LEFT, padx=200, pady=25, height=3,width=20, text=explanation).pack()
        def popup_done():
            popup.destroy()
            popup.grab_release()
        
        self.B1 = Button(popup, text="OK", command=popup_done).pack()
        

    def end(self):
        self.root.destroy()
        rospy.is_shutdown()


        
        def popup_done(): 
            global guide_z 
            guide_z = False
            popup.destroy()
            popup.grab_release()

        self.B1 = Button(popup, text="Done", command=popup_done).pack()

      

class Robot(threading.Thread): 

    def __init__(self, freq= 20):
        threading.Thread.__init__(self)
        # Default frequency 125Hz
        rospy.init_node('Controller', anonymous=True)
        self.rate = rospy.Rate(freq)
       
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

        
 
        #Subscribers and Publisher
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=5)
        #self.pub = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self.jointStateCallback)
        rospy.Subscriber("/tool_velocity", TwistStamped, self.toolVelocityCallback)
  
        #Sensor publishers and subscribers
        #self.pub_reset = rospy.Publisher('/ethdaq_zero', Bool, queue_size=1)
        
        # Publish plot data
        """
        self.pub_x = rospy.Publisher('/goal_x', Float32, queue_size=1)
        self.pub_y = rospy.Publisher('/goal_y', Float32, queue_size=1)
        self.pub_z = rospy.Publisher('/goal_z', Float32, queue_size=1)
        self.pub_xd = rospy.Publisher('/position_x', Float32, queue_size=1)
        self.pub_yd = rospy.Publisher('/position_y', Float32, queue_size=1)
        self.pub_zd = rospy.Publisher('/position_z', Float32, queue_size=1)
        self.pub_xb = rospy.Publisher('/camera_pos_x', Float32, queue_size=1)
        self.pub_yb = rospy.Publisher('/camera_pos_y', Float32, queue_size=1)
        self.pub_zb = rospy.Publisher('/camera_pos_z', Float32, queue_size=1)
        """
        
        self.pub_endeff_x = rospy.Publisher('/tool_x', Float32, queue_size=1)
        self.pub_endeff_y = rospy.Publisher('/tool_y', Float32, queue_size=1)
        self.pub_endeff_z = rospy.Publisher('/tool_z', Float32, queue_size=1)
        self.pub_endeff_q1 = rospy.Publisher('/tool_q1', Float32, queue_size=1)
        self.pub_endeff_q2 = rospy.Publisher('/tool_q2', Float32, queue_size=1)
        self.pub_endeff_q3 = rospy.Publisher('/tool_q3', Float32, queue_size=1)
 
        self.pub_goal_x = rospy.Publisher('/goal_x', Float32, queue_size=1)
        self.pub_goal_y = rospy.Publisher('/goal_y', Float32, queue_size=1)
        self.pub_goal_z = rospy.Publisher('/goal_z', Float32, queue_size=1)
        self.pub_goal_q1 = rospy.Publisher('/goal_q1', Float32, queue_size=1)
        self.pub_goal_q2 = rospy.Publisher('/goal_q2', Float32, queue_size=1)
        self.pub_goal_q3 = rospy.Publisher('/goal_q3', Float32, queue_size=1)
        
        """
        self.pub_joint1 = rospy.Publisher('/q_1', Float32, queue_size=1)
        self.pub_joint2 = rospy.Publisher('/q_2', Float32, queue_size=1)
        self.pub_joint3 = rospy.Publisher('/q_3', Float32, queue_size=1)
        self.pub_joint4 = rospy.Publisher('/q_4', Float32, queue_size=1)
        self.pub_joint5 = rospy.Publisher('/q_5', Float32, queue_size=1)
        self.pub_joint6 = rospy.Publisher('/q_6', Float32, queue_size=1)


        self.pub_dq1 = rospy.Publisher('/q_1_max', Float32, queue_size=1)
        self.pub_dq2 = rospy.Publisher('/q_2_max', Float32, queue_size=1)
        self.pub_dq3 = rospy.Publisher('/q_3_max', Float32, queue_size=1)
        self.pub_dq4 = rospy.Publisher('/q_4_max', Float32, queue_size=1)
        self.pub_dq5 = rospy.Publisher('/q_5_max', Float32, queue_size=1)
        self.pub_dq6 = rospy.Publisher('/q_6_max', Float32, queue_size=1)

        self.pub_dq1_no_limit = rospy.Publisher('/q_1_min', Float32, queue_size=1)
        self.pub_dq2_no_limit = rospy.Publisher('/q_2_min', Float32, queue_size=1)
        self.pub_dq3_no_limit = rospy.Publisher('/q_3_min', Float32, queue_size=1)
        self.pub_dq4_no_limit = rospy.Publisher('/q_4_min', Float32, queue_size=1)
        self.pub_dq5_no_limit = rospy.Publisher('/q_5_min', Float32, queue_size=1)
        self.pub_dq6_no_limit = rospy.Publisher('/q_6_min', Float32, queue_size=1)
        """
        rospy.Subscriber("/ethdaq_data_raw", WrenchStamped, self.wrenchSensorCallback)
        rospy.Subscriber("/ethdaq_data", WrenchStamped, self.wrenchSensorCallback)
        rospy.Subscriber("/geom2", String,  self.cameraCallback2)
        rospy.Subscriber("/geom4", String,  self.cameraCallback4)
        rospy.Subscriber("/geom9", String,  self.cameraCallback9)
        #self.pub_x = rospy.Publisher('/positionx', Float32, queue_size=1) 
        
        rospy.sleep(0.1) #time needed for initialization 



#######################################
############ Calibration FUNCTIONS #############
#######################################


    def calibration(self, Tmatrix4, desired_pose, collectedPoints):
        global pointsCam
        global pointsRobo
        global rotGen
        actual_position = self.getTaskPosi()
        desired_position_base = desired_pose[0]
        desired_quaternion = desired_pose[1]      
        roundVariable = 1
        switchonoff = True
        actual_pos_camera = Tmatrix4[0:4,3:4].flatten()/1000
        
        # Round numbers
        actual_pos_camera[len(actual_pos_camera) - 1] = actual_pos_camera[len(actual_pos_camera)-1]*1000
        actual_pos_camera_tmp = np.copy(actual_pos_camera)
        for i in range(len(actual_pos_camera)):
            actual_pos_camera_tmp[i] = round(actual_pos_camera[i], roundVariable)
        
        
        if(len(pointsCam) > (100 + collectedPoints)):
            switchonoff = False
            pointsCamReal = np.delete(pointsCam,[0,0,0,0],0)
            pointsRoboReal = np.delete(pointsRobo,[0,0,0],0)
            
            dCam = np.copy(pointsCamReal)
            dRobo = np.copy(pointsRoboReal)
            meanCam = np.mean(pointsCamReal,axis=0)
            meanRobo = np.mean(pointsRoboReal,axis=0)

            # d_ny = d_i - d_hat
            for i in range(len(pointsCamReal)):
                for j in range(len(pointsCamReal[0])-1):
                    dCam[i][j] = pointsCamReal[i][j] - meanCam[j] 
                    dRobo[i][j] = pointsRoboReal[i][j] - meanRobo[j]
            
            
         
            s = 12
            l = 0
            b = 0
            Mmat = np.zeros((len(pointsCamReal)*3, len(pointsCamReal) + 12))
            # Create matrix
            for i in range(len(pointsCamReal)):
                for j in range(len(pointsCamReal[0])):     
                    Mmat[b][j] = dCam[i][j]
                    Mmat[b+1][j+4] = dCam[i][j]
                    Mmat[b+2][j+8] = dCam[i][j]   
                b = b+3

            
            for i in range(len(pointsCamReal)):
                for v in range(len(pointsRoboReal[0])):
                    Mmat[l+v][s] = -dRobo[i][v]
                l=l+3
                s=s+1
             
            U1, S1, V1 = np.linalg.svd(Mmat)
            V1_end = V1[len(V1)-1][0:12]
            Pmat = np.reshape(V1_end,(3,4))
            rotGen = Pmat[0:3,0:3]
        
        # Collect 3D points
        
        if(switchonoff == True):
            if(actual_pos_camera_tmp[0] != round(pointsCam[len(pointsCam) - 1][0],roundVariable) or actual_pos_camera_tmp[1] != round(pointsCam[len(pointsCam) - 1][1],roundVariable) or actual_pos_camera_tmp[2] != round(pointsCam[len(pointsCam) - 1][2],roundVariable)):
                pointsCam = np.vstack([pointsCam, actual_pos_camera])
                pointsRobo = np.vstack([pointsRobo, actual_position])

        #Returns rotGen from SVD
        return rotGen
#######################################
############ Control FUNCTIONS #############
#######################################
    def impedance_control(self,Tmatrix2, Tmatrix4, desired_pose, rotGen, speedK,  calib_pointerTmatrix, prePlanMatrix, flipTarget):
        # Gain K
        c = 5.0
        K1 = 1.0/c
        K2 = 3.0/c
        if(flipTarget == True):
            K2 = 0
        k1 = (K1*speedK)
        k2 = (K2*speedK)
        
        k = np.array([[k1,0,0,0,0,0], [0,k1,0,0,0,0], [0,0,k1,0,0,0], [0,0,0,k2,0,0], [0,0,0,0,k2,0], [0,0,0,0,0,k2]])
        
        global pointsCam
        global pointsRobo
        self.mode = 'impedance controller'
        quaternion = self.getTaskQuaternion()
        actual_position = self.getTaskPosi()
        desired_position_base = desired_pose[0]
        desired_quaternion = desired_pose[1]

        
        rotFix = np.array([[1,0,0,0], [0,-1,0,0], [0,0,-1,0], [0,0,0,1]])
        if(np.mean(calib_pointerTmatrix) == 0):
            calib_pointerTmatrix = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        if(np.mean(prePlanMatrix) == 0):
            prePlanMatrix = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        

        # Verification marker
        if(flipTarget == True):
            prePlanMatrix = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
            calib_pointerTmatrix = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])   
        else: 
            calib_pointerTmatrix = np.array([[-3.11358795e-01, -1.08359100e-01, -9.44092484e-01, -2.85460008e+00], [ 6.26798808e-01, -7.70144464e-01, -1.18330750e-01, -1.44690665e+02], [-7.14267999e-01, -6.28597794e-01,  3.07713883e-01, -1.50225090e+02], [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

        prePlanMatrix = np.array([[-3.45769419e-01, -5.88027866e-02,  9.36478012e-01,  2.81442090e+01], [ 2.49882453e-02, -9.98257213e-01, -5.34606259e-02, -3.43502257e+01], [ 9.37986627e-01,  4.91765570e-03,  3.46628721e-01, -1.15093392e+02], [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

        goal_marker = np.matmul(Tmatrix2, prePlanMatrix)
        #print(goal_marker)
        #goal_marker = np.matmul(Tmatrix2, rotFix)
        end_eff_marker = np.matmul(Tmatrix4, calib_pointerTmatrix)  
        #goal_marker = Tmatrix2
        #end_eff_marker = Tmatrix4
        
        
        #calibration_marker = Tmatrix9
        
        desired_pos_camera = goal_marker[0:3,3:4].flatten()/1000
        actual_pos_camera = end_eff_marker[0:3,3:4].flatten()/1000
        
        ############### Calibration camera ##########################
        
        rotMC = end_eff_marker[0:3, 0:3]
        rotBE = tf.transformations.quaternion_matrix(quaternion)[0:3, 0:3]
        rotEM = np.matmul(np.matmul(np.linalg.inv(rotMC),rotGen), np.linalg.inv(rotBE))

        rotGen2 = np.matmul(np.matmul(rotMC, rotEM), rotBE)
        #rotGen2 = rotGen
        ############### p - p_d ##########################
        
        

        err = (desired_pos_camera  - actual_pos_camera)
        #error_pos_camera = np.matmul(rotBC, np.matmul(rotY , err))
        error_pos_camera = np.matmul(rotGen2 , err)
       

        quaternions_goal = tf.transformations.quaternion_from_matrix(goal_marker)
        quaternions_endeffector = tf.transformations.quaternion_from_matrix(end_eff_marker)
       
        #error_angle = tf.transformations.quaternion_multiply(desired_quaternion, tf.transformations.quaternion_inverse(quaternion))
        error_angle = tf.transformations.quaternion_multiply(quaternions_goal, tf.transformations.quaternion_inverse(quaternions_endeffector))


        error_ang = np.matmul(rotGen2 , error_angle[0:3])
        error = np.append(error_pos_camera, error_ang)
  
        # Control Law
        control_law = np.matmul(error,k)
        

        # data for plots
        data_quat_end_eff = quaternions_endeffector[0:3]
        data_quat_goal = quaternions_goal[0:3]
        data_pos_end_eff = np.array([end_eff_marker[0][3], end_eff_marker[1][3], end_eff_marker[2][3]])
        data_pos_goal = np.array([goal_marker[0][3], goal_marker[1][3], goal_marker[2][3]])

        return [control_law, data_quat_end_eff, data_quat_goal, data_pos_end_eff, data_pos_goal]
        

    def tipToolCalibration(self, Tmatrix4, Tmatrix9):
        global checkrow1, checkrow2, checkrow3, calib_pointerTmatrix
        PointerTmatrix = np.array([[-0.500604, -0.865676, -0.000642, 12.067809], [0.865676, -0.500604, 0.000444, -21.939008], [-0.000714, -0.000329, 1.000000, -2.537459], [ 0.0, 0.0, 0.0, 1.0]])

        # Math from orthoma 
        if(checkrow1 != np.mean(Tmatrix9[0]) and checkrow2 != np.mean(Tmatrix9[1]) and checkrow3 != np.mean(Tmatrix9[2])):
            calibPointer = np.matmul(Tmatrix9, PointerTmatrix)
            calib_pointerTmatrix = np.matmul(np.linalg.inv(Tmatrix4), calibPointer)
            
        checkrow1 = np.mean(Tmatrix9[0])
        checkrow2 = np.mean(Tmatrix9[1])
        checkrow3 = np.mean(Tmatrix9[2])
        return calib_pointerTmatrix


    def prePlanPosition(self, calib_pointerTmatrix, Tmatrix4, Tmatrix2):
        global checkrowPrePlan1, checkrowPrePlan2, checkrowPrePlan3, prePlan_pointerTmatrix

        # Math to pre planned pose
        prePlan_pointerTmatrix = np.array([[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]])
        if(checkrowPrePlan1 != np.mean(Tmatrix4[0]) and checkrowPrePlan2 != np.mean(Tmatrix4[1]) and checkrowPrePlan3 != np.mean(Tmatrix4[2])):
            CtoPPMatrix = np.matmul(Tmatrix4, calib_pointerTmatrix)
            prePlan_pointerTmatrix = np.matmul(np.linalg.inv(Tmatrix2), CtoPPMatrix)             
        
        checkrowPrePlan1 = np.mean(Tmatrix4[0])
        checkrowPrePlan2 = np.mean(Tmatrix4[1])
        checkrowPrePlan3 = np.mean(Tmatrix4[2])
        return prePlan_pointerTmatrix

    # Trajectory controller, tested controller, not verified in report
    def trajGenPoints(self, desired_pose, count):
        global trajCount
        c = 1
        k = 0.5
        x = 0
        y = 1
        z = 2
        threshhold = 0.1  

        self.mode = 'impedance controller'
        quaternion = self.getTaskQuaternion()
        actual_position = self.getTaskPosi()
        positions = np.array([[0.82486, -0.51018, 0.21066], [0.82738, -0.54933, 0.65417], [0.94009, 0.29343, 0.66002], [1.02991, 0.23619, 0.25965], [ 0.62244, 0.63766, 0.33269], [0.59919, 0.70732, 0.76191], [-0.31424, 0.83937, 0.7834], [-0.34466, 0.87406, 0.26547], [-0.8256, 0.45144, 0.29477], [-0.87887, 0.41303, 0.78414], [-0.95871, -0.22587, 0.75526], [-0.86459, -0.41394, 0.33856], [-0.76049, -0.74522, 0.50626], [-0.53952, -0.83571, 0.22054], [-0.5795, -0.7196, 0.64038], [0.12494, -0.92563, 0.82685], [0.13152, -0.47089, 0.28201], [-0.23391, -0.43449, 0.52768], [-0.38107, 0.31347, 0.38621], [0.33293, 0.3641, 0.38627], [0.35508, -0.34246, 0.38626]])


        quats = np.array([[-0.38292308,  0.45933626, -0.59887538,  0.53266161], [-0.40914157,  0.51389096, -0.54640928,  0.51957305],  [-0.4826565,   0.41776057, -0.39745305,  0.65920397], [-0.49307207,  0.50383661, -0.44756228,  0.55019688],  [-0.6718397,   0.10352994, -0.05911462,  0.73103928],  [-0.68133928,  0.09291008, -0.03919293,  0.72498856], [-0.63853426, -0.05921477,  0.20528296,  0.73934195], [-0.72144138, -0.10371669,  0.15037521,  0.66794647], [-0.53357887, -0.34190591,  0.49526981,  0.59422366], [-0.55541995, -0.4360819,   0.49133063,  0.50983867], [-0.46665292, -0.51137324,  0.56836963,  0.44462167], [-0.33972754, -0.59958032,  0.65153409,  0.31716237], [-0.18740591, -0.67400559,  0.69192061,  0.17844149],  [-0.10934784, -0.70206126,  0.68978216,  0.13911727], [-0.14582507, -0.61320037,  0.77455579,  0.05276061],[0.5681533,  -0.40237072,  0.33911757,  0.6326918 ], [-0.67774699,  0.01613773,  0.48899118,  0.54889545], [-0.00365793,  0.67665133,  0.7345087,   0.05124996], [0.32931651, 0.62052238, 0.40866113, 0.58266516], [ 0.67676412,  0.18838271, -0.14129164,  0.69752344], [ 0.60708547, -0.35347507, -0.5982922,   0.38542062]])
        
        
        desired_position_base = positions[count]
        desired_quaternion = quats[count]
        error_position = desired_position_base - actual_position
        error_angle = tf.transformations.quaternion_multiply(desired_quaternion, tf.transformations.quaternion_inverse(quaternion))        
        error = np.append(error_position, error_angle[0:3])

        if(trajCount > 15):
            x = 3
            y = 4
            z = 5
        if(abs(error[x]) < threshhold and abs(error[y]) < threshhold and abs(error[z]) < threshhold):
            trajCount = trajCount + 1
        if(trajCount == len(positions)-1):
            trajCount = 0
        control_law = k*error

        return control_law




#######################################
############ JACOBIAN #################
#######################################

    def jac_function(self, lambdaK):   
	q1 = self.q[0]
	q2 = self.q[1]
	q3 = self.q[2]
	q4 = self.q[3]
	q5 = self.q[4]
	q6 = self.q[5]


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

        #Jac = matrix( [[(((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)*math.cos(q3)+math.sin(q3)*(-math.sin(q4)*math.sin(q5)*d6-math.cos(q4)*d5)-a2)*math.cos(q2)-math.sin(q2)*((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)))*math.sin(q1)+math.cos(q1)*(math.cos(q5)*d6+d4), math.cos(q1)*(((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)*math.cos(q3)+math.sin(q3)*(-math.sin(q4)*math.sin(q5)*d6-math.cos(q4)*d5)-a2)*math.sin(q2)+((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3))*math.cos(q2)), (((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3))*math.cos(q2)+math.sin(q2)*((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)*math.cos(q3)-math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)))*math.cos(q1), math.cos(q1)*(((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5))*math.cos(q2)+math.sin(q2)*((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5)*math.cos(q3)-math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5))), -(((math.cos(q3)*math.cos(q4)-math.sin(q3)*math.sin(q4))*math.cos(q2)-math.sin(q2)*(math.cos(q3)*math.sin(q4)+math.sin(q3)*math.cos(q4)))*math.cos(q5)*math.cos(q1)+math.sin(q1)*math.sin(q5))*d6, 0],[(((-math.cos(q4)*math.sin(q5)*d6+math.sin(q4)*d5+a3)*math.cos(q3)+math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)+a2)*math.cos(q2)+math.sin(q2)*((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)))*math.cos(q1)+math.sin(q1)*(math.cos(q5)*d6+d4), math.sin(q1)*(((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)*math.cos(q3)+math.sin(q3)*(-math.sin(q4)*math.sin(q5)*d6-math.cos(q4)*d5)-a2)*math.sin(q2)+((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3))*math.cos(q2)), math.sin(q1)*(((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3))*math.cos(q2)+math.sin(q2)*((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)*math.cos(q3)-math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5))), (((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5))*math.cos(q2)+math.sin(q2)*((math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5)*math.cos(q3)-math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)))*math.sin(q1), -d6*(math.sin(q1)*((math.cos(q3)*math.cos(q4)-math.sin(q3)*math.sin(q4))*math.cos(q2)-math.sin(q2)*(math.cos(q3)*math.sin(q4)+math.sin(q3)*math.cos(q4)))*math.cos(q5)-math.cos(q1)*math.sin(q5)), 0],[0, ((-math.cos(q4)*math.sin(q5)*d6+math.sin(q4)*d5+a3)*math.cos(q3)+math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)+a2)*math.cos(q2)+math.sin(q2)*((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)), ((-math.cos(q4)*math.sin(q5)*d6+math.sin(q4)*d5+a3)*math.cos(q3)+math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5))*math.cos(q2)+math.sin(q2)*((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5-a3)), (math.cos(q3)*(-math.cos(q4)*math.sin(q5)*d6+math.sin(q4)*d5)+math.sin(q3)*(math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5))*math.cos(q2)+((math.sin(q4)*math.sin(q5)*d6+math.cos(q4)*d5)*math.cos(q3)+math.sin(q3)*(math.cos(q4)*math.sin(q5)*d6-math.sin(q4)*d5))*math.sin(q2), ((-math.sin(q3)*math.cos(q4)-math.cos(q3)*math.sin(q4))*math.cos(q2)+math.sin(q2)*(math.sin(q3)*math.sin(q4)-math.cos(q3)*math.cos(q4)))*d6*math.cos(q5), 0],[0,0,0,0,0,0],[0, -math.cos(q1), -math.cos(q1), -math.cos(q1), -math.sin(q1)*(math.sin(q4)*(math.sin(q2)*math.sin(q3)-math.cos(q2)*math.cos(q3))-math.cos(q4)*(math.sin(q3)*math.cos(q2)+math.sin(q2)*math.cos(q3))), (math.sin(q2)*(math.cos(q3)*math.sin(q4)+math.sin(q3)*math.cos(q4))+math.cos(q2)*(math.sin(q3)*math.sin(q4)-math.cos(q3)*math.cos(q4)))*math.sin(q5)*math.sin(q1)-math.cos(q1)*math.cos(q5)],[1, 0, 0, 0, math.sin(q4)*(math.sin(q3)*math.cos(q2)+math.sin(q2)*math.cos(q3))-math.cos(q4)*(math.cos(q2)*math.cos(q3)-math.sin(q2)*math.sin(q3)), (math.sin(q4)*(math.sin(q2)*math.sin(q3)-math.cos(q2)*math.cos(q3))-math.cos(q4)*(math.sin(q3)*math.cos(q2)+math.sin(q2)*math.cos(q3)))*math.sin(q5)]])
        lambdaa = 0
        """
        if(lambdaK == 1):
             lambdaa = 0
        if(lambdaK == 2):
             lambdaa = 0.1
        if(lambdaK == 3):
             lambdaa = 0.01
        if(lambdaK == 4):
             lambdaa = 0.001
        if(lambdaK == 5):
             lambdaa = 0.0001
        if(lambdaK == 6):
             lambdaa = 0.00001
        if(lambdaK == 7):
             lambdaa = 0.000001
        if(lambdaK == 8):
             lambdaa = 1
        if(lambdaK == 9):
             lambdaa = 10
        if(lambdaK == 10):
             lambdaa = 100
        #print("lambda", lambdaa)
        """
        #Jac_psudo = np.linalg.pinv(Jac)
        Jac_psudo = np.matmul(np.transpose(Jac), np.linalg.inv(np.matmul(Jac, np.transpose(Jac)) + np.multiply(lambdaa, np.identity(6))))

        return [Jac_psudo, Jac]

#######################################
############ Joint constraints FUNCTION #############
#######################################



    def get_jointAngles(self):
        q1 = self.q[0]
        q2 = self.q[1]
	q3 = self.q[2]
	q4 = self.q[3]
	q5 = self.q[4]
	q6 = self.q[5]
        return [q1,q2,q3,q4,q5,q5]




    # Joint constraint function
    def q_dotNfunc(self, joint1):
        q1 = self.q[0]
        q2 = self.q[1]
	q3 = self.q[2]
	q4 = self.q[3]
	q5 = self.q[4]
	q6 = self.q[5]
        
        q_array = np.array([q1,q2,q3,q4,q5,q6])
        q_max = np.array([0.0, -(5.0*np.pi/12.0), 0.0, (8.0*np.pi)/9.0, ((29.0*np.pi)/36.0), 2.0*np.pi])
        q_min = np.array([-(np.pi/2.0), -(8.0*np.pi/9.0), (-3.0*np.pi/4.0), -np.pi/9.0, ((-11.0*np.pi)/18.0), -2.0*np.pi])
        q_dotN = np.zeros(6)
        delta_q = np.zeros(6)
        q_bar = np.zeros(6)
        q_dotN = np.zeros(6)
        for i in range(6):
            delta_q[i] = q_max[i] - q_min[i]
            q_bar[i] = (q_max[i] + q_min[i])/2

        # Gradient
        summ = ((q1-q_bar[0])/delta_q[0])**2 + ((q2-q_bar[1])/delta_q[1])**2 + ((q3-q_bar[2])/delta_q[2])**2 + ((q4-q_bar[3])/delta_q[3])**2 + ((q5-q_bar[4])/delta_q[4])**2 + ((q6-q_bar[5])/delta_q[5])**2 
        summ = (summ)/6


        for i in range(6):
            q_dotN[i] = ((q_array[i]-q_bar[i])/delta_q[i])*(2.0/6.0)

        return [q_dotN, q_max, q_min]

#######################################
############ Useful FUNCTIONS #############
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


    def goalPosition(self, position, quaternion):
        ############ 1st point ###################
	position = np.array([0.5, -1.05908, 0.76039])
        quaternion = np.array([ 0.05793482,  0.64728254, -0.64398721,  0.40366984])
        
	return position, quaternion
#######################################
############ UR FUNCTIONS #############
#######################################


    def velocity_cmd(self, control_law, acceleration = 5, time = 0.05):
        command = "speedl(" + np.array2string(control_law, precision= 3, separator=',') +","+ \
        str(acceleration) + "," + str(time) + ")" #0.3,0.2
        self.pub.publish(command)

    def q_dot(self, dq_value, acceleration = 1, time = 0.15):
        command = "speedj(" + np.array2string(dq_value, precision= 3, separator=',') +","+ \
        str(acceleration) + "," + str(time) + ")" #0.3,0.2
        self.pub.publish(command)


#######################################
######### ROTATION MATRICES ###########
#######################################

# matrix from euler ZYZ velocities and angular velocities 

    def Matrix_T(self, phi,theta):
        Matrix = np.matrix( [ [0 , -sin(phi) , cos(phi)*sin(theta)] , [0, cos(phi), sin(phi)*sin(theta)] , [1.0, 0, cos(theta)]]) 

        return Matrix

    def reference_frame(self, frame, control_law):
        rot_mat = self.getTransMatrix()
        if frame == 'base': 
            control_law = control_law
        elif frame == 'tool': 
            linear_velocity = np.matmul(rot_mat[0:3,0:3], control_law[0:3]) 
            angular_velocity = np.matmul(rot_mat[0:3,0:3], control_law[3:6])  
            control_law = np.concatenate((linear_velocity, angular_velocity))
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


    def cameraCallback2(self, data):
        stringform = data.data.replace("[", "")
        stringform = stringform.replace("]", "")
        stringform = stringform.replace(",", " ")
        Tmatrix2 = np.fromstring(stringform, dtype=float, sep=' ')
        Tmatrix2 = np.reshape(Tmatrix2, (4,4))
        self.cameraData2 = Tmatrix2

    def cameraCallback4(self, data):
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
            
            euler = tf.transformations.euler_from_matrix(transrotM, 'sxyz'); 
            quaternion = tf.transformations.quaternion_from_matrix(transrotM)
            return trans, euler,transrotM, quaternion
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2.TransformException):
            print("EXCEPTION")
            pass 


   
  
    
        
                
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

   	

    def run(self):
        global rotGen
        global joint1
        global flipTarget
        #calib_pointerTmatrix = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
	Tmatrix4_temp = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        Tmatrix2_temp = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
	position = np.array([0, 0, 0])
        quaternion = np.array([0, 0,  0,  0])
        position, quaternion = self.goalPosition(position,quaternion)
	prev_error_0 = 1000
        prev_error_1 = 1000
        prev_error_2 = 1000
        prev_error_3 = 1000
        prev_error_4 = 1000
        prev_error_5 = 1000
        start_pose = np.array([position, quaternion])
        global clockCount
        while (not rospy.is_shutdown()): 
	    global speedK,atHome,isReversed,readyToGo,readyToCollect3D,prePlannedReady,collectedPoints, testMarker
            pose = start_pose
            #######################################
            ############ Markers  ####################
            #######################################
            Tmatrix2 = self.cameraData2
            Tmatrix4 = self.cameraData4
            Tmatrix9 = self.cameraData9


            #######################################
            ############ Stop when out of view  ####################
            #######################################
            if(Tmatrix4[0][3] != 0 and Tmatrix4[1][3] != 0 and Tmatrix4[2][3] != 0):
                Tmatrix4_temp = Tmatrix4
            if(Tmatrix2[0][3] != 0 and Tmatrix2[1][3] != 0 and Tmatrix2[2][3] != 0):
                Tmatrix2_temp = Tmatrix2
            #######################################
            ############  Buttons logic  ####################
            #######################################
            #self.findPosition()
            
            
            if(isTipToolCalibrated == False):
                calib_pointerTmatrix = self.tipToolCalibration(Tmatrix4, Tmatrix9)
            if(prePlannedReady == False):
                prePlanMatrix = self.prePlanPosition(calib_pointerTmatrix, Tmatrix4, Tmatrix2)
            
            if(readyToCollect3D == True and np.mean(rotGen) == 0 and prePlannedReady == True):
                rotGen = self.calibration(Tmatrix4_temp, pose, collectedPoints)
            if(readyToGo == False):
                joint1,_,_,_,_,_ = self.get_jointAngles()
                
            if(testMarker == True):
                Tmatrix2_temp = self.cameraData9
                flipTarget = True
                #prePlanMatrix = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
                #calib_pointerTmatrix = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
            


            #######################################
            ############ Active Controller  ####################
            #######################################
            controller, quat_endeff, quat_goal, pos_endeff, pos_goal = self.impedance_control(Tmatrix2_temp, Tmatrix4_temp, pose, rotGen, speedK, calib_pointerTmatrix, prePlanMatrix, flipTarget)

            #controller = self.trajGenPoints(pose, trajCount)
            #print(controller)

            #######################################
            ############ Joint Constraint  ####################
            #######################################
            Jac_psudo, Jac = self.jac_function(lambdaK)
	    V_ref = np.transpose(controller)
            #Jac_dag = np.matmul(np.transpose(Jac), np.linalg.inv(np.matmul(Jac, np.transpose(Jac))))
            #limiEq = np.identity(6) - np.matmul(Jac_dag, Jac)
	    dq = np.matmul(Jac_psudo, V_ref)
	    Jpdot = np.asarray(dq).reshape(-1)
            q_dotN, q_max, q_min = self.q_dotNfunc(joint1)
            Ident_eq = np.identity(6) - np.matmul(Jac_psudo,Jac)
            limitAvoid1 = np.matmul(Ident_eq, q_dotN)
            limitAvoid = np.asarray(limitAvoid1).reshape(-1)
            dq_value = Jpdot
            
            if(Tmatrix4[0][3] == 0 and Tmatrix4[1][3] == 0 and Tmatrix4[2][3] == 0):
                readyToGo = False
            
            if(Tmatrix2[0][3] == 0 and Tmatrix2[1][3] == 0 and Tmatrix2[2][3] == 0):
                readyToGo = False
            
            
            #######################################
            ############ Velocity Command to robot ####################
            #######################################
     
            if(readyToGo == True or testMarker == True):
                self.q_dot(dq_value)
            

            #######################################
            ############ Inverse movement  ####################
            #######################################
            if(abs(round(controller[0],4)) > prev_error_0 and abs(round(controller[1],4)) > prev_error_1 and abs(round(controller[2],4)) > prev_error_2 and readyToGo == True and np.mean(abs(controller[0:2])) > 0.014):
                if(isReversed == False):             
                    print("############# \n ############# \n ############# \n ###########")
                    #rotGen = rotGen*-1
                    #isReversed = True
                       
            prev_error_0 = abs(round(controller[0],4))
            prev_error_1 = abs(round(controller[1],4))
            prev_error_2 = abs(round(controller[2],4))
            prev_error_3 = abs(round(controller[3],4))
            prev_error_4 = abs(round(controller[4],4))
            prev_error_5 = abs(round(controller[5],4))
            

            #######################################
            ############ Plots  ###################
            ####################################### 
            """
	    self.pub_x.publish(Tmatrix2[0][3])
            self.pub_y.publish(Tmatrix2[1][3])
            self.pub_z.publish(Tmatrix2[2][3])
            self.pub_xd.publish(Tmatrix4[0][3])
            self.pub_yd.publish(Tmatrix4[1][3])
            self.pub_zd.publish(Tmatrix4[2][3])
            self.pub_xb.publish(Tmatrix9[0][3])
            self.pub_yb.publish(Tmatrix9[1][3])
            self.pub_zb.publish(Tmatrix9[2][3])
            """
            



            """
            self.pub_endeff_x.publish(pos_endeff[0])
            self.pub_endeff_y.publish(pos_endeff[1])
            self.pub_endeff_z.publish(pos_endeff[2])
            self.pub_endeff_q1.publish(quat_endeff[0])
            self.pub_endeff_q2.publish(quat_endeff[1])
            self.pub_endeff_q3.publish(quat_endeff[2])
            
            self.pub_goal_x.publish(pos_goal[0])
            self.pub_goal_y.publish(pos_goal[1])
            self.pub_goal_z.publish(pos_goal[2])
            self.pub_goal_q1.publish(quat_goal[0])
            self.pub_goal_q2.publish(quat_goal[1])
            self.pub_goal_q3.publish(quat_goal[2])
            """
            radToDeg = 180.0/np.pi
            j1,j2,j3,j4,j5,j6 = self.get_jointAngles()
            """
            self.pub_joint1.publish(j1*radToDeg)
            self.pub_joint2.publish(j2*radToDeg)
            self.pub_joint3.publish(j3*radToDeg)
            self.pub_joint4.publish(j4*radToDeg)
            self.pub_joint5.publish(j5*radToDeg)
            self.pub_joint6.publish(j6*radToDeg)
            
            self.pub_dq1.publish(q_max[0]*radToDeg)
            self.pub_dq2.publish(q_max[1]*radToDeg)
            self.pub_dq3.publish(q_max[2]*radToDeg)
            self.pub_dq4.publish(q_max[3]*radToDeg)
            self.pub_dq5.publish(q_max[4]*radToDeg)
            self.pub_dq6.publish(q_max[5]*radToDeg)
 
            self.pub_dq1_no_limit.publish(q_min[0]*radToDeg)
            self.pub_dq2_no_limit.publish(q_min[1]*radToDeg)
            self.pub_dq3_no_limit.publish(q_min[2]*radToDeg)
            self.pub_dq4_no_limit.publish(q_min[3]*radToDeg)
            self.pub_dq5_no_limit.publish(q_min[4]*radToDeg)
            self.pub_dq6_no_limit.publish(q_min[5]*radToDeg)
            """

            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot_thread = Robot()
        robot_thread.setName("Robot Thread")
        robot_thread.daemon = True
        robot_thread.start()

        gui_thread = Interface(root)
        gui_thread.setName("GUI Thread")
       
        root.mainloop()
         
    except rospy.ROSInterruptException:
        pass
