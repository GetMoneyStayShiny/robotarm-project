#!/usr/bin/env python2.7
import roslib
import rospy 
import numpy as np 
import time
import tf
import select
import math
import sys, termios, tty, os, time
import Tkinter as tk
import threading
from numpy import matrix
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

root = Tk()
#popup = Toplevel()
home = 0
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

    def __init__(self, freq= 125):
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
        #self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
        self.pub = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)
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
        rospy.Subscriber("/ethdaq_data_raw", WrenchStamped, self.wrenchSensorCallback)
        rospy.Subscriber("/ethdaq_data", WrenchStamped, self.wrenchSensorCallback)

        
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



    def impedance_control(self, desired_pose, k, c = 80, force_scaling=0.3):
        c=float(c)
        k=float(k)
        self.mode = 'impedance controller'
        quaternion = self.getTaskQuaternion()
        actual_position = self.getTaskPosi()
        desired_position_base = desired_pose[0]
        desired_quaternion = desired_pose[1]
        error_position = desired_position_base - actual_position

        # multiplication of two quaternions gives the rotations of doing the two rotations consecutive, 
        # could be seen as "adding" two rotations
        # multiplying the desired quaternion with the inverse of the current quaternion gives the error, 
        # could be seen as the "difference" between them
        error_angle = tf.transformations.quaternion_multiply(desired_quaternion, tf.transformations.quaternion_inverse(quaternion))
        error = np.append(error_position, error_angle[0:3]) #Why can we use only error_angle[0:3]?
        SelectionVector = np.array([1,2,2,2,2,2])
        error = error*SelectionVector

        # measured_force is in tool frame and must be transformed to base frame

        #print(np.linalg.norm(self.getWrench()*np.array([1,1,1,0,0,0]))/10000)
        velocity = np.append(np.array(self.getToolLinearVelocity()) ,np.array([0,0,0]))
        measured_force = self.reference_frame('tool', self.getWrench())
        force_array = np.array([0,0,0])

        measured = np.append(np.array(self.force_sensor), force_array)
	#print measured
	#measured_force = self.reference_frame('tool', self.getWrench())
        #print self.force_sensor
	kp = 2
        kd = 1
        #print(k)
        #control_law = kp*error + measured_force - kd*velocity
        #control_law = kp*error - kd*velocity
        #control_law = kp*error
        #control_law = k/c*error
	#print measured_force
        control_law = k/c*error + force_scaling/c*measured_force
        #control_law = force_scaling/c * measured_force
       	#print control_law
        #print("error ", error)
        #print("force ", measured_force)
        #testSelectionVector = np.array([0.5,1,0.5,1,1,1])
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

    def q_dot(self, dq_value, acceleration = 5, time = 0.05):
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
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
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
	global home
        #position = np.array([-0.12, -0.43, 0.26])
	position = np.array([-0.17239, -0.38559, 0.46098])
        #quaternion = np.array([-2.91e-04, 9.998e-01, 1.245e-02, 1.25e-02]) 
	
        #quaternion = np.array([3.49848602e-06,  9.99961671e-01, -3.99984690e-04,  8.74621458e-03])
        quaternion = np.array([0.68450141, -0.07245186,  0.15660954,  0.70829514])
	#--Real robot--  
        #position = np.array([0.20864163268953798, 0.2901168672870089, 0.7255072413646502])
        #quaternion = np.array([0.95663979,-0.29068509,-0.01717201,-0.00689979])

        #Start-pose for rowing exercise
        #position = np.array([0.15756176236973007, 0.3590548461660456, 0.41006310959573894])
        #quaternion = np.array([-0.70700723, -0.00474353, 0.00277345, 0.7071849])
    
        #New start-pose(start-pose 2), only testing, not compatile with safety limits (yet)
        #position = np.array([0.3988074665985663, -0.2106094069148962, 0.39359153019942167])
        #quaternion = np.array([0.4993599, 0.50327672, 0.50019774, 0.49714627])
	
        
        roll = 1.5155    
        pitch = -0.3226
        yaw = 0.1297

        rospy.logwarn("ROLL %f",roll)
        rospy.logwarn("PITCH %f", pitch)
        rospy.logwarn("YAW %f", yaw)

        newQut = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        rospy.loginfo(newQut)

        
        #print Jac_psudo
	
        start_pose = np.array([position, quaternion])
        while (not rospy.is_shutdown()): 
	
            # Publishing, for being able to plot later
            self.pub_x.publish(self.getTaskPosi()[0]-start_pose[0][0])
            self.pub_y.publish(self.getTaskPosi()[1]-start_pose[0][1])
            self.pub_z.publish(self.getTaskPosi()[2]-start_pose[0][2])
            

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
            atHome = self.proximityCheck(start_pose, 0.03)
            #print(run_robot)
            #print(atHome)

            controller = self.impedance_control(pose,res)
            #controller = self.pose_control(pose)
            #print controller
            """
            if(guide_z and atHome):
                #controller = self.guide_z_control()
		controller = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
                position = np.array([0.15756176236973007, 0.3590548461660456, self.getTaskPosi()[2]])
                quaternion = np.array([-0.70700723, -0.00474353, 0.00277345, 0.7071849])
                start_pose = np.array([position,quaternion])
	        #print('zguide')
            else:
                #print('impedance')
                controller = self.impedance_control(pose,res)
            """
                

            #######################################
            ############ Safety Limits ############
            #######################################    
            # ---Limits for rowing exercise--- 
            # Limits movement in z-direction, "floor" at z=0.192 and "roof" at z=0.725
            if (self.getTaskPosi()[2] <= 0.192 and controller[2]<0):
                controller[2] = 0 
            if (self.getTaskPosi()[2] >= 0.725 and controller[2]>0):
                controller[2] = 0 

            # Limits the movement in x-direction by 20 cenitmeters in each direction
            # from start position
            if (self.getTaskPosi()[0] <= (position[0]-0.2) and controller[0]<0):
                controller[0]=0
            if(self.getTaskPosi()[0] >= (position[0]    )+0.2 and controller[0]>0):
                controller[0] = 0 

            #Limits the movement in y-direction by 5 centimeters behind the start position 
            if(self.getTaskPosi()[1] <= (position[1]-0.05) and controller[1]<0):
                controller[1] = 0       

            # Limits the rotation around the x-axis
            #+- 0.3 rad
            if(self.getTaskEuler()[0] <= -1.872 and not controller[3]>0):
                controller[3] = 0

            if(self.getTaskEuler()[0] >= -1.272 and controller[3]>0):
                controller[3] = 0

            # Limits the rotation around z-axis
            # +- 0.3 rad 
            if(self.getTaskEuler()[2] >= 0.311 and controller[5]>0):
                controller[5] = 0
            if(self.getTaskEuler()[2] <= -0.289 and controller[5]<0):
                controller[5] = 0           
	    V_ref = matrix([[0],[0],[0],[0],[0],[0]])
	    Jac_psudo = self.jac_function()
	    print Jac_psudo
            #dq = np.matmul(Jac_psudo, np.transpose(controller))
            
	    
	    #q = Jac_psudo*matrix ( [[controller[0]],[controller[1]],[controller[2]],[controller[3]],[controller[4]],[controller[5]]] )
            
	    #dq_value = np.array([-6.1784, -1.57, -2.35619, -2.3212, -1.56, -0.01014])
            #print self.q
	    if( self.q[0] < -1.3 and self.q[1] < -1.3 and self.q[2] < -1.5 and self.q[3] < -1.1 and self.q[4] > 1.5):

	    	#dq = Jac_psudo*V_ref
		#self.mode = 'free drive mode'
                home = 1
		#print("hej")
	    if home == 1:
		#controller = np.array([0.0,-0.01,0.0,0.0,0.0,0.0])
                #position = np.array([-0.1346, -0.93141, 0.05459])
                #quaternion = np.array([-0.04883505, 0.68830866, -0.05110866, -0.72196554])
                #start_pose = np.array([position, quaternion])
		print("hej")
            #print(np.linalg.norm(self.getWrench()*np.array([1,1,1,0,0,0]))/10000)
            
	    #dq = Jac_psudo*V_ref
	    dq = np.matmul(Jac_psudo, np.transpose(controller))
	    dq_value = np.asarray(dq).reshape(-1)
	    #print dq_value
	    if(run_robot):
            	self.q_dot(dq_value)
		
		#self.velocity_cmd(controller)
            
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
