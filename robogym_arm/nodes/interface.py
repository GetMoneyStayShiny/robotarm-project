#!/usr/bin/env python2.7
import rospy
import numpy as np
from threading import Thread, Lock
from Tkinter import *
from PIL import ImageTk, Image
from robogym_arm.msg import InterfaceStamped, RobotStamped

offset = 0
force_now = 0
guide_z = False
red = 30
green = 35
blue = 191

exercise = [
    ("Horizontal",1),
    ("Vertical",2),
    ("BicepCurl",3)
]

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
exercise_string_forex2 = """
Exercise string for exercise 2 
"""

# threading.Thread
class Interface():
    def __init__(self):
        self.root = Tk()
        self.mutex = Lock()

        rospy.init_node('RoboGymInterface')
        rospy.Subscriber('/robogym/robot_interface_status',RobotStamped,self.robot_status_callback)
        self.interface_cmd = rospy.Publisher('/robogym/interface_cmd',InterfaceStamped,queue_size=1)

        self.onoff_light = StringVar()
        self.forceVar = StringVar()
        self.onoff_light.set("red")
        self.str_res = StringVar()
        self.atHome = False
        self.run_robot = False
        self.resistance = 50
        self.exercise_val = IntVar()
        self.exercise_val.set(1)

        force_plot = np.zeros(10)

        self.rate = rospy.Rate(1)

        ros_thread = Thread(target=self.ros_loop)
        ros_thread.start()
        self.initilize_gui()

    def ros_loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def initilize_gui(self):
        print "In the inti gui"
        self.root.title("RobotGym")
        frame = Frame(self.root)
        Grid.rowconfigure(self.root, 0, weight=1)
        Grid.columnconfigure(self.root, 0, weight=1)
        frame.grid(row=0, column=0, sticky=N + S + E + W)
        grid = Frame(frame)
        grid.grid(sticky=N + S + E + W, column=0, row=0, columnspan=2)
        Grid.rowconfigure(frame, 0, weight=1)
        Grid.columnconfigure(frame, 0, weight=1)

        for x in range(80):
            Grid.columnconfigure(frame, x, weight=1)

        for y in range(40):
            Grid.rowconfigure(frame, y, weight=1)

        # Unlock button
        self.start_button = Button(frame, text="Unlock", command=self.starter)
        self.start_button.grid(row=10, column=30, padx=2, pady=2, sticky=N + S + E + W)

        # Z-Position button
        self.set_position_button = Button(frame, text="Set height", command=self.poition_set)
        self.set_position_button.grid(row=12, column=30, padx=2, pady=2, sticky=N + S + E + W)

        # Resistance button
        self.get_weight = Button(frame, text="Set resistance", command=self.getWeight)
        self.get_weight.grid(row=14, column=30, padx=2, pady=2, sticky=N + S + E + W)

        # Lock button
        self.stop_button = Button(frame, text="Lock", command=self.stop)
        self.stop_button.grid(row=16, column=30, padx=2, pady=2, sticky=N + S + E + W)

        # Unit label
        self.slide_label = Label(frame, text="Resistance (N/m)", font=("Helvetica", 12))
        self.slide_label.grid(row=39, column=30, sticky=E)

        # Slidebar
        self.slideBar = Scale(frame, from_=125, to=25, length=400, width=50, tickinterval=25)
        self.slideBar.grid(row=30, rowspan=9, column=30, sticky=W)
        self.slideBar.set(50)

        # Current resistance label
        self.current_resistans = Label(frame, textvariable=self.str_res, font=("Helvetica", 12))
        self.current_resistans.grid(row=26, rowspan=4, column=30)

        # Text widget explaning interface
        text_button = Text(frame, height=23, width=64)
        text_button.tag_configure('bold_italics', font=('Arial', 12, 'bold', 'italic'))
        text_button.tag_configure('big', font=('Verdana', 12, 'bold'))
        text_button.tag_configure('color', foreground='#476042', font=('Tempus Sans ITC', 12, 'bold'))

        text_button.insert(END, 'Unlock', 'big')
        text_button.insert(END, start_string)

        text_button.insert(END, '\nSet height', 'big')
        text_button.insert(END, set_pos_string)

        text_button.insert(END, '\nSet resistance', 'big')
        text_button.insert(END, set_res_string)

        text_button.insert(END, '\nLock', 'big')
        text_button.insert(END, stop_string)

        text_button.insert(END, '\nShutdown', 'big')
        text_button.insert(END, s_down_string)

        text_button.config(state=DISABLED)
        text_button.grid(row=10, rowspan=22, column=50, columnspan=10, padx=6, pady=6, sticky=N)

        exersice_details = Label(frame, text="Choose your exercise:")
        # exersice_details.config(height=5, width=25)
        exersice_details.configure(font=('Verdana', 12, 'bold'))
        exersice_details.grid(row=10, column=66, pady=2, sticky=W)

        for index,language in enumerate(exercise):
            exercise_select =  Radiobutton(frame,
                               text=language,
                               variable=self.exercise_val,
                               command=self.ShowChoice,
                               value=index+1)
            if index == 0:
                exercise_select.grid(row=12, column=65, padx=2,sticky=W)
            elif index == 1:
                exercise_select.grid(row=12, column=75, padx=2, sticky=W)
            elif index == 2:
                exercise_select.grid(row=32, column=65, padx=2, sticky=W)

        # Exercise image
        orgrinal_ex1 = Image.open("ex1_s.jpg")
        resized_ex1 = orgrinal_ex1.resize((320, 280), Image.ANTIALIAS)
        self.photo_ex1 = ImageTk.PhotoImage(resized_ex1)
        ex1_image = Text(frame, height=20, width=40)
        ex1_image.insert(END, '\n')
        ex1_image.image_create(END, image=self.photo_ex1)
        ex1_image.config(state=DISABLED)
        ex1_image.grid(row=13, rowspan=20, column=65, columnspan=10, sticky=N)

        orgrinal_ex2 = Image.open("ex2_s.jpg")
        resized_ex2 = orgrinal_ex2.resize((320, 280), Image.ANTIALIAS)
        self.photo_ex2 = ImageTk.PhotoImage(resized_ex2)
        ex2_image = Text(frame, height=20, width=40)
        ex2_image.insert(END, '\n')
        ex2_image.image_create(END, image=self.photo_ex2)
        ex2_image.config(state=DISABLED)
        ex2_image.grid(row=13, rowspan=20, column=75, columnspan=10, sticky=N)

        orgrinal_ex3 = Image.open("ex3_s.jpg")
        resized_ex3 = orgrinal_ex3.resize((320, 280), Image.ANTIALIAS)
        self.photo_ex3 = ImageTk.PhotoImage(resized_ex3)
        ex3_image = Text(frame, height=20, width=40)
        ex3_image.insert(END, '\n')
        ex3_image.image_create(END, image=self.photo_ex3)
        ex3_image.config(state=DISABLED)
        ex3_image.grid(row=33, rowspan=20, column=65, columnspan=10, sticky=N)

        # Test widget explaning exercise
        self.text_exercise = Text(frame, height=10, width=40)
        self.text_exercise.tag_configure('bold_italics', font=('Arial', 12, 'bold', 'italic'))
        self.text_exercise.tag_configure('big', font=('Verdana', 12, 'bold'))
        self.text_exercise.tag_configure('color', foreground='#476042', font=('Tempus Sans ITC', 12, 'bold'))

        self.text_exercise.tag_bind('follow', '<1>', lambda e, t=self.text_exercise: t.insert(END, "Not now, maybe later!"))

        self.text_exercise.insert(END, exercise_string)
        self.text_exercise.config(state=DISABLED)
        self.text_exercise.grid(row=33, rowspan=25, column=75, columnspan=10, padx=6, pady=6, sticky=N)

        # Force display label
        self.force = Label(frame, textvariable=self.forceVar, font=("Helvetica", 100),
                           fg="#%02x%02x%02x" % (red, green, blue), borderwidth=3, relief="ridge")
        self.force.config(height=1, width=5)
        self.force.grid(row=33, rowspan=4, column=52, columnspan=4, pady=4, padx=4)

        # Force display unit
        self.slide_label = Label(frame, text="Force (N)", font=("Helvetica", 12))
        self.slide_label.grid(row=32, column=53, sticky=S + E)

        # Shutdown button
        self.end_button = Button(frame, text="Shutdown", command=self.end)
        self.end_button.grid(row=39, column=79, padx=8, pady=2, sticky=W)

        # ON/Off label
        self.onoff = Label(frame, bg=self.onoff_light.get())
        self.onoff.config(height=3, width=5)
        self.onoff.grid(row=18, column=30, pady=2, sticky=E)

        self.onoff_status = Label(frame, text="Robot active:")
        self.onoff.config(height=3, width=5)
        self.onoff_status.grid(row=18, column=30, pady=2, sticky=W)

        '''
        #at home label
        self.at_home = Label(frame, bg=at_home_light.get())
        self.at_home.config(height = 3, width=5)
        self.at_home.grid(row=21, column=30,pady = 2, columnspan= 2, sticky = E)
        self.at_home_status = Label(frame, text = "Home position:")
        self.at_home_status.config(height = 3, width=5)
        self.at_home_status.grid(row = 21, column = 30, pady = 2, columnspan=2, sticky = W)
        '''

        global w, h
        # Till for samus dator
        # w,h = self.root.winfo_screenwidth(), self.root.winfo_screenwidth()
        w = 1366
        h = 1366

        # root.minsize(width=w/2, height=h/2)
        # root.maxsize(width=w, height=h)
        self.root.geometry(str(w) + 'x' + str(h))
        # self.root.geometry( '500x300')
        self.root.mainloop()

    def ShowChoice(self):
        interface_msg = InterfaceStamped()
        interface_msg.interface.exercise_type = self.exercise_val.get()
        self.interface_cmd.publish(interface_msg)

    def robot_status_callback(self,data):
        self.mutex.acquire()
        _var = data
        self.mutex.release()
        self.forceVar = data.robot.force_str
        self.atHome = data.robot.at_home
        self.onoff_light.set(data.robot.onoff_light)
        self.onoff.config(bg=self.onoff_light.get())

    def guide_z_false(self):
        global guide_z
        guide_z = False

    def guide_z_true(self):
        global guide_z
        guide_z = True

    def getWeight(self):
        if (self.atHome):
            self.resistance = self.slideBar.get()
            self.str_res.set("Current resistance: " + str(self.resistance))
            interface_msg = InterfaceStamped()
            interface_msg.interface.resistance = self.resistance
            interface_msg.interface.exercise_type = 0
            self.interface_cmd.publish(interface_msg)
        else:
            popup = Toplevel()
            # self.root.withdraw()
            popup.grab_set()
            popup.title("Error, robot must be in home position")
            popup.geometry(str(w / 2) + 'x' + str(h / 10))
            popup.geometry("+300+400")
            # popup.geometry("+d%")
            explanation = """Wait for the robot to return to its home position and try again."""
            popw2 = Label(popup, justify=LEFT, padx=200, pady=25, height=3, width=15, text=explanation).pack()

        def popup_done():
            popup.destroy()
            # self.root.deiconify()
            popup.grab_release()

        self.B1 = Button(popup, text="Done", command=popup_done).pack()

    def starter(self):
        self.run_robot = True
        self.onoff_light.set("green")
        self.onoff.config(bg=self.onoff_light.get())
        interface_msg = InterfaceStamped()
        interface_msg.interface.resistance = self.resistance
        interface_msg.interface.exercise_type = 0
        interface_msg.interface.robot_ready = self.run_robot
        self.interface_cmd.publish(interface_msg)


    def stop(self):
        if (self.atHome):
            self.run_robot = False
            self.onoff_light.set("red")
            self.onoff.config(bg=self.onoff_light.get())

            interface_msg = InterfaceStamped()
            interface_msg.interface.resistance = self.resistance
            interface_msg.interface.exercise_type = 0
            interface_msg.interface.robot_ready = self.run_robot
            self.interface_cmd.publish(interface_msg)

    def end(self):
        self.root.destroy()
        self.run_robot = False
        interface_msg = InterfaceStamped()
        interface_msg.interface.resistance = self.resistance
        interface_msg.interface.exercise_type = 0
        interface_msg.interface.robot_ready = self.run_robot
        self.interface_cmd.publish(interface_msg)
        rospy.is_shutdown()

    def get(self):
        global offset
        offset = float(z.get())
        print(offset)

    def rgb_to_hex(rgb):
        return "#%02x%02x%02x" % rgb

    def poition_set(self):
        if (self.atHome):
            popup = Toplevel()
            popup.grab_set()
            global guide_z
            guide_z = True
            # self.root.withdraw()

            popup.title("Set you start position")
            popup.geometry(str(w / 2) + 'x' + str(h / 10))
            popup.geometry("+300+400")
            explanation = """Move the robot vertically to find your prefered position"""
            popw2 = Label(popup, justify=LEFT, padx=200, pady=25, height=3, width=15, text=explanation).pack()
        else:
            popup = Toplevel()
            popup.grab_set()
            # self.root.withdraw()
            popup.title("Error, robot must be in home position")
            popup.geometry(str(w / 2) + 'x' + str(h / 10))
            popup.geometry("+300+400")
            explanation = """Wait for the robot to return to its home position and try again."""
            popw2 = Label(popup, justify=LEFT, padx=200, pady=25, height=3, width=15, text=explanation).pack()

        def popup_done():
            global guide_z
            guide_z = False
            popup.destroy()
            popup.grab_release()

        self.B1 = Button(popup, text="Done", command=popup_done).pack()

if __name__ == '__main__':
    Interface()
