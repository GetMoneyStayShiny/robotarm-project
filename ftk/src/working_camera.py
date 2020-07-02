#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import atracsys.ftk as tracker_sdk

import platform
if platform.system() == 'Darwin':
    import matplotlib
    matplotlib.use("TkAgg")

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import os
from collections import deque
import numpy as np

x = 0
fig, ax = plt.subplots(2, 2)
ax[-1, -1].axis('off')
data = deque([(x, (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0))], maxlen=50)
max_number_of_tracked_markers = 4
axis_x_lines = []
axis_y_lines = []
axis_z_lines = []
markers_indexes = {}
current_marker_index = 1



def talker():
    ch2 = rospy.Publisher('/geom2', String, queue_size=10)
    ch4 = rospy.Publisher('/geom4', String, queue_size=10)
    ch9 = rospy.Publisher('/geom9', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    

    def exit_with_error(error, tracking_system):
        print(error)
        errors_dict = {}
        if tracking_system.get_last_error(errors_dict) == tracker_sdk.Status.Ok:
            for level in ['errors', 'warnings', 'messages']:
                if level in errors_dict:
                    print(errors_dict[level])
        exit(1)

    tracking_system = tracker_sdk.TrackingSystem()

    if tracking_system.initialise() != tracker_sdk.Status.Ok:
        exit_with_error(
            "Error, can't initialise the atracsys SDK api.", tracking_system)

    if tracking_system.enumerate_devices() != tracker_sdk.Status.Ok:
        exit_with_error("Error, can't enumerate devices.", tracking_system)

    frame = tracker_sdk.FrameData()

    if tracking_system.create_frame(False, 10, 20, 20, 10) != tracker_sdk.Status.Ok:
        exit_with_error("Error, can't create frame object.", tracking_system)

    print("Tracker with serial ID {0} detected".format(
        hex(tracking_system.get_enumerated_devices()[0].serial_number)))

#geometry_path = tracking_system.get_data_option("Data Directory")

    geometry_path = '/home/robert/Documents/School/masterthesis/AtracsysFusionTrack250_Camera/OneDrive_1_3-9-2020/'




    for geometry in ['geometry2.ini', 'geometry4.ini', 'geometry9.ini']:
        if tracking_system.set_geometry(os.path.join(geometry_path, geometry)) != tracker_sdk.Status.Ok:
            exit_with_error("Error, can't create frame object.", tracking_system)

    global Tmatrix2, Tmatrix4, Tmatrix9
    Tmatrix2 = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
    Tmatrix4 = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
    Tmatrix9 = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
    
    while not rospy.is_shutdown():
        global x
        global markers_indexes
        global current_marker_index
        
        
     
        x += 1
        tracking_system.get_last_frame(frame)

        marker_data = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]
        
        for marker in frame.markers:
            if not marker.geometry_id in markers_indexes:
                markers_indexes[marker.geometry_id] = current_marker_index
                current_marker_index += 1
	        #print("data: ", marker)
                #print("type:", marker.type())

            if markers_indexes[marker.geometry_id] < max_number_of_tracked_markers:
                
                if(marker.geometry_id == 2):
                    Tmatrix2 = np.array([[marker.rotation[0][0], marker.rotation[0][1], marker.rotation[0][2], marker.position[0]], [marker.rotation[1][0], marker.rotation[1][1], marker.rotation[1][2], marker.position[1]], [marker.rotation[2][0], marker.rotation[2][1], marker.rotation[2][2], marker.position[2]], [0,0,0,1]])                     
                 

                if(marker.geometry_id == 4):            
                    Tmatrix4 = np.array([[marker.rotation[0][0], marker.rotation[0][1], marker.rotation[0][2], marker.position[0]], [marker.rotation[1][0], marker.rotation[1][1], marker.rotation[1][2], marker.position[1]], [marker.rotation[2][0], marker.rotation[2][1], marker.rotation[2][2], marker.position[2]], [0,0,0,1]])
                   

                if(marker.geometry_id == 9):
                    Tmatrix9 = np.array([[marker.rotation[0][0], marker.rotation[0][1], marker.rotation[0][2], marker.position[0]], [marker.rotation[1][0], marker.rotation[1][1], marker.rotation[1][2], marker.position[1]], [marker.rotation[2][0], marker.rotation[2][1], marker.rotation[2][2], marker.position[2]], [0,0,0,1]])
                    
   
        print(Tmatrix2)
        print(Tmatrix4)
        print(Tmatrix9)
        rosFormatMatrix2 = np.array2string(Tmatrix2, precision=20, separator=',', suppress_small=True)
        rosFormatMatrix4 = np.array2string(Tmatrix4, precision=20, separator=',', suppress_small=True)
        rosFormatMatrix9 = np.array2string(Tmatrix9, precision=20, separator=',', suppress_small=True)
        #hello_str = rosFormatMatrix
        #rospy.loginfo(hello_str)
        ch2.publish(rosFormatMatrix2)
        ch4.publish(rosFormatMatrix4)
        ch9.publish(rosFormatMatrix9)
        
        #rospy.Publisher('/cameraData', hello_str, queue_size=1)

        #print(hello_str)
        rate.sleep()







if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
