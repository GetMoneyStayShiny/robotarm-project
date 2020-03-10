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
    pub = rospy.Publisher('chatter', String, queue_size=10)
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


    def animate(i):
        global x
        global markers_indexes
        global current_marker_index
        global Tmatrix
        Tmatrix = []
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
            
                if marker.geometry_id == 2:
                    for a in range(len(marker.rotation)):
                        tmp = [marker.position[a]]
                        tmpmatrix = marker.rotation[a] + tmp
                        Tmatrix = Tmatrix + tmpmatrix
                    Tmatrix = Tmatrix + [0,0,0,1]
                    Tmatrix = np.reshape(Tmatrix, (4,4))
                    
                    print(Tmatrix)
                marker_data[markers_indexes[marker.geometry_id]] = (
                    marker.position[0], marker.position[1], marker.position[2])
		

        current_frame_data = (
            x, marker_data[0], marker_data[1], marker_data[2], marker_data[3])

        data.append(current_frame_data)
        ax[0, 0].relim()
        ax[0, 1].relim()
        ax[1, 0].relim()
        ax[0, 0].autoscale_view()
        ax[0, 1].autoscale_view()
        ax[1, 0].autoscale_view()
        for i in range(0, max_number_of_tracked_markers):
            axis_x_data = [data[x][0] for x in range(0, len(data))], [
                data[x][i+1][0] for x in range(0, len(data))]
            axis_x_lines[i].set_data(axis_x_data)
            axis_y_data = [data[x][0] for x in range(0, len(data))], [
                data[x][i+1][1] for x in range(0, len(data))]
            axis_y_lines[i].set_data(axis_y_data)
            axis_z_data = [data[x][0] for x in range(0, len(data))], [
                data[x][i+1][2] for x in range(0, len(data))]
            axis_z_lines[i].set_data(axis_z_data)



    for i in range(0, max_number_of_tracked_markers):
        axis_x_data = [data[x][0] for x in range(0, len(data))], [
            data[x][i+1][0] for x in range(0, len(data))]
        axis_x_lines.append(ax[0, 0].plot(axis_x_data)[0])
        axis_y_data = [data[x][0] for x in range(0, len(data))], [
            data[x][i+1][1] for x in range(0, len(data))]
        axis_y_lines.append(ax[0, 1].plot(axis_y_data)[0])
        axis_z_data = [data[x][0] for x in range(0, len(data))], [
            data[x][i+1][2] for x in range(0, len(data))]
        axis_z_lines.append(ax[1, 0].plot(axis_z_data)[0])

    ax[0, 0].title.set_text("Axis x")
    ax[0, 1].title.set_text("Axis y")
    ax[1, 0].title.set_text("Axis z")

    ax[0, 0].set_ylabel("Position [mm]")
    ax[0, 1].set_ylabel("Position [mm]")
    ax[1, 0].set_ylabel("Position [mm]")

    ani = animation.FuncAnimation(fig, animate, interval=10)

    plt.subplots_adjust(top=0.87, bottom=0.11, left=0.18, right=0.90, hspace=0.34, wspace=0.60)
    plt.suptitle("x/y/z positions of markers")
    plt.show()
    
    while not rospy.is_shutdown():
        rosFormatMatrix = np.array2string(Tmatrix, precision=20, separator=',', suppress_small=True)
        hello_str = rosFormatMatrix
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        #rospy.Publisher('/cameraData', hello_str, queue_size=1)

        print(rosFormatMatrix)
        rate.sleep()







if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
