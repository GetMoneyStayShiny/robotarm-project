**RoboGym Arm**

This ROS package will provide an interface and a controller for a collaborative robot. UR10 is a robot that was used in this project. The key point that should be noted is that the system is controlled by joint velocity control. The file structure of the package is given below. 

- launch : This directory includes all the launch files that will be used in this project
  * interface - To launch the iterface class
  * robot - To launch the robot class  
  * robogym - To launch the system. This will launch both the classes and the requred files to run system (like the ur_mordern_drive package)
- msg : The directory which includes the custom made messages for this project
  * Interface - Message which is published by the interface. This message includes interface values/status
  * InterfaceStamped - Time-stamped message of the Interface message.
  * Robot -  Message which is published by the robot class(RoboGym node). This message includes robot values/status.
  * RobotStamped -  Time-stamped message of the Robot message.
- nodes : Two python classes 
  * interface - Interface class. This defines the GUI
  * robot - This is the RoboGym node. This node is also know as the main controller of this project (Robot class) 
- resources : Image files for the GUI
- CMakeLists.Txt : CMake project configuration file
- package.xml : package.xml 
- README.md : readme file (this file)

**HOW TO RUN THIS PACKAGE**

This package supports both real-time robot minupulation and simulation robot control. If you wish to run the program in the real robot, you need the robot ip, which can be found in the robot intialize page in UR10. Once the value is found run,
- _roslaunch robogym_arm robogym.launch robot_ip:=<ROBOT_IP>_ (Please note that there is already a default value set for the robot ip, if you don't wish to specify every iteration, it is recommended to edit this value in the launch file itself)

If you wish to run in the simulation mode then follw the below given steps.
- Open up the UR_SIM. First go the correct directory where you installed your ur-sim software from UR.
    * cd ~/ursim-3.11.0.82155/ 
    * ./start-ursim.sh
- Install the ROS ackage _ur_robot_driver_ . Installation details can be found in \ref <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>
- _roslaunch ur_calibration calibration_correction.launch robot_ip:=localhost target_filename:="<FILE PATH>/ur10_calibration.yaml"_. This will create a calibration file in the specified directory (basically this is the tf tree). 
- _roslaunch robogym_arm robogym.launch is_sim:=true kinematics_config:=<CALIBRATION_FILE_PATH>_. Also kinematics_config is optional one could edit this value directly in the launch file.

**REFERENCE**
1. Robotics Operating System (ROS) \ref <https://www.ros.org/>
2. ROS-Industrial \ref <https://rosindustrial.org/>
3. URScript API : URScript manual for the UR10 robot control
4. ur_modern_driver \ref <https://github.com/ros-industrial/ur_modern_driver>
5. ur_robot_driver \ref <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>
6. EtherDAQ ROS driver \ref <https://github.com/OptoForce/etherdaq_ros>