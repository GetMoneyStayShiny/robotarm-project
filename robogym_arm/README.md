In the run function;
  We have Start-pose for [home/rowing.....]
  in the form of position(x,y,z) and quaternion(x,y,z,w)
  init start pose
  
  while not rospy.shutdown()
     we get the x from;
	getTaskPosi --> transMat("The 2 frames") 
			In this case we are using the 1st branch from
			tf graph, which gives us the tf from base 
			to tool0_control. 

			This function basically listen to tf service (ROS service) 
                        tf --> This is a ros library (http://wiki.ros.org/tf) basically ehat this does 
                        is gives us the connection between each link. Remeber how we construct the 
		        trasformation matrix for forward kinematics. 
			
			So this service will give us the Rotational matrix (R) and the Traslational matrix (T)
                        for given 2 frames. (In the old code we only care about the base frame and the 
			end tool frame, since they only control the spped of the tool -- speedl command)
                        but (I guess) for us we have to get these trasformation mats for each frame from base to tool 

			We can of course the the euler and then the quaternions (we have to command using quaternion)
			So the function transMat will return 4 values (vectors/matrix) and for this getTaskPosi we
			extract the position. Then we can calculate position diffrence (I assume this will be 3x1
			vector not sure need to see)                        
			
			Then we publish this value --> According to thwm its for the purpose of ploting the data

  
    Then we have,
	forceVar --> StringVar() (A class defined in the Tkinter lib -- not related to our case but basically we access data from the GUI)
	force_now --> This is value that is shown by the above variable. 	
        getWrench() --> currently returning a vector of zeros (1x6)  but, they do have a variable called wrench which is given by the 
                        expression below,  
                        wrench = [(force_sensor, torque_sensor)] - self.force_offset

        force_sensor --> We have a callback named, wrenchSensorCallback() --> 
                         msg type --> geometry_msgs/WrenchStamped (http://docs.ros.org/melodic/api/geometry_msgs/html/msg/WrenchStamped.html)
                         I assume this is the data coming from the foce sensor


        torque_sensor --> The same sensor msg also include the value of this as well... 
       
        force_offset --> I think this could be the initial force value. Because in the initial stage they set that to the 
                         force_sensor & torque_sensor values and then I dont see its being updated anywhere...  
	
        force_now = int(np.linalg.norm(self.getWrench() * np.array([1, 1, 1, 0, 0, 0])) / 10000) --> This will return a single value but I dont
		    understand what is this.... 

