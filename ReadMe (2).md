ReadMe: Project Update 2/3
Code and steps used for this video/project update:

1) Launch Gazebo

Terminal 1:

	$ catkin_make   							/*in catkin directory */
	$ source devel/setup.bash
	$ roslaunch ur3_driver ur3_gazebo.launch  	/*launches the UR3 in Gazebo*/ 
	


2) Inverse + Forward Kinematics to Pick and Place "Can" from User Specified Location


Terminal 2:

	$ rosrun lab2pkg_py lab2_spawn.py --simulator True			/*Spawns 3 colored blocks, into a tower location specivified by the user */	
	
	User input: 
	which tower location should the blocks spawn (1, 2, or 3)		

	$ rosrun lab2pkg_py Upd2_lab2_exec.py --simulator True		/*runs the code to move the block based on user input below*/
