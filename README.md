# RG
ReadMe: Project Update 1
Code and steps used for this video/project update:

1) Launch Gazebo

Terminal 1:
	$ catkin_make   							/*in catkin directory */
	$ source devel/setup.bash
	$roslaunch ur3_driver ur3_gazebo.launch  				/*launches the UR3 in Gazebo*/ 
	


2) Tower of Hanoi move + accessing suction gripper input data:

Terminal 3:
	$ source devel/setup.bash 						/*in catkin directory */
	$ rostopic list 							/*lsit of messages used to identify the " /ur3/gripper_input " message. /*
	$ rostopic echo /ur3/gripper_input 					/*continously streams value of gripper sensor data */
										/* $ rostopic echo/ur3/gripper_input -n 1     :shows only a snapshot of the same sensor data*/
Terminal 2:
	$ rosrun lab2pkg_py lab2_spawn.py --simlator True			/*Spawns 3 colored blocks, into a tower location specivified by the user */	
	
	User input: 
	which tower location should the blocks spawn (1, 2, or 3)		

	$ rosrun lab2pkg_py lab2_exec.py --simlator True			/*runs the code to move the block based on user input below*/

	User input:
	Enter either 1, 2, or 3 to define the location of starting tower.
	Enter 0 to quit.
	Enter one of the two remaining tower locations as the end tower.



---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

If required for testing (not shown in video):

3) Move arm:  /*uses forward kinematics from lab3 code to move the arm based on 6 joint angles from user input*/

Terminal 2:
	$ source devel/setup.bash 						/*in catkin directory */
	$ rosrun lab3pkg_py lab3_exec.py 0 -90 0 0 0 90 --simulator True 	/*these angles move the robot to be vertically straight up, up to the end effector. --simulator True dictates that this is running on the simulator, gazebo. */
