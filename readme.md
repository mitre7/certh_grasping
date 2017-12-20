Build Requirements:
	1) Boost
	2) Eigen
	3) robot_helpers
	4) cvx_util
	5) clopema_robot
	6) camera_helpers
	7) spring_detector
	8) push_debris

This package contains functions that give all the tranformations from spring coordinate system to both camera and world coordinate system respectively. Importantly, this is developed based on Certh testbed configuration.


---------------------------------------------------------------------------------------------------


-- test_grasping: is a ros node that calls spring_detection service and moves the arm to grasp the detected springs. Attention: It's important for the end effector frame to be on the static gripper finger and not in the centrer point between the two fingers(while it's opened). 

In order to run it, type:
	- roslaunch camera_helpers gphoto2.launch
	- rosrun spring_detector test_detector
	- rosrun certh_grasping test_grasping 


---------------------------------------------------------------------------------------------------


-- clean_debris_node: contains the complete pipeline for spring detection, cleaning the area around the spring and finally grasping it. Attention: It's important for the end effector frame to be in the centrer point between the two fingers(while it's opened). removeDebris function:

1) Get an rgb image from the dslr mounted on the ceiling.

2) Calls the spring_detection service in order to detect the springs.

3) Calls the push_debris service in order to find the push points and direction.

4) Move arm1 to the points defined by the push_debris agent.

5) If the spring is clean of swarf then move arm2 to grasp it. Otherwise, repeat the procedure above.

In order to run it, type:
	- roslaunch camera_helpers gphoto2.launch
	- rosrun spring_detector test_detector
	- rosrun certh_grasping clean_debris
