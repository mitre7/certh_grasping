cmake_minimum_required(VERSION 2.8.3)
project(certh_grasping)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    moveit_visual_tools
    robot_helpers	
    cvx_util
    clopema_robot
    camera_helpers
    spring_detector
)


catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        certh_grasping_lib
    CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning_interface
        moveit_msgs
        moveit_visual_tools
	cvx_util
	clopema_robot
	robot_helpers
        spring_detector
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

ADD_DEFINITIONS( -std=c++11 )

add_executable(test_grasping test/test_grasping.cpp src/certh_grasping.cpp)
target_link_libraries(test_grasping ${catkin_LIBRARIES})

