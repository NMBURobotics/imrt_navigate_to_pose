cmake_minimum_required(VERSION 3.0.2)
project(imrt_navigate_to_pose)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  message_runtime
  rospy
  std_msgs
)

catkin_python_setup()

# Generate actions in the 'action' folder
 add_action_files(
   FILES
   NavigateToPose.action
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   geometry_msgs 
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imrt_navigate_to_pose
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation message_runtime rospy std_msgs
#  DEPENDS system_lib
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 catkin_install_python(PROGRAMS
   nodes/action_server
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 catkin_install_python(PROGRAMS
 nodes/action_client
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
