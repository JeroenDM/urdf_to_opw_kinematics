cmake_minimum_required(VERSION 2.8.3)
project(urdf_to_opw_kinematics)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  urdfdom_py
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES urdf_to_opw_kinematics
#  CATKIN_DEPENDS rospy urdfdom_py
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_rostest(test/test_robot1.launch)
  add_rostest(test/test_irb2400.launch)
  add_rostest(test/test_kr6r700.launch)
endif()