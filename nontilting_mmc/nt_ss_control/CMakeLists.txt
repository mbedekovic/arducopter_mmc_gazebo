cmake_minimum_required(VERSION 2.8.3)
project(nt_ss_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

#add_executable(attitude_ss_ctl_node nodes/attitude_ctl_node.cpp src/AttitudeControlSS.cpp src/AttitudeFeedForward.cpp)

add_executable(quad_ss_ctl_node nodes/quad_attitude_ctl_node.cpp src/AttitudeControlSS.cpp src/AttitudeFeedForward.cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(attitude_ss_ctl_node
#  ${catkin_LIBRARIES}
#)

target_link_libraries(quad_ss_ctl_node
  ${catkin_LIBRARIES}
)
