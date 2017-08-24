cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)

find_package(catkin REQUIRED message_generation std_msgs dynamic_reconfigure)

add_message_files(
  FILES
  PIDController.msg
  KalmanFilter.msg
  HSIAController.msg
  SSController.msg
)

add_service_files(
    FILES
    StandStill.srv
)

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(
  cfg/MavCtlParams.cfg
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

