cmake_minimum_required(VERSION 2.8.3)
project(mav_msgs)

find_package(catkin REQUIRED message_generation std_msgs dynamic_reconfigure)

add_message_files(
  FILES
  CommandAttitudeThrust.msg
  CommandMotorSpeed.msg
  CommandRateThrust.msg
  CommandTrajectory.msg
  MotorSpeed.msg
  PIDController.msg
  SSController.msg
)

generate_messages(DEPENDENCIES std_msgs)


generate_dynamic_reconfigure_options(
cfg/MavCtlParams.cfg
cfg/MavZCtlParams.cfg
cfg/MavXYCtlParams.cfg
cfg/MavAttitudeCtlParams.cfg
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

