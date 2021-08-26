cmake_minimum_required(VERSION 2.8.3)
project(rmui_msgs)

find_package(catkin
  REQUIRED
  actionlib
  actionlib_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  LED.msg
  ImuCalibStatus.msg
)

add_action_files(
  DIRECTORY action
  FILES ExecuteStatenet.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime 
)
