cmake_minimum_required(VERSION 3.0.2)
project(piezosensorROS)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  message_runtime
)

add_message_files(
  FILES
  Piezosensor.msg
  Thresholds.msg
)

add_service_files(
  FILES
  Tare.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/ReadPiezo.py
  scripts/PlotPiezoData.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
