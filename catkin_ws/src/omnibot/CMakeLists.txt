cmake_minimum_required(VERSION 3.0.2)
project(omnibot)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  roscpp
  std_msgs
  tf
  tf2
  message_generation
)
add_message_files(
  FILES
  MotorSpeed.msg
  MotorEncoder.msg
)


catkin_python_setup()

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
   nodes/node_odometry.py
   nodes/node_robot_mock.py
   nodes/node_teleop.py
   nodes/node_play.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
