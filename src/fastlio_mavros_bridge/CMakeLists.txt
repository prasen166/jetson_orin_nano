cmake_minimum_required(VERSION 3.0.2)
project(fastlio_mavros_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  tf
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs mavros_msgs tf tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/fastlio_to_mavros_improved.py
  scripts/slam_safety_monitor_improved.py
  scripts/fastlio_diagnostics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
