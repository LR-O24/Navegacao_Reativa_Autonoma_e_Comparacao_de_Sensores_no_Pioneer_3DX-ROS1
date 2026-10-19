cmake_minimum_required(VERSION 2.8.3)
project(pioneer_reactive_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package()

install(PROGRAMS
  scripts/reactive_avoider.py
  scripts/analyze_runs.py
  scripts/static_noise.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
