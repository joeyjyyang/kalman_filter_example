cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter_example)

find_package(catkin)

catkin_package()

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
   PROGRAMS scripts/kalman_filter_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)