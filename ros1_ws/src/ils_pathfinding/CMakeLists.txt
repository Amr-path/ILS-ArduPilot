cmake_minimum_required(VERSION 3.0.2)
project(ils_pathfinding)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs nav_msgs mavros mavros_msgs)
catkin_package()
catkin_install_python(PROGRAMS scripts/drone_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
