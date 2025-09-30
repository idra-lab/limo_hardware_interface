cmake_minimum_required(VERSION 3.0.2)
project(optitrack_interface)

find_package(catkin REQUIRED nav_msgs geometry_msgs)




catkin_package(
  CATKIN_DEPENDS rospy  nav_msgs geometry_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/nat_net_client.py
    scripts/optitrack.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
  

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  PATTERN ".svn" EXCLUDE)


