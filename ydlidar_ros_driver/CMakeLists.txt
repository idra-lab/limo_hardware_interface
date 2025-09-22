# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 2.8)

project(ydlidar_ros_driver)



############## YDLIDAR SDK END#####################################

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  	message_generation
	roscpp
	rospy
	sensor_msgs
	)
 
 
catkin_package(CATKIN_DEPENDS
	LIBRARIES ydlidar_ros_driver ydlidar_sdk
	roscpp rospy sensor_msgs message_runtime 
	)
#---------------------------------------------------------------------------------------
# Set default build to release
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

#---------------------------------------------------------------------------------------
# Compiler config
#---------------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(YDLIDAR_SDK_LIBRARIES ydlidar_sdk)

#Include directories
include_directories(${catkin_INCLUDE_DIRS} 
		    ${YDLIDAR_SDK_INCLUDE_DIRS}   
		    ${CMAKE_CURRENT_SOURCE_DIR}/sdk            # SDK root (so <core/base/utils.h>, src/CYdLidar.h work)
  		    ${CMAKE_CURRENT_BINARY_DIR}/sdk            # for generated ydlidar_config.h
  		    )

#link library directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

# ---- Add SDK ----
add_subdirectory(sdk)

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/ydlidar_ros_driver.cpp)

# -----------------------------------------------------------------------
# link libraries
#---------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}_node
	${catkin_LIBRARIES}
	${YDLIDAR_SDK_LIBRARIES})


#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

