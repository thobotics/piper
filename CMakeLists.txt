cmake_minimum_required(VERSION 2.8.3)
project(piper)

# version indicator
set(PIPER_VERSION_MAJOR 0)
set(PIPER_VERSION_MINOR 1)
set(PIPER_VERSION_PATCH 0)
set(PIPER_VERSION_STRING "${PIPER_VERSION_MAJOR}.${PIPER_VERSION_MINOR}.${PIPER_VERSION_PATCH}")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  control_msgs
  actionlib
  roslib
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  nav_msgs
  navfn
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   FieldGrid.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
)

# C++11 flags for GTSAM 4.0
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# find GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)

# find GPMP2
find_package(gpmp2 REQUIRED)
include_directories(${gpmp2_INCLUDE_DIR})
set(gpmp2_LIBRARIES gpmp2)

catkin_package(
	CATKIN_DEPENDS rospy message_runtime std_msgs nav_msgs
)

# Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})


################################################
##                    base                    ##
################################################
include_directories(piper/base/)

file(GLOB BASE_SRC "piper/base/*.cpp")
add_library(piperbase ${BASE_SRC})

target_link_libraries(piperbase
  ${catkin_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${gpmp2_LIBRARIES}
)

install(DIRECTORY piper/base/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# options for which interfaces need to be built and installed
option(BUILD_ALL_INTERFACE "whether to build all interfaces" OFF)
option(BUILD_GPMP2_INTERFACE "whether to build interface for batch GPMP2" OFF)
option(BUILD_STEAP_INTERFACE "whether to build interface for STEAP" OFF)
option(BUILD_TURTLEBOT_INTERFACE "whether to build interface for TURTLEBOT" OFF)

#######################################################
##                    mobilerobot                    ##
#######################################################
include_directories(piper/mobilerobot/)

file(GLOB MOBILEROBOT_SRC "piper/mobilerobot/*.cpp")
add_library(pipermobilerobot ${MOBILEROBOT_SRC})

target_link_libraries(pipermobilerobot
  ${catkin_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${gpmp2_LIBRARIES}
)

install(DIRECTORY piper/mobilerobot/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

################################################
##                    all                     ##
################################################
if(BUILD_ALL_INTERFACE)
  set(BUILD_GPMP2_INTERFACE ON)
  set(BUILD_STEAP_INTERFACE ON)
  set(BUILD_TURTLEBOT_INTERFACE ON)
endif()


################################################
##              gpmp2_interface               ##
################################################
if(BUILD_GPMP2_INTERFACE)
  include_directories(piper/gpmp2_interface/)

  file(GLOB GPMP2_INTERFACE_SRC "piper/gpmp2_interface/*.cpp")
  add_executable(gpmp2_interface ${GPMP2_INTERFACE_SRC})

  target_link_libraries(gpmp2_interface piperbase)

  install(TARGETS gpmp2_interface
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY piper/gpmp2_interface/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
  )
endif()


################################################
##              steap_interface               ##
################################################
if(BUILD_STEAP_INTERFACE)
  include_directories(piper/steap_interface/)

  file(GLOB STEAP_INTERFACE_SRC "piper/steap_interface/*.cpp")
  add_executable(steap_interface ${STEAP_INTERFACE_SRC})

  target_link_libraries(steap_interface piperbase)

  install(TARGETS steap_interface
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY piper/steap_interface/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
  )
endif()

####################################################
##              turtlebot_interface               ##
####################################################
if(BUILD_TURTLEBOT_INTERFACE)
  include_directories(piper/turtlebot_interface/)

  # file(GLOB TURTLEBOT_INTERFACE_SRC "piper/turtlebot_interface/*.cpp")
  # add_executable(turtlebot_interface ${TURTLEBOT_INTERFACE_SRC})
  add_executable(turtlebot_interface piper/turtlebot_interface/turtlebot_interface.cpp)
  add_executable(diff_interface piper/turtlebot_interface/diff_interface.cpp)

  target_link_libraries(turtlebot_interface piperbase)
  target_link_libraries(diff_interface piperbase pipermobilerobot)

  install(TARGETS turtlebot_interface diff_interface
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY piper/turtlebot_interface/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
  )
endif()

####################################################
##                 ros_application                ##
####################################################
include_directories(piper/ros_application/library/)
file(GLOB ROS_LIB_SRC "piper/ros_application/library/*.cpp")
add_library(rosapplib ${ROS_LIB_SRC})

include_directories(piper/ros_application/)
add_executable(ros_sdf piper/ros_application/ros_sdf.cpp)
add_executable(turtlebot_server piper/ros_application/turtlebot_server.cpp)

target_link_libraries(ros_sdf piperbase)
target_link_libraries(turtlebot_server rosapplib ${catkin_LIBRARIES})

install(TARGETS ros_sdf turtlebot_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY piper/ros_application/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

################################################
##                   launch                   ##
################################################
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
