cmake_minimum_required(VERSION 2.8.3)
project(daq)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions

#find_package(Boost REQUIRED COMPONENTS system)

set(raspicam_DIR "/usr/local/lib/cmake")

find_package(OpenCV REQUIRED)
find_package(raspicam)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES daq
#  CATKIN_DEPENDS roscpp rospy std_msgs
   DEPENDS raspicam raspicam_cv opencv
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${raspicam_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(daq
#   src/${PROJECT_NAME}/daq.cpp
# )


set(CMAKE_C_FLAGS "-DMPU9150 -DMPU_DEBUGON ${CMAKE_C_FLAGS}")

add_library(MPULib src/inv_mpu_lib/inv_mpu.c)
add_library(DMPLib src/inv_mpu_lib/inv_mpu_dmp_motion_driver.c)
add_library(I2Cdev src/I2Cdev/I2Cdev.c)

add_library(sensor src/MotionSensor/sensor.cpp)
target_link_libraries(sensor I2Cdev ${catkin_LIBRARIES})

add_executable(RPi_Camera src/RPiCam/RpiCamOpenCV.cpp)
target_link_libraries( RPi_Camera raspicam raspicam_cv ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(daq ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(mono-vo src/mono-vo/visodo.cpp)
target_link_libraries (mono-vo raspicam raspicam_cv ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(daq_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(daq_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS daq daq_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_daq.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
