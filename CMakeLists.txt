cmake_minimum_required(VERSION 2.8.3)
project(daq)



### Linking Libraries ###
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
find_package(OpenCV REQUIRED)

#The following line is to avoid a dependency issue associated with Raspicam library linking
set(raspicam_DIR "/usr/local/lib/cmake")
find_package(raspicam)

################################
###### Message Generation ######
################################

add_message_files( FILES SixDoF.msg)

generate_messages(DEPENDENCIES std_msgs)

######################################
#### Catkin Package Configuration ####
######################################

catkin_package(INCLUDE_DIRS include DEPENDS raspicam raspicam_cv opencv)


###########
## Build ##
###########

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${raspicam_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "-DMPU9150 -DMPU_DEBUGON -std=c++11 ${CMAKE_C_FLAGS}")

add_library(MPULib src/inv_mpu_lib/inv_mpu.c)
add_library(DMPLib src/inv_mpu_lib/inv_mpu_dmp_motion_driver.c)
add_library(I2Cdev src/I2Cdev/I2Cdev.c)
add_library(MonoVOdometry src/Monocular-VO/Mono-VO.cpp)

add_executable(MPU_Demo src/MotionSensor/MPU-9150_Demo.cpp)
target_link_libraries(MPU_Demo I2Cdev DMPLib MPULib ${catkin_LIBRARIES})
add_dependencies(MPU_Demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(RPi_Camera src/RPiCam/RpiCamOpenCV.cpp)
#target_link_libraries( RPi_Camera raspicam raspicam_cv ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(mono-vo src/mono-vo/visodo.cpp)
#target_link_libraries (mono-vo raspicam raspicam_cv ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(camera-calibration src/RPiCam/camera_calibration.cpp)
target_link_libraries (camera-calibration raspicam raspicam_cv ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
