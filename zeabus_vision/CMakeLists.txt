cmake_minimum_required(VERSION 2.8.3)
project(zeabus_vision)

add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    OpenCV REQUIRED
)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  vision_path.msg
  vision_shoot_craps.msg
  vision_qualifying_gate.msg
  vision_qualifying_marker.msg
  vision_casino_gate.msg
  vision_roulette.msg
  vision_buy_a_gold_chip.msg
  vision_slots.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  vision_srv_path.srv
  vision_srv_shoot_craps.srv
  vision_srv_qualifying_gate.srv
  vision_srv_qualifying_marker.srv
  vision_srv_casino_gate.srv
  vision_srv_roulette.srv
  vision_srv_buy_a_gold_chip.srv
  vision_srv_slots.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

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
  INCLUDE_DIRS 
  LIBRARIES zeabus_vision
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)

## Declare a cpp executable
add_executable(bag2mp4 src/bag2mp4.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(bag2mp4
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
