cmake_minimum_required(VERSION 2.8.3)
project(volta_hardware)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  volta_msgs
  controller_manager
  hardware_interface
  diagnostic_updater
)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS hardware_interface volta_msgs roscpp
DEPENDS Boost )

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

install(
  DIRECTORY include src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
add_library(tableToRos
	src/tableToRos.cpp
)
add_library(can_monitor
	src/can_monitor.cpp
)
add_library(queue
	src/queue.cpp
)

add_library(conversion
	src/conversion.cpp
)
add_library(voltaDataStruct
	src/voltaDataStruct.cpp
)
add_library(volta_hardware src/volta_hardware.cpp)
add_executable(volta_base src/volta_base.cpp)
target_link_libraries(volta_base volta_hardware ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(rpmSend src/rpmSend.cpp)
target_link_libraries(rpmSend ${catkin_LIBRARIES})
target_link_libraries(rpmSend tableToRos can_monitor queue voltaDataStruct conversion ${catkin_LIBRARIES} ${Boost_LIBRARIES})
