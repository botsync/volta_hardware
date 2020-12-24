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
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/controls.cfg
)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS hardware_interface volta_msgs roscpp dynamic_reconfigure
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
add_executable(volta_hardware_node src/volta_hardware_node.cpp)
target_link_libraries(volta_hardware_node volta_hardware ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(serial_node src/serialMain.cpp)
target_link_libraries(serial_node tableToRos queue voltaDataStruct conversion ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(serial_node ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		PATTERN ".svn" EXCLUDE
		)

install(TARGETS
		volta_hardware_node
		serial_node
		tableToRos
		queue
		conversion
		voltaDataStruct
		volta_hardware
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		)