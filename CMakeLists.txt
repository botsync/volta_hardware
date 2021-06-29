cmake_minimum_required(VERSION 3.5)
project(volta_hardware)
add_compile_options(-std=c++14)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(volta_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS} ${volta_msgs_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(tableToRos
	src/tableToRos.cpp
)
ament_target_dependencies(tableToRos volta_msgs)

add_library(queue
	src/queue.cpp
)

add_library(conversion
	src/conversion.cpp
)
add_library(voltaDataStruct
	src/voltaDataStruct.cpp
)

add_library(
	${PROJECT_NAME}
	SHARED
	src/volta_hardware.cpp
)
target_include_directories(
	${PROJECT_NAME}
	PRIVATE
	include
)
ament_target_dependencies(${PROJECT_NAME} hardware_interface pluginlib rclcpp volta_msgs)

add_executable(volta_hardware_ROS src/volta_hardware_to_ROS.cpp)
ament_target_dependencies(volta_hardware_ROS rclcpp volta_msgs)

add_executable(volta_hardware_node src/volta_hardware_node.cpp)
ament_target_dependencies(volta_hardware_node hardware_interface pluginlib rclcpp controller_manager volta_msgs)
target_link_libraries(volta_hardware_node ${Boost_LIBRARIES})

add_executable(serial_node src/serialMain.cpp)
ament_target_dependencies(serial_node rclcpp volta_msgs)
target_link_libraries(serial_node tableToRos queue voltaDataStruct conversion ${Boost_LIBRARIES})

pluginlib_export_plugin_description_file(hardware_interface volta_hardware.xml)

install(DIRECTORY include
	DESTINATION share/${PROJECT_NAME})

install(TARGETS
		volta_hardware_node
		volta_hardware_ROS
		serial_node
		tableToRos
		queue
		conversion
		voltaDataStruct
		${PROJECT_NAME}
	DESTINATION
		lib/${PROJECT_NAME}
		)
		
## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rosidl_default_runtime
)

ament_package()
