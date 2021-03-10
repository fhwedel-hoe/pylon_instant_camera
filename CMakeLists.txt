cmake_minimum_required(VERSION 3.5)
project(pylon_usb_instant_camera)

set (CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager QUIET) # in development at https://github.com/ros-perception/image_common/tree/ros2
if (NOT ${camera_info_manager_FOUND})
    message(WARNING "image_transport not found. Rectification via camera info parameters will not be possible.")
endif()

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
if (NOT ${Pylon_FOUND})
    message(FATAL "Unable to find Basler Pylon SDK.")
endif()

add_executable(node src/main.cpp)
target_include_directories(node PUBLIC
  ${Pylon_INCLUDE_DIRS}
)
target_link_libraries(node
  ${Pylon_LIBRARIES}
)
ament_target_dependencies(node 
	rclcpp
	image_transport
	sensor_msgs
)

install(TARGETS 
  node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
