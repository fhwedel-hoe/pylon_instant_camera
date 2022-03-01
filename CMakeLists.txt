cmake_minimum_required(VERSION 3.5)
project(pylon_instant_camera)

set (CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
if (NOT ${Pylon_FOUND})
    message(FATAL "Unable to find Basler Pylon SDK.")
endif()

set(NODE_NAME "PylonCameraNode")
add_library(${NODE_NAME} SHARED src/main.cpp)
target_include_directories(${NODE_NAME} PUBLIC
    ${Pylon_INCLUDE_DIRS}
)
target_link_libraries(${NODE_NAME}
    ${Pylon_LIBRARIES}
)
ament_target_dependencies(${NODE_NAME} 
    rclcpp
    rclcpp_components
    image_transport
    camera_calibration_parsers
)
rclcpp_components_register_node(${NODE_NAME} PLUGIN "${PROJECT_NAME}::${NODE_NAME}" EXECUTABLE "node")

install(TARGETS 
  ${NODE_NAME}
  DESTINATION lib
)

add_library(AddressPrinter SHARED src/addressprinter.cpp)
ament_target_dependencies(AddressPrinter 
    rclcpp
    rclcpp_components
    sensor_msgs
)
rclcpp_components_register_node(AddressPrinter PLUGIN "AddressPrinter" EXECUTABLE "addressprinter")
install(TARGETS 
  AddressPrinter
  DESTINATION lib
)

add_library(StaticBufferPublisher SHARED src/staticbufferpublisher.cpp)
ament_target_dependencies(StaticBufferPublisher 
    rclcpp
    rclcpp_components
    sensor_msgs
)
rclcpp_components_register_node(StaticBufferPublisher PLUGIN "StaticBufferPublisher" EXECUTABLE "staticbufferpublisher")
install(TARGETS 
  StaticBufferPublisher
  DESTINATION lib
)

# install launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
