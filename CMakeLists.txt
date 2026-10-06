cmake_minimum_required(VERSION 3.5...4.2)
project(pylon_instant_camera)

set (CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)

# Install from https://www2.baslerweb.com/en/downloads/software-downloads/.
find_package(Pylon 8 REQUIRED)

set(NODE_NAME "PylonCameraNode")
add_library(${NODE_NAME} SHARED src/main.cpp)
target_link_libraries(${NODE_NAME}
    pylon::pylon
    rclcpp::rclcpp
    rclcpp_components::component
    image_transport::image_transport
    camera_calibration_parsers::camera_calibration_parsers
)
rclcpp_components_register_node(${NODE_NAME} PLUGIN "${PROJECT_NAME}::${NODE_NAME}" EXECUTABLE "node")

install(TARGETS
  ${NODE_NAME}
  DESTINATION lib
)

ament_package()
