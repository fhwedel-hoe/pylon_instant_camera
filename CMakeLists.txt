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

# Install from https://www2.baslerweb.com/en/downloads/software-downloads/.
find_package(Pylon 7.4 REQUIRED)

set(NODE_NAME "PylonCameraNode")
add_library(${NODE_NAME} SHARED src/main.cpp)
target_link_libraries(${NODE_NAME}
    pylon::pylon
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

ament_package()
