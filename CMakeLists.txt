cmake_minimum_required(VERSION 3.8)
project(cr_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

# NVIDIA VPI library
find_library(VPI_LIB nvvpi PATHS "/usr/lib/aarch64-linux-gnu" NO_DEFAULT_PATH)
if(NOT VPI_LIB)
    message(WARNING "VPI library not found at /usr/lib/aarch64-linux-gnu/")
    find_library(VPI_LIB nvvpi)
    if(VPI_LIB)
        message(STATUS "Found VPI at: ${VPI_LIB}")
    else()
        message(FATAL_ERROR "VPI library not found, required for VIC conversion")
    endif()
else()
    message(STATUS "Found VPI at: ${VPI_LIB}")
endif()

# VPI include directory
set(VPI_INCLUDE_DIR "/usr/include/vpi" CACHE STRING "VPI include directory")
if(EXISTS ${VPI_INCLUDE_DIR})
    message(STATUS "VPI includes found at: ${VPI_INCLUDE_DIR}")
    include_directories(${VPI_INCLUDE_DIR})
else()
    message(FATAL_ERROR "VPI include directory not found at ${VPI_INCLUDE_DIR}")
endif()

# Include OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Include project headers
include_directories(include)

# Add executable
add_executable(cr_camera_node 
  src/cr_camera_node.cpp
  src/cr_vic_converter.cpp
  src/cr_image_publisher.cpp
  src/cr_v4l2_camera.cpp
  src/cr_yaml_config.cpp
)
ament_target_dependencies(cr_camera_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
  ament_index_cpp
)

# Link libraries
target_link_libraries(cr_camera_node 
  ${OpenCV_LIBRARIES} 
  pthread
  ${VPI_LIB}
)

# Install targets
install(TARGETS
  cr_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
