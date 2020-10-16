cmake_minimum_required(VERSION 3.5)
project(robosub2020_assignment2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(motion_planner src/motion_planner.cpp)
ament_target_dependencies(motion_planner rclcpp std_msgs)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
        motion_planner
        DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
        scripts/video_streamer.py
        scripts/gate_detector.py
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
