cmake_minimum_required(VERSION 3.5)
project(petra_services)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(petra_core REQUIRED)

include_directories(
  include 
  ${petra_core_INCLUDE_DIRS}
)

set(dependencies 
  "rclcpp" 
  "petra_core"
)

#make executables
add_executable(Communication src/Communication.cpp)
ament_target_dependencies(Communication ${dependencies})

add_executable(Navigation2Dummy src/Navigation2Dummy.cpp)
ament_target_dependencies(Navigation2Dummy ${dependencies} "rclcpp_action")

add_executable(PatientMonitoringDummy src/PatientMonitoringDummy.cpp)
ament_target_dependencies(PatientMonitoringDummy ${dependencies})


install(TARGETS
  Communication
  Navigation2Dummy
  PatientMonitoringDummy
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
