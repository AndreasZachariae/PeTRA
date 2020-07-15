cmake_minimum_required(VERSION 3.5)
project(petra_central_control)

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
find_package(BehaviorTree REQUIRED)

include_directories(
  include 
  ${petra_core_INCLUDE_DIRS}
)
 
set(dependencies 
  "rclcpp" 
  "rclcpp_action"
  "petra_core"
)
file (GLOB SOURCES src/*.cpp)
file (GLOB SOURCES_CONDITIONS src/conditions/*.cpp)
file (GLOB SOURCES_ACTIONS src/actions/*.cpp)
file (GLOB SOURCES_ACTIONS_BASE src/actions/base/*.cpp)
file (GLOB SOURCES_ACTIONS_TYPES src/actions/types/*.cpp)

add_executable(PeTRACentralControl
    ${SOURCES}
    ${SOURCES_CONDITIONS}
    ${SOURCES_ACTIONS}
    ${SOURCES_ACTIONS_BASE}
    ${SOURCES_ACTIONS_TYPES})

target_link_libraries(PeTRACentralControl
    petra_core::petra_core_LIB
    BehaviorTree::behaviortree_cpp_v3)

ament_target_dependencies(PeTRACentralControl ${dependencies})

install(TARGETS
  PeTRACentralControl
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
