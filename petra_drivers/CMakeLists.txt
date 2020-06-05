cmake_minimum_required(VERSION 3.5)
project(petra_drivers)

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
add_executable(Screen src/Screen.cpp)
ament_target_dependencies(Screen ${dependencies})
target_link_libraries(Screen petra_core::petra_core_LIB)

add_executable(Keyboard src/Keyboard.cpp)
ament_target_dependencies(Keyboard ${dependencies})

add_executable(RobotDummy src/RobotDummy.cpp)
ament_target_dependencies(RobotDummy ${dependencies})

install(TARGETS
  Screen
  Keyboard
  RobotDummy
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()