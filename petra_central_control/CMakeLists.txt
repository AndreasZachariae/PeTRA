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

include_directories(
  include 
  ${petra_core_INCLUDE_DIRS}
)
 
set(dependencies 
  "rclcpp" 
  "rclcpp_action"
  "petra_core"
)

set(CentralControlUnit_SRC
  src/node.cpp
  src/CentralControlUnit.cpp
  src/Skill.cpp
  src/SkillSelection.cpp
  src/SystemMonitor.cpp
  src/BaseMovement.cpp
  src/UserDialog.cpp
  src/DevicePairing.cpp
  src/ArmMovement.cpp
  src/BatteryCharging.cpp
)

#make executables
add_executable(CCU ${CentralControlUnit_SRC})
target_link_libraries(CCU petra_core::petra_core_LIB)
ament_target_dependencies(CCU ${dependencies})

install(TARGETS
  CCU
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
