cmake_minimum_required(VERSION 3.5)
project(petra_core)

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
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(dependencies
  builtin_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DialogDataType.msg"
  "msg/PatientCondition.msg"
  "msg/EmergencyStop.msg"
  "srv/UserDialog.srv"
  "action/ChargeBattery.action"
  "action/NavigateToPose.action"
  "action/PairDevice.action"
  "action/MoveArm.action"
  DEPENDENCIES ${dependencies}
 )

set(petra_core_LIB_SRC
    src/events/Event.cpp
    src/tools/Chrono.cpp
    src/tools/ChronoTime.cpp
    src/tools/IO.cpp
    src/tools/Logger.cpp
    src/tools/LogLevel.cpp
    src/tools/ThreadExecuter.cpp
)
add_library(petra_core_LIB ${petra_core_LIB_SRC})

target_include_directories(petra_core_LIB PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install library
install(TARGETS petra_core_LIB
  EXPORT export_petra_core_LIB
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_interfaces(export_petra_core_LIB HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime ${dependencies})
ament_package()
