cmake_minimum_required(VERSION 3.5)
project(whi_arm_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
# find_package(joint_limits_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(angles REQUIRED)
find_package(serial REQUIRED)
find_package(pluginlib REQUIRED)
find_package(whi_interfaces REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/include/jsoncpp/
)

## Declare a C++ library
FILE(GLOB WHI_SDK_SRC
  "src/sdk/*.cpp"
)
## Define third party link directory
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm # arm64
  # ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86 # amd64
)
add_library(${PROJECT_NAME}
  ${WHI_SDK_SRC}
  src/whi_arm_interface.cpp
)
## Link to the third party non-ros libraries
target_link_libraries(${PROJECT_NAME}
  sockpp
  jsoncpp
  boost_system
  jakaAPI
  # fairino
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  std_srvs
  trajectory_msgs
  serial
  angles
  hardware_interface
  pluginlib
  whi_interfaces
)

add_executable(whi_arm_interface_util
  src/whi_arm_interface_util.cpp
  src/whi_arm_interface_util_node.cpp
)
ament_target_dependencies(whi_arm_interface_util
  rclcpp
  std_msgs
  std_srvs
  whi_interfaces
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "WHI_MOTION_HW_INTERFACE_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
    whi_arm_interface_util
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# arm64
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so"
)
# amd64
# install(
  # DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86
  # DESTINATION lib
  # FILES_MATCHING PATTERN "*.so"
# )

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
