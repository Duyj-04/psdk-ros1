cmake_minimum_required(VERSION 3.0.2)
project(psdk_ros1)

# This package is the ROS1 boundary around the locally vendored DJI Payload
# SDK.  Message/service/action generation and the two runtime nodes are kept
# here so the package can be built in a standard ROS Noetic catkin workspace.
add_definitions(-DSYSTEM_ARCH_LINUX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  image_transport
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  vision_msgs
)

file(GLOB PSDK_ROS1_MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg msg/*.msg)
file(GLOB PSDK_ROS1_SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/srv srv/*.srv)
file(GLOB PSDK_ROS1_ACTION_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/action action/*.action)

# These interfaces are generated before either executable is compiled.  The
# generated targets are also used by add_dependencies below.
add_message_files(DIRECTORY msg FILES ${PSDK_ROS1_MSG_FILES})
add_service_files(DIRECTORY srv FILES ${PSDK_ROS1_SRV_FILES})
add_action_files(DIRECTORY action FILES ${PSDK_ROS1_ACTION_FILES})

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    image_transport
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf
    tf2_ros
    vision_msgs
)

# The wrapper consumes the SDK headers, prebuilt static library, and selected
# platform/sample support sources from this sibling directory.
set(PSDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../Payload-SDK-3.13.0 CACHE PATH "Path to DJI Payload SDK 3.13")
set(DJI_OSAL_PATH ${PSDK_PATH}/samples/sample_c++/platform/linux/common/osal)
set(DJI_HAL_PATH ${PSDK_PATH}/samples/sample_c++/platform/linux/manifold2/hal)
set(DJI_UTILS_PATH ${PSDK_PATH}/samples/sample_c/module_sample)
set(DJI_STREAMING_PATH ${PSDK_PATH}/samples/sample_c++/module_sample/liveview)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  set(TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
  add_definitions(-DPLATFORM_ARCH_x86_64=1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
  add_definitions(-DPLATFORM_ARCH_aarch64=1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7|arm")
  set(TOOLCHAIN_NAME arm-linux-gnueabihf-gcc)
  add_definitions(-DPLATFORM_ARCH_arm=1)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Link the architecture-specific DJI static library shipped with the SDK.
# The explicit checks make a missing or mismatched SDK installation fail at
# configure time instead of producing a less useful linker error later.
find_library(PSDK_LIB NAMES payloadsdk libpayloadsdk.a PATHS ${PSDK_PATH}/psdk_lib/lib/${TOOLCHAIN_NAME} NO_DEFAULT_PATH)
if(NOT PSDK_LIB)
  message(FATAL_ERROR "Cannot find DJI PSDK library under ${PSDK_PATH}/psdk_lib/lib/${TOOLCHAIN_NAME}")
endif()

find_library(USB_1_LIBRARY usb-1.0)
if(NOT USB_1_LIBRARY)
  message(FATAL_ERROR "Cannot find libusb-1.0. Install libusb-1.0-0-dev.")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PSDK_PATH}/psdk_lib/include
  ${DJI_OSAL_PATH}
  ${DJI_HAL_PATH}
  ${DJI_UTILS_PATH}
  ${DJI_STREAMING_PATH}
)

add_executable(psdk_ros1_node
  src/psdk_ros1_node.cpp
  src/psdk_ros1_wrapper.cpp
  ${DJI_OSAL_PATH}/osal.c
  ${DJI_OSAL_PATH}/osal_fs.c
  ${DJI_OSAL_PATH}/osal_socket.c
  ${DJI_HAL_PATH}/hal_network.c
  ${DJI_HAL_PATH}/hal_uart.c
  ${DJI_HAL_PATH}/hal_usb_bulk.c
  ${DJI_UTILS_PATH}/utils/cJSON.c
  ${DJI_UTILS_PATH}/utils/dji_config_manager.c
  ${DJI_UTILS_PATH}/utils/util_file.c
  ${DJI_UTILS_PATH}/utils/util_misc.c
)

# psdk_ros1_node owns PSDK initialization, JSON dispatch, ROS service/topic
# bridges, and the platform callback registration.  CMAKE_DL_LIBS is needed by
# the DJI static library on Linux for its dynamic-loading helpers.
add_dependencies(psdk_ros1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(psdk_ros1_node
  ${catkin_LIBRARIES}
  ${PSDK_LIB}
  ${USB_1_LIBRARY}
  ${CMAKE_DL_LIBS}
  pthread
)

add_executable(psdk_visual_target_tracker_node
  src/psdk_visual_target_tracker_node.cpp
)

# The tracker is intentionally separate from the PSDK wrapper: it consumes
# detector/gimbal topics and sends high-level gimbal/planner commands through
# ROS, while the wrapper remains the owner of PSDK calls.
add_dependencies(psdk_visual_target_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(psdk_visual_target_tracker_node
  ${catkin_LIBRARIES}
)

# Install both runtime nodes and the package-side resources used by launch
# files and operators.
install(TARGETS psdk_ros1_node psdk_visual_target_tracker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
