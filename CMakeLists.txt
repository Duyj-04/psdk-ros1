cmake_minimum_required(VERSION 3.0.2)
project(psdk_ros1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

set(PSDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../Payload-SDK-3.13.0" CACHE PATH "Path to DJI Payload SDK root")
set(PSDK_LIB_DIR "${PSDK_ROOT}/build/bin")

if(NOT EXISTS "${PSDK_ROOT}")
  message(FATAL_ERROR "Expected DJI Payload SDK at ${PSDK_ROOT}")
endif()

find_library(PSDK_CORE_LIBRARY
  NAMES payloadsdk libpayloadsdk
  PATHS "${PSDK_LIB_DIR}" "${PSDK_ROOT}"
  NO_DEFAULT_PATH
)

if(NOT PSDK_CORE_LIBRARY)
  message(FATAL_ERROR "Could not find Payload SDK library under ${PSDK_ROOT}")
endif()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PSDK_ROOT}
  ${PSDK_ROOT}/psdk_lib/include
  ${PSDK_ROOT}/psdk_lib/include/dji_platform
)

add_executable(psdk_ros1_node src/psdk_ros1_node.cpp)
target_link_libraries(psdk_ros1_node
  ${catkin_LIBRARIES}
  ${PSDK_CORE_LIBRARY}
)
