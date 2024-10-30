cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

project(custom_gz_plugins)

find_package(gz-rendering8 REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport14 REQUIRED)

# Define a variable 'GZ_MSGS_VER' holding the version number
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

# Define a variable 'MSGS_PROTOS' listing the .proto files
set(MSGS_PROTOS
   ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/custom_msgs/GeneratedPath.proto
)

# Call 'gz_msgs_generate_messages()' to process the .proto file
gz_msgs_generate_messages(
  # The cmake target to be generated for libraries/executables to link
  TARGET msgs
  # The protobuf package to generate (Typically based on the path)
  PROTO_PACKAGE "gz.custom_msgs"
  # The path to the base directory of the proto files
  # All import paths should be relative to this (eg gz/custom_msgs/vector3d.proto)
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
  # List of proto files to process
  MSGS_PROTOS ${MSGS_PROTOS}
  # Depenency on gz-msgs
  DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}

)

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/custom_gz_plugins-msgs_genmsg
)

set(CMAKE_AUTOMOC ON)

find_package(gz-gui8 REQUIRED)

QT5_ADD_RESOURCES(resources_RCC PathVisualizer.qrc)

add_library(PathVisualizer SHARED
  PathVisualizer.cc
  ${resources_RCC}
)
target_link_libraries(PathVisualizer
  PRIVATE
    gz-gui8::gz-gui8
    gz-rendering8::gz-rendering8
    gz-transport14::gz-transport14
)

install(TARGETS PathVisualizer DESTINATION ${$GZ_SIM_SYSTEM_PLUGIN_PATH})