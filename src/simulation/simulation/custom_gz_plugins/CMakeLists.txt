cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

project(custom_gz_plugins)

# Common to both plugins
find_package(gz-rendering8 REQUIRED)

set(CMAKE_AUTOMOC ON)

find_package(gz-gui8 REQUIRED)

QT5_ADD_RESOURCES(resources_RCC PathVisualizer.qrc)

add_library(PathVisualizer SHARED
  PathVisualizer.cc
  ${resources_RCC}
)
target_link_libraries(PathVisualizer
  PUBLIC
    gz-gui8::gz-gui8
    gz-rendering8::gz-rendering8
)

install(TARGETS PathVisualizer DESTINATION ${$GZ_SIM_SYSTEM_PLUGIN_PATH})