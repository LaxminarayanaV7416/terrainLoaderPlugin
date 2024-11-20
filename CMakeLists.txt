# Minimum version required
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

# Project name
project(gazebo_terrain_loader)

# Set C11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Protobuf REQUIRED)
#--------------------#
# Message Generation #
#--------------------#
PROTOBUF_GENERATE_CPP(VECTOR_PROTO_SRCS VECTOR_PROTO_HDRS msgs/vector3d.proto)
PROTOBUF_GENERATE_CPP(PHY_PROTO_SRCS PHY_PROTO_HDRS msgs/Wind.proto)
add_library(vector_msgs SHARED ${VECTOR_PROTO_SRCS})
set_target_properties(vector_msgs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)
add_library(physics_msgs SHARED ${PHY_PROTO_SRCS})
set_target_properties(physics_msgs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)
link_libraries(physics_msgs vector_msgs)

# Include the built proto messages as part of import in plugins
include_directories(${CMAKE_BINARY_DIR})

# Include the headers from the gazeboSUASHCCPlugins/include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/gazeboSUASHCCPlugins/include)


# Add all source files in gazeboSUASHCCPlugins/src
file(GLOB SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/gazeboSUASHCCPlugins/src/gazebo_terrain_loader.cpp
)

# Find gazebo package
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

# find the eigen package
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add the library (shared object)
add_library(gazebo_terrain_loader_plugin SHARED ${SOURCES})

# target_link_libraries(gazebo_terrain_loader_plugin ${GAZEBO_INCLUDE_DIRS})

# Set output directory for libraries
set_target_properties(gazebo_terrain_loader_plugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

file(GLOB WIND_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/gazeboSUASHCCPlugins/src/gazebo_suas_hcc_wind_plugin.cpp
)
# Add the library (shared object)
add_library(gazebo_suas_hcc_wind_plugin SHARED ${WIND_SOURCES})
set_target_properties(gazebo_suas_hcc_wind_plugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# Print output directory message
message(STATUS "Library will be placed in: ${CMAKE_BINARY_DIR}/lib")
