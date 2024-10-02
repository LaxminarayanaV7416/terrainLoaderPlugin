# Minimum version required
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

# Project name
project(gazebo_terrain_loader)

# Set c11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# find_package(ignition-math3 REQUIRED)
# include_directories(${IGNITION_MATH_INCLUDE_DIRS})

# adding gazebo package
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

# Add the library (shared object)
add_library(gazebo_terrain_loader_plugin SHARED gazeboTerrainLoader/src/gazebo_terrain_loader.cpp)

# Make sure output directories for libraries and executables are defined correctly
# This ensures that both shared and static libraries are placed in the lib directory
set_target_properties(gazebo_terrain_loader_plugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# Ensure that the build is successful and print the paths
message(STATUS "Library will be placed in: ${CMAKE_BINARY_DIR}/lib")
