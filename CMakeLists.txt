# Minimum version required
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

# Project name
project(gazebo_terrain_loader)

# Set the C++ standard to C++20
set (CMAKE_CXX_STANDARD 20)

# adding gazebo package
find_package(gazebo REQUIRED)

# Add the library (shared object)
add_library(gazebo_terrain_loader_plugin SHARED gazeboTerrainLoader/src/gazebo_terrain_loader.cpp)

# Make sure output directories for libraries and executables are defined correctly
# This ensures that both shared and static libraries are placed in the lib directory
set_target_properties(gazebo_terrain_loader_plugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# Ensure that the build is successful and print the paths
message(STATUS "Library will be placed in: ${CMAKE_BINARY_DIR}/lib")
