# Minimum version required
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

# Project name
project(gazebo_terrain_loader)

# Set C11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include the headers from the gazeboTerrainLoader/include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/gazeboTerrainLoader/include)

# Add all source files in gazeboTerrainLoader/src
file(GLOB SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/gazeboTerrainLoader/src/*.cpp
)

# Find gazebo package
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

# Add the library (shared object)
add_library(gazebo_terrain_loader_plugin SHARED ${SOURCES})

# Set output directory for libraries
set_target_properties(gazebo_terrain_loader_plugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# Print output directory message
message(STATUS "Library will be placed in: ${CMAKE_BINARY_DIR}/lib")
