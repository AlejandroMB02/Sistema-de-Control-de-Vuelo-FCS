cmake_minimum_required(VERSION 3.16)
project(drone_fcs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile_commands.json (NECESARIO para clang-tidy)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Librería principal (core, independiente de ESP-IDF)
add_library(drone_core
    src/math/Vector3D.cpp
)

target_include_directories(drone_core
    PUBLIC include
)

# Tests
enable_testing()
add_subdirectory(test)
