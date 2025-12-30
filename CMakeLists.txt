cmake_minimum_required(VERSION 3.16)
project(drone_fcs LANGUAGES CXX)

# -------------------------
# Configuración general
# -------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Necesario para clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -------------------------
# Librería principal
# -------------------------
add_library(drone_core
    src/math/Vector3D.cpp
)

target_include_directories(drone_core
    PUBLIC include
)

# -------------------------
# Ejecutable de Simulación (main_sim)
# -------------------------
add_executable(simulation
    src/main_sim.cpp
)

target_link_libraries(simulation
    PRIVATE drone_core
)

# -------------------------
# GoogleTest (SUBMÓDULO)
# -------------------------
# Evita instalar gtest y compilar cosas innecesarias
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)

if(EXISTS "${CMAKE_SOURCE_DIR}/extern/googletest/CMakeLists.txt")
    add_subdirectory(extern/googletest)
else()
    message(WARNING "GoogleTest no encontrado en extern/. Los tests no se compilarán.")
endif()

# -------------------------
# Tests
# -------------------------
enable_testing()
add_subdirectory(test)
