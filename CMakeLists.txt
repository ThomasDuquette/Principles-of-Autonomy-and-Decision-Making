cmake_minimum_required(VERSION 3.10)

project(
    Principles
    VERSION 1.0
    DESCRIPTION "Principles of Autonomy and Decision Making"
    LANGUAGES CXX
)

set(CMAKE_INCLUDE_CURRENT_DIR   ON)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(extern/blast_rush)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
add_subdirectory(recitations)
add_subdirectory(assignments)
include(CTest)
enable_testing()
add_subdirectory(test)
endif()
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/CMakeFiles/main.dir)
