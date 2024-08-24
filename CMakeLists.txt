cmake_minimum_required(VERSION 3.10)

# Project name and version
project(dogzilla_driver VERSION 1.0)

# Specify the C++ standard
if (NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 23)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++23")
endif()
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_definitions(-O3)

set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Add the include directory for headers
# This should add `include` directory to the search path
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add a library from the source files in src/
add_library(dogzilla_driver
    src/dogzilla_driver.cpp
)

# Link include directory to the library target
target_include_directories(dogzilla_driver PUBLIC
    "${PROJECT_SOURCE_DIR}/include"
)

add_executable(main
    src/main.cpp)
    target_include_directories(main PUBLIC
    "${PROJECT_SOURCE_DIR}/include"
)
target_link_libraries(main dogzilla_driver pthread)


# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)