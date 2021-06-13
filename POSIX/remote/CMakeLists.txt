# file: remote.cpp
# author: Hikari hashida
# brief:
# date: 04/05/2021
# last visited: 04/05/2021

cmake_minimum_required(VERSION 3.9.1)
project(run_remote)

#support for C++14 or later
set(CMAKE_CXX_STANDARD 14)

#adding executables *make sure to include src files one by one
add_executable(${PROJECT_NAME} remote.cpp)

#including common source/header files
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE} ${COMMON})

#linking common source/header files
target_link_directories(${PROJECT_NAME} PRIVATE ${INCLUDE} ${COMMON})

#make sure SDL2 is installed
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARIES})
