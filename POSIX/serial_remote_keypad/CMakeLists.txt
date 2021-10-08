project(serial_comm_tester)

set(CMAKE_CXX_STANDARD 14)

#adding executables *make sure to include src files one by one
add_executable(
    ${PROJECT_NAME}
    main.cpp
    sercom.cpp
)

#including common source/header files
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE})

#linking common source/header files
target_link_directories(${PROJECT_NAME} PRIVATE ${INCLUDE})

#make sure SDL2 is installed
#find_package(SDL2 REQUIRED)
#include_directories(${SDL2_INCLUDE_DIRS})

#target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARIES})