project(serial_comm_tester)

set(CMAKE_CXX_STANDARD 14)

add_executable(
    ${PROJECT_NAME}
    main.cpp
    sercom.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE .)

target_link_directories(${PROJECT_NAME} PRIVATE .)
