cmake_minimum_required(VERSION 3.10)
project(mavsdk_code)

find_package(MAVSDK REQUIRED)

add_executable(mavsdk_code main.cpp)

target_link_libraries(mavsdk_code
    MAVSDK::mavsdk
)

