cmake_minimum_required(VERSION 3.13)
project(mavlink_natnet_bridge)
add_compile_options(-std=c++17)

find_package(Threads REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "mavlink/install")
find_package(MAVLink REQUIRED)

add_executable(mavlink_natnet_bridge mavlink_natnet_bridge.cpp mavlink_socket.cpp natnet_socket.cpp)

target_link_libraries(mavlink_natnet_bridge Threads::Threads)
target_link_libraries(mavlink_natnet_bridge MAVLink::mavlink)
target_link_libraries(mavlink_natnet_bridge ${CMAKE_SOURCE_DIR}/natnet/lib/libNatNet.so)
target_include_directories(mavlink_natnet_bridge PUBLIC ${CMAKE_SOURCE_DIR}/natnet/include)