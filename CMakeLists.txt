cmake_minimum_required(VERSION 2.8.3)
project(rm_suspension)
#add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    robot_msgs
    )

catkin_package(
)

# Source/Include files
include_directories(
    ${catkin_INCLUDE_DIRS}
    include/
    ./
    ../robot_common/include
    ../robot_third_party/include
)

file(GLOB_RECURSE sources "src/*.cpp")

add_executable(sim "main.cpp" ${sources})

target_link_libraries(sim
    ${catkin_LIBRARIES}
    )
