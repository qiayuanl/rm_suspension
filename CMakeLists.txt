cmake_minimum_required(VERSION 2.8.3)
project(rm_suspension)
#add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Ofast)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    message_generation
    )
add_message_files(
    FILES
    JointData.msg
)

generate_messages(
    DEPENDENCIES
)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include/
    ./
)
file(GLOB_RECURSE sources "src/*.cpp")

add_executable(sim "main.cpp" ${sources})

target_link_libraries(sim
    ${catkin_LIBRARIES}
    )
