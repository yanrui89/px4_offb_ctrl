cmake_minimum_required(VERSION 3.0.2)

project(rapid_traj_gen)

set(CMAKE_CXX_STANDARD 17)

set(SRC_FILES src/RapidTrajectoryGenerator.cpp src/SingleAxisTrajectory.cpp)

add_library(${PROJECT_NAME}_lib ${SRC_FILES})

target_include_directories(${PROJECT_NAME}_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)