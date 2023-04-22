cmake_minimum_required(VERSION 2.8.3)
project(libbspline)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

set(BUILD_SHARED_LIBS ON)

file(GLOB INCLUDE_FILES "include/*.h")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} include)

include(CTest)

# add_library(${PROJECT_NAME} STATIC ${SRC_FILES} ${INCLUDE_FILES})

add_executable(m_matrix test/m_matrix.cpp)
add_test(m_matrix m_matrix)

add_executable(txt_reader test/txt_reader.cpp)
add_test(txt_reader txt_reader ${CMAKE_SOURCE_DIR}/data/control_point.txt)

add_executable(1d_bspline test/1d_bspline.cpp)
add_test(1d_bspline 1d_bspline 
  ${CMAKE_SOURCE_DIR}/data/control_point.txt
  ${CMAKE_SOURCE_DIR}/data/position.txt
  ${CMAKE_SOURCE_DIR}/data/velocity.txt
  ${CMAKE_SOURCE_DIR}/data/time.txt)

add_executable(3d_bspline test/3d_bspline.cpp)
add_test(3d_bspline 3d_bspline 
  ${CMAKE_SOURCE_DIR}/data/cpx.txt
  ${CMAKE_SOURCE_DIR}/data/cpy.txt
  ${CMAKE_SOURCE_DIR}/data/cpz.txt
  ${CMAKE_SOURCE_DIR}/data/posx.txt
  ${CMAKE_SOURCE_DIR}/data/posy.txt
  ${CMAKE_SOURCE_DIR}/data/posz.txt
  ${CMAKE_SOURCE_DIR}/data/time.txt)

  add_executable(3d_bspline_single test/3d_bspline_single.cpp)
  add_test(3d_bspline_single 3d_bspline_single 
    ${CMAKE_SOURCE_DIR}/data/cpx.txt
    ${CMAKE_SOURCE_DIR}/data/cpy.txt
    ${CMAKE_SOURCE_DIR}/data/cpz.txt
    ${CMAKE_SOURCE_DIR}/data/posx.txt
    ${CMAKE_SOURCE_DIR}/data/posy.txt
    ${CMAKE_SOURCE_DIR}/data/posz.txt
    ${CMAKE_SOURCE_DIR}/data/time.txt)

# target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
# target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/..)