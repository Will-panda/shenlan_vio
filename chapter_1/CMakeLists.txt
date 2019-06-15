cmake_minimum_required(VERSION 3.0)
project(chapter1)

find_package(Sophus REQUIRED)
INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS})


add_executable(compare src/compare.cpp)

target_link_libraries(compare ${Sophus_LIBRARIES})