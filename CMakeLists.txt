cmake_minimum_required(VERSION 3.7)
project(exploration_transform_implement)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp transform_explorer.cpp transform_explorer.h)
add_executable(exploration_transform_implement ${SOURCE_FILES})