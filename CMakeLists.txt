cmake_minimum_required(VERSION 3.14)
project(scurvecpp)

set(CMAKE_CXX_STANDARD 17)

find_package(pybind11 REQUIRED)

#add_library(scurvecpp MODULE bindings.cpp scurvecpperator.cpp)

pybind11_add_module(scurvecpp bindings.cpp)
target_include_directories(scurvecpp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(scurvecpp PRIVATE pybind11::module)
