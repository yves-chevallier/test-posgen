cmake_minimum_required(VERSION 3.14)
project(positiongen)

set(CMAKE_CXX_STANDARD 17)

find_package(pybind11 REQUIRED)

#add_library(positiongen MODULE bindings.cpp positionGenerator.cpp)

pybind11_add_module(positiongen bindings.cpp positionGenerator.cpp)
target_include_directories(positiongen PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(positiongen PRIVATE pybind11::module)
