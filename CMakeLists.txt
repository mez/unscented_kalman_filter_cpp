project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x -Wall)

set(sources
   utility.cc
   main.cc)

add_executable(UnscentedKF ${sources})
