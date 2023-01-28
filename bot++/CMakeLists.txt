cmake_minimum_required(VERSION 3.23)
project(bot++)

set(CMAKE_CXX_STANDARD 14)

add_executable(bot++ main.cpp inverse_kinematics.h transformation_matrix.h)
