cmake_minimum_required(VERSION 3.23)
project(Bot)

set(CMAKE_CXX_STANDARD 14)

add_executable(Bot main.cpp robot.h forward_kinematics.h transformation_matrix.h)

