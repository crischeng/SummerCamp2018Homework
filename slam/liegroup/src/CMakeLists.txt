cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)

include(PICMake)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(NOT USER_NAME)
    set(USER_NAME src)
endif()

include_directories(../${USER_NAME})

pi_add_target(LieGroup BIN main.cpp REQUIRED GSLAM Eigen3 System)
pi_report_target()
pi_install(TARGETS LieGroup)
