PROJECT(eslam)
cmake_minimum_required(VERSION 2.6)

include(FindPkgConfig)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(eslamBase)


# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

