PROJECT(eslam)
cmake_minimum_required(VERSION 2.6)

include(FindPkgConfig)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

option( DEBUG_VIZ "Switch on visualization of environment (only for DEBUG)." )

if( DEBUG_VIZ )
    message( "Using debug visualization." )
    add_definitions(-DDEBUG_VIZ)
    find_package(Qt4 REQUIRED)
    set(QT_USE_OPENGL TRUE)
    include(${QT_USE_FILE})
endif( DEBUG_VIZ )

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(eslamBase)


# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

