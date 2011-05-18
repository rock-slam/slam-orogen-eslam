PROJECT(eslam)
cmake_minimum_required(VERSION 2.6)

include(FindPkgConfig)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

option( DEBUG_VIZ "Switch on visualization of environment (only for DEBUG)." )

if( DEBUG_VIZ )
    message( "Using debug visualization." )
    add_definitions(-DDEBUG_VIZ)

    pkg_check_modules(ESLAMVIZ "eslam-viz")
    message(vizdirs ${ESLAMVIZ_LIBRARIES})

    include_directories(${ESLAMVIZ_INCLUDE_DIRS})
    link_directories(${ESLAMVIZ_LIBRARY_DIRS})
endif( DEBUG_VIZ )

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(eslamBase)


# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

