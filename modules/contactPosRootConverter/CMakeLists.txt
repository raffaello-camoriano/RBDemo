# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Author: Giulia Pasquale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME contactPosRootConverter)

PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)

if(NOT ICUB_USE_IPOPT)
    MESSAGE(FATAL_ERROR "IPOPT is required")
endif(NOT ICUB_USE_IPOPT)

include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} main.cpp)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} skinDynLib iKin)

install(TARGETS ${PROJECTNAME} DESTINATION bin)