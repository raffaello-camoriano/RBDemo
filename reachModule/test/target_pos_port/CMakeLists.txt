# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Author: Matej Hoffmann
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

SET(PROJECTNAME target_pos_port)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp)

find_package(YARP)
find_package(ICUB)

include_directories(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME}  ${YARP_LIBRARIES})