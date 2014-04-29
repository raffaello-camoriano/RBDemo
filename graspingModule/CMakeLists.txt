# Copyright: (C) 2010 RobotCub Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME graspingModule)

PROJECT(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)

SET(folder_source main.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS}
					${actionPrimitives_INCLUDE_DIRS}
					${skinDynLib_INCLUDE_DIRS}					
                    ${perceptiveModels_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} actionPrimitives
					${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

########## application
#file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
#yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME})


