# Copyright: 2014 iCub Facility, Istituto Italiano di Tecnologia
# Author: Giulia Pasquale

CMAKE_MINIMUM_REQUIRED(VERSION 2.7)
SET(PROJECTNAME touchDetector)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
#find_package(GSL)
#find_package(OpenCV)
## optionally: use cmake find scripts provided by YARP and iCub

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)
#include(iCubHelpers)
#include(iCubOptions)
#include(iCubFindDependencies)

## load some macros useful for applications (see below)
include(YarpInstallationHelpers)

if(NOT ICUB_USE_IPOPT)
    MESSAGE(FATAL_ERROR "IPOPT is required")
endif(NOT ICUB_USE_IPOPT)

## set CMAKE_INSTALL_PREFIX to the ICUBcontrib one and print warning if user modifies it
icubcontrib_set_default_prefix() 

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
#include_directories(${GSL_INCLUDE_DIRS})
#include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECTNAME} touchDetector.cpp)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
#target_link_libraries(${PROJECTNAME} ${GSL_LIBRARIES} ${OpenCV_LIBRARIES})

## if you want to link against all libraries in the iCub build, rely on the ICUB_LIBRARIES variable:
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")	
#target_link_libraries(${PROJECT_NAME} ${ICUB_LIBRARIES})
## but you might prefer to just link the one you actually need: (important: notice that we use the name of the project)
target_link_libraries(${PROJECT_NAME} skinDynLib iKin)

## to install data files:
include(YarpInstallationHelpers)
set(conf touchDetector/conf/touchDetector.ini)
## alternatively, if you are too lazy to list all files explicitly, you can "GLOB" them with:
#file(GLOB conf conf/*.ini)

install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/touchDetector/conf)
