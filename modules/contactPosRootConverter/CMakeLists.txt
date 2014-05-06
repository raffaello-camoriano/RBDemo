# Copyright: 2014 iCub Facility, Istituto Italiano di Tecnologia
# Author: Giulia Pasquale

CMAKE_MINIMUM_REQUIRED(VERSION 2.7)
SET(PROJECTNAME contactPosRootConverter)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)

## optionally: use cmake find scripts provided by YARP and iCub
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)

icubcontrib_set_default_prefix() 
include(ICUBcontribOptions)

if(NOT ICUB_USE_IPOPT)
    MESSAGE(FATAL_ERROR "IPOPT is required")
endif(NOT ICUB_USE_IPOPT)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} main.cpp)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")	
#target_link_libraries(${PROJECTNAME} ${ICUB_LIBRARIES})
target_link_libraries(${PROJECTNAME} skinDynLib iKin)

include(YarpInstallationHelpers)
install(TARGETS ${PROJECTNAME} DESTINATION bin)

icubcontrib_add_uninstall_target()