# Copyright: 2014 iCub Facility, IIT
# Author: Tanis Mar
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.7)
set(PROJECTNAME detectNear)
project(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(OpenCV)
find_package(ICUB)
find_package(ICUBcontrib)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

## include(YarpIDL)
## yarp_idl_to_dir(${PROJECTNAME}.thrift ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)
#set(idl ${PROJECTNAME}.thrift)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})
# source_group("IDL Files" FILES ${idl})

add_subdirectory(app)


include_directories(#${PCL_INCLUDE_DIRS} 
                    ${CMAKE_SOURCE_DIR} 
                    ${PROJECT_SOURCE_DIR}/include 
                    #${thrift3D_INCLUDE_DIRS} 
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS} 
                    )                    



                    
add_executable (${PROJECTNAME} ${source} ${header}  ${idl})
target_link_libraries (${PROJECTNAME} ${ICUB_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)
