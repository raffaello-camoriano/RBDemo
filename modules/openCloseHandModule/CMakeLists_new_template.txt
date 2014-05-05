# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME iCubWriter)
project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
include(YarpInstallationHelpers)

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

set(TinyXML_INCLUDE_DIRS "$ENV{YARP_ROOT}/extern/tinyxml/tinyxml" CACHE PATH "TinyXML include directory" FORCE)
set(TinyXML_CODE ${TinyXML_INCLUDE_DIRS}/tinystr.cpp
                 ${TinyXML_INCLUDE_DIRS}/tinyxml.cpp
                 ${TinyXML_INCLUDE_DIRS}/tinyxmlerror.cpp
                 ${TinyXML_INCLUDE_DIRS}/tinyxmlparser.cpp)

source_group("Source Files" FILES src/main.cpp)
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

add_executable(${PROJECTNAME} src/main.cpp ${TinyXML_CODE})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} actionPrimitives)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
icubcontrib_add_uninstall_target()

########## application
file(GLOB conf_ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB conf_svg ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.svg)
yarp_install(FILES ${conf_ini} ${conf_svg} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME})

