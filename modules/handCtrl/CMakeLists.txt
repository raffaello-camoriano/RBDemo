# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME handCtrl)
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

source_group("Source Files" FILES src/main.cpp)
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} src/main.cpp)
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} actionPrimitives)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
icubcontrib_add_uninstall_target()

########## application
file(GLOB conf_ini ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/*.ini)
file(GLOB app_xml ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${conf_ini} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME})
yarp_install(FILES ${app_xml} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR}/${PROJECTNAME})

# Debug: Print out all variables
get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()



