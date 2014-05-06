# Copyright: (C) 2014 RobotCub Consortium
# Author: ---
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


cmake_minimum_required(VERSION 2.8)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(RBDemo)

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

add_definitions(${YARP_DEFINES})   # add yarp definitions - this contains also -D_REENTRANT
include(YarpInstallationHelpers)   # pick up yarp's cmake scripts


## add modules

add_subdirectory(disparityComp)
add_subdirectory(faceDetector)
add_subdirectory(graspingModule)
add_subdirectory(nearThingsDetector)
add_subdirectory(reachModule)
add_subdirectory(touchdetector)

## then apps
add_subdirectory(app)

icubcontrib_finalize_export(periPersonalSpace)
icubcontrib_add_uninstall_target()
