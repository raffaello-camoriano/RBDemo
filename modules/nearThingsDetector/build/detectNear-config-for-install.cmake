# Copyright: (C) 2013 Istituto Italiano di Tecnologia
# Author: Elena Ceseracciu
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if (NOT detectNear_FOUND)

set(detectNear_LIBRARIES "" CACHE INTERNAL "List of detectNear libraries")

include("C:/ProgramsTanis/robotology/ICUBcontrib/lib/detectNear/detectNear-export-install.cmake")
include("C:/ProgramsTanis/robotology/ICUBcontrib/lib/detectNear/detectNear-export-install-includes.cmake")

set (detectNear_FOUND TRUE)
endif (NOT detectNear_FOUND)
