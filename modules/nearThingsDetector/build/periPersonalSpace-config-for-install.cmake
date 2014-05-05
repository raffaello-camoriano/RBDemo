# Copyright: (C) 2013 Istituto Italiano di Tecnologia
# Author: Elena Ceseracciu
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if (NOT periPersonalSpace_FOUND)

set(periPersonalSpace_LIBRARIES "" CACHE INTERNAL "List of periPersonalSpace libraries")

include("C:/ProgramsTanis/robotology/ICUBcontrib/lib/periPersonalSpace/periPersonalSpace-export-install.cmake")
include("C:/ProgramsTanis/robotology/ICUBcontrib/lib/periPersonalSpace/periPersonalSpace-export-install-includes.cmake")

set (periPersonalSpace_FOUND TRUE)
endif (NOT periPersonalSpace_FOUND)
