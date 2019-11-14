#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

option(BUILD_MODULES       "Build modules"              OFF)
option(USE_YARP            "Use YARP"                   OFF)

add_subdirectory(ground-truth-library)

if (BUILD_MODULES)
  add_subdirectory(ground-truth-module)
endif()
