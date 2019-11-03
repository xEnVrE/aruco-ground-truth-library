#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

option(USE_YARP        "Use YARP" OFF)
option(USE_ICUB        "Use ICUB" OFF)
option(USE_VTK         "Use VTK" OFF)

add_subdirectory(camera-library)
add_subdirectory(utils-library)
add_subdirectory(ground-truth-library)

if (USE_YARP)
  option(BUILD_MODULES "Build YARP-based modules" OFF)

  if (BUILD_MODULES)
    add_subdirectory(ground-truth-module)
  endif()

endif()

if (USE_VTK)
  option(BUILD_VISUALIZATION "Build visualization tool" OFF)

  if (BUILD_VISUALIZATION)
    add_subdirectory(viz-library)
  endif()

endif()
