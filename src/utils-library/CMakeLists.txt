#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

project("UtilsLibrary")

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

# BayesFilters
find_package(BayesFilters REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

if (USE_YARP)
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             cv
             eigen
             sig
             )
endif()

set(${LIBRARY_TARGET_NAME}_HDR
    include/Probe.h
    include/ProbeContainer.h
    )

set(${LIBRARY_TARGET_NAME}_SRC
    src/Probe.cpp
    src/ProbeContainer.cpp
    )

if (USE_YARP)
  list(APPEND ${LIBRARY_TARGET_NAME}_HDR
    include/YarpBufferedPort.hpp
    include/YarpVectorOfProbe.hpp
    include/YarpImageOfProbe.hpp
    )
endif()

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_HDR} ${${LIBRARY_TARGET_NAME}_SRC})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                                                  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                  )

target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                      BayesFilters::BayesFilters
                      Eigen3::Eigen
                      ${OpenCV_LIBS}
                      )
if (USE_YARP)
target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                      YARP::YARP_init
                      YARP::YARP_OS
                      YARP::YARP_eigen
                      YARP::YARP_cv
                      )
endif()
