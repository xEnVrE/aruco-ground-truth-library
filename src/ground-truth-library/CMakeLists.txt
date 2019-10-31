#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

project("GroundTruthLibrary")

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

# BayesFilters
find_package(BayesFilters REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

set(${LIBRARY_TARGET_NAME}_HDR
    include/ArucoBoardMeasurement.h
    include/ArucoMarkerMeasurement.h
    include/ArucoMeasurement.h
    include/ReverseLinkMeasurement.h
    )

set(${LIBRARY_TARGET_NAME}_SRC
    src/ArucoBoardMeasurement.cpp
    src/ArucoMarkerMeasurement.cpp
    src/ArucoMeasurement.cpp
    src/ReverseLinkMeasurement.cpp
    )


add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_HDR} ${${LIBRARY_TARGET_NAME}_SRC})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                                                  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                  )

target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                      BayesFilters::BayesFilters
                      CameraLibrary
                      Eigen3::Eigen
                      UtilsLibrary
                      ${OpenCV_LIBS}
                      )
