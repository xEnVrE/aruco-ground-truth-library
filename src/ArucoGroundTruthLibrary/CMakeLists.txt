#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(LIBRARY_TARGET_NAME ArucoGroundTruthLibrary)

# BayesFilters
find_package(BayesFilters REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# RobotsIO
find_package(RobotsIO REQUIRED)

set(${LIBRARY_TARGET_NAME}_HDR
    include/ArucoGroundTruthLibrary/ArucoBoardMeasurement.h
    include/ArucoGroundTruthLibrary/ArucoMarkerMeasurement.h
    include/ArucoGroundTruthLibrary/ArucoMeasurement.h
    include/ArucoGroundTruthLibrary/ReverseLinkMeasurement.h
    include/ArucoGroundTruthLibrary/TunableReverseLinkMeasurement.h
    include/ArucoGroundTruthLibrary/ThreePointReverseLinkMeasurement.h
    )

set(${LIBRARY_TARGET_NAME}_SRC
    src/ArucoBoardMeasurement.cpp
    src/ArucoMarkerMeasurement.cpp
    src/ArucoMeasurement.cpp
    src/ReverseLinkMeasurement.cpp
    src/TunableReverseLinkMeasurement.cpp
    src/ThreePointReverseLinkMeasurement.cpp
    )

# Add library
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

# Library properties
set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION       ${${PROJECT_NAME}_VERSION}
                                                        PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

# Include directories
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

# Linker configuration
target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                      BayesFilters::BayesFilters
                      Eigen3::Eigen
                      RobotsIO::RobotsIO
                      ${OpenCV_LIBS}
)

# Specify installation targets, typology and destination folders.
install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                   COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                   COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                                   COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}"        COMPONENT dev
)

set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS ${LIBRARY_TARGET_NAME})
