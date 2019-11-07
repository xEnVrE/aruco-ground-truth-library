#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

project("CameraLibrary")

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

# Eigen
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

if (USE_YARP AND USE_ICUB)
# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             cv
             dev
             eigen
             os
             sig
             )

# ICUB
find_package(ICUB REQUIRED)
endif()

set(${LIBRARY_TARGET_NAME}_HDR
    include/Camera.h
    include/CameraParameters.h
    )

set(${LIBRARY_TARGET_NAME}_SRC
    src/Camera.cpp
    src/CameraParameters.cpp
    )

if (USE_YARP)
  list(APPEND ${LIBRARY_TARGET_NAME}_HDR
    include/YarpCamera.h
    )

  list(APPEND ${LIBRARY_TARGET_NAME}_SRC
    src/YarpCamera.cpp
    )
endif()

if (USE_YARP AND USE_ICUB)
  list(APPEND ${LIBRARY_TARGET_NAME}_HDR
    include/iCubCamera.h
    include/iCubCameraRelative.h
    )

  list(APPEND ${LIBRARY_TARGET_NAME}_SRC
    src/iCubCamera.cpp
    src/iCubCameraRelative.cpp
    )
endif()

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_HDR} ${${LIBRARY_TARGET_NAME}_SRC})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                                                  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                  )

target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                      Eigen3::Eigen
                      ${OpenCV_LIBS}
                      )

if (USE_YARP AND USE_ICUB)
  target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                        YARP::YARP_cv
                        YARP::YARP_dev
                        YARP::YARP_eigen
                        YARP::YARP_init
                        YARP::YARP_os
                        YARP::YARP_sig
                        ${ICUB_LIBARIES}
                        )
endif()
