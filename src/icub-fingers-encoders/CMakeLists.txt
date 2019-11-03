#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

project("iCubFingersEncoders")

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

# Eigen
find_package(Eigen3 REQUIRED)

# iCUB
find_package(ICUB REQUIRED)

# YARP
find_package(YARP CONFIG REQUIRED
                  COMPONENTS
                  OS
                  dev
                  math
                  REQUIRED)

set(${LIBRARY_TARGET_NAME}_HDR include/iCubFingersEncoders.h)
set(${LIBRARY_TARGET_NAME}_SRC src/iCubFingersEncoders.cpp)
set(${LIBRARY_TARGET_NAME}_CONF config/analogs_configuration.ini)

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_HDR} ${${LIBRARY_TARGET_NAME}_SRC})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                                                  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                  )

target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                                             Eigen3::Eigen
                                             YARP::YARP_dev
                                             YARP::YARP_math
                                             YARP::YARP_OS
                                             ${ICUB_LIBRARIES}
                                             )

yarp_install(FILES ${${LIBRARY_TARGET_NAME}_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/icub-fingers-encoders)
