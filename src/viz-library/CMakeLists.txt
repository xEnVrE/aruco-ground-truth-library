#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

project("VisualizationLibrary")

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

include(CMakeRC)

# Eigen
find_package(Eigen3 REQUIRED)

# VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

if (USE_YARP AND USE_ICUB)
# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             cv
             dev
             eigen
             sig
             )

# ICUB
find_package(ICUB REQUIRED)
endif()

set(${LIBRARY_TARGET_NAME}_HDR
    include/MeshResources.h
    include/VtkContainer.h
    include/VtkContent.h
    include/VtkMeshOBJ.h
    include/VtkUpdateHandler.h
    include/vtkOBJResource.h
    )

set(${LIBRARY_TARGET_NAME}_SRC
    src/MeshResources.cpp
    src/VtkContainer.cpp
    src/VtkContent.cpp
    src/VtkMeshOBJ.cpp
    src/VtkUpdateHandler.cpp
    src/vtkOBJResource.cpp
    )

if (USE_YARP AND USE_ICUB)
  list(APPEND ${LIBRARY_TARGET_NAME}_HDR
    include/iCubForwardKinematics.h
    include/VtkiCubHand.h
    )

  list(APPEND ${LIBRARY_TARGET_NAME}_SRC
    src/iCubForwardKinematics.cpp
    src/VtkiCubHand.cpp
    )
endif()

cmrc_add_resource_library(${LIBRARY_TARGET_NAME}_RCShader
    ALIAS VisualizationLibrary::RCShader
    NAMESPACE meshes
    PREFIX __prc
    meshes/full_LeftHandPalm_red.obj
    meshes/full_LeftThumb0.obj
    meshes/full_LeftThumbFinger1.obj
    meshes/full_LeftThumbFinger2.obj
    meshes/full_LeftThumbFingerTip.obj
    meshes/full_LeftIndexFinger0.obj
    meshes/full_LeftIndexFinger1.obj
    meshes/full_LeftIndexFinger2.obj
    meshes/full_LeftIndexFingerTip_b.obj
    meshes/full_LeftMiddleFinger0.obj
    meshes/full_LeftMiddleFinger1.obj
    meshes/full_LeftMiddleFinger2.obj
    meshes/full_LeftMiddleFingerTip_b.obj
    meshes/full_LeftRingFinger0.obj
    meshes/full_LeftRingFinger1.obj
    meshes/full_LeftRingFinger2.obj
    meshes/full_LeftRingFingerTip_b.obj
    meshes/full_LeftLittleFinger0.obj
    meshes/full_LeftLittleFinger1.obj
    meshes/full_LeftLittleFinger2.obj
    meshes/full_LeftLittleFingerTip_b.obj
   )

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_HDR} ${${LIBRARY_TARGET_NAME}_SRC})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                                                  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                  )

target_link_libraries(${LIBRARY_TARGET_NAME}
                      PUBLIC
                      Eigen3::Eigen
                      ${VTK_LIBRARIES}
                      PRIVATE
                      VisualizationLibrary::RCShader
                      )

if (USE_YARP AND USE_ICUB)
  target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                        YARP::YARP_OS
                        YARP::YARP_cv
                        YARP::YARP_dev
                        YARP::YARP_eigen
                        YARP::YARP_init
                        YARP::YARP_sig
                        ${ICUB_LIBARIES}
                        )
endif()
