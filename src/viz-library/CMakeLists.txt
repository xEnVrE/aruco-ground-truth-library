#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

option(USE_SUPERIMPOSE "Use SuperimposeMesh" OFF)
option(USE_VTK         "Use VTK"             OFF)

project("VisualizationLibrary")

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

include(CMakeRC)

# Eigen
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# RobotsIO
find_package(RobotsIO REQUIRED)

if (USE_VTK)
    # VTK
    find_package(VTK REQUIRED)
    include(${VTK_USE_FILE})
endif()

if (USE_SUPERIMPOSE)
  # Superimposemesh
  find_package(SuperimposeMesh 0.11.100 REQUIRED)
endif()

if (USE_YARP)
    # YARP
    find_package(YARP CONFIG REQUIRED
                 COMPONENTS
                 eigen
                 os
                 sig
                 )
endif()

set(${LIBRARY_TARGET_NAME}_HDR
    include/iCubForwardKinematics.h
    )

set(${LIBRARY_TARGET_NAME}_SRC
    src/iCubForwardKinematics.cpp
    )

if (USE_VTK)
    list(APPEND ${LIBRARY_TARGET_NAME}_HDR
        include/MeshResources.h
        include/VtkContainer.h
        include/VtkContent.h
        include/VtkMeshOBJ.h
        include/VtkPointCloud.h
        include/VtkUpdateHandler.h
        include/iCubForwardKinematics.h
        include/vtkOBJResource.h
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC
        src/MeshResources.cpp
        src/VtkContainer.cpp
        src/VtkContent.cpp
        src/VtkMeshOBJ.cpp
        src/VtkPointCloud.cpp
        src/VtkUpdateHandler.cpp
        src/iCubForwardKinematics.cpp
        src/vtkOBJResource.cpp
    )
endif()

if (USE_SUPERIMPOSE AND USE_YARP)
    list(APPEND ${LIBRARY_TARGET_NAME}_HDR
         include/SIiCubHand.h
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC
         src/SIiCubHand.cpp
    )
endif()

if (USE_VTK AND USE_YARP)
    list(APPEND ${LIBRARY_TARGET_NAME}_HDR
         include/VtkiCubHand.h
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC
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
                      RobotsIO::RobotsIO
                      ${OpenCV_LIBS}
                      PRIVATE
                      VisualizationLibrary::RCShader
                      )

if (USE_VTK)
    target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                          ${VTK_LIBRARIES}
                          )
endif()

if (USE_SUPERIMPOSE)
    target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                          SI::SuperimposeMesh
                          )
endif()

if (USE_YARP)
    target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                          YARP::YARP_eigen
                          YARP::YARP_init
                          YARP::YARP_os
                          YARP::YARP_sig
                          )
endif()
