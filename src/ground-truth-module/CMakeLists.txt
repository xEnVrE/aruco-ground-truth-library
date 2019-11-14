#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

find_package(RobotsIO REQUIRED)

set(EXE_TARGET_NAME test-aruco-measurement)

set(${EXE_TARGET_NAME}_HDR
    )

set(${EXE_TARGET_NAME}_SRC
    src/TestArucoMeasurement.cpp
    )

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      GroundTruthLibrary
                      )

# install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)


if (USE_YARP)
    find_package(YARP COMPONENTS sig)

    set(EXE_TARGET_NAME test-reverse-link-measurement)

    set(${EXE_TARGET_NAME}_HDR
        )

    set(${EXE_TARGET_NAME}_SRC
        src/TestReverseLinkMeasurement.cpp
        )

    add_executable(${EXE_TARGET_NAME}
                   ${${EXE_TARGET_NAME}_HDR}
                   ${${EXE_TARGET_NAME}_SRC}
                   )

    target_include_directories(${EXE_TARGET_NAME}
                               PRIVATE
                               )

    target_link_libraries(${EXE_TARGET_NAME}
                          PRIVATE
                          GroundTruthLibrary
                          YARP::YARP_sig
                          )
endif()
# install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

if (USE_VTK AND USE_YARP)
    find_package(VTK REQUIRED)
    include(${VTK_USE_FILE})

    set(EXE_TARGET_NAME test-visualization)

    set(${EXE_TARGET_NAME}_HDR
        )

    set(${EXE_TARGET_NAME}_SRC
        src/TestVisualization.cpp
        )

    add_executable(${EXE_TARGET_NAME}
                   ${${EXE_TARGET_NAME}_HDR}
                   ${${EXE_TARGET_NAME}_SRC}
                   )

    target_include_directories(${EXE_TARGET_NAME}
                               PRIVATE
                               )

    target_link_libraries(${EXE_TARGET_NAME}
                          PRIVATE
                          RobotsIO::RobotsIO
                          VisualizationLibrary
                          ${VTK_LIBRARIES}
                          )

    # install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
endif()

find_package(OpenCV REQUIRED)

if (USE_SUPERIMPOSE)
    find_package(BayesFilters REQUIRED)

    set(EXE_TARGET_NAME test-superimposition)

    set(${EXE_TARGET_NAME}_HDR
        )

    set(${EXE_TARGET_NAME}_SRC
        src/TestSuperimposition.cpp
        )

    add_executable(${EXE_TARGET_NAME}
                   ${${EXE_TARGET_NAME}_HDR}
                   ${${EXE_TARGET_NAME}_SRC}
                   )

    target_include_directories(${EXE_TARGET_NAME}
                               PRIVATE
                               )

    target_link_libraries(${EXE_TARGET_NAME}
                          PRIVATE
                          BayesFilters::BayesFilters
                          RobotsIO::RobotsIO
                          VisualizationLibrary
                          ${OpenCV_LIBS}
                          )

    # install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
endif()

set(EXE_TARGET_NAME test-camera)

set(${EXE_TARGET_NAME}_HDR
    )

set(${EXE_TARGET_NAME}_SRC
    src/TestCamera.cpp
    )

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      RobotsIO::RobotsIO
                      )

# install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)


set(EXE_TARGET_NAME test-camera-log)

set(${EXE_TARGET_NAME}_HDR
    )

set(${EXE_TARGET_NAME}_SRC
    src/TestCameraLog.cpp
    )

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      RobotsIO::RobotsIO
                      )

# install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)


set(EXE_TARGET_NAME test-camera-log-read)

set(${EXE_TARGET_NAME}_HDR
    )

set(${EXE_TARGET_NAME}_SRC
    src/TestCameraLogRead.cpp
    )

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      RobotsIO::RobotsIO
                      )

# install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)


option(USE_ICUB        "Use iCub" OFF)

if (USE_YARP AND USE_ICUB)

    set(EXE_TARGET_NAME icub-extr-calib-collector)

    find_package(YARP CONFIG REQUIRED
                 COMPONENTS
                 dev
                 )
    find_package(ICUB REQUIRED)

    set(${EXE_TARGET_NAME}_HDR
        )

    set(${EXE_TARGET_NAME}_SRC
        src/iCubExtrinsicCalibrationCollector.cpp
        )

    add_executable(${EXE_TARGET_NAME}
                   ${${EXE_TARGET_NAME}_HDR}
                   ${${EXE_TARGET_NAME}_SRC}
                   )

    target_include_directories(${EXE_TARGET_NAME}
                               PRIVATE
                               )

    target_link_libraries(${EXE_TARGET_NAME}
                          PRIVATE
                          RobotsIO::RobotsIO
                          YARP::YARP_dev
                          ${ICUB_LIBRARIES}
                          )

    # install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
endif()

set(EXE_TARGET_NAME icub-extr-calib-processor)

find_package(YARP CONFIG REQUIRED
             COMPONENTS
             dev
             )
find_package(ICUB REQUIRED)

set(${EXE_TARGET_NAME}_HDR
    )

set(${EXE_TARGET_NAME}_SRC
    src/iCubExtrinsicCalibrationProcessor.cpp
    )

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      RobotsIO::RobotsIO
                      GroundTruthLibrary
                      )

# install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)


set(EXE_TARGET_NAME test-extrinsic-aruco)

find_package(YARP CONFIG REQUIRED
             COMPONENTS
             dev
             )
find_package(ICUB REQUIRED)

set(${EXE_TARGET_NAME}_HDR
    )

set(${EXE_TARGET_NAME}_SRC
    src/TestExtrinsicAruco.cpp
    )

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      RobotsIO::RobotsIO
                      GroundTruthLibrary
                      )

# install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)


set(EXE_TARGET_NAME icub-extr-calib-train)

find_package(YARP CONFIG REQUIRED
             COMPONENTS
             eigen
             sig
             )
find_package(ICUB REQUIRED)

set(${EXE_TARGET_NAME}_HDR
    )

set(${EXE_TARGET_NAME}_SRC
    src/iCubExtrinsicCalibrationTrain.cpp
    )

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      YARP::YARP_eigen
                      YARP::YARP_sig
                      ${ICUB_LIBRARIES}
                      )

# install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)


set(EXE_TARGET_NAME test-trained-model)

find_package(YARP CONFIG REQUIRED
             COMPONENTS
             eigen
             sig
             )
find_package(ICUB REQUIRED)

set(${EXE_TARGET_NAME}_HDR
    )

set(${EXE_TARGET_NAME}_SRC
    src/TestTrainedModel.cpp
    )

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      YARP::YARP_eigen
                      YARP::YARP_sig
                      ${ICUB_LIBRARIES}
                      )

# install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
