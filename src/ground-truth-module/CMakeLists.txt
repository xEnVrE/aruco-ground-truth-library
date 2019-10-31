#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

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

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)


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
                      )

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
