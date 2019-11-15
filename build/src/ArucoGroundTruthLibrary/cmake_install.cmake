# Install script for directory: /home/xenvre/robot-code/aruco-ground-truth-library/src/ArucoGroundTruthLibrary

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/xenvre/robot-install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "0")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xlibx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/xenvre/robot-code/aruco-ground-truth-library/build/lib/libArucoGroundTruthLibary.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ArucoGroundTruthLibary" TYPE FILE FILES
    "/home/xenvre/robot-code/aruco-ground-truth-library/src/ArucoGroundTruthLibrary/include/ArucoGroundTruthLibrary/ArucoBoardMeasurement.h"
    "/home/xenvre/robot-code/aruco-ground-truth-library/src/ArucoGroundTruthLibrary/include/ArucoGroundTruthLibrary/ArucoMarkerMeasurement.h"
    "/home/xenvre/robot-code/aruco-ground-truth-library/src/ArucoGroundTruthLibrary/include/ArucoGroundTruthLibrary/ArucoMeasurement.h"
    "/home/xenvre/robot-code/aruco-ground-truth-library/src/ArucoGroundTruthLibrary/include/ArucoGroundTruthLibrary/ReverseLinkMeasurement.h"
    "/home/xenvre/robot-code/aruco-ground-truth-library/src/ArucoGroundTruthLibrary/include/ArucoGroundTruthLibrary/ThreePointReverseLinkMeasurement.h"
    )
endif()

