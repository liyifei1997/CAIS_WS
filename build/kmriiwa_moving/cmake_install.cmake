# Install script for directory: /home/yfl/CAIS_WS/src/kmriiwa_moving

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yfl/CAIS_WS/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yfl/CAIS_WS/build/kmriiwa_moving/catkin_generated/installspace/kmriiwa_moving.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kmriiwa_moving/cmake" TYPE FILE FILES
    "/home/yfl/CAIS_WS/build/kmriiwa_moving/catkin_generated/installspace/kmriiwa_movingConfig.cmake"
    "/home/yfl/CAIS_WS/build/kmriiwa_moving/catkin_generated/installspace/kmriiwa_movingConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kmriiwa_moving" TYPE FILE FILES "/home/yfl/CAIS_WS/src/kmriiwa_moving/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kmriiwa_moving" TYPE PROGRAM FILES
    "/home/yfl/CAIS_WS/src/kmriiwa_moving/scripts/arm_base_moving.py"
    "/home/yfl/CAIS_WS/src/kmriiwa_moving/scripts/arm_base_movingV2.py"
    "/home/yfl/CAIS_WS/src/kmriiwa_moving/scripts/arm_base_movingV4.py"
    "/home/yfl/CAIS_WS/src/kmriiwa_moving/scripts/arm_base_movingV5.py"
    "/home/yfl/CAIS_WS/src/kmriiwa_moving/scripts/arm_base_movingV6.py"
    "/home/yfl/CAIS_WS/src/kmriiwa_moving/scripts/kmr_move_square.py"
    "/home/yfl/CAIS_WS/src/kmriiwa_moving/scripts/kmr_square.py"
    "/home/yfl/CAIS_WS/src/kmriiwa_moving/scripts/listener.py"
    )
endif()

