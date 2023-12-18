# Install script for directory: /home/bien/ros1/clone_git/clone_project/humainoid/op3/OP3_Humanoid_Robot/no1_source/no2_modun/no1_ROBOTIS-OP3/src/op3_localization

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/bien/ros1/clone_git/clone_project/humainoid/op3/OP3_Humanoid_Robot/no1_source/no2_modun/no1_ROBOTIS-OP3/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/bien/ros1/clone_git/clone_project/humainoid/op3/OP3_Humanoid_Robot/no1_source/no2_modun/no1_ROBOTIS-OP3/build/op3_localization/catkin_generated/installspace/op3_localization.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/op3_localization/cmake" TYPE FILE FILES
    "/home/bien/ros1/clone_git/clone_project/humainoid/op3/OP3_Humanoid_Robot/no1_source/no2_modun/no1_ROBOTIS-OP3/build/op3_localization/catkin_generated/installspace/op3_localizationConfig.cmake"
    "/home/bien/ros1/clone_git/clone_project/humainoid/op3/OP3_Humanoid_Robot/no1_source/no2_modun/no1_ROBOTIS-OP3/build/op3_localization/catkin_generated/installspace/op3_localizationConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/op3_localization" TYPE FILE FILES "/home/bien/ros1/clone_git/clone_project/humainoid/op3/OP3_Humanoid_Robot/no1_source/no2_modun/no1_ROBOTIS-OP3/src/op3_localization/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/op3_localization/op3_localization" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/op3_localization/op3_localization")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/op3_localization/op3_localization"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/op3_localization" TYPE EXECUTABLE FILES "/home/bien/ros1/clone_git/clone_project/humainoid/op3/OP3_Humanoid_Robot/no1_source/no2_modun/no1_ROBOTIS-OP3/devel/lib/op3_localization/op3_localization")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/op3_localization/op3_localization" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/op3_localization/op3_localization")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/op3_localization/op3_localization"
         OLD_RPATH "/opt/ros/noetic/lib:/home/bien/ros1/clone_git/clone_project/humainoid/op3/OP3_Humanoid_Robot/no1_source/no2_modun/no1_ROBOTIS-OP3/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/op3_localization/op3_localization")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/op3_localization" TYPE DIRECTORY FILES "/home/bien/ros1/clone_git/clone_project/humainoid/op3/OP3_Humanoid_Robot/no1_source/no2_modun/no1_ROBOTIS-OP3/src/op3_localization/include/op3_localization/")
endif()

