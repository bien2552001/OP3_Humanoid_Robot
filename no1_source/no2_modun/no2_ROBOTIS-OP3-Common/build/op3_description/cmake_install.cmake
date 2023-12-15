# Install script for directory: /home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/src/op3_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install" TYPE PROGRAM FILES "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install" TYPE PROGRAM FILES "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install/setup.bash;/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install" TYPE FILE FILES
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/catkin_generated/installspace/setup.bash"
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install/setup.sh;/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install" TYPE FILE FILES
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/catkin_generated/installspace/setup.sh"
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install/setup.zsh;/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install" TYPE FILE FILES
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/catkin_generated/installspace/setup.zsh"
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/install" TYPE FILE FILES "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/catkin_generated/installspace/op3_description.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/op3_description/cmake" TYPE FILE FILES
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/catkin_generated/installspace/op3_descriptionConfig.cmake"
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/catkin_generated/installspace/op3_descriptionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/op3_description" TYPE FILE FILES "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/src/op3_description/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/op3_description" TYPE DIRECTORY FILES
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/src/op3_description/doc"
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/src/op3_description/launch"
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/src/op3_description/meshes"
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/src/op3_description/rviz"
    "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/src/op3_description/urdf"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/bien/ros1/ros1/clone_project/humainoid/op3/project_test/op3_test/op3_test1/no2_ROBOTIS-OP3-Common/build/op3_description/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
