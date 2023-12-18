# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;roslib;std_msgs;geometry_msgs;robotis_controller_msgs;cmake_modules;robotis_framework_common;robotis_device;robotis_math;op3_kinematics_dynamics;op3_tuning_module_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lop3_tuning_module".split(';') if "-lop3_tuning_module" != "" else []
PROJECT_NAME = "op3_tuning_module"
PROJECT_SPACE_DIR = "/home/bien/ros1/clone_git/clone_project/humainoid/op3/OP3_Humanoid_Robot/no1_source/no2_modun/no1_ROBOTIS-OP3/install"
PROJECT_VERSION = "0.2.1"
