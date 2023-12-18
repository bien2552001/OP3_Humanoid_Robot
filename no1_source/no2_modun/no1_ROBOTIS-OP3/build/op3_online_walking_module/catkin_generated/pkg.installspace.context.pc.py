# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include;/usr/include/eigen3;/usr/share/orocos_kdl/cmake/../../../include".split(';') if "${prefix}/include;/usr/include;/usr/include/eigen3;/usr/share/orocos_kdl/cmake/../../../include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;roslib;std_msgs;sensor_msgs;geometry_msgs;robotis_controller_msgs;op3_online_walking_module_msgs;cmake_modules;robotis_framework_common;robotis_device;robotis_math;op3_balance_control".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lop3_online_walking_module;-lorocos-kdl".split(';') if "-lop3_online_walking_module;-lorocos-kdl" != "" else []
PROJECT_NAME = "op3_online_walking_module"
PROJECT_SPACE_DIR = "/home/bien/ros1/clone_git/clone_project/humainoid/op3/OP3_Humanoid_Robot/no1_source/no2_modun/no1_ROBOTIS-OP3/install"
PROJECT_VERSION = "0.2.1"
