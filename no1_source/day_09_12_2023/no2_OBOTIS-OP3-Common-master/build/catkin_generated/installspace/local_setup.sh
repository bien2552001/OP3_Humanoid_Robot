#!/usr/bin/env sh
# generated from catkin/cmake/template/local_setup.sh.in

# since this file is sourced either use the provided _CATKIN_SETUP_DIR
# or fall back to the destination set at configure time
: ${_CATKIN_SETUP_DIR:=/home/bien/ros1/clone_git/clone_project/humainoid/no1_ROBOTIS_OP3_BRANCH_MASTER/op3_test/day_09_12_2023/no2_OBOTIS-OP3-Common-master/install}
CATKIN_SETUP_UTIL_ARGS="--extend --local"
. "$_CATKIN_SETUP_DIR/setup.sh"
unset CATKIN_SETUP_UTIL_ARGS