#!/usr/bin/python3
# -*- coding: utf-8 -*-
# generated from catkin/cmake/template/script.py.in
# creates a relay to a python script source file, acting as that file.
# The purpose is that of a symlink
python_script = '/home/bien/ros1/clone_git/clone_project/humainoid/op3/OP3_Humanoid_Robot/no1_source/no2_modun/no1_ROBOTIS-OP3/src/dynamixel_sdk/src/dynamixel_sdk/group_bulk_write.py'
with open(python_script, 'r') as fh:
    context = {
        '__builtins__': __builtins__,
        '__doc__': None,
        '__file__': python_script,
        '__name__': __name__,
        '__package__': None,
    }
    exec(compile(fh.read(), python_script, 'exec'), context)
