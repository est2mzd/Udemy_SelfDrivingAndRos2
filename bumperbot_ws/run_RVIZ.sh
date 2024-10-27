#!/bin/bash

XACRO_FILE_PATH=/work/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro
ros2 launch urdf_tutorial display.launch.py model:=${XACRO_FILE_PATH}