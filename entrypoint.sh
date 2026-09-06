#!/bin/bash
set -e

# ROS 2 ve workspace ortamını her komut için yükle
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash

# Araç Foxy'nin VARSAYILAN DDS'ini (Fast-DDS) kullanıyor → eşleşmek için Fast-DDS.
# Dışarıdan -e RMW_IMPLEMENTATION verilirse ona saygı gösterir, yoksa fastrtps.
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

exec "$@"
