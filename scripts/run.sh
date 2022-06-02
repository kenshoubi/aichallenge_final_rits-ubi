#!/bin/bash -e

SCRIPT_DIR=$(cd $(dirname $0); pwd)


if [ -z $1 ]; then
  MAP_PATH=$SCRIPT_DIR/../map/data
else
  MAP_PATH=$1
fi


ros2 launch autoware_launch autoware.launch.xml map_path:=$MAP_PATH vehicle_model:=ymc_golfcart sensor_model:=aip_aichallenge

