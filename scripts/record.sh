#!/bin/bash -x

# save directory (absolute path)
MNTBASE='/mnt/AIC'
DIR_ROOT="${MNTBASE}/log"
DEVFILE="/dev/disk/by-label/AIC"
DURATION=60
SCRIPT_DIR=$(cd $(dirname $0); pwd)
QOS_PROFILE_PATH=$SCRIPT_DIR/config/qos_profile_overrides_aichallenge.yaml
DIR_SPECIFIED=0

# parse options
while getopts o:d: opt; do
  case "$opt" in
    o)
      DIR_ROOT="$(echo "$OPTARG" | sed 's/^\(.*\)\/log\/*$/\1/')/log"
      DIR_SPECIFIED=1;;
    d)
      DURATION="$OPTARG";;
    *)
      echo "Usage: $0 [-n] [-o DIRNAME] [-d DURATION_SEC]"
      exit 1;;
  esac
done


# check if the SSD is mounted
MNTPT=$(dirname "$DIR_ROOT")

if [ $DIR_SPECIFIED -eq 0 ]; then
  if [ ! -e "$DIR_ROOT" ]; then
    # try mount
    sudo mkdir -p $MNTBASE >> /dev/null
    sudo mount -t ext4 $DEVFILE $MNTBASE >> /dev/null
    if [ $? -ne 0 ]; then
      echo "Mount failed at ${MNTPT}. Connect external SSD for logging or specify record directory [ -o DIRNAME ]"
			exit 1
    fi
  fi
fi

RECORD_LIST="\
.*/velodyne_packets|\
/api/.*|\
/autoware/engage|\
/awapi/.*|\
/control/.*|\
/diagnostics|\
/diagnostics_agg|\
/foa/operation/log|\
/localization/kinematic_state|\
/localization/pose_estimator/exe_time_ms|\
/localization/pose_estimator/initial_pose_with_covariance|\
/localization/pose_estimator/iteration_num|\
/localization/pose_estimator/pose_with_covariance|\
/localization/pose_estimator/pose|\
/localization/pose_estimator/transform_probability|\
/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias|\
/perception/object_recognition/detection/euclidean/camera_lidar_fusion/.*|\
/perception/object_recognition/detection/labeled_clusters|\
/perception/object_recognition/detection/objects|\
/perception/object_recognition/detection/objects_with_feature|\
/perception/object_recognition/detection/rois(.)|\
/perception/object_recognition/objects|\
/perception/object_recognition/tracking/objects|\
/planning/mission_planning/checkpoint|\
/planning/mission_planning/goal|\
/planning/mission_planning/route|\
/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/output/path_candidate|\
/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/.*|\
/planning/scenario_planning/lane_driving/behavior_planning/debug/traffic_signal|\
/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id|\
/planning/scenario_planning/lane_driving/behavior_planning/path|\
/planning/scenario_planning/lane_driving/force_lane_change|\
/planning/scenario_planning/lane_driving/lane_change_approval|\
/planning/scenario_planning/lane_driving/lane_change_candidate_path|\
/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/trajectory|\
/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/debug/wall_marker|\
/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/adaptive_cruise_control/debug_values|\
/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/debug/marker|\
/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/obstacle_stop/debug_values|\
/planning/scenario_planning/lane_driving/motion_planning/surround_obstacle_checker/debug/marker|\
/planning/scenario_planning/lane_driving/motion_planning/surround_obstacle_checker/trajectory|\
/planning/scenario_planning/lane_driving/trajectory|\
/planning/scenario_planning/max_velocity|\
/planning/scenario_planning/motion_velocity_smoother/distance_to_stopline|\
/planning/scenario_planning/parking/trajectory|\
/planning/scenario_planning/scenario_selector/trajectory|\
/planning/scenario_planning/status/stop_reasons|\
/planning/scenario_planning/trajectory|\
/remote/.*|\
/rosout|\
/sensing/camera/.*/camera_info|\
/sensing/camera/.*/image_rect_color/compressed|\
/sensing/camera/.*/image_raw|\
/sensing/camera/traffic_light/image_raw/compressed|\
/sensing/lidar/top/outlier_filtered/pointcloud|\
/system/.*|\
/tf_static|\
/tf|\
/vehicle/.*|\
"

DATE=`date +'%y%m%d'`
TIME=`date +'%H%M%S'`
START_TIME="$DATE-$TIME"
ROSBAG_DEST="$DIR_ROOT/$START_TIME"

ros2 bag record -o $ROSBAG_DEST --qos-profile-overrides-path $QOS_PROFILE_PATH -e $RECORD_LIST



