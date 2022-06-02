#!/bin/bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:=$1 log_directory:=/tmp no_validation:=True
