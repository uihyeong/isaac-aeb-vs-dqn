#!/bin/bash
# AEB 4개 노드 실행 (Isaac 씬이 떠있는 상태에서)
source /opt/ros/humble/setup.bash
cd ~/isaac_aeb/aeb_nodes
WP=~/isaac_aeb/aeb_nodes/waypoints_trimmed.yaml

echo "[run_aeb] e_stop / aeb / oa / path_follower 시작"
python3 scout_mini_e_stop.py > /tmp/aeb_estop.log 2>&1 &
python3 scout_mini_aeb.py > /tmp/aeb_aeb.log 2>&1 &
python3 scout_mini_obstacle_avoidance.py > /tmp/aeb_oa.log 2>&1 &
python3 sm_path_follower_test4.py --ros-args \
    -p waypoint_file:="$WP" \
    -p global_frame:=world \
    -p base_frame:=base_link > /tmp/aeb_follower.log 2>&1 &
echo "[run_aeb] PIDs: $(jobs -p | tr '\n' ' ')"
wait
