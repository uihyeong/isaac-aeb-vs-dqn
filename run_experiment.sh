#!/bin/bash
# AEB/DQN 반복 측정 (씬은 이미 떠있어야 함). 사용법: run_experiment.sh <aeb|dqn> <N>
source /opt/ros/humble/setup.bash
MODE=$1; N=${2:-3}
START="3.648659787390237 -1.0394418076000558 -1.25"
OUTDIR=~/isaac_aeb/results; mkdir -p "$OUTDIR"

for i in $(seq 0 $((N-1))); do
  echo "=========== $MODE run $i ==========="
  # 1) 로봇 리셋
  rm -f /tmp/robot_ready.txt
  printf "0.0 0.0\n" > /tmp/cmd_vel.txt
  echo "$START" > /tmp/reset_robot.txt
  for t in $(seq 1 60); do [ -f /tmp/robot_ready.txt ] && break; sleep 0.2; done
  sleep 1.0
  OUT="$OUTDIR/metrics_${MODE}_run${i}.json"; rm -f "$OUT"
  # 2) 지표 노드
  python3 ~/isaac_aeb/metrics_node.py --ros-args -p tag:="$MODE" -p out:="$OUT" -p timeout:=400.0 \
      > /tmp/metrics_${MODE}_${i}.log 2>&1 &
  MPID=$!
  sleep 0.5
  # 3) 컨트롤러
  if [ "$MODE" = "aeb" ]; then
    bash ~/isaac_aeb/run_aeb.sh > /tmp/ctrl_${MODE}_${i}.log 2>&1 &
  else
    ~/grid_nav/venv/bin/python ~/isaac_aeb/dqn_drive.py > /tmp/ctrl_${MODE}_${i}.log 2>&1 &
  fi
  # 4) 지표 JSON 생성까지 대기 (최대 ~460s)
  for t in $(seq 1 920); do [ -f "$OUT" ] && break; sleep 0.5; done
  # 5) 정리
  kill $MPID 2>/dev/null
  pkill -f "scout_mini_e_stop|scout_mini_aeb|scout_mini_obstacle|sm_path_follower|dqn_drive.py" 2>/dev/null
  printf "0.0 0.0\n" > /tmp/cmd_vel.txt
  sleep 2.0
  echo "RESULT $MODE run $i: $(cat "$OUT" 2>/dev/null | tr -d '\n')"
done
echo "=== $MODE $N회 완료 ==="
