#!/bin/bash
# 단일 측정 실행 (포그라운드). 사용법: run_measure.sh <dqn|aeb> <run_idx> [people_seed] [out_tag]
# 씬+브리지는 이미 떠있어야 함. 지표 JSON 생성 시 종료.
set +e
source /opt/ros/humble/setup.bash 2>/dev/null
MODE=$1; IDX=${2:-0}; SEED=${3:-}; TAGOUT=${4:-run${IDX}}
START="3.648659787390237 -1.0394418076000558 -1.25"
OUT=/home/sejong/isaac_aeb/results/metrics_${MODE}_${TAGOUT}.json
mkdir -p /home/sejong/isaac_aeb/results
rm -f "$OUT"

# 1) 리셋 (seed 주면 사람도 재배치)
rm -f /tmp/robot_ready.txt
printf "0.0 0.0\n" > /tmp/cmd_vel.txt
echo "$START $SEED" > /tmp/reset_robot.txt
for t in $(seq 1 50); do [ -f /tmp/robot_ready.txt ] && break; sleep 0.2; done
sleep 1
echo "[measure] $MODE $TAGOUT (seed=$SEED): reset done"

# 2) 지표 노드
python3 /home/sejong/isaac_aeb/metrics_node.py --ros-args \
    -p tag:="$MODE" -p out:="$OUT" -p timeout:=400.0 >/tmp/measure_metrics.log 2>&1 &
MPID=$!
sleep 1

# 3) 컨트롤러
if [ "$MODE" = "dqn" ]; then
    /home/sejong/grid_nav/venv/bin/python /home/sejong/isaac_aeb/dqn_drive.py >/tmp/measure_ctrl.log 2>&1 &
else
    bash /home/sejong/isaac_aeb/run_aeb.sh >/tmp/measure_ctrl.log 2>&1 &
fi
CPID=$!
echo "[measure] controller PID $CPID, metrics PID $MPID"

# 4) JSON 대기 (최대 ~440s)
for t in $(seq 1 880); do
    [ -f "$OUT" ] && break
    sleep 0.5
done

# 5) 정리
kill $MPID 2>/dev/null
pkill -9 -f "scout_mini_e_stop|scout_mini_aeb|scout_mini_obstacle|sm_path_follower|dqn_drive.py" 2>/dev/null
printf "0.0 0.0\n" > /tmp/cmd_vel.txt
sleep 1
echo "[measure] RESULT $MODE $TAGOUT:"
cat "$OUT" 2>/dev/null || echo "  (no JSON — timed out)"
