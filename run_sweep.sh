#!/bin/bash
# 밀집도 스윕 + 시드별 N회: 각 density마다 씬 재기동 → seed별 (AEB,DQN) 동일 장애물 측정.
# 교차형 장애물(AEB_OBSTACLE_MODE=cross). 결과: results/metrics_<mode>_d<D>_s<S>.json
set +e
source /opt/ros/humble/setup.bash 2>/dev/null
export AEB_OBSTACLE_MODE="${AEB_OBSTACLE_MODE:-cross}"
export AEB_PERSON_SPEED="${AEB_PERSON_SPEED:-0.3}"
export AEB_CROSS_HALF="${AEB_CROSS_HALF:-0.9}"
DENSITIES="${DENSITIES:-2 4 6 8 10}"
NSEED="${NSEED:-3}"
SWEEP_TAG="${SWEEP_TAG:-}"            # 결과 파일 접두 (예: harsh_, along_)
LOG=/tmp/sweep_progress.log
: > "$LOG"
echo "[sweep] densities=[$DENSITIES] nseed=$NSEED mode=$AEB_OBSTACLE_MODE speed=$AEB_PERSON_SPEED crosshalf=$AEB_CROSS_HALF tag=$SWEEP_TAG" | tee -a "$LOG"

kill_scene() {
  ps -ef | grep -E "aeb_scene|isaac_sim/kit/python/bin" | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null
  sleep 5
}
wait_ready() {
  for i in $(seq 1 60); do
    grep -q "Phase 3 완료" /tmp/scene.log 2>/dev/null && return 0
    grep -qE "Traceback|Error:" /tmp/scene.log 2>/dev/null && return 1
    sleep 5
  done
  return 1
}

for D in $DENSITIES; do
  echo "[sweep] === density $D 명 : 씬 재기동 ===" | tee -a "$LOG"
  kill_scene
  pkill -9 -f "cmdvel_to_file|dqn_drive|metrics_node" 2>/dev/null
  rm -f /tmp/scene.log /tmp/robot_ready.txt /tmp/reset_robot.txt; printf "0.0 0.0\n" > /tmp/cmd_vel.txt
  export AEB_NUM_PEOPLE=$D
  nohup ~/isaac_sim/python.sh /home/sejong/isaac_aeb/aeb_scene.py --headless --sx 3.65 --sy -1.04 --syaw -1.25 > /tmp/scene.log 2>&1 &
  sleep 8
  if ! wait_ready; then echo "[sweep] density $D 씬 기동 실패 — skip" | tee -a "$LOG"; continue; fi
  grep "사람 [0-9]*명 생성" /tmp/scene.log | tail -1 | tee -a "$LOG"
  # 브리지
  pkill -9 -f cmdvel_to_file 2>/dev/null; sleep 1
  setsid python3 /home/sejong/isaac_aeb/cmdvel_to_file.py >/tmp/bridge.log 2>&1 </dev/null & disown
  sleep 3
  for S in $(seq 1 $NSEED); do
    for M in ${MODES:-aeb dqn}; do
      TAG="${SWEEP_TAG}d${D}_s${S}"
      R=$(bash /home/sejong/isaac_aeb/run_measure.sh $M $S $S "$TAG" 2>&1 | tail -1)
      J=/home/sejong/isaac_aeb/results/metrics_${M}_${TAG}.json
      SUM=$(python3 -c "import json;d=json.load(open('$J'));print(f\"succ={d['success']} t={d['time_to_goal']} estop={d['n_estop']} coll={d['n_collision']} clr={d['min_clearance']}\")" 2>/dev/null)
      echo "[sweep] d=$D s=$S $M : $SUM" | tee -a "$LOG"
    done
  done
done
echo "[sweep] ===== 전체 완료 =====" | tee -a "$LOG"
