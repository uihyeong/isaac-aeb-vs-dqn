"""
공정 비교 지표 수집 노드 (py3.10 + ROS2). AEB·DQN 동일하게 측정.
구독: /tf(world→base_link 포즈) · /scan(이격거리·충돌) · /scout_mini/e_stop(E-stop 카운트)
산출(JSON): 성공여부 · 목표도달시간 · 경로길이 · 충돌수(벽/사람 분해) · 최소이격 · 끼임 · E-stop수

실행: source /opt/ros/humble/setup.bash
  python3 metrics_node.py --ros-args -p tag:=AEB -p out:=/tmp/metrics_AEB_run0.json
"""
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

HOME = os.path.expanduser("~")
PGM_PATH = os.path.join(HOME, "Downloads", "maps_extracted", "maps", "map_5floor.pgm")
PGM_RES = 0.05
OCC_THRESH = 100          # pgm <= 이면 벽

# 목표 (waypoints_trimmed.yaml 마지막)
GOAL_X, GOAL_Y = 3.513, 2.473
GOAL_R = 0.6              # 도달 반경(m)
COLLISION_THRESH = 0.30  # 최소 scan < 이면 충돌(로봇 반폭 0.25 + 여유)
MOVE_EPS = 0.10          # 이 거리 이상 움직이면 "이동"으로 간주(m)
STALL_COUNT_SEC = 10.0   # 정지가 이 시간 넘으면 정지이벤트 1회 카운트(s)
WEDGE_SEC = 90.0         # 정지가 이 시간 넘으면 끼임(종료)(s)
LASER_YAW = math.pi      # 라이다 후방 장착(yaw 180°)


class Metrics(Node):
    def __init__(self):
        super().__init__("metrics_node")
        self.tag = self.declare_parameter("tag", "RUN").value
        self.out = self.declare_parameter("out", "/tmp/metrics.json").value
        self.timeout = float(self.declare_parameter("timeout", 240.0).value)
        self.csv = self.declare_parameter("csv", "").value   # per-step (전방거리,명령속도) 로깅
        self.csv_rows = []                                    # (t, front_dist, vcmd, min_dist)

        # 벽 맵 로드 (충돌 분류용)
        pgm = cv2.imread(PGM_PATH, cv2.IMREAD_GRAYSCALE)
        self.h, self.w = pgm.shape
        self.wall = (pgm <= OCC_THRESH)
        self.origin = np.array([-(self.w * PGM_RES) / 2.0, -(self.h * PGM_RES) / 2.0])

        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, "/scout_mini/e_stop", self.on_estop, 10)
        from geometry_msgs.msg import Twist
        self.last_vcmd = 0.0
        self.create_subscription(Twist, "/cmd_vel", self.on_cmdvel, 10)   # 파일대신 토픽(충돌 방지)

        # 상태
        self.scan = None
        self.traj = []              # (t, x, y)
        self.t0 = None              # 첫 이동 시각
        self.min_clear = 9.9
        self.n_collision = 0
        self.n_wall = 0
        self.n_person = 0
        self.in_collision = False
        self.estop_prev = False
        self.n_estop = 0
        self.wedged = False
        self.n_stall = 0           # 10s 넘는 정지 횟수
        self.in_stall = False
        self.last_move_t = None
        self.last_move_xy = None
        self.max_stall = 0.0
        self.success = False
        self.t_goal = None
        self.start_wall = time.time()
        self.done = False

        self.create_timer(0.05, self.tick)
        self.get_logger().info(f"[metrics:{self.tag}] 측정 시작 → {self.out}")

    def on_scan(self, msg):
        self.scan = msg

    def on_estop(self, msg):
        if msg.data and not self.estop_prev:
            self.n_estop += 1
        self.estop_prev = msg.data

    def _robot_pose(self):
        try:
            tf = self.buf.lookup_transform("world", "base_link", rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def _front_dist(self):
        """로봇 전방 ±20° 콘 내 최소 scan 거리(전방 장애물거리). 라이다 후방장착(LASER_YAW) 보정."""
        if self.scan is None:
            return None
        s = self.scan
        r = np.array(s.ranges, dtype=np.float32)
        n = len(r)
        ang = s.angle_min + np.arange(n) * s.angle_increment
        rob = np.arctan2(np.sin(LASER_YAW + ang), np.cos(LASER_YAW + ang))  # 로봇프레임 빔방향
        front = np.abs(rob) < math.radians(20)
        valid = front & np.isfinite(r) & (r >= s.range_min) & (r <= s.range_max)
        return float(r[valid].min()) if valid.any() else None

    def _flush_csv(self):
        try:
            with open(self.csv, "w") as f:
                f.write("t,front_dist,vcmd\n")
                for row in self.csv_rows:
                    f.write(",".join(map(str, row)) + "\n")
        except Exception:
            pass

    def on_cmdvel(self, msg):
        self.last_vcmd = abs(msg.linear.x)

    def _read_vcmd(self):
        return self.last_vcmd

    def _is_wall(self, wx, wy):
        cx = int(round((wx - self.origin[0]) / PGM_RES))
        cy = int(round(self.h - (wy - self.origin[1]) / PGM_RES))
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                yy, xx = cy + dy, cx + dx
                if 0 <= yy < self.h and 0 <= xx < self.w and self.wall[yy, xx]:
                    return True
        return False

    def tick(self):
        if self.done:
            return
        pose = self._robot_pose()
        if pose is None:
            if time.time() - self.start_wall > 30:
                self.get_logger().warn("TF 없음 30s → 종료")
                self.finish()
            return
        x, y, yaw = pose
        now = time.time()

        # per-step 로깅 (게인 반응곡선용): 전방거리 vs 명령속도 (정지 포함 위해 t0 무관 기록)
        if self.csv:
            fd = self._front_dist(); vc = self._read_vcmd()
            if fd is not None and vc is not None:
                self.csv_rows.append((round(now - self.start_wall, 3), round(fd, 3), round(vc, 3)))
                if len(self.csv_rows) % 40 == 0:      # 주기적 flush (끼임/강제종료 대비)
                    self._flush_csv()

        # 첫 이동 감지 → 타이머 시작
        if self.t0 is None:
            if self.traj and math.hypot(x - self.traj[0][1], y - self.traj[0][2]) > 0.1:
                self.t0 = now
            self.traj.append((now, x, y))
        else:
            self.traj.append((now, x, y))

        # 최소 이격 + 충돌 (scan 기반)
        if self.scan is not None:
            r = np.array(self.scan.ranges, dtype=np.float32)
            r = r[np.isfinite(r)]
            r = r[(r >= self.scan.range_min) & (r <= self.scan.range_max)]
            if r.size:
                mn = float(r.min())
                self.min_clear = min(self.min_clear, mn)
                if mn < COLLISION_THRESH and not self.in_collision:
                    self.in_collision = True
                    self.n_collision += 1
                    # 충돌점 분류: 최소빔 월드좌표가 벽 위면 벽충돌
                    i = int(np.argmin(np.nan_to_num(np.array(self.scan.ranges), nan=1e9)))
                    ang = self.scan.angle_min + i * self.scan.angle_increment
                    wb = yaw + LASER_YAW + ang
                    px = x + mn * math.cos(wb)
                    py = y + mn * math.sin(wb)
                    if self._is_wall(px, py):
                        self.n_wall += 1
                    else:
                        self.n_person += 1
                elif mn >= COLLISION_THRESH + 0.1:
                    self.in_collision = False

        # 목표 도달
        if math.hypot(x - GOAL_X, y - GOAL_Y) < GOAL_R and self.t0 is not None:
            self.success = True
            self.t_goal = now - self.t0
            self.get_logger().info(f"[metrics:{self.tag}] ★목표 도달★ {self.t_goal:.1f}s")
            self.finish()
            return

        # 정지/끼임 추적 (이동 기준점 대비)
        if self.last_move_xy is None:
            self.last_move_xy = (x, y)
            self.last_move_t = now
        elif math.hypot(x - self.last_move_xy[0], y - self.last_move_xy[1]) > MOVE_EPS:
            self.last_move_xy = (x, y)
            self.last_move_t = now
            self.in_stall = False
        else:
            stall = now - self.last_move_t
            self.max_stall = max(self.max_stall, stall)
            if stall > STALL_COUNT_SEC and not self.in_stall and self.t0 is not None:
                self.in_stall = True
                self.n_stall += 1
                self.get_logger().info(f"[metrics:{self.tag}] 정지 이벤트 #{self.n_stall} (>{STALL_COUNT_SEC}s)")
            if stall > WEDGE_SEC and self.t0 is not None:
                self.wedged = True
                self.get_logger().warn(f"[metrics:{self.tag}] 끼임(영구) {stall:.0f}s → 종료")
                self.finish()
                return

        # 타임아웃
        if self.t0 is not None and (now - self.t0) > self.timeout:
            self.get_logger().warn(f"[metrics:{self.tag}] 타임아웃 {self.timeout}s")
            self.finish()

    def _path_len(self):
        L = 0.0
        for a, b in zip(self.traj[:-1], self.traj[1:]):
            L += math.hypot(b[1] - a[1], b[2] - a[2])
        return L

    def finish(self):
        if self.done:
            return
        self.done = True
        if self.csv and self.csv_rows:
            with open(self.csv, "w") as f:
                f.write("t,front_dist,vcmd\n")
                for row in self.csv_rows:
                    f.write(",".join(map(str, row)) + "\n")
            self.get_logger().info(f"[metrics:{self.tag}] CSV {len(self.csv_rows)}행 저장: {self.csv}")
        res = {
            "tag": self.tag,
            "success": self.success,
            "time_to_goal": round(self.t_goal, 2) if self.t_goal else None,
            "path_length": round(self._path_len(), 2),
            "n_collision": self.n_collision,
            "n_wall_collision": self.n_wall,
            "n_person_collision": self.n_person,
            "min_clearance": round(self.min_clear, 3),
            "wedged": self.wedged,
            "n_stall": self.n_stall,
            "max_stall_sec": round(self.max_stall, 1),
            "n_estop": self.n_estop,
        }
        with open(self.out, "w") as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
        self.get_logger().info(f"[metrics:{self.tag}] 저장: {json.dumps(res, ensure_ascii=False)}")


def main():
    rclpy.init()
    n = Metrics()
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.1)
    finally:
        n.finish()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
