"""
DQN 드라이브 래퍼 (grid_nav venv: py3.10 + rclpy + SB3).
격자 학습 DQN(best_model.zip)을 Isaac 연속 씬에서 로컬 플래너로 사용.
구독: /tf(world→base_link) · /scan(동적 장애물 추출)
발행: /cmd_vel (pure-pursuit으로 DQN이 고른 인접 셀로 주행)

관측(152) 구성은 grid_nav_env._get_obs와 동일하게 재현:
  7×7 ego window × 3채널(벽 / 동적장애물 현재 / 직전=이동방향) + 경로방향(2)+거리(1)+경유지(2)

실행: cd ~/isaac_aeb
  ~/grid_nav/venv/bin/python dqn_drive.py
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.expanduser("~/grid_nav"))
import grid_nav_env as G          # 맵/경로/상수 재사용
from stable_baselines3 import DQN

WINDOW = G.WINDOW
HALF = WINDOW // 2
CELL_PX = G.CELL_PX
PGM_RES = G.PGM_RES
LOOKAHEAD = G.LOOKAHEAD
CLOSE_TOL = G.CLOSE_TOL
ACTIONS = G.ACTIONS

LASER_YAW = math.pi          # 후방 장착
V_MAX = 0.5                  # AEB와 동일 상한 (공정)
W_MAX = 1.0
GOAL_WORLD = (3.513, 2.473)  # 최종 목표 (waypoints_trimmed 끝)
GOAL_APPROACH = 4.0         # 경유지 완료+이 거리 내 → 목표 직접 조준(루프 폐합 혼동 방지)
GOAL_STOP = 0.5             # 이 반경 내 → 정지(도착)
DECISION_DT = 0.6            # DQN 재결정 간격(s)
DODGE_DIST = 0.7            # DQN 측면 회피 시 캐럿 수직 오프셋(m)
LOOKAHEAD_DIST = 1.3        # pure-pursuit 전방 주시 거리(m) > 회전반경(~0.7m)
MODEL = os.path.expanduser("~/grid_nav/best_model.zip")
PURSUIT_ONLY = os.environ.get("PURSUIT_ONLY", "0") == "1"  # 1=회피 끄고 순수 경로추종(대조군)


class DQNDrive(Node):
    def __init__(self):
        super().__init__("dqn_drive")
        # 정적 맵·경로 (env에서 로드)
        env = G.GridNavEnv(num_obstacles=2, seed=0)
        self.free = env.free
        self.origin = env.origin
        self.hc, self.wc = env.hc, env.wc
        self.path = env.path
        self.path_arr = env.path_arr
        self.sub1_cell = env.sub1_cell
        self.sub2_cell = env.sub2_cell
        self.goal_cell = env.goal_cell

        if PURSUIT_ONLY:
            self.model = None
            self.get_logger().info("PURSUIT_ONLY 모드: 회피 없이 순수 경로추종 (대조군)")
        else:
            self.model = DQN.load(MODEL, device="cpu")
            self.get_logger().info(f"DQN 로드: {MODEL}")

        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.cmd_path = "/tmp/cmd_vel.txt"   # Isaac이 읽는 파일에 직접 기록(브리지 우회)
        self.pos_hist = []                   # anti-orbit: (t,x,y) 이력
        self.escape_until = 0.0              # 탈출 모드 종료 시각
        self.scan = None
        self.path_progress = 0
        self.sub1_done = False
        self.sub2_done = False
        self.obs_prev_cells = []     # 직전 결정의 동적장애물 셀
        self.target_world = None     # 현재 주행 목표 (world x,y)
        self.last_decision = 0.0
        self.stop_cmd = False

        self.create_timer(0.05, self.control_tick)   # 20Hz 주행
        self.get_logger().info("DQN drive 시작")

    # ── 좌표 변환 ──
    def world_to_cell(self, x, y):
        return G._world_to_cell(x, y, self.origin, self.hc)

    def cell_to_world(self, cx, cy):
        wx = self.origin[0] + (cx + 0.5) * CELL_PX * PGM_RES
        wy = self.origin[1] + (self.hc - cy - 1 + 0.5) * CELL_PX * PGM_RES
        return wx, wy

    def on_scan(self, msg):
        self.scan = msg

    def robot_pose(self):
        try:
            tf = self.buf.lookup_transform("world", "base_link", rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    # ── 동적 장애물 셀 추출 (scan 중 벽 아닌 반환) ──
    def dynamic_cells(self, x, y, yaw, rcx, rcy):
        cells = set()
        if self.scan is None:
            return cells
        r = np.array(self.scan.ranges, dtype=np.float32)
        n = len(r)
        amin, ainc = self.scan.angle_min, self.scan.angle_increment
        rng_max = WINDOW * CELL_PX * PGM_RES   # 윈도우 반경만
        for i in range(0, n, 4):               # 4빔마다 (속도)
            d = r[i]
            if not (self.scan.range_min < d < rng_max):
                continue
            wb = yaw + LASER_YAW + amin + i * ainc
            wx = x + d * math.cos(wb)
            wy = y + d * math.sin(wb)
            cx, cy = self.world_to_cell(wx, wy)
            if not (0 <= cx < self.wc and 0 <= cy < self.hc):
                continue
            if not self.free[cy, cx]:
                continue                       # 벽 반환 → 동적 아님
            if abs(cx - rcx) <= HALF and abs(cy - rcy) <= HALF and (cx, cy) != (rcx, rcy):
                cells.add((cx, cy))
        return cells

    def build_obs(self, rcx, rcy, obs_now_cells):
        wall_w = np.zeros((WINDOW, WINDOW), dtype=np.float32)
        obs_now = np.zeros((WINDOW, WINDOW), dtype=np.float32)
        obs_prev = np.zeros((WINDOW, WINDOW), dtype=np.float32)
        for j in range(WINDOW):
            for i in range(WINDOW):
                cx = rcx + (i - HALF)
                cy = rcy + (j - HALF)
                if not (0 <= cx < self.wc and 0 <= cy < self.hc) or not self.free[cy, cx]:
                    wall_w[j, i] = 1.0
        for (cx, cy) in obs_now_cells:
            ix, iy = cx - rcx + HALF, cy - rcy + HALF
            if 0 <= ix < WINDOW and 0 <= iy < WINDOW:
                obs_now[iy, ix] = 1.0
        for (cx, cy) in self.obs_prev_cells:
            ix, iy = cx - rcx + HALF, cy - rcy + HALF
            if 0 <= ix < WINDOW and 0 <= iy < WINDOW:
                obs_prev[iy, ix] = 1.0
        idx = min(self.path_progress + LOOKAHEAD, len(self.path) - 1)
        tgt = self.path[idx]
        ddx, ddy = tgt[0] - rcx, tgt[1] - rcy
        dist = math.hypot(ddx, ddy)
        dirx, diry = (ddx / dist, ddy / dist) if dist > 1e-6 else (0.0, 0.0)
        dist_n = min(dist / WINDOW, 1.0)
        scal = np.array([dirx, diry, dist_n,
                         1.0 if self.sub1_done else 0.0,
                         1.0 if self.sub2_done else 0.0], dtype=np.float32)
        return np.concatenate([wall_w.ravel(), obs_now.ravel(), obs_prev.ravel(), scal])

    def nearest_path_idx(self, cell):
        d = np.abs(self.path_arr - np.array(cell)).sum(axis=1)
        return int(np.argmin(d))

    def cell_close(self, a, b, tol=CLOSE_TOL):
        return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol

    def decide(self, x, y, yaw):
        rcx, rcy = self.world_to_cell(x, y)
        # 경로 진행도(단조 전진)
        prog = self.nearest_path_idx((rcx, rcy))
        if prog > self.path_progress:
            self.path_progress = prog
        # 경유지 갱신
        if not self.sub1_done and self.cell_close((rcx, rcy), self.sub1_cell):
            self.sub1_done = True
            self.get_logger().info("sub1(point2) 통과")
        if not self.sub2_done and self.cell_close((rcx, rcy), self.sub2_cell):
            self.sub2_done = True
            self.get_logger().info("sub2(point3) 통과")
        # ── 경로 캐럿(pure-pursuit): 로봇에서 직선거리 ≥ LOOKAHEAD_DIST 인 첫 경로점 ──
        rpos = np.array([x, y])
        cidx = self.path_progress
        while cidx < len(self.path) - 1:
            cw = np.array(self.cell_to_world(*self.path[cidx]), dtype=float)
            if np.linalg.norm(cw - rpos) >= LOOKAHEAD_DIST:
                break
            cidx += 1
        carrot = np.array(self.cell_to_world(*self.path[cidx]), dtype=float)
        pdir = carrot - rpos
        nrm = np.linalg.norm(pdir)
        pdir = pdir / nrm if nrm > 1e-6 else np.array([math.cos(yaw), math.sin(yaw)])

        # ── 대조군: 회피 없이 경로만 추종 ──
        if PURSUIT_ONLY:
            self.stop_cmd = False
            self.target_world = tuple(carrot)
            return

        # ── DQN 회피: 동적 장애물 셀 → 격자관측 → 정책 ──
        now_cells = self.dynamic_cells(x, y, yaw, rcx, rcy)
        obs = self.build_obs(rcx, rcy, now_cells)
        self.obs_prev_cells = list(now_cells)
        action, _ = self.model.predict(obs, deterministic=True)
        dx, dy = ACTIONS[int(action)]
        self.get_logger().info(
            f"cell=({rcx},{rcy}) act={int(action)}({dx},{dy}) prog={self.path_progress} nObs={len(now_cells)}")

        if (dx, dy) == (0, 0) and len(now_cells) > 0:
            # DQN 정지 = 장애물 회피 대기
            self.stop_cmd = True
            self.target_world = (x, y)
            return
        self.stop_cmd = False

        dodge = np.zeros(2)
        if (dx, dy) != (0, 0) and len(now_cells) > 0:
            # DQN이 고른 방향의 경로-수직 성분 = 측면 회피
            dvec = np.array(self.cell_to_world(rcx + dx, rcy + dy), dtype=float) - np.array([x, y])
            dn = np.linalg.norm(dvec)
            if dn > 1e-6:
                dvec /= dn
                perp = dvec - np.dot(dvec, pdir) * pdir   # 경로 수직 성분
                pn = np.linalg.norm(perp)
                if pn > 0.2:                                # 의미있는 측면 의도일 때만
                    dodge = perp / pn * DODGE_DIST
        self.target_world = tuple(carrot + dodge)

    def _emit(self, cmd):
        """/cmd_vel 발행 + Isaac이 읽는 파일에 직접 기록(브리지 우회)."""
        self.pub.publish(cmd)
        try:
            with open(self.cmd_path, "w") as f:
                f.write(f"{cmd.linear.x} {cmd.angular.z}\n")
        except Exception:
            pass

    def control_tick(self):
        try:
            pose = self.robot_pose()
            if pose is None:
                self._emit(Twist())     # 포즈 없으면 정지(stale 명령 폭주 방지)
                return
            x, y, yaw = pose
            now = time.time()

            # ── anti-orbit: 최근 4s 순변위 작으면 공전 → 경로 캐럿으로 직진 탈출 ──
            self.pos_hist.append((now, x, y))
            self.pos_hist = [(t, px, py) for (t, px, py) in self.pos_hist if now - t <= 4.0]
            if now > self.escape_until and len(self.pos_hist) > 20:
                t0p = self.pos_hist[0]
                if (now - t0p[0]) >= 3.5 and math.hypot(x - t0p[1], y - t0p[2]) < 0.4:
                    self.escape_until = now + 2.5      # 2.5s 직진 탈출
            if now < self.escape_until:
                cidx = min(self.path_progress + LOOKAHEAD + 2, len(self.path) - 1)
                cx, cy = self.cell_to_world(*self.path[cidx])
                ang = math.atan2(cy - y, cx - x)
                herr = math.atan2(math.sin(ang - yaw), math.cos(ang - yaw))
                cmd = Twist()
                cmd.linear.x = float(V_MAX * max(0.3, math.cos(herr)))   # 항상 전진
                cmd.angular.z = float(max(-0.6, min(0.6, 0.8 * herr)))   # 약한 선회만
                self._emit(cmd)
                return

            # ── 종단: 경유지 둘 다 통과 + 목표 근처 → 경로 대신 목표 직접 조준 ──
            gd = math.hypot(x - GOAL_WORLD[0], y - GOAL_WORLD[1])
            if self.sub1_done and self.sub2_done and gd < GOAL_APPROACH:
                cmd = Twist()
                if gd < GOAL_STOP:
                    self._emit(cmd)               # 도착 → 정지
                    return
                ang = math.atan2(GOAL_WORLD[1] - y, GOAL_WORLD[0] - x)
                herr = math.atan2(math.sin(ang - yaw), math.cos(ang - yaw))
                cmd.linear.x = float(V_MAX * max(0.1, math.cos(herr)) if math.cos(herr) > 0 else 0.05)
                cmd.angular.z = float(max(-0.8, min(0.8, 1.0 * herr)))
                self._emit(cmd)
                return

            if (self.target_world is None or (now - self.last_decision) > DECISION_DT):
                self.decide(x, y, yaw)
                self.last_decision = now

            cmd = Twist()
            if self.stop_cmd or self.target_world is None:
                self._emit(cmd)
                return
            tx, ty = self.target_world
            ang = math.atan2(ty - y, tx - x)
            herr = math.atan2(math.sin(ang - yaw), math.cos(ang - yaw))
            # 부드러운 pure-pursuit (낮은 게인·각속도 상한 → 진동/spin 억제)
            v = V_MAX * max(0.1, math.cos(herr)) if math.cos(herr) > 0 else 0.05
            w = max(-0.8, min(0.8, 1.0 * herr))
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self._emit(cmd)
        except Exception as e:
            self.get_logger().error(f"control_tick 예외: {e!r}")
            self._emit(Twist())


def main():
    rclpy.init()
    n = DQNDrive()
    try:
        rclpy.spin(n)
    finally:
        n.pub.publish(Twist())
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
