"""
격자 기반 동적 장애물 회피 환경 (Gymnasium).

Plan B (~/wjdals.readme) 구현:
  - 맵(map_5floor.pgm)을 0.5m 격자로 이산화
  - Voronoi 경로를 breadcrumb로 따라가며 동적 장애물(사람) 회피
  - 경유지 2개(point2, point3) 통과 + 목표(end) 도달 = terminal
  - 행동: 상/하/좌/우/정지 (Discrete 5)
  - ★예측형★: 관측에 장애물 현재+직전 위치(=이동방향)를 포함 → 선제 회피 학습
  - 리워드: step cost(음수) + 경로진행 + 경유지/목표 보너스 + 충돌 패널티(terminal)

좌표계: nav_dynobs_env와 동일 (CENTER_MAP, RES=0.05)
  origin = [-(W*RES)/2, -(H*RES)/2]
"""

import math
import os

import numpy as np
import yaml
import cv2
import gymnasium as gym
from gymnasium import spaces


# ─── 경로/상수 ────────────────────────────────────────────────────────────────
HOME          = os.path.expanduser("~")
PGM_PATH      = os.path.join(HOME, "Downloads", "map_5floor.pgm")
VORONOI_YAML  = os.path.join(HOME, "voronoi_waypoints.yaml")
ROOMS_YAML    = os.path.join(HOME, "rooms_v2.yaml")

PGM_RES       = 0.05      # m/px
CELL_SIZE     = 0.5       # m/cell  → 1.7m 복도 ≈ 3칸 (주행용. 0.25m는 학습 실패 → 복귀)
CELL_PX       = int(round(CELL_SIZE / PGM_RES))   # 10 px/cell
FREE_FRAC     = 0.60      # 셀 내 자유픽셀 비율 ≥ 이 값이면 이동가능 셀

WINDOW        = 7         # 에고센트릭 로컬 관측 창 (홀수), 7칸×0.5m=3.5m 시야
LOOKAHEAD     = 3         # 경로 lookahead (칸, ≈1.5m)

# 리워드
R_STEP        = -0.5      # 매 스텝 시간 비용 (음수) → 최단 유도
R_PROGRESS    =  2.0      # 경로 인덱스 1칸 전진 시 (1회성, 포텐셜)
R_SUBGOAL     =  50.0     # 경유지 통과 (1회성)
R_GOAL        =  500.0    # 목표 도달 (terminal)
R_COLLISION   = -200.0    # 장애물 충돌 (terminal)
R_WALL        = -2.0      # 벽으로 이동 시도 (제자리 유지)

# 동적 장애물
OBS_CELLS_PER_STEP = 0.5  # 로봇 1스텝당 장애물 이동 칸수 (로봇 절반 속도, 셀크기 무관 비율)
COLLISION_CELLS    = 0    # 충돌 판정: 체비셰프 거리 ≤ 이 값 (0=같은 칸)
CLOSE_TOL          = 1    # 경유지/목표 도달 판정 반경 (칸, ≈0.5m)

ACTIONS = {
    0: (0, +1),   # 상 (+y)
    1: (0, -1),   # 하 (-y)
    2: (-1, 0),   # 좌 (-x)
    3: (+1, 0),   # 우 (+x)
    4: (0, 0),    # 정지
}


def _load_grid():
    """PGM → (free 격자 bool[H_cell, W_cell], 원점, 픽셀크기)."""
    pgm = cv2.imread(PGM_PATH, cv2.IMREAD_GRAYSCALE)
    if pgm is None:
        raise FileNotFoundError(f"PGM 없음: {PGM_PATH}")
    H, W = pgm.shape
    free_px = pgm > 250                      # 흰색(254)만 자유공간. 회색(205=unknown)은 벽 취급
    origin = np.array([-(W * PGM_RES) / 2.0, -(H * PGM_RES) / 2.0])

    hc, wc = H // CELL_PX, W // CELL_PX
    free = np.zeros((hc, wc), dtype=bool)
    for cy in range(hc):
        for cx in range(wc):
            block = free_px[cy*CELL_PX:(cy+1)*CELL_PX, cx*CELL_PX:(cx+1)*CELL_PX]
            free[cy, cx] = block.mean() >= FREE_FRAC
    return free, origin, (H, W)


def _bresenham(a, b):
    """두 셀 사이 직선을 셀 리스트로 (양 끝 포함)."""
    (x0, y0), (x1, y1) = a, b
    cells = []
    dx = abs(x1 - x0); dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy; x += sx
        if e2 < dx:
            err += dx; y += sy
    return cells


def _world_to_cell(x, y, origin, hc):
    """월드 (x,y)m → 격자 (cx, cy). y축은 이미지에서 뒤집힘."""
    px = (x - origin[0]) / PGM_RES
    py = (y - origin[1]) / PGM_RES
    cx = int(px // CELL_PX)
    # 이미지 y는 아래로 증가 → 격자 cy = hc - (py//CELL_PX) - 1
    cy = hc - int(py // CELL_PX) - 1
    return cx, cy


class GridNavEnv(gym.Env):
    """격자 동적 장애물 회피 환경."""

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, num_obstacles=2, max_steps=400, render_mode=None, seed=None):
        super().__init__()
        self.num_obstacles = num_obstacles
        self.max_steps = max_steps
        self.render_mode = render_mode

        # 맵 격자
        self.free, self.origin, _ = _load_grid()
        self.hc, self.wc = self.free.shape

        # Voronoi 경로 → 격자 셀 리스트 (연속 중복 제거 + Bresenham 보간으로 셀단위 밀집화)
        with open(VORONOI_YAML) as f:
            raw = yaml.safe_load(f).get("waypoints", [])
        wp_cells = []
        for wp in raw:
            c = _world_to_cell(wp[0], wp[1], self.origin, self.hc)
            if not wp_cells or wp_cells[-1] != c:
                wp_cells.append(c)
        # 연속 웨이포인트 사이를 셀단위로 채움 (작은 셀에서 띄엄띄엄해지는 것 방지)
        path = [wp_cells[0]]
        for a, b in zip(wp_cells[:-1], wp_cells[1:]):
            for cell in _bresenham(a, b)[1:]:
                if cell != path[-1]:
                    path.append(cell)
        self.path = path                      # [(cx,cy), ...] 셀단위 밀집
        self.path_arr = np.array(path, dtype=np.int32)

        # 시작/경유지/목표 셀
        with open(ROOMS_YAML) as f:
            rooms = yaml.safe_load(f)
        self.start_cell = _world_to_cell(rooms["start"]["x"], rooms["start"]["y"], self.origin, self.hc)
        self.sub1_cell  = _world_to_cell(rooms["point2"]["x"], rooms["point2"]["y"], self.origin, self.hc)
        self.sub2_cell  = _world_to_cell(rooms["point3"]["x"], rooms["point3"]["y"], self.origin, self.hc)
        self.goal_cell  = _world_to_cell(rooms["end"]["x"], rooms["end"]["y"], self.origin, self.hc)

        # 공간 정의
        self.action_space = spaces.Discrete(5)
        # 관측: 로컬창 3채널(벽/장애물now/장애물prev) + [dir_x,dir_y,dist, sub1,sub2]
        win = WINDOW * WINDOW
        obs_dim = 3 * win + 5
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(obs_dim,), dtype=np.float32)

        self._rng = np.random.default_rng(seed)

        # 상태 변수
        self.robot = None
        self.obs_idx = None        # 각 장애물의 경로 float 인덱스
        self.obs_dir = None        # 각 장애물 이동 방향 (+1/-1)
        self.obs_cells = None      # 현재 칸
        self.obs_cells_prev = None # 직전 칸
        self.path_progress = 0
        self.sub1_done = False
        self.sub2_done = False
        self.steps = 0

    # ── 유틸 ──────────────────────────────────────────────────────────────────
    def _is_free(self, cx, cy):
        return 0 <= cx < self.wc and 0 <= cy < self.hc and self.free[cy, cx]

    def _nearest_path_idx(self, cell):
        d = np.abs(self.path_arr - np.array(cell)).sum(axis=1)
        return int(d.argmin())

    # ── reset ─────────────────────────────────────────────────────────────────
    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self.robot = list(self.path[0])       # 경로 시작점
        self.path_progress = 0
        self.sub1_done = False
        self.sub2_done = False
        self.steps = 0

        # 장애물: 경로 위 랜덤 인덱스(시작점 근처 제외)에 배치
        n_path = len(self.path)
        self.obs_idx = []
        self.obs_dir = []
        for _ in range(self.num_obstacles):
            idx = self._rng.integers(10, n_path - 1)
            self.obs_idx.append(float(idx))
            self.obs_dir.append(int(self._rng.choice([-1, 1])))
        self.obs_idx = np.array(self.obs_idx, dtype=np.float32)
        self.obs_dir = np.array(self.obs_dir, dtype=np.int32)
        self._update_obstacle_cells(init=True)

        return self._get_obs(), {}

    def _update_obstacle_cells(self, init=False):
        cells = []
        for i in range(self.num_obstacles):
            idx = int(round(self.obs_idx[i]))
            idx = max(0, min(len(self.path) - 1, idx))
            cells.append(self.path[idx])
        if init:
            self.obs_cells_prev = list(cells)
        else:
            self.obs_cells_prev = self.obs_cells
        self.obs_cells = cells

    # ── step ──────────────────────────────────────────────────────────────────
    def step(self, action):
        self.steps += 1
        reward = R_STEP
        terminated = False
        truncated = False

        # 1) 로봇 이동
        dx, dy = ACTIONS[int(action)]
        nx, ny = self.robot[0] + dx, self.robot[1] + dy
        if dx == 0 and dy == 0:
            pass  # 정지
        elif self._is_free(nx, ny):
            self.robot = [nx, ny]
        else:
            reward += R_WALL  # 벽 → 제자리

        # 2) 장애물 이동 (경로 왕복)
        for i in range(self.num_obstacles):
            self.obs_idx[i] += self.obs_dir[i] * OBS_CELLS_PER_STEP
            if self.obs_idx[i] >= len(self.path) - 1:
                self.obs_idx[i] = len(self.path) - 1
                self.obs_dir[i] = -1
            elif self.obs_idx[i] <= 0:
                self.obs_idx[i] = 0
                self.obs_dir[i] = +1
        self._update_obstacle_cells()

        # 3) 충돌 판정
        for c in self.obs_cells:
            if max(abs(self.robot[0] - c[0]), abs(self.robot[1] - c[1])) <= COLLISION_CELLS:
                reward += R_COLLISION
                terminated = True
                return self._get_obs(), reward, terminated, truncated, {"event": "collision"}

        # 4) 경로 진행 보상 (포텐셜)
        prog = self._nearest_path_idx(self.robot)
        if prog > self.path_progress:
            reward += R_PROGRESS * (prog - self.path_progress)
            self.path_progress = prog

        # 5) 경유지/목표
        if not self.sub1_done and self._cell_close(self.robot, self.sub1_cell):
            self.sub1_done = True
            reward += R_SUBGOAL
        if not self.sub2_done and self._cell_close(self.robot, self.sub2_cell):
            self.sub2_done = True
            reward += R_SUBGOAL
        # 목표는 경유지 2개 모두 통과한 뒤에만 인정 (루프 전체 주행 강제)
        if self.sub1_done and self.sub2_done and self._cell_close(self.robot, self.goal_cell):
            reward += R_GOAL
            terminated = True
            return self._get_obs(), reward, terminated, truncated, {"event": "goal"}

        # 6) 시간 초과
        if self.steps >= self.max_steps:
            truncated = True

        return self._get_obs(), reward, terminated, truncated, {}

    def _cell_close(self, a, b, tol=CLOSE_TOL):
        return max(abs(a[0] - b[0]), abs(a[1] - b[1])) <= tol

    # ── 관측 ──────────────────────────────────────────────────────────────────
    def _get_obs(self):
        half = WINDOW // 2
        wall_w = np.zeros((WINDOW, WINDOW), dtype=np.float32)
        obs_now = np.zeros((WINDOW, WINDOW), dtype=np.float32)
        obs_prev = np.zeros((WINDOW, WINDOW), dtype=np.float32)
        rx, ry = self.robot
        for j in range(WINDOW):
            for i in range(WINDOW):
                cx = rx + (i - half)
                cy = ry + (j - half)
                if not (0 <= cx < self.wc and 0 <= cy < self.hc) or not self.free[cy, cx]:
                    wall_w[j, i] = 1.0
        for c in self.obs_cells:
            ix, iy = c[0] - rx + half, c[1] - ry + half
            if 0 <= ix < WINDOW and 0 <= iy < WINDOW:
                obs_now[iy, ix] = 1.0
        for c in self.obs_cells_prev:
            ix, iy = c[0] - rx + half, c[1] - ry + half
            if 0 <= ix < WINDOW and 0 <= iy < WINDOW:
                obs_prev[iy, ix] = 1.0

        # 경로 lookahead 방향
        idx = min(self.path_progress + LOOKAHEAD, len(self.path) - 1)
        tgt = self.path[idx]
        ddx, ddy = tgt[0] - rx, tgt[1] - ry
        dist = math.hypot(ddx, ddy)
        if dist > 1e-6:
            dirx, diry = ddx / dist, ddy / dist
        else:
            dirx, diry = 0.0, 0.0
        dist_n = min(dist / WINDOW, 1.0)

        scal = np.array([dirx, diry, dist_n,
                         1.0 if self.sub1_done else 0.0,
                         1.0 if self.sub2_done else 0.0], dtype=np.float32)
        return np.concatenate([wall_w.ravel(), obs_now.ravel(), obs_prev.ravel(), scal])

    # ── render (디버그) ─────────────────────────────────────────────────────────
    def render(self):
        img = np.ones((self.hc, self.wc, 3), dtype=np.uint8) * 255
        img[~self.free] = (40, 40, 40)
        for c in self.path:
            img[c[1], c[0]] = (200, 220, 255)
        sg = [(self.sub1_cell, (255, 180, 0)), (self.sub2_cell, (255, 120, 0)),
              (self.goal_cell, (0, 200, 0))]
        for cell, col in sg:
            img[cell[1], cell[0]] = col
        for c in self.obs_cells:
            img[c[1], c[0]] = (0, 0, 255)
        img[self.robot[1], self.robot[0]] = (255, 0, 0)
        return cv2.resize(img, (self.wc*6, self.hc*6), interpolation=cv2.INTER_NEAREST)


if __name__ == "__main__":
    # 간단 자체 점검
    env = GridNavEnv(num_obstacles=2, seed=0)
    obs, _ = env.reset()
    print("관측 shape:", obs.shape, "| 행동 수:", env.action_space.n)
    print("격자 크기 (hc,wc):", env.hc, env.wc)
    print("start/sub1/sub2/goal 셀:", env.start_cell, env.sub1_cell, env.sub2_cell, env.goal_cell)
    print("경로 길이(셀):", len(env.path))
    total = 0.0
    for _ in range(50):
        o, r, term, trunc, info = env.step(env.action_space.sample())
        total += r
        if term or trunc:
            print("에피소드 종료:", info, "보상합:", round(total, 1))
            break
    print("자체 점검 완료")
