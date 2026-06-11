"""
Phase 1 — Isaac Sim 씬: Scout Mini + 후방장착 LiDAR + map_5floor 벽 + ROS2 bridge.

목표(Phase 1): Isaac Sim이 "실제 로봇 + 세상" 역할을 해서
  - /scan (LaserScan) 발행  (RPLidar처럼 후방 장착)
  - TF map→odom→base_link 발행 (GT 위치)
  - /clock 발행
  - /cmd_vel 구독 → 차동구동으로 바퀴 구동
이게 되면 zip의 AEB 노드(e_stop/aeb/oa/path_follower)를 그대로 붙일 수 있다.

실행:
  ~/isaac_sim/python.sh ~/isaac_aeb/aeb_scene.py
  # 헤드리스: 끝에 --headless
검증(다른 터미널):
  ros2 topic echo /scan --once
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"   # 로봇 전진해야
"""

import argparse
import math
import numpy as np

# ─── 인자 ─────────────────────────────────────────────────────────────────────
ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
ap.add_argument("--sx", type=float, default=3.65)   # 로봇 시작 x (기본: idx27, 여유 1.06m)
ap.add_argument("--sy", type=float, default=-1.04)  # 로봇 시작 y
ap.add_argument("--syaw", type=float, default=-1.25)  # 시작 yaw(rad), 경로방향
args, _ = ap.parse_known_args()

# ─── SimulationApp 먼저 (다른 omni import보다 반드시 선행) ────────────────────────
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": args.headless})

# ── 이후 omni/isaacsim import ──────────────────────────────────────────────────
import os
import cv2
import yaml
import omni
import omni.graph.core as og
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux, Sdf
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.extensions import enable_extension

# ROS2 bridge 확장 활성화
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

HOME = os.path.expanduser("~")
PGM_PATH   = os.path.join(HOME, "Downloads", "maps_extracted", "maps", "map_5floor.pgm")
SCOUT_USD  = os.path.join(HOME, "ros2_ws2", "src", "scout_ros2", "scout_description",
                          "urdf", "scout_mini", "scout_mini.usd")
VORONOI_YAML = os.path.join(HOME, "voronoi_waypoints.yaml")
PGM_RES    = 0.05
OCC_THRESH = 250        # 흰색(>250)=자유. 그 외(회색/검정)=벽
WALL_H     = 1.5        # 벽 높이(m)
WALL_MERGE = 4          # 셀 병합 단위(픽셀) → 박스 수 절감 (0.2m 박스)

# ── 동적 장애물(사람) 파라미터 ──
NUM_PEOPLE   = int(os.environ.get("AEB_NUM_PEOPLE", "2"))   # 밀집도 실험용 환경변수
PEOPLE_SEED  = int(os.environ.get("AEB_PEOPLE_SEED", "0"))  # run마다 다른 배치
PERSON_SPEED = float(os.environ.get("AEB_PERSON_SPEED", "0.3"))  # m/s (가혹 시나리오용 환경변수)
PERSON_R     = 0.25     # 캡슐 반지름(m)
PERSON_H     = 1.2      # 캡슐 원통부 높이(m) → 총 ~1.7m
PERSON_ZC    = 0.85     # 중심 높이(m)

# 좌표계: grid_nav와 동일 centered origin
def _origin_from_pgm(w, h):
    return np.array([-(w * PGM_RES) / 2.0, -(h * PGM_RES) / 2.0])


def configure_wheel_drives(stage):
    """휠 revolute 조인트를 속도제어용으로 설정 (stiffness=0, damping 높게).
    velocityCommand가 힘으로 전달되려면 angular drive damping 필요."""
    from pxr import UsdPhysics
    wheel_joints = ["front_left_wheel", "rear_left_wheel",
                    "front_right_wheel", "rear_right_wheel"]
    for j in wheel_joints:
        jp = f"/World/Scout/joints/{j}"
        prim = stage.GetPrimAtPath(jp)
        if not prim or not prim.IsValid():
            print(f"[scene] 휠조인트 못찾음: {jp}")
            continue
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(15000.0)
        drive.CreateMaxForceAttr(1e6)
    print("[scene] 휠 드라이브 설정 (stiffness=0, damping=15000)")


def configure_wheel_friction(stage, mat):
    """휠 충돌면에 저마찰 재질 → skid-steer 제자리 회전 가능 (옆 미끄러짐 허용)."""
    from pxr import UsdShade
    for j in ["front_left_wheel_link", "rear_left_wheel_link",
              "front_right_wheel_link", "rear_right_wheel_link"]:
        for sub in ["/collisions", ""]:
            p = f"/World/Scout/{j}{sub}"
            prim = stage.GetPrimAtPath(p)
            if prim and prim.IsValid():
                UsdShade.MaterialBindingAPI(prim).Bind(mat, materialPurpose="physics")
    print("[scene] 휠 저마찰 재질 적용 (회전 가능하게)")


def make_lowfric_material(stage, path="/World/Looks/lowfric"):
    """저마찰 물리 재질 (벽에 적용 → skid-steer 차체가 코너에서 미끄러지게)."""
    from pxr import UsdShade
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(0.05)
    pm.CreateDynamicFrictionAttr(0.05)
    pm.CreateRestitutionAttr(0.0)
    return mat


def build_world():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    stage = omni.usd.get_context().get_stage()

    # 조명
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
    dome.CreateIntensityAttr(1000.0)

    return world, stage


def spawn_walls(stage, lowfric_mat=None):
    """map_5floor의 벽(흰색 아닌 영역)을 박스로 생성. 가로 run 병합으로 박스 수 절감."""
    from pxr import UsdShade
    pgm = cv2.imread(PGM_PATH, cv2.IMREAD_GRAYSCALE)
    h, w = pgm.shape
    origin = _origin_from_pgm(w, h)
    # 벽 = 자유공간(흰색)이 아닌 곳
    wall = (pgm <= OCC_THRESH)
    # WALL_MERGE 단위로 다운샘플 (블록 내 벽 비율 > 0.5면 벽)
    bh, bw = h // WALL_MERGE, w // WALL_MERGE
    cell = PGM_RES * WALL_MERGE
    block_wall = np.zeros((bh, bw), bool)
    for by in range(bh):
        for bx in range(bw):
            blk = wall[by*WALL_MERGE:(by+1)*WALL_MERGE, bx*WALL_MERGE:(bx+1)*WALL_MERGE]
            block_wall[by, bx] = blk.mean() > 0.5

    walls_root = "/World/map_walls"
    UsdGeom.Xform.Define(stage, Sdf.Path(walls_root))
    n = 0
    for by in range(bh):
        bx = 0
        while bx < bw:
            if not block_wall[by, bx]:
                bx += 1
                continue
            # 가로 연속 run 병합
            x0 = bx
            while bx < bw and block_wall[by, bx]:
                bx += 1
            run = bx - x0
            # 박스 중심 (월드)
            # 픽셀(col, row) → 월드: x = origin_x + col*RES ; y는 상하 반전
            cx_px = (x0 + run / 2.0) * WALL_MERGE
            cy_px = (by + 0.5) * WALL_MERGE
            wx = origin[0] + cx_px * PGM_RES
            wy = origin[1] + (h - cy_px) * PGM_RES
            path = f"{walls_root}/w{n}"
            cube = UsdGeom.Cube.Define(stage, Sdf.Path(path))
            cube.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(cube)
            xf.AddTranslateOp().Set(Gf.Vec3d(float(wx), float(wy), WALL_H / 2.0))
            xf.AddScaleOp().Set(Gf.Vec3f(run * cell, cell, WALL_H))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            if lowfric_mat is not None:
                UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(
                    lowfric_mat, materialPurpose="physics")
            n += 1
    print(f"[scene] 벽 박스 {n}개 생성 (병합 {WALL_MERGE}px, {cell:.2f}m)")
    return n


def load_voronoi_path():
    """voronoi_waypoints.yaml → 월드 (x,y) 리스트 + 누적 arc-length."""
    with open(VORONOI_YAML) as f:
        raw = yaml.safe_load(f).get("waypoints", [])
    pts = np.array([[w[0], w[1]] for w in raw], dtype=float)
    seg = np.sqrt(((pts[1:] - pts[:-1]) ** 2).sum(axis=1))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    return pts, cum


def _pos_at_s(pts, cum, s):
    """경로 arc-length s(m)에서의 (x,y) 보간."""
    s = max(0.0, min(cum[-1], s))
    i = int(np.searchsorted(cum, s) - 1)
    i = max(0, min(len(pts) - 2, i))
    seg_len = cum[i + 1] - cum[i]
    t = 0.0 if seg_len < 1e-6 else (s - cum[i]) / seg_len
    p = pts[i] * (1 - t) + pts[i + 1] * t
    return float(p[0]), float(p[1])


OBSTACLE_MODE = os.environ.get("AEB_OBSTACLE_MODE", "along")  # along | cross
CROSS_HALF    = float(os.environ.get("AEB_CROSS_HALF", "0.9"))  # 교차 모드 경로수직 왕복 반폭(m)


def _tangent_at_s(pts, cum, s):
    """경로 arc-length s에서의 진행방향 단위벡터."""
    s = max(0.0, min(cum[-1], s))
    i = int(np.searchsorted(cum, s) - 1)
    i = max(0, min(len(pts) - 2, i))
    t = pts[i + 1] - pts[i]
    n = float(np.hypot(t[0], t[1]))
    return (t / n) if n > 1e-6 else np.array([1.0, 0.0])


def _person_state(rng, pts, cum):
    """사람 1명의 초기 상태(모드별). 재배치(reseed)에도 재사용."""
    total = cum[-1]
    cs = float(rng.uniform(0.2 * total, 0.85 * total))   # 경로 위 중심 위치
    st = {"dir": int(rng.choice([-1, 1]))}
    if OBSTACLE_MODE == "cross":
        cx, cy = _pos_at_s(pts, cum, cs)
        tg = _tangent_at_s(pts, cum, cs)
        st.update({"cx": cx, "cy": cy, "px": -tg[1], "py": tg[0],
                   "off": float(rng.uniform(-CROSS_HALF, CROSS_HALF))})
    else:
        st["s"] = cs
    return st


def _person_xy(st, pts, cum):
    if OBSTACLE_MODE == "cross":
        return st["cx"] + st["off"] * st["px"], st["cy"] + st["off"] * st["py"]
    return _pos_at_s(pts, cum, st["s"])


def spawn_people(stage, pts, cum):
    """동적 장애물 사람(캡슐) 생성. RTX LiDAR가 감지하도록 렌더링 geometry."""
    rng = np.random.default_rng(PEOPLE_SEED)
    people = []
    for i in range(NUM_PEOPLE):
        path = f"/World/people/person_{i}"
        cap = UsdGeom.Capsule.Define(stage, Sdf.Path(path))
        cap.CreateRadiusAttr(PERSON_R)
        cap.CreateHeightAttr(PERSON_H)
        cap.CreateAxisAttr("Z")
        cap.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.55, 0.35)])
        UsdPhysics.CollisionAPI.Apply(cap.GetPrim())   # 물리 충돌(선택)
        st = _person_state(rng, pts, cum)
        op = UsdGeom.XformCommonAPI(cap.GetPrim())
        x, y = _person_xy(st, pts, cum)
        op.SetTranslate(Gf.Vec3d(x, y, PERSON_ZC))
        st["op"] = op
        people.append(st)
    print(f"[scene] 동적 장애물 사람 {NUM_PEOPLE}명 생성 (속도 {PERSON_SPEED} m/s, mode={OBSTACLE_MODE})")
    return people


def reseed_people(people, pts, cum, seed):
    """run마다 사람 배치를 새 시드로 재배치(prim 유지, 상태만 갱신)."""
    rng = np.random.default_rng(seed)
    for p in people:
        st = _person_state(rng, pts, cum)
        for k, v in st.items():
            p[k] = v


def update_people(people, pts, cum, dt):
    """매 스텝 사람 이동: along=경로따라, cross=경로 수직 왕복."""
    total = cum[-1]
    for p in people:
        if OBSTACLE_MODE == "cross":
            p["off"] += p["dir"] * PERSON_SPEED * dt
            if p["off"] >= CROSS_HALF:
                p["off"] = CROSS_HALF; p["dir"] = -1
            elif p["off"] <= -CROSS_HALF:
                p["off"] = -CROSS_HALF; p["dir"] = 1
        else:
            p["s"] += p["dir"] * PERSON_SPEED * dt
            if p["s"] >= total:
                p["s"] = total; p["dir"] = -1
            elif p["s"] <= 0.0:
                p["s"] = 0.0; p["dir"] = 1
        x, y = _person_xy(p, pts, cum)
        p["op"].SetTranslate(Gf.Vec3d(x, y, PERSON_ZC))


ROBOT_PRIM   = "/World/Scout"
BASE_LINK    = "/World/Scout/base_link"
WHEEL_JOINTS = ["front_left_wheel", "rear_left_wheel", "front_right_wheel", "rear_right_wheel"]
WHEEL_RADIUS = 0.16      # Scout Mini 바퀴 반지름(m, URDF 실측 1.600e-01)
WHEEL_BASE   = 0.4165    # 좌우 트랙 폭(m, URDF y-offset 0.2082515 ×2)


def setup_lidar_and_ros2(stage):
    """RTX LiDAR(후방 장착) + ROS2 bridge OmniGraph (clock/scan/cmd_vel→drive/TF)."""
    import omni.kit.commands
    import omni.replicator.core as rep
    import omni.graph.core as og

    # ── RTX LiDAR 생성 (base_link 자식, 후방 장착: yaw 180° → quat z=1) ──
    _, lidar = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path="/lidar",
        parent=BASE_LINK,
        config="RPLIDAR_S2E",      # 실제 로봇과 동일한 RPLidar S2E (2D, LaserScan)
        translation=(0.0, 0.0, 0.25),
        orientation=Gf.Quatd(0.0, 0.0, 0.0, 1.0),   # (w,x,y,z)=(0,0,0,1) = yaw 180° (후방)
    )
    lidar_path = lidar.GetPath().pathString
    # RTX lidar 데이터가 흐르려면 적정 해상도 + lidar render_vars 필요 (공식 테스트 기준)
    render_product = rep.create.render_product(
        lidar_path, resolution=(128, 128),
        render_vars=["GenericModelOutput", "RtxSensorMetadata"], name="lidar_rp")
    print(f"[scene] RTX LiDAR 생성 @ {lidar_path} (후방 장착)")

    # ── ROS2 OmniGraph (clock/scan/tf만; cmd_vel→이동은 운동학으로 처리) ──
    og.Controller.edit(
        {"graph_path": "/AEBGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("PubScan", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ("PubTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PubClock.inputs:execIn"),
                ("OnTick.outputs:tick", "PubScan.inputs:execIn"),
                ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PubClock.inputs:topicName", "/clock"),
                ("PubScan.inputs:topicName", "/scan"),
                ("PubScan.inputs:frameId", "laser"),
                ("PubScan.inputs:type", "laser_scan"),
                ("PubScan.inputs:renderProductPath", render_product.path),
                ("PubTF.inputs:topicName", "/tf"),
                ("PubTF.inputs:targetPrims", [BASE_LINK]),
            ],
        },
    )
    print("[scene] ROS2 graph 생성 (/clock /scan /tf; 이동은 운동학)")


def main():
    world, stage = build_world()
    lowfric = make_lowfric_material(stage)
    spawn_walls(stage, lowfric_mat=lowfric)

    # Scout Mini 로드 (start 위치). 참조 USD의 xformOp이 incompatible →
    # ClearXformOpOrder 후 깨끗한 translate 추가 (XformCommonAPI 실패 회피)
    rooms_start = (args.sx, args.sy, 0.2)
    add_reference_to_stage(usd_path=SCOUT_USD, prim_path=ROBOT_PRIM)
    scout_xf = UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT_PRIM))
    scout_xf.ClearXformOpOrder()
    scout_xf.AddTranslateOp().Set(Gf.Vec3d(*rooms_start))
    # ★경로 방향을 보고 스폰★ (안 그러면 path_follower가 heading오차>80°로 v=0 freeze)
    # AEB 트림경로 idx0→idx1 방향 ≈ atan2(-0.09, 0.03) ≈ -1.25 rad
    spawn_yaw = args.syaw
    scout_xf.AddRotateZOp().Set(np.degrees(spawn_yaw))
    print(f"[scene] Scout 로드 @ {rooms_start}, yaw={np.degrees(spawn_yaw):.0f}°")

    # 운동학 제어: base_link을 kinematic rigid body로(브로드페이즈 NaN 안전) + articulation 제거.
    # 매 프레임 base_link 자체 pose를 set → TF가 정확히 따라옴 (부모 xform 이동과 달리 동기 유지).
    from pxr import UsdPhysics as _UP
    nkin = nart = 0
    for prim in stage.Traverse():
        if "/World/Scout" not in str(prim.GetPath()):
            continue
        if prim.HasAPI(_UP.RigidBodyAPI):
            _UP.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
            nkin += 1
        if prim.HasAPI(_UP.ArticulationRootAPI):
            prim.RemoveAPI(_UP.ArticulationRootAPI)
            nart += 1
    print(f"[scene] base_link kinematic: RigidBody {nkin}개, ArticulationRoot {nart}개 제거")
    setup_lidar_and_ros2(stage)

    # 동적 장애물 사람 (경로 따라 왕복, LiDAR가 감지)
    pts, cum = load_voronoi_path()
    people = spawn_people(stage, pts, cum)

    world.reset()

    from isaacsim.core.prims import SingleRigidPrim
    robot = SingleRigidPrim(prim_path=BASE_LINK, name="scout")
    robot.initialize()

    dt = world.get_physics_dt()
    print(f"[scene] Phase 3 완료 (LiDAR + ROS2 + 사람 {NUM_PEOPLE}명, kinematic rigidprim 구동). dt={dt:.4f}")

    # ── 운동학 제어 상태 (cmd_vel 파일 브리지에서 읽음) ──
    CMD_PATH = "/tmp/cmd_vel.txt"
    rx, ry, ryaw = float(args.sx), float(args.sy), float(spawn_yaw)
    rz = rooms_start[2]

    def _read_cmd():
        try:
            with open(CMD_PATH) as f:
                v, w = f.read().split()[:2]
                return float(v), float(w)
        except Exception:
            return 0.0, 0.0

    def _set_pose(x, y, yaw):
        q = np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)], dtype=np.float32)
        robot.set_world_pose(position=np.array([x, y, rz], dtype=np.float32), orientation=q)

    RESET_PATH = "/tmp/reset_robot.txt"
    READY_PATH = "/tmp/robot_ready.txt"

    def _check_reset():
        nonlocal rx, ry, ryaw, log_t
        if os.path.exists(RESET_PATH):
            seed = None
            try:
                with open(RESET_PATH) as f:
                    parts = f.read().split()
                rx, ry, ryaw = float(parts[0]), float(parts[1]), float(parts[2])
                if len(parts) >= 4:                # 4번째 값 = 사람 재배치 시드
                    seed = int(float(parts[3]))
            except Exception:
                rx, ry, ryaw = float(args.sx), float(args.sy), float(spawn_yaw)
            if seed is not None:
                reseed_people(people, pts, cum, seed)
            os.remove(RESET_PATH)
            open(READY_PATH, "w").write("1\n")
            print(f"[kin] ★리셋★ → ({rx:.2f},{ry:.2f},{np.degrees(ryaw):.0f}°) seed={seed}", flush=True)
            log_t = 0.0

    step = 0
    log_t = 0.0
    while simulation_app.is_running():
        _check_reset()
        v, w = _read_cmd()
        # 차동구동 운동학 적분 (base_link +x = 전진)
        ryaw += w * dt
        rx += v * math.cos(ryaw) * dt
        ry += v * math.sin(ryaw) * dt
        _set_pose(rx, ry, ryaw)
        update_people(people, pts, cum, dt)
        world.step(render=True)
        step += 1
        log_t += dt
        if step % 60 == 0:
            print(f"[kin] t={log_t:5.1f}s pos=({rx:6.2f},{ry:6.2f}) yaw={np.degrees(ryaw):6.1f}° "
                  f"cmd v={v:+.2f} w={w:+.2f}", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
