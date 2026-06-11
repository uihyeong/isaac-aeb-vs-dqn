"""Scout USD 구조 점검: articulation root, 바퀴 조인트, base_link prim 경로."""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import os
import omni
from pxr import UsdGeom, UsdPhysics, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage

HOME = os.path.expanduser("~")
SCOUT_USD = os.path.join(HOME, "ros2_ws2", "src", "scout_ros2", "scout_description",
                         "urdf", "scout_mini", "scout_mini.usd")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
add_reference_to_stage(usd_path=SCOUT_USD, prim_path="/World/Scout")
UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/Scout")).SetTranslate(Gf.Vec3d(0, 0, 0.2))
world.reset()

lines = ["===== Scout prim 트리 (관련 항목) ====="]
for prim in stage.Traverse():
    p = str(prim.GetPath())
    if "/World/Scout" not in p:
        continue
    t = prim.GetTypeName()
    flags = []
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        flags.append("ARTICULATION_ROOT")
    if "Joint" in str(t):
        flags.append(f"JOINT({t})")
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        flags.append("RIGID")
    if any(k in p.lower() for k in ["wheel", "base_link", "chassis", "lidar", "laser"]) or flags:
        lines.append(f"  {p}  [{t}] {' '.join(flags)}")

with open("/tmp/scout_struct.txt", "w") as f:
    f.write("\n".join(lines) + "\n")

simulation_app.close()
