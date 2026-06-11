# sm_path_follower_test4.py
import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def yaw_from_quaternion(q):
    # q: geometry_msgs/Quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    # [-pi, pi]로 정규화
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PathFollower(Node):
    def __init__(self):
        super().__init__('sm_path_follower_test4')

        # === 파라미터 ===
        # waypoint 파일 경로
        self.declare_parameter('waypoint_file', '')
        # 기준 lookahead 거리 (가변 lookahead의 중심값 역할)
        self.declare_parameter('lookahead_dist', 0.8)
        # 기준 선속도 (직선 구간에서의 최대 속도)
        self.declare_parameter('linear_speed', 0.5)
        # Pure Pursuit gain (ω = v * k_ang * curvature_pp)
        self.declare_parameter('k_ang', 1.0)
        # 최대 각속도 (rad/s)
        self.declare_parameter('max_ang_vel', 1.0)
        # 최종 goal 근처 허용 오차 (직선거리 기준)
        self.declare_parameter('goal_tolerance', 0.3)
        # TF 프레임 (recorder와 동일하게)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # === 종방향(속도) 제어용 추가 파라미터 ===
        self.declare_parameter('kappa_v_gain', 0.25)
        self.declare_parameter('slowdown_distance', 0.7)
        self.declare_parameter('min_speed', 0.40)

        # 곡률 관련 추가 파라미터
        self.declare_parameter('kappa_deadband', 0.22)
        self.declare_parameter('min_curv_factor', 0.90)

        # === Obstacle Avoidance 관련 파라미터 ===
        # 회피 시 추가로 넣을 최대 각속도 (절댓값)
        self.declare_parameter('oa_max_omega', 0.5)
        # 회피 시 속도 줄이는 정도 (0~1, 0.5면 최대 50%까지 감속)
        self.declare_parameter('oa_v_slow_gain', 0.5)

        # === 파라미터 값 읽기 ===
        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value

        if waypoint_file == '':
            waypoint_file = os.path.join(
                os.path.expanduser('~'),
                'ros2_ws',
                'src',
                'scout_mini_tools',
                'waypoint',
                'waypoints_map_5floor_smoothSpline_developed.yaml' #맵 바꾸려면 여기 파일 바꿔주면됨 
            )
            self.get_logger().warn(
                f'waypoint_file 파라미터가 비어 있어서, '
                f'기본 경로 {waypoint_file} 를 사용합니다.'
            )

        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_ang_vel = float(self.get_parameter('max_ang_vel').value)

        # 종방향 제어용 파라미터
        self.kappa_v_gain = float(self.get_parameter('kappa_v_gain').value)
        self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        self.min_speed = float(self.get_parameter('min_speed').value)

        # 곡률 관련 파라미터
        self.kappa_deadband = float(self.get_parameter('kappa_deadband').value)
        self.min_curv_factor = float(self.get_parameter('min_curv_factor').value)

        # OA 파라미터
        self.oa_max_omega = float(self.get_parameter('oa_max_omega').value)
        self.oa_v_slow_gain = float(self.get_parameter('oa_v_slow_gain').value)

        # 가변 lookahead 거리 범위 설정
        self.min_lookahead = 0.5 * self.lookahead_dist
        self.max_lookahead = 1.5 * self.lookahead_dist
        self.current_lookahead = self.lookahead_dist

        # === waypoints 로드 (x, y, yaw, curvature, cumlength) ===
        if not os.path.exists(waypoint_file):
            self.get_logger().error(f'waypoint 파일을 찾을 수 없습니다: {waypoint_file}')
            raise FileNotFoundError(waypoint_file)

        with open(waypoint_file, 'r') as f:
            data = yaml.safe_load(f)

        raw_wps = data.get('waypoints', [])
        if len(raw_wps) == 0:
            self.get_logger().error('waypoints 리스트가 비어 있습니다.')
            raise RuntimeError('empty waypoints')

        tmp_points = []
        for wp in raw_wps:
            if len(wp) < 2:
                self.get_logger().warn(f"잘못된 waypoint 형식(2개 미만): {wp}, 건너뜁니다.")
                continue

            x = float(wp[0])
            y = float(wp[1])
            yaw = float(wp[2]) if len(wp) >= 3 else 0.0

            if len(wp) >= 5:
                kappa = float(wp[3])
                s_val = float(wp[4])
            else:
                kappa = None
                s_val = None

            tmp_points.append({
                'x': x,
                'y': y,
                'yaw': yaw,
                'kappa': kappa,
                's': s_val,
            })

        if len(tmp_points) == 0:
            self.get_logger().error('유효한 waypoints가 없습니다.')
        xs = [p['x'] for p in tmp_points]
        ys = [p['y'] for p in tmp_points]
        s_list = [0.0]
        for i in range(1, len(xs)):
            ds = math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1])
            s_list.append(s_list[-1] + ds)

        self.waypoints = []
        for i, p in enumerate(tmp_points):
            kappa = p['kappa'] if p['kappa'] is not None else 0.0
            s_val = p['s'] if p['s'] is not None else s_list[i]

            self.waypoints.append({
                'x': p['x'],
                'y': p['y'],
                'yaw': p['yaw'],
                'kappa': kappa,
                's': s_val,
            })

        self.goal_s = self.waypoints[-1]['s']

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints (x,y,yaw,kappa,s) from {waypoint_file}'
        )

        self.current_idx = 0
        self.goal_reached = False

        # TF 버퍼/리스너 (map -> base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # === E-STOP 관련 플래그 및 구독 ===
        self.estop_active = False
        self.create_subscription(
            Bool,
            '/scout_mini/e_stop',
            self.estop_callback,
            10
        )

        # === AEB 스케일 구독 ===
        self.aeb_scale = 1.0  # 기본값: AEB 없음
        self.create_subscription(
            Float32,
            '/scout_mini/aeb_scale',
            self.aeb_callback,
            10
        )

        # === OA 조향 구독 (/scout_mini/oa_steer: -1.0 오른쪽 ~ +1.0 왼쪽) ===
        self.oa_steer = 0.0
        self.create_subscription(
            Float32,
            '/scout_mini/oa_steer',
            self.oa_steer_callback,
            10
        )

        # 제어 주기: 0.02s => 50Hz
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(
            f'PathFollower started. global_frame={self.global_frame}, base_frame={self.base_frame}'
        )

    # === E-STOP 콜백 ===
    def estop_callback(self, msg: Bool):
        if msg.data and not self.estop_active:
            self.get_logger().warn('E-STOP 활성화: 로봇을 정지합니다.')
        elif not msg.data and self.estop_active:
            self.get_logger().info('E-STOP 해제.')
        self.estop_active = msg.data

    # === AEB 콜백 ===
    def aeb_callback(self, msg: Float32):
        s = float(msg.data)
        if s < 0.0:
            s = 0.0
        elif s > 1.0:
            s = 1.0
        if abs(s - self.aeb_scale) > 1e-3:
            self.get_logger().debug(f'AEB scale: {self.aeb_scale:.2f} -> {s:.2f}')
        self.aeb_scale = s

    # === OA 조향 콜백 ===
    def oa_steer_callback(self, msg: Float32):
        # AEB가 거의 풀브레이크 상태면 OA 억제
        if self.aeb_scale < 0.2:
            self.oa_steer = 0.0
            return
        self.oa_steer = max(-1.0, min(1.0, float(msg.data)))

    def timer_callback(self):
        # ★ 최상단에서 E-STOP 우선 체크
        if self.estop_active:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        if self.goal_reached:
            return

        # === 현재 로봇 pose (map -> base_link) ===
        try:
            now = Time()
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                now
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(
                f'TF ({self.global_frame} -> {self.base_frame}) 를 아직 가져오지 못했습니다.'
            )
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        yaw = yaw_from_quaternion(transform.transform.rotation)

        # === goal까지 남은 거리(직선거리) ===
        goal_x = self.waypoints[-1]['x']
        goal_y = self.waypoints[-1]['y']
        goal_dist = math.hypot(goal_x - x, goal_y - y)

        if goal_dist < self.goal_tolerance:
            self.get_logger().info('Goal reached! Stopping.')
            self.goal_reached = True
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        # === 현재 경로 상 진행 정도 (s_current) ===
        s_current = self.waypoints[self.current_idx]['s']
        remaining_s = max(0.0, self.goal_s - s_current)

        # === 가변 lookahead 거리 사용 ===
        Ld = self.current_lookahead

        # === lookahead target 선택 ===
        target_idx = self.current_idx
        for i in range(self.current_idx, len(self.waypoints)):
            wx = self.waypoints[i]['x']
            wy = self.waypoints[i]['y']
            dist = math.hypot(wx - x, wy - y)
            if dist > Ld:
                target_idx = i
                break
        else:
            target_idx = len(self.waypoints) - 1

        self.current_idx = target_idx
        tx = self.waypoints[target_idx]['x']
        ty = self.waypoints[target_idx]['y']
        kappa_target = self.waypoints[target_idx]['kappa']

        # === 횡방향 제어: Pure Pursuit ===
        dx = tx - x
        dy = ty - y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        Ld_actual = math.hypot(local_x, local_y)
        if Ld_actual < 1e-6:
            curvature_pp = 0.0
        else:
            curvature_pp = 2.0 * local_y / (Ld_actual ** 2)

        angle_to_target = math.atan2(ty - y, tx - x)
        heading_error = normalize_angle(angle_to_target - yaw)
        heading_mag = abs(heading_error)

        # === 종방향 제어: 선속도 결정 (기본 pure pursuit 부분 그대로) ===
        v_nominal = self.linear_speed

        kappa_mag = abs(kappa_target)
        if kappa_mag <= self.kappa_deadband:
            curv_factor = 1.0
        else:
            kappa_eff = kappa_mag - self.kappa_deadband
            curv_factor = 1.0 / (1.0 + self.kappa_v_gain * kappa_eff)
            curv_factor = max(self.min_curv_factor, curv_factor)

        if self.slowdown_distance > 0.0:
            slow_factor = min(1.0, remaining_s / self.slowdown_distance)
        else:
            slow_factor = 1.0

        # 기본 path-following 속도
        v = v_nominal * curv_factor * slow_factor
        v = max(self.min_speed, min(v, v_nominal))

        # === AEB 스케일 적용 (0~1) ===
        v *= self.aeb_scale

        # === OA 감속: oa_steer 크기에 비례 ===
        oa_alpha = abs(self.oa_steer)
        if oa_alpha > 0.0:
            v *= (1.0 - self.oa_v_slow_gain * oa_alpha)

        # heading error가 너무 크면 정지 회전
        if heading_mag > math.radians(80.0):
            v = 0.0

        # === 각속도 계산 ===
        omega = v * curvature_pp * self.k_ang

        # OA 조향: +1이면 왼쪽, -1이면 오른쪽
        if oa_alpha > 0.0:
            omega += self.oa_max_omega * self.oa_steer

        # 각속도 제한
        if omega > self.max_ang_vel:
            omega = self.max_ang_vel
        elif omega < -self.max_ang_vel:
            omega = -self.max_ang_vel

        # === cmd_vel publish ===
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.cmd_pub.publish(twist)

        # === 다음 스텝을 위한 가변 Lookahead 업데이트 ===
        self.current_lookahead = self._update_lookahead(v, remaining_s)

        self.get_logger().debug(
            f'idx={self.current_idx}, pos=({x:.2f},{y:.2f}), '
            f'target=({tx:.2f},{ty:.2f}), Ld={Ld:.2f}, '
            f'heading_err(deg)={math.degrees(heading_error):.1f}, '
            f'v={v:.2f}, w={omega:.2f}, kappa_path={kappa_target:.3f}, '
            f'rem_s={remaining_s:.2f}, aeb={self.aeb_scale:.2f}, '
            f'oa_steer={self.oa_steer:.2f}, oa_alpha={oa_alpha:.2f}'
        )

    def _update_lookahead(self, v, remaining_s):
        v_ref = max(self.linear_speed, 1e-3)
        alpha_v = max(0.0, min(1.0, abs(v) / v_ref))

        if self.slowdown_distance > 0.0:
            alpha_s = max(0.0, min(1.0, remaining_s / self.slowdown_distance))
        else:
            alpha_s = 1.0

        alpha = 0.7 * alpha_v + 0.3 * alpha_s

        Ld = self.min_lookahead + alpha * (self.max_lookahead - self.min_lookahead)
        return max(self.min_lookahead, min(Ld, self.max_lookahead))


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down PathFollower node')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

######################################################################################################################################################################################################################################
# # sm_path_follower_test4.py
# import math
# import os
# import yaml

# import rclpy
# from rclpy.node import Node
# from rclpy.time import Time

# from geometry_msgs.msg import Twist
# from std_msgs.msg import Bool, Float32   # ★ AEB, OA용
# from tf2_ros import Buffer, TransformListener
# from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


# def yaw_from_quaternion(q):
#     # q: geometry_msgs/Quaternion
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)


# def normalize_angle(angle):
#     # [-pi, pi]로 정규화
#     while angle > math.pi:
#         angle -= 2.0 * math.pi
#     while angle < -math.pi:
#         angle += 2.0 * math.pi
#     return angle


# class PathFollower(Node):
#     def __init__(self):
#         super().__init__('sm_path_follower_test4')

#         # === 파라미터 ===
#         self.declare_parameter('waypoint_file', '')
#         self.declare_parameter('lookahead_dist', 0.8)
#         self.declare_parameter('linear_speed', 0.5)
#         self.declare_parameter('k_ang', 1.0)
#         self.declare_parameter('max_ang_vel', 1.0)
#         self.declare_parameter('goal_tolerance', 0.3)
#         self.declare_parameter('global_frame', 'map')
#         self.declare_parameter('base_frame', 'base_link')

#         # 종방향 제어
#         self.declare_parameter('kappa_v_gain', 0.25)
#         self.declare_parameter('slowdown_distance', 0.7)
#         self.declare_parameter('min_speed', 0.40)

#         # 곡률 관련
#         self.declare_parameter('kappa_deadband', 0.22)
#         self.declare_parameter('min_curv_factor', 0.90)

#         # ★ OA 관련 (Pure Pursuit 위에 살짝 더해줄 gain)
#         self.declare_parameter('oa_steer_gain', 0.8)   # rad/s per 1.0 steer

#         # === 파라미터 값 읽기 ===
#         waypoint_file = self.get_parameter(
#             'waypoint_file'
#         ).get_parameter_value().string_value

#         if waypoint_file == '':
#             waypoint_file = os.path.join(
#                 os.path.expanduser('~'),
#                 'ros2_ws',
#                 'src',
#                 'scout_mini_tools',
#                 'waypoint',
#                 'waypoints_floor12_1127_smoothSpline_developed.yaml'
#             )
#             self.get_logger().warn(
#                 f'waypoint_file 파라미터가 비어 있어서, '
#                 f'기본 경로 {waypoint_file} 를 사용합니다.'
#             )

#         self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
#         self.linear_speed = float(self.get_parameter('linear_speed').value)
#         self.k_ang = float(self.get_parameter('k_ang').value)
#         self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
#         self.global_frame = self.get_parameter('global_frame').value
#         self.base_frame = self.get_parameter('base_frame').value
#         self.max_ang_vel = float(self.get_parameter('max_ang_vel').value)

#         self.kappa_v_gain = float(self.get_parameter('kappa_v_gain').value)
#         self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
#         self.min_speed = float(self.get_parameter('min_speed').value)

#         self.kappa_deadband = float(self.get_parameter('kappa_deadband').value)
#         self.min_curv_factor = float(self.get_parameter('min_curv_factor').value)

#         # OA gain
#         self.oa_steer_gain = float(self.get_parameter('oa_steer_gain').value)

#         # 가변 lookahead
#         self.min_lookahead = 0.5 * self.lookahead_dist
#         self.max_lookahead = 1.5 * self.lookahead_dist
#         self.current_lookahead = self.lookahead_dist

#         # === waypoints 로드 ===
#         if not os.path.exists(waypoint_file):
#             self.get_logger().error(f'waypoint 파일을 찾을 수 없습니다: {waypoint_file}')
#             raise FileNotFoundError(waypoint_file)

#         with open(waypoint_file, 'r') as f:
#             data = yaml.safe_load(f)

#         raw_wps = data.get('waypoints', [])
#         if len(raw_wps) == 0:
#             self.get_logger().error('waypoints 리스트가 비어 있습니다.')
#             raise RuntimeError('empty waypoints')

#         tmp_points = []
#         for wp in raw_wps:
#             if len(wp) < 2:
#                 self.get_logger().warn(f"잘못된 waypoint 형식(2개 미만): {wp}, 건너뜁니다.")
#                 continue

#             x = float(wp[0])
#             y = float(wp[1])
#             yaw = float(wp[2]) if len(wp) >= 3 else 0.0

#             if len(wp) >= 5:
#                 kappa = float(wp[3])
#                 s_val = float(wp[4])
#             else:
#                 kappa = None
#                 s_val = None

#             tmp_points.append({
#                 'x': x,
#                 'y': y,
#                 'yaw': yaw,
#                 'kappa': kappa,
#                 's': s_val,
#             })

#         if len(tmp_points) == 0:
#             self.get_logger().error('유효한 waypoints가 없습니다.')
#             raise RuntimeError('no valid waypoints')

#         xs = [p['x'] for p in tmp_points]
#         ys = [p['y'] for p in tmp_points]
#         s_list = [0.0]
#         for i in range(1, len(xs)):
#             ds = math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1])
#             s_list.append(s_list[-1] + ds)

#         self.waypoints = []
#         for i, p in enumerate(tmp_points):
#             kappa = p['kappa'] if p['kappa'] is not None else 0.0
#             s_val = p['s'] if p['s'] is not None else s_list[i]

#             self.waypoints.append({
#                 'x': p['x'],
#                 'y': p['y'],
#                 'yaw': p['yaw'],
#                 'kappa': kappa,
#                 's': s_val,
#             })

#         self.goal_s = self.waypoints[-1]['s']

#         self.get_logger().info(
#             f'Loaded {len(self.waypoints)} waypoints (x,y,yaw,kappa,s) from {waypoint_file}'
#         )

#         self.current_idx = 0
#         self.goal_reached = False

#         # TF
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # cmd_vel
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # === E-STOP ===
#         self.estop_active = False
#         self.create_subscription(
#             Bool,
#             '/scout_mini/e_stop',
#             self.estop_callback,
#             10
#         )

#         # === AEB scale ===
#         self.aeb_scale = 1.0
#         self.create_subscription(
#             Float32,
#             '/scout_mini/aeb_scale',
#             self.aeb_callback,
#             10
#         )

#         # === OA steer (새로 추가) ===
#         self.oa_steer = 0.0  # -1.0 ~ 1.0, 0이면 영향 없음
#         self.create_subscription(
#             Float32,
#             '/scout_mini/oa_steer',
#             self.oa_steer_callback,
#             10
#         )

#         # timer
#         self.timer = self.create_timer(0.02, self.timer_callback)
#         self.get_logger().info(
#             f'PathFollower started. global_frame={self.global_frame}, base_frame={self.base_frame}'
#         )

#     # === 콜백들 ===
#     def estop_callback(self, msg: Bool):
#         if msg.data and not self.estop_active:
#             self.get_logger().warn('E-STOP 활성화: 로봇을 정지합니다.')
#         elif not msg.data and self.estop_active:
#             self.get_logger().info('E-STOP 해제.')
#         self.estop_active = msg.data

#     def aeb_callback(self, msg: Float32):
#         s = float(msg.data)
#         if s < 0.0:
#             s = 0.0
#         elif s > 1.0:
#             s = 1.0
#         if abs(s - self.aeb_scale) > 1e-3:
#             self.get_logger().debug(f'AEB scale: {self.aeb_scale:.2f} -> {s:.2f}')
#         self.aeb_scale = s

#     def oa_steer_callback(self, msg: Float32):
#         # -1.0 ~ 1.0 범위로 클램프
#         v = float(msg.data)
#         if v < -1.0:
#             v = -1.0
#         elif v > 1.0:
#             v = 1.0
#         self.oa_steer = v

#     def timer_callback(self):
#         # 최상단 E-STOP
#         if self.estop_active:
#             twist = Twist()
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0
#             self.cmd_pub.publish(twist)
#             return

#         if self.goal_reached:
#             return

#         # 현재 pose
#         try:
#             now = Time()
#             transform = self.tf_buffer.lookup_transform(
#                 self.global_frame,
#                 self.base_frame,
#                 now
#             )
#         except (LookupException, ConnectivityException, ExtrapolationException):
#             self.get_logger().warn(
#                 f'TF ({self.global_frame} -> {self.base_frame}) 를 아직 가져오지 못했습니다.'
#             )
#             return

#         x = transform.transform.translation.x
#         y = transform.transform.translation.y
#         yaw = yaw_from_quaternion(transform.transform.rotation)

#         # goal까지 거리
#         goal_x = self.waypoints[-1]['x']
#         goal_y = self.waypoints[-1]['y']
#         goal_dist = math.hypot(goal_x - x, goal_y - y)

#         if goal_dist < self.goal_tolerance:
#             self.get_logger().info('Goal reached! Stopping.')
#             self.goal_reached = True
#             twist = Twist()
#             self.cmd_pub.publish(twist)
#             return

#         # 경로 진행 정도
#         s_current = self.waypoints[self.current_idx]['s']
#         remaining_s = max(0.0, self.goal_s - s_current)

#         # 가변 lookahead
#         Ld = self.current_lookahead

#         # target waypoint 선택
#         target_idx = self.current_idx
#         for i in range(self.current_idx, len(self.waypoints)):
#             wx = self.waypoints[i]['x']
#             wy = self.waypoints[i]['y']
#             dist = math.hypot(wx - x, wy - y)
#             if dist > Ld:
#                 target_idx = i
#                 break
#         else:
#             target_idx = len(self.waypoints) - 1

#         self.current_idx = target_idx
#         tx = self.waypoints[target_idx]['x']
#         ty = self.waypoints[target_idx]['y']
#         kappa_target = self.waypoints[target_idx]['kappa']

#         # === Pure Pursuit lateral ===
#         dx = tx - x
#         dy = ty - y
#         local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
#         local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

#         Ld_actual = math.hypot(local_x, local_y)
#         if Ld_actual < 1e-6:
#             curvature_pp = 0.0
#         else:
#             curvature_pp = 2.0 * local_y / (Ld_actual ** 2)

#         angle_to_target = math.atan2(ty - y, tx - x)
#         heading_error = normalize_angle(angle_to_target - yaw)
#         heading_mag = abs(heading_error)

#         # === longitudinal speed ===
#         v_nominal = self.linear_speed

#         kappa_mag = abs(kappa_target)
#         if kappa_mag <= self.kappa_deadband:
#             curv_factor = 1.0
#         else:
#             kappa_eff = kappa_mag - self.kappa_deadband
#             curv_factor = 1.0 / (1.0 + self.kappa_v_gain * kappa_eff)
#             curv_factor = max(self.min_curv_factor, curv_factor)

#         if self.slowdown_distance > 0.0:
#             slow_factor = min(1.0, remaining_s / self.slowdown_distance)
#         else:
#             slow_factor = 1.0

#         v = v_nominal * curv_factor * slow_factor
#         v = max(self.min_speed, min(v, v_nominal))

#         # AEB 적용
#         v *= self.aeb_scale

#         # heading error 너무 크면 제자리 회전
#         # if heading_mag > math.radians(80.0):
#         #     v = 0.0
        
#         # === heading error가 너무 크면 정지 회전 (단, OA가 없을 때만) ===
#         if heading_mag > math.radians(80.0) and abs(self.oa_steer) < 0.2:
#             # OA가 거의 없는 순수 path tracking 상황에서만 제자리 회전
#             v = 0.0

#         # === Pure Pursuit 각속도 ===
#         omega = v * curvature_pp * self.k_ang

#         # === OA 조향 바이어스 추가 (있으면) ===
#         # oa_steer: -1(오른쪽으로 크게 피하기) ~ +1(왼쪽으로 크게 피하기)
#         if abs(self.oa_steer) > 1e-3:
#             omega += self.oa_steer_gain * self.oa_steer
#             # OA 중에는 살짝 감속 (너무 과격하지 않게)
#             v *= 0.8

#         # 각속도 제한
#         if omega > self.max_ang_vel:
#             omega = self.max_ang_vel
#         elif omega < -self.max_ang_vel:
#             omega = -self.max_ang_vel

#         # 명령 publish
#         twist = Twist()
#         twist.linear.x = v
#         twist.angular.z = omega
#         self.cmd_pub.publish(twist)

#         # lookahead 업데이트
#         self.current_lookahead = self._update_lookahead(v, remaining_s)

#         self.get_logger().debug(
#             f'idx={self.current_idx}, pos=({x:.2f},{y:.2f}), '
#             f'target=({tx:.2f},{ty:.2f}), Ld={Ld:.2f}, '
#             f'heading_err(deg)={math.degrees(heading_error):.1f}, '
#             f'v={v:.2f}, w={omega:.2f}, kappa_path={kappa_target:.3f}, '
#             f'rem_s={remaining_s:.2f}, aeb={self.aeb_scale:.2f}, oa={self.oa_steer:.2f}'
#         )

#     def _update_lookahead(self, v, remaining_s):
#         v_ref = max(self.linear_speed, 1e-3)
#         alpha_v = max(0.0, min(1.0, abs(v) / v_ref))

#         if self.slowdown_distance > 0.0:
#             alpha_s = max(0.0, min(1.0, remaining_s / self.slowdown_distance))
#         else:
#             alpha_s = 1.0

#         alpha = 0.7 * alpha_v + 0.3 * alpha_s

#         Ld = self.min_lookahead + alpha * (self.max_lookahead - self.min_lookahead)
#         return max(self.min_lookahead, min(Ld, self.max_lookahead))


# def main(args=None):
#     rclpy.init(args=args)
#     node = PathFollower()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.get_logger().info('Shutting down PathFollower node')
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()
