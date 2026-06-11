# # scout_mini_obstacle_avoidance.py
# import math

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from std_msgs.msg import Float32
# from rclpy.qos import qos_profile_sensor_data


# class ScoutMiniObstacleAvoidance(Node):
#     """
#     LiDAR /scan을 이용해서 좌/우 어느 쪽이 더 막혀있는지 보고
#     회피 조향 힌트를 publish 하는 노드.
#     - 출력: /scout_mini/oa_steer (Float32, -1.0 ~ 1.0)
#       *  0.0 : 회피 없음 (Path follower 그대로)
#       *  >0  : 왼쪽으로 피하기 (양수일수록 강하게)
#       *  <0  : 오른쪽으로 피하기
#     """

#     def __init__(self):
#         super().__init__('scout_mini_obstacle_avoidance')

#         # 파라미터 (필요시 launch에서 튜닝)
#         self.declare_parameter('scan_deg', 30.0)          # 전방 ±30°
#         self.declare_parameter('detect_distance', 1.0)    # 이 이내 장애물만 회피 고려
#         self.declare_parameter('min_valid_range', 0.05)   # 너무 가까운 노이즈 제거
#         self.declare_parameter('min_points', 5)           # 최소 점 개수 (노이즈 방지)

#         self.scan_rad = math.radians(
#             float(self.get_parameter('scan_deg').value)
#         )
#         self.detect_distance = float(self.get_parameter('detect_distance').value)
#         self.min_valid_range = float(self.get_parameter('min_valid_range').value)
#         self.min_points = int(self.get_parameter('min_points').value)

#         # LiDAR 구독
#         self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.laser_callback,
#             qos_profile_sensor_data
#         )

#         # OA 조향 publish
#         self.pub_steer = self.create_publisher(Float32, '/scout_mini/oa_steer', 10)

#         self.get_logger().info(
#             f'ScoutMiniObstacleAvoidance started. FOV=±{math.degrees(self.scan_rad):.1f}deg, '
#             f'detect<= {self.detect_distance:.2f}m'
#         )

#     def laser_callback(self, msg: LaserScan):
#         angle_min = msg.angle_min
#         angle_inc = msg.angle_increment

#         left_weight = 0.0   # +각도 쪽
#         right_weight = 0.0  # -각도 쪽
#         count = 0

#         for i, r in enumerate(msg.ranges):
#             if math.isinf(r) or math.isnan(r):
#                 continue
#             if r <= self.min_valid_range or r > self.detect_distance:
#                 continue

#             # 1) LiDAR 프레임 각도
#             angle = angle_min + i * angle_inc

#             # 2) base_link 기준으로 180도 회전 보정 (라이다가 뒤를 보고 있을 때)
#             angle += math.pi

#             # 3) [-pi, pi] 정규화
#             if angle > math.pi:
#                 angle -= 2.0 * math.pi
#             elif angle < -math.pi:
#                 angle += 2.0 * math.pi

#             # 전방 FOV 내만 사용
#             if abs(angle) > self.scan_rad:
#                 continue

#             # 가까울수록 weight 크게 (detect_distance - r)
#             weight = self.detect_distance - r
#             if weight <= 0.0:
#                 continue

#             if angle >= 0.0:
#                 # 좌측(+) 쪽 장애물
#                 left_weight += weight
#             else:
#                 # 우측(-) 쪽 장애물
#                 right_weight += weight

#             count += 1

#         steer_msg = Float32()

#         # 장애물 점이 너무 적으면 -> 회피 안 함
#         if count < self.min_points:
#             steer_msg.data = 0.0
#         else:
#             total = left_weight + right_weight
#             if total <= 1e-6:
#                 steer_msg.data = 0.0
#             else:
#                 # left_weight 크다 = 왼쪽이 더 막힘 -> 오른쪽(음수)으로 피하기
#                 steer = (right_weight - left_weight) / total
#                 # [-1, 1] 클램프
#                 if steer > 1.0:
#                     steer = 1.0
#                 elif steer < -1.0:
#                     steer = -1.0

#                 steer_msg.data = steer
#                 self.get_logger().debug(
#                     f'OA: left_w={left_weight:.3f}, right_w={right_weight:.3f}, '
#                     f'steer={steer:.2f}, points={count}'
#                 )

#         self.pub_steer.publish(steer_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ScoutMiniObstacleAvoidance()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
# scout_mini_obstacle_avoidance.py
# scout_mini_obstacle_avoidance.py
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data


class ScoutMiniObstacleAvoidance(Node):
    """
    LiDAR /scan을 이용해서 좌/우 어느 쪽이 더 막혀있는지 보고
    회피 조향 힌트를 publish 하는 노드.

    - 출력: /scout_mini/oa_steer (Float32, -1.0 ~ 1.0)
      *  0.0 : 회피 없음 (Path follower 그대로)
      *  >0  : 왼쪽으로 피하기 (양수일수록 강하게)
      *  <0  : 오른쪽으로 피하기
    """

    def __init__(self):
        super().__init__('scout_mini_obstacle_avoidance')

        # === 파라미터 (필요하면 launch에서 튜닝) ===
        # 전방 FOV (각도)
        self.declare_parameter('scan_deg', 30.0)           # 전방 ±30°
        # 전방 박스 영역 x 범위 (m)
        self.declare_parameter('x_min', 0.4)               # 0.4m 보다 앞부터
        self.declare_parameter('x_max', 1.5)               # 1.5m 이내까지만
        # 전방 박스 영역 y 반폭 (m) → |y| <= y_half 인 점만 사용
        self.declare_parameter('y_half', 0.45)             # 좌우 45cm 박스
        # 장애물 너무 가까이 붙어서 돌면 → x_min 살짝 키우기 (예: 0.5~0.6)

        # 너무 일찍 피해서 경로 많이 벗어나면 → x_max 줄이기 (예: 1.3)

        # 좌우 여유가 너무 좁다면 → y_half 키우기 (예: 0.5~0.6)

        # 기타 필터
        self.declare_parameter('min_valid_range', 0.05)    # 너무 가까운 노이즈 제거
        self.declare_parameter('min_points', 5)            # 최소 점 개수 (노이즈 방지)

        # 중앙에 두꺼운 장애물이 있을 때 어느 쪽으로 피할지 (기본: 왼쪽)
        self.declare_parameter('central_bias_side', -1.0)   # +1: 왼쪽, -1: 오른쪽
        self.declare_parameter('central_ratio_thresh', 0.3)  # 좌/우 거의 비슷할 때 기준

        # 파라미터 값 읽기
        self.scan_rad = math.radians(
            float(self.get_parameter('scan_deg').value)
        )
        self.x_min = float(self.get_parameter('x_min').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.y_half = float(self.get_parameter('y_half').value)

        self.min_valid_range = float(self.get_parameter('min_valid_range').value)
        self.min_points = int(self.get_parameter('min_points').value)
        self.central_bias_side = float(self.get_parameter('central_bias_side').value)
        self.central_ratio_thresh = float(self.get_parameter('central_ratio_thresh').value)

        # LiDAR 구독
        self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )

        # OA 조향 publish
        self.pub_steer = self.create_publisher(Float32, '/scout_mini/oa_steer', 10)

        self.get_logger().info(
            f'ScoutMiniObstacleAvoidance started. FOV=±{math.degrees(self.scan_rad):.1f}deg, '
            f'box: x∈[{self.x_min:.2f},{self.x_max:.2f}], |y|<={self.y_half:.2f}'
        )

    def laser_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        left_weight = 0.0   # y >= 0 (왼쪽)
        right_weight = 0.0  # y <  0 (오른쪽)
        count = 0

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r <= self.min_valid_range:
                continue

            # 1) LiDAR 프레임 각도
            angle = angle_min + i * angle_inc

            # 2) base_link 기준으로 180도 회전 (라이다가 뒤를 보고 있을 때)
            angle += math.pi

            # 3) [-pi, pi] 정규화
            if angle > math.pi:
                angle -= 2.0 * math.pi
            elif angle < -math.pi:
                angle += 2.0 * math.pi

            # 전방 FOV 필터 (안전장치)
            if abs(angle) > self.scan_rad:
                continue

            # 4) base_link 기준 좌표
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # 뒤쪽은 무시
            if x <= 0.0:
                continue

            # 전방 박스 영역 내의 점만 사용
            if not (self.x_min <= x <= self.x_max):
                continue
            if abs(y) > self.y_half:
                continue

            # 가까울수록 weight 크게 (앞쪽 x 기준)
            weight = self.x_max - x
            if weight <= 0.0:
                continue

            if y >= 0.0:
                left_weight += weight
            else:
                right_weight += weight

            count += 1

        steer_msg = Float32()

        # 장애물 점이 너무 적으면 → 회피 안 함
        if count < self.min_points:
            steer_msg.data = 0.0
            self.pub_steer.publish(steer_msg)
            return

        total = left_weight + right_weight
        if total <= 1e-6:
            steer_msg.data = 0.0
            self.pub_steer.publish(steer_msg)
            return

        # 기본 steer: 오른쪽 - 왼쪽 (왼쪽이 많으면 음수 → 오른쪽으로, 반대도 마찬가지)
        diff = right_weight - left_weight
        base_steer = diff / total  # [-1,1] 근사

        # 좌/우 비중이 거의 비슷하면 = 정면에 두꺼운 장애물
        ratio = abs(diff) / total
        if ratio < self.central_ratio_thresh:
            # 한쪽으로 일관되게 피하도록 bias 적용
            steer = 0.8 * (1.0 if self.central_bias_side >= 0.0 else -1.0)
        else:
            steer = base_steer

        # [-1, 1] 클램프
        if steer > 1.0:
            steer = 1.0
        elif steer < -1.0:
            steer = -1.0

        steer_msg.data = steer

        self.get_logger().debug(
            f'OA: left_w={left_weight:.3f}, right_w={right_weight:.3f}, '
            f'diff={diff:.3f}, ratio={ratio:.2f}, steer={steer:.2f}, points={count}'
        )

        self.pub_steer.publish(steer_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScoutMiniObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
