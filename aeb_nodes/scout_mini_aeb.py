import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from rclpy.qos import qos_profile_sensor_data


class ScoutMiniAEB(Node):
    """
    LiDAR /scan을 이용해서 전방 최소 거리 기반 AEB scale 계산 노드.
    - 출력: /scout_mini/aeb_scale (Float32, 0.0 ~ 1.0)
      * 1.0  : AEB 없음 (정상 속도)
      * 0.0  : 완전 정지
      * 0~1  : 감속 스케일
    """

    def __init__(self):
        super().__init__('scout_mini_aeb')

        # 파라미터 (필요하면 나중에 launch에서 튜닝)
        self.declare_parameter('fov_deg', 20.0)          # 전방 ±20도
        self.declare_parameter('stop_distance', 0.5)     # 이 거리 이하면 정지
        self.declare_parameter('slow_distance', 1.4)     # 이 이내부터 감속 시작
        self.declare_parameter('min_valid_range', 0.05)  # 이 이하는 노이즈로 무시

        self.fov = math.radians(
            float(self.get_parameter('fov_deg').value)
        )
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.min_valid_range = float(self.get_parameter('min_valid_range').value)

        if self.stop_distance >= self.slow_distance:
            self.get_logger().warn(
                f'stop_distance({self.stop_distance}) >= slow_distance({self.slow_distance}) '
                f'이므로 값을 조정하는 것이 좋습니다.'
            )

        # 전방 최소 거리 (디버깅용)
        self.front_min_dist = math.inf

        # 구독: LiDAR
        self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )

        # 발행: AEB scale
        self.aeb_pub = self.create_publisher(Float32, '/scout_mini/aeb_scale', 10)

        self.get_logger().info(
            f'ScoutMiniAEB started. fov=±{math.degrees(self.fov):.1f}deg, '
            f'stop={self.stop_distance:.2f}m, slow={self.slow_distance:.2f}m'
        )

    def laser_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        d_min = math.inf

        # 전방 ±fov 범위에서 최소 거리 계산
        for i, r in enumerate(msg.ranges):
            # 유효하지 않은 값 필터링
            if math.isinf(r) or math.isnan(r):
                continue
            if r <= self.min_valid_range:
                continue

            # 1) 라이다 프레임 기준 각도
            angle = angle_min + i * angle_inc

            # 2) base_link 기준으로 180도 회전 보정
            angle += math.pi

            # 3) [-pi, pi]로 정규화
            if angle > math.pi:
                angle -= 2.0 * math.pi
            elif angle < -math.pi:
                angle += 2.0 * math.pi

            # 4) 이제 angle은 base_link 기준 각도 → 전방 ±fov만 사용
            if -self.fov <= angle <= self.fov:
                if r < d_min:
                    d_min = r

        self.front_min_dist = d_min

        aeb_scale = Float32()
        aeb_scale.data = 1.0  # 기본값: AEB 없음

        if math.isinf(d_min):
            # 전방에 유의미한 측정 없음 → AEB 적용 안 함
            self.aeb_pub.publish(aeb_scale)
            return

        # 거리 기반 스케일 계산
        d = d_min

        if d <= self.stop_distance:
            # 긴급 정지
            aeb_scale.data = 0.0
            self.get_logger().warn(f'[AEB] STOP: d={d:.2f} m')

        elif d <= self.slow_distance:
            # 감속 구간: stop ~ slow 사이를 0~1로 선형 스케일링
            scale = (d - self.stop_distance) / (self.slow_distance - self.stop_distance)
            scale = max(0.0, min(1.0, scale))
            aeb_scale.data = scale
            self.get_logger().info(
                f'[AEB] SLOW: d={d:.2f} m, scale={scale:.2f}'
            )
        else:
            # slow_distance 밖 → 정상 주행
            aeb_scale.data = 1.0

        self.aeb_pub.publish(aeb_scale)


def main(args=None):
    rclpy.init(args=args)
    node = ScoutMiniAEB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
