"""
cmd_vel → 파일 브리지 (시스템 py3.10 + ROS2).
Isaac(py3.11)이 og/rclpy로 cmd_vel을 못 읽는 문제 우회용.
/cmd_vel 구독 → /tmp/cmd_vel.txt 에 "v w" 기록 (Isaac이 매 프레임 읽어 kinematic 이동).

실행: source /opt/ros/humble/setup.bash && python3 cmdvel_to_file.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PATH = "/tmp/cmd_vel.txt"


class Bridge(Node):
    def __init__(self):
        super().__init__("cmdvel_to_file")
        self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
        open(PATH, "w").write("0.0 0.0\n")
        self.get_logger().info(f"cmd_vel → {PATH} 브리지 시작")

    def cb(self, msg: Twist):
        with open(PATH, "w") as f:
            f.write(f"{msg.linear.x} {msg.angular.z}\n")


def main():
    rclpy.init()
    n = Bridge()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
