import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool 
from sensor_msgs.msg import LaserScan
from math import cos, sin

class ScoutMiniEStop(Node):

    def __init__(self):
        super().__init__('scout_mini_e_stop')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Bool, '/scout_mini/e_stop', 10)

        self.lidar_flag = False

        # 스카우트 미니용 감지 박스 (임의 값, 추후 튜닝)
        self.front_min_x = 0.2   # 20cm 이상 앞
        self.front_max_x = 0.8   # 80cm 이내
        self.side_y      = 0.25  # 좌우 25cm
        
    def laser_callback(self, msg: LaserScan):
        estop = Bool()
        estop.data = False

        if not self.lidar_flag:
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment
            self.lidar_flag = True

        for i, data in enumerate(msg.ranges):
            current_angle = self.angle_min + self.angle_increment*i

            # laser 프레임 기준 좌표
            lx = data * cos(current_angle)
            ly = data * sin(current_angle)

            # base_link 프레임 기준으로 180도 회전 보정
            bx = -lx
            by = -ly

            if (self.front_min_x < bx < self.front_max_x and
                -self.side_y < by < self.side_y):
                estop.data = True
                break
            #cx = data * cos(current_angle)
            #cy = data * sin(current_angle)
            #if (self.front_min_x < cx < self.front_max_x and
            #    -self.side_y < cy < self.side_y):
            #    estop.data = True
            #    break

        self.publisher_.publish(estop)

def main(args=None):
    rclpy.init(args=args)

    scout_mini_e_stop = ScoutMiniEStop()

    rclpy.spin(scout_mini_e_stop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scout_mini_e_stop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()