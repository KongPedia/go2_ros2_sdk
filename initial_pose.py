import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from functools import partial

class MultiRobotAutoInit(Node):
    def __init__(self, num_envs):
        super().__init__('multi_robot_auto_init')
        self.num_envs = num_envs
        self.completed_robots = set() # 완료된 로봇 ID 저장

        self.subs = []
        self.pubs = []

        qos = QoSProfile(depth=10)

        self.get_logger().info(f'Starting Auto Initial Pose for {num_envs} robots...')

        for i in range(num_envs):
            # 1. Publisher 생성 (/robot{i}/initialpose)
            pub = self.create_publisher(
                PoseWithCovarianceStamped, 
                f'/robot{i}/initialpose', 
                qos
            )
            self.pubs.append(pub)

            # 2. Subscriber 생성 (/robot{i}/odom)
            # partial을 사용하여 콜백 함수에 robot_id(i)를 전달
            sub = self.create_subscription(
                Odometry,
                f'/robot{i}/odom',
                partial(self.odom_callback, robot_id=i),
                qos
            )
            self.subs.append(sub)

    def odom_callback(self, msg, robot_id):
        # 이미 처리된 로봇이면 패스
        if robot_id in self.completed_robots:
            return

        self.get_logger().info(f'[Robot {robot_id}] Odom received. Publishing Initial Pose...')

        # 3. 메시지 변환 (Odom -> PoseWithCovarianceStamped)
        init_msg = PoseWithCovarianceStamped()
        
        # [중요] AMCL은 map 프레임 기준이어야 함. 
        # Odom 메시지의 header.frame_id는 보통 'odom'이지만, 초기화할 땐 'map'이라 우겨야 함.
        init_msg.header = msg.header
        init_msg.header.frame_id = 'map' 
        init_msg.header.stamp = self.get_clock().now().to_msg()

        # 위치 및 방향 복사
        init_msg.pose.pose = msg.pose.pose

        # [옵션] Covariance(공분산) 재설정
        # Odom에서 오는 공분산이 너무 작으면(0이면) AMCL 입자가 안 퍼질 수 있음.
        # 초기화 시에는 약간 넓게 퍼뜨려주는 것이 안전하므로 수동 설정 권장.
        # (필요 없다면 msg.pose.covariance 그대로 써도 됨)
        init_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.068
        ]

        # 4. 발행
        self.pubs[robot_id].publish(init_msg)
        
        # 완료 목록에 추가
        self.completed_robots.add(robot_id)

        # 5. 모든 로봇이 완료되었는지 체크
        if len(self.completed_robots) == self.num_envs:
            self.get_logger().info('All robots initialized! Shutting down...')
            # 잠시 대기 후 종료 (메시지 전송 보장)
            import time
            time.sleep(1.0)
            raise SystemExit # 예외를 발생시켜 spin을 멈춤

def main():
    # 인자 파싱
    if len(sys.argv) < 2:
        print("Usage: python3 auto_init_pose.py <num_envs>")
        print("Example: python3 auto_init_pose.py 5")
        return

    num_envs = int(sys.argv[1])

    rclpy.init()
    node = MultiRobotAutoInit(num_envs)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
