import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 설정: 이동 속도 및 회전 속도
LINEAR_SPEED = 0.5  # m/s
ANGULAR_SPEED = 1.0 # rad/s

class Go2KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('go2_keyboard_teleop')
        
        # TwistMux의 Priority 10 (joy) 슬롯으로 발행
        # go2_ros2_sdk/config/twist_mux.yaml 설정을 따름
        self.pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.print_usage()
        
        # 타이머 기반으로 주기적 발행 (키를 누르고 있을 때 부드러운 이동을 위해)
        self.timer = self.create_timer(0.1, self.run_loop)
        self.target_twist = Twist()

    def print_usage(self):
        print(f"""
        =============================================
        🐕 Go2 Keyboard Teleop (Infrastructure) 🐕
        =============================================
        
        이동 제어 (Holonomic Control):
        -------------------------
           Q    W    E
           (↶)  (↑)  (↷)
           
           A    S    D
           (←)  (↓)  (→)
        -------------------------
        W / S : 전진 / 후진 (Linear X)
        A / D : 좌  / 우  이동 (Linear Y - Strafing)
        Q / E : 좌  / 우  회전 (Angular Z - Yaw)
        
        Space : 🛑 정지 (Stop)
        CTRL-C: 종료
    
        현재 속도 설정:
        Linear : {LINEAR_SPEED} m/s
        Angular: {ANGULAR_SPEED} rad/s
        =============================================
        """)

    def get_key(self):
        """Non-blocking 키 입력 감지"""
        tty.setraw(sys.stdin.fileno())
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_loop(self):
        try:
            key = self.get_key()
            
            # 키 입력이 없으면 기존 속도 유지 혹은 감속 로직을 넣을 수 있음
            # 여기서는 안전을 위해 키를 뗄 때 멈추게 하거나, 
            # 계속 누르고 있어야 가는 방식 중 '누를 때만 갱신' 방식을 사용합니다.
            
            twist = Twist()
            publish = False

            if key == 'w':
                twist.linear.x = LINEAR_SPEED
                publish = True
            elif key == 's':
                twist.linear.x = -LINEAR_SPEED
                publish = True
            elif key == 'a':
                twist.linear.y = LINEAR_SPEED
                publish = True
            elif key == 'd':
                twist.linear.y = -LINEAR_SPEED
                publish = True
            elif key == 'q':
                twist.angular.z = ANGULAR_SPEED
                publish = True
            elif key == 'e':
                twist.angular.z = -ANGULAR_SPEED
                publish = True
            elif key == ' ':
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                publish = True
            elif key == '\x03': # CTRL-C
                rclpy.shutdown()
                return

            if publish:
                self.target_twist = twist
                self.pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Go2KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 정지 명령 전송
        stop_twist = Twist()
        node.pub.publish(stop_twist)
        # 터미널 설정 복구
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()