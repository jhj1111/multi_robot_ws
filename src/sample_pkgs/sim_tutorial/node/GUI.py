import sys
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from PySide2.QtCore import *
from PySide2.QtWidgets import *

class NODE(QThread, Node):
    message_received = Signal(str)

    def __init__(self, node_name='gui_node'):
        QThread.__init__(self)
        Node.__init__(self, node_name)

        self.subscription = self.create_subscription(
            String, 'current_pose', self.subscription_callback, 10)  # 현재 위치 구독
        self.publisher = self.create_publisher(String, 'robot_commands', 10)  # 로봇 명령 퍼블리셔 생성

    def subscription_callback(self, msg):
        message = msg.data
        self.get_logger().info(f'Received message: {message}')
        self.message_received.emit(message)

    def publish_command(self, command):
        """ 명령을 퍼블리시하는 함수 """
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {command}')


class GUI(QMainWindow):
    def __init__(self, ros_thread):
        super().__init__()
        self.ros_thread = ros_thread
        self.ros_thread.message_received.connect(self.add_message)
        self.setupUi()

    def setupUi(self):
        self.window = QMainWindow()
        if not self.window.objectName():
            self.window.setObjectName(u"MainWindow")

        self.window.setObjectName("MainWindow")
        self.window.resize(375, 350)
        
        self.centralwidget = QWidget(self.window)
        self.textBrowser = QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QRect(60, 40, 256, 192))
        
        self.patrol_button = QPushButton("순찰 시작", self.centralwidget)
        self.patrol_button.setGeometry(QRect(60, 250, 120, 30))
        self.patrol_button.clicked.connect(lambda: self.ros_thread.publish_command("robot_start"))
        
        self.rescue_button = QPushButton("구조 시작", self.centralwidget)
        self.rescue_button.setGeometry(QRect(200, 250, 120, 30))
        self.rescue_button.clicked.connect(lambda: self.ros_thread.publish_command("drone_start"))

        self.patrol_stop_button = QPushButton("순찰 중지", self.centralwidget)
        self.patrol_stop_button.setGeometry(QRect(60, 300, 120, 30))
        self.patrol_stop_button.clicked.connect(lambda: self.ros_thread.publish_command("robot_stop"))
        
        self.rescue_stop_button = QPushButton("구조 중지", self.centralwidget)
        self.rescue_stop_button.setGeometry(QRect(200, 300, 120, 30))
        self.rescue_stop_button.clicked.connect(lambda: self.ros_thread.publish_command("drone_stop"))
        
        self.window.setCentralWidget(self.centralwidget)

    def add_message(self, message):
        self.textBrowser.append(f"Current Pose: {message}")


def main():
    rclpy.init()
    node = NODE()
    ros_thread = threading.Thread(target=lambda : rclpy.spin(node), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = GUI(node)
    gui.window.show()

    try:
        sys.exit(app.exec_())

    except KeyboardInterrupt:
        sys.exit(0)

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
