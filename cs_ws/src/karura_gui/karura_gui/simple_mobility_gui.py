import sys
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PySide6.QtWidgets import QApplication, QWidget, QLabel
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QTimer, QFile
from PySide6.QtWebEngineWidgets import QWebEngineView


class CmdVelSubscriber(Node):
    """ROS2 Twistメッセージを購読して保持するノード"""
    def __init__(self):
        super().__init__('cmd_vel_gui_subscriber')
        self.sub = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10
        )
        self.latest_twist = Twist()

    def listener_callback(self, msg):
        self.latest_twist = msg


class MainWindow(QWidget):
    """PySide6で作るGUIクラス"""
    def __init__(self, node: CmdVelSubscriber, stop_event):
        super().__init__()
        self.node = node
        self.stop_event = stop_event

        # Qt Designerで作ったUIを読み込み
        loader = QUiLoader()
        ui_file = QFile("resource/simple_mobility_gui.ui")
        ui_file.open(QFile.ReadOnly)
        self.ui = loader.load(ui_file, self)
        ui_file.close()

        # WebView設定
        self.webView: QWebEngineView = self.ui.findChild(QWebEngineView, "webView")
        self.webView.setUrl("http://192.168.137.20:8080/html/p2p.html")

        # ラベル取得
        self.label_linear_x: QLabel = self.ui.findChild(QLabel, "label_lin_x_value")
        self.label_linear_y: QLabel = self.ui.findChild(QLabel, "label_lin_y_value")
        self.label_angular_z: QLabel = self.ui.findChild(QLabel, "label_ang_z_value")

        # タイマーでUI更新
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(100)  # 100msごとに更新

    def update_labels(self):
        twist = self.node.latest_twist
        self.label_linear_x.setText(f"linear.x: {twist.linear.x:.2f}")
        self.label_linear_y.setText(f"linear.y: {twist.linear.y:.2f}")
        self.label_angular_z.setText(f"angular.z: {twist.angular.z:.2f}")

    def closeEvent(self, event):
        """ウィンドウが閉じられるときに呼ばれる"""
        print("GUI closed — stopping ROS...")
        self.stop_event.set()  # ROSスレッド停止を通知
        QCoreApplication.quit()
        event.accept()


def ros_spin(node, stop_event):
    """ROSスレッドを停止イベント付きで動作"""
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main():
    rclpy.init()
    node = CmdVelSubscriber()

    stop_event = threading.Event()

    app = QApplication(sys.argv)
    window = MainWindow(node, stop_event)
    window.ui.show()

    # ROS2を別スレッドで動かす
    thread = threading.Thread(target=ros_spin, args=(node, stop_event), daemon=True)
    thread.start()

    try:
        sys.exit(app.exec())
    finally:
        # Ctrl+C や閉じるボタンでも確実に終了
        print("Shutting down...")
        stop_event.set()
        thread.join(timeout=2.0)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
