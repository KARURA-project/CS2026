import sys
import threading
import signal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PySide6.QtWidgets import QApplication, QWidget, QLabel
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QTimer, QFile, QCoreApplication, QThread, Signal
from PySide6.QtWebEngineWidgets import QWebEngineView


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_gui_subscriber')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.latest_twist = Twist()

    def listener_callback(self, msg):
        self.latest_twist = msg


class MainWindow(QWidget):
    def __init__(self, node, stop_event):
        super().__init__()
        self.node = node
        self.stop_event = stop_event
        loader = QUiLoader()
        ui_file = QFile("resource/simple_mobility_gui.ui")
        ui_file.open(QFile.ReadOnly)
        self.ui = loader.load(ui_file, self)
        ui_file.close()

        self.webView: QWebEngineView = self.ui.findChild(QWebEngineView, "webView")
        if self.webView is None:
            raise RuntimeError("webView (QWebEngineView) not found in UI. Make sure UI contains a widget named 'webView' of type QWebEngineView.")

        self.label_linear_x = self.ui.findChild(QLabel, "label_lin_x_value")
        self.label_linear_y = self.ui.findChild(QLabel, "label_lin_y_value")
        self.label_angular_z = self.ui.findChild(QLabel, "label_ang_z_value")

        self.webView.setUrl("http://192.168.137.21:8080/html/p2p.html")

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(100)

    def update_labels(self):
        twist = self.node.latest_twist
        self.label_linear_x.setText(f"linear.x: {twist.linear.x:.2f}")
        self.label_linear_y.setText(f"linear.y: {twist.linear.y:.2f}")
        self.label_angular_z.setText(f"angular.z: {twist.angular.z:.2f}")

    def closeEvent(self, event):
        print("GUI closed — stopping ROS...")
        self.stop_event.set()
        QCoreApplication.quit()
        event.accept()


def ros_spin(node, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def run_qt(stop_event, node):
    """Qtアプリを別スレッドで動かす"""
    app = QApplication(sys.argv)
    window = MainWindow(node, stop_event)
    window.ui.show()
    app.exec()
    stop_event.set()
    print("Qt thread finished")


def main():
    rclpy.init()
    node = CmdVelSubscriber()
    stop_event = threading.Event()

    # Qtを別スレッドで起動
    qt_thread = threading.Thread(target=run_qt, args=(stop_event, node), daemon=True)
    qt_thread.start()

    # ROSスレッド（メインスレッド）で動かす
    try:
        while not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received — shutting down...")
        stop_event.set()
    finally:
        stop_event.set()
        qt_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        app.quit()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
