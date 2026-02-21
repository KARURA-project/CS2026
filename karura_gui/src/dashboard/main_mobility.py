import sys
from PySide6.QtWidgets import QApplication, QMessageBox
import traceback
from PySide6.QtCore import QTimer
from karura_gui.mobility import MobilityMainWindow, MobilityBridge
# raise RuntimeError("SENTINEL: dashboard.main-mobility is running")
from pathlib import Path

def main():
    app = QApplication(sys.argv)
    qss_path = Path(__file__).resolve().parent / "core" / "karura_dark.qss"
    app.setStyleSheet(qss_path.read_text(encoding="utf-8"))
    print(f"[STYLE] Loaded QSS: {qss_path}", file=sys.stderr, flush=True)
    
    bridge = None
    window = None
    print("[MAIN] hello", file=sys.stderr, flush=True)

    try:
        print("[Main] Starting main_mobility.py")
        print("[Main] Creating mobility bridge")
        bridge = MobilityBridge()
        print("[Main] Mobility bridge created")
        print("[Main] starting mobility bridge")
        bridge.start()
        print("[Main] Mobility bridge started")

        window = MobilityMainWindow()
        window.bridge = bridge
        window.connect_signals(bridge)

        #Connects signals to UI to display ROS2 data
        if hasattr(window, "battery_status"):
            bridge.battery_data_signal.connect(
                lambda msg: window.battery_status.set_values(remaining=msg.data)
            )
        if hasattr(window, "actual_rads_signal"):
            bridge.actual_rads_signal.connect(
                lambda msg: window.actual_rads_signal.set_values(remaining=msg.data)
            )
        if hasattr(window, "roll_pitch_yaw_signal"):
            bridge.roll_pitch_yaw_signal.connect(
                lambda msg: window.roll_pitch_yaw_signal.set_values(remaining=msg.data)
            )

        window.show()
        
        def start_camera_if_present():
            if window is None:
                return

            candidates = [
                window,
                getattr(window, "central", None),
                getattr(getattr(window, "central", None), "ui", None),
            ]

            # 1) First: try known attribute names
            attr_names = (
                "camera_widget", "video_widget", "cameraView", "videoView",
                "maincameravideo", "mainCameraVideo",
            )

            for obj in candidates:
                if obj is None:
                    continue
                for attr in attr_names:
                    if hasattr(obj, attr):
                        w = getattr(obj, attr)
                        if hasattr(w, "start_camera"):
                            #print("Camera on pause for now. Change in main_mobility.py")
                            print(f"[Main] Starting camera via {obj.__class__.__name__}.{attr}",
                                file=sys.stderr, flush=True)
                            w.start_camera()
                            return

            # 2) Fallback: scan attributes for anything with start_camera()
            for obj in candidates:
                if obj is None:
                    continue
                for name in dir(obj):
                    try:
                        w = getattr(obj, name)
                    except Exception:
                        continue
                    if hasattr(w, "start_camera"):
                        print(f"[Main] Starting camera via discovered {obj.__class__.__name__}.{name}",
                            file=sys.stderr, flush=True)
                        w.start_camera()
                        return

            print("[Main] No camera widget found to start.", file=sys.stderr, flush=True)


        # Schedule camera start after the event loop starts and widgets are realized
        #print("[Main] Camera startup disabled for debugging.")
        QTimer.singleShot(0, start_camera_if_present)

        #Handles starting and stopping timer
        # def connect_timer_logic():
        #     window.ui.timerbuttonpanel.ui.startButton.clicked.connect(window.ui.timerprogressbar.start)    
        #     window.ui.timerbuttonpanel.ui.stopButton.clicked.connect(window.ui.timerprogressbar.stop)

        # connect_timer_logic()

        def on_exit():
            print("[Main] Shutting down mobility bridge")
            try:
                if window is not None:
                    for attr in ("camera_widget", "video_widget", "cameraView", "videoView"):
                        if hasattr(window, attr):
                            w = getattr(window, attr)
                            if hasattr(w, "stop_camera"):
                                w.stop_camera()
                if bridge is not None:
                    bridge.shutdown()
            except Exception:
                traceback.print_exc(file=sys.stderr)
        
        app.aboutToQuit.connect(on_exit)

        sys.exit(app.exec())
    
    except Exception as e:
        tb = traceback.format_exc
        error_msg = f"Failed to start Mobility Dashboard: {e}\n\n{tb}"
        print(error_msg, file=sys.stderr)
        
        # Show error dialog
        QMessageBox.critical(None, "Mobility Dashboard Error", error_msg)

        try:
            if bridge is not None:
                bridge.shutdown()
        except Exception:
            pass
        sys.exit(1)


if __name__ == "__main__":
    main()