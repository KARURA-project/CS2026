import sys
from PySide6.QtWidgets import QApplication, QMessageBox
import traceback
from karura_gui.mobility import MobilityMainWindow, MobilityBridge
raise RuntimeError("SENTINEL: dashboard.main-mobility is running")


def main():
    app = QApplication(sys.argv)
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
        window.connect_signals(bridge)
        window.show()
        
        def start_camera_if_present():
            for attr in ("camera_widget", "video_widget", "cameraView", "videoView"):
                if hasattr(window, attr):
                    w = getattr(window, attr)
                    if hasattr(w, "start_camera"):
                        w.start_camera()
                        return

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