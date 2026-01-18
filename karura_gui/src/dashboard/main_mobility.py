import sys
from PySide6.QtWidgets import QApplication, QMessageBox
import traceback
from PySide6.QtCore import QTimer
from karura_gui.mobility import MobilityMainWindow, MobilityBridge
# raise RuntimeError("SENTINEL: dashboard.main-mobility is running")


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
        QTimer.singleShot(0, start_camera_if_present)

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