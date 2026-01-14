import sys
from PySide6.QtWidgets import QApplication, QMessageBox

from karura_gui.mobility import MobilityMainWindow, MobilityBridge


def main():
    app = QApplication(sys.argv)
    
    try:
        bridge = MobilityBridge()
        bridge.start()

        window = MobilityMainWindow()
        window.connect_signals(bridge)
        window.show()
        
        def on_exit():
            bridge.shutdown()
        
        app.aboutToQuit.connect(on_exit)

        sys.exit(app.exec())
    
    except Exception as e:
        error_msg = f"Failed to start Mobility Dashboard: {str(e)}"
        print(error_msg, file=sys.stderr)
        
        # Show error dialog
        QMessageBox.critical(None, "Mobility Dashboard Error", error_msg)
        sys.exit(1)


if __name__ == "__main__":
    main()