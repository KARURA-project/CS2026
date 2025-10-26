import subprocess

def main():
    for i in range(5):
        subprocess.Popen(["ros2", "run", "camera_gui", "image_publisher", str(i)])
    print("Launched 5 camera publishers.")
    input("Press Enter to stop all publishers...")

if __name__ == "__main__":
    main()
