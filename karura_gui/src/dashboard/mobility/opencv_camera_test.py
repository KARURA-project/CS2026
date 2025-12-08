import cv2
import time

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # 0 usually refers to the first camera


print("A")
if not cap.isOpened():
    print("Error: Could not open camera.")
else:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))


    print("B")
    while True:
        ret, frame = cap.read()
        print(ret, frame)
        print("C")
        if not ret:
            print("Error: Could not read frame.")
            break
        print("HI")
        cv2.imshow('Camera Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(.100)

    cap.release()
    cv2.destroyAllWindows()