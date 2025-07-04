import cv2
#test code to get camera port of webcam
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap is not None and cap.isOpened():
        print(f"Webcam found at index {i}")
        cap.release()
