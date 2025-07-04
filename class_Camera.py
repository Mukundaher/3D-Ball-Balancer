from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import serial

class Camera:
    def __init__(self, video_path=None, buffer_size=64):
       
        self.video_path = video_path
        self.buffer_size = buffer_size
       

# For magenta pinks (160-180° in 0-360 scale)
        # self.lower_pink2 = np.array([168, 102, 141])  # with red in back ground
        # # self.upper_pink2 = np.array([179, 255, 255])

        ## HSV range for specific pink ball detection also it ignores noise of orange and red very well
        self.lower_pink2 = np.array([116, 92, 168])  # without red in back ground
        self.upper_pink2 = np.array([179, 255, 255])
        self.pts = deque(maxlen=self.buffer_size)
        
        # Initializing  video stream
        if not self.video_path:
            self.vs = VideoStream(src=0).start()
        else:
            self.vs = cv2.VideoCapture(self.video_path)
        time.sleep(2.0) 

    def find_ball(self, frame):
        #processing to get center of ball
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_pink2, self.upper_pink2)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        if len(cnts) > 0:
            #it gets max contour with area greater that 100 ,depending of specific ball
            cnts = [c for c in cnts if cv2.contourArea(c) > 100]
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                
                try:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                except:
                    center = (int(x), int(y))
                
                if radius > 10:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    
                    h, w = frame.shape[:2]
                    #brings camera coordinates to real world position according to co ordinates of system
                    x -= w / 2
                    y -= h / 2
                    error_x = -(0 - x)
                    error_y = -(0 - y)
                    area = cv2.contourArea(c)
                    return int(x), int(y), int(error_x**2+error_y**2), center, frame

        return -1, -1, 0, None, frame
    
    def take_pic(self):
        #this reads live feed from camera
      frame = self.vs.read() if not self.video_path else self.vs.read()[1]
      return frame.copy() if frame is not None else None


    def run(self):
        #for simulation purposes shows image processing
        while True:
            frame = self.vs.read() if not self.video_path else self.vs.read()[1]
            if frame is None:
                break

            x, y, e , center, processed_frame = self.find_ball(frame)
            self.pts.appendleft(center)

            # Draw the trail
            for i in range(1, len(self.pts)):
                if self.pts[i - 1] is None or self.pts[i] is None:
                    continue
                thickness = int(np.sqrt(self.buffer_size / float(i + 1)) * 2.5)
                cv2.line(processed_frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

            cv2.imshow("Pink Ball Tracking", processed_frame)
            if center:
                print(f"error: {e}")
                print(f"x,y: {x,y}")
            else:
                print("Ball not found")
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.vs.stop() if not self.video_path else self.vs.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="path to the video file")
    ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
    args = vars(ap.parse_args())

    tracker = Camera(video_path=args.get("video"), buffer_size=args["buffer"])
    tracker.run()
