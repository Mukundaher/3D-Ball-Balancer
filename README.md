# 3D-Ball-Balancer

# Ball Balancing STEWART Platform
![alt text](https://github.com/Mukundaher/3D-Ball-Balancer/blob/main/Ball_Balancer.png)

---

## 📌 Overview 

  ball-balancing **3-RRS (Revolute-Revolute-Spherical)** Platform. It can dynamically balance a ball in real time and perform motion tasks such as moving in straight lines, quadrants, and even circles. The robot uses real-time computer vision, inverse kinematics, and PID control for its operation.

---

## 🔧 Tools & Concepts Used

- Python
- SolidWorks
- 3d Printing
- OpenCV (Contour Detection, HSV Tuning)
- Inverse Kinematics (for 3-RRS)
- PID Control
- Arduino Board

---

## 🎥 Video of robot balancing the ball
![Video]([https://github.com/Mukundaher/3D-Ball-Balancer/blob/main/Demo.mp4])

---


## 🧰 List of Components 

| S.No | Component | Quantity | Purpose | 
|------|-----------|----------|---------|
| 1. | **Arduino R4 Minima** | 1 | Microcontroller |
| 2. | **Power Supply Or 5V batteries** | 1 | Servo Control | 
| 3. | **5MP Webcam Camera ** | 1 | Vision (ball detection) | 
| 4. | **Usb Cable** | 1 | Connects Laptop to Arduino | 
| 5. | **RKI1206 Servo Motors** | 3 | Actuation of platform | 
| 6. | **Ball Joint (M6)** | 3 | Platform joints (RRS design) | 
| 7. |**Ball Bearings (ID - 6mm, OD - 16mm)** | 3 | For joint rotation |

---

## 🧮 Inverse Kinematics [(Snippets)]([https://github.com/Mukundaher/3D-Ball-Balancer/blob/main/Inverse_kinematics.pdf])

We derived and implemented the IK for the 3-RRS platform to calculate the exact servo angles required to achieve desired platform tilt and height. This allowed precise control over the ball's motion.

---

## 👁️ Ball Detection with OpenCV

- HSV Trackbar for color tuning
- Faster response than BGR and no noise Like Grayscale ,for robust tracking 
- `cv2.findContours` and `cv2.minEnclosingCircle` to detect the ball
- Real-time position and area returned for control logic

---

## 🎛️ PID Tuning

after manual tuning and testing, a finely tuned PID controller was implemented, keeping the ball at centre.

---

## 📐 CAD Files [(Link)](https://github.com/Mukundaher/3D-Ball-Balancer/blob/main/STL_Files.zip)

All the 3D models used in this project—including the base, platform, servo holders, and linkages—were custom-designed using Fusion360. You can find the .stl files in the  STLs directory. These files are ready for 3d printing.

---


