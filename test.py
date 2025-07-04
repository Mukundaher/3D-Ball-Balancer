from class_pid import PID
from class_bbrobot import BBrobot
from class_audrino import SPort
import time
import math
# another test program to get platform orientation by specific ball co-ordinates  and goal

# Initialize PID, Robot

K_PID = [1, 0.0, 0.0] 
k = 1                              
alpha = 1                           
pid = PID(K_PID, k, alpha)
robot = BBrobot(ids=[1, 2, 3])


goal_position = (0, 0)       # Target is center
current_position = (100, 0)    

# Call PID controller
theta, phi = pid.compute(goal_position, current_position)

# Print result
# print(f"[PID Output] theta = {theta:.2f}°, phi = {phi:.2f}°")
# error_cases = [
#     [0, 0, 0.141],     # Flat
#     [90, 10, 0.141],   # Tilt right
#     [180, 15, 0.141],  # Tilt backward
#     [270, 5, 0.141],   # Tilt left
# ]

posi_vectors = [
     [100,0],[0,100],[-100,0],[0,-100],[80,80]
  ]

for pos_vector in posi_vectors:
    vector = [0,0]
    vector[0]=0-pos_vector[0]
    vector[1]=0-pos_vector[1]
    theta = math.degrees(math.atan2(vector[1], vector[0]))
    if theta < 0:
        theta += 360
    phi = k * math.sqrt(vector[0]**2 + vector[1]**2)

    pose=[theta,phi,.141]
    robot.control_t_posture(pose, t=0.02)
    input("Press Enter to continue...")

    


