import time
import cv2
from class_Camera import Camera
from class_pid import PID
from class_bbrobot import BBrobot
from class_audrino import SPort

def main():
    # Configuration parameters
    K_PID = [.024,.0031,.013]
    # K_PID = [.024,.0031,.012]

    # K_PID = [.024,.0025,.012]
    # K_PID = [.0255,.003,.0125]
    # K_PID = [.026,.003,.013]
    # K_PID = [.025,.003,.012]
  
    k = 1                     # Scaling factor for phi
    alpha = 1                 # Low-pass filter coefficient
    servo_ids = [1, 2, 3]       # Servo IDs
    Pz = 0.141               # Platform height (meters)
    goal = (0,0)               # Target position (center)
    control_interval = .02 #sholud be 0.02     

    # Initialize components
    robot = BBrobot(servo_ids)
    camera = Camera()
    pid = PID(K_PID, k, alpha)
    port=SPort()

    try:
        # Move to initial position
        # robot.Initialize_posture()
        port.send_angles([90,90,90])
        print("Robot initialized. Starting balance control...")
        time.sleep(.02)
        
        while True:
            start_time = time.perf_counter()
            
            # Capture and process frame
            frame = camera.take_pic()
            t1=time.time()
            if frame is None:
                continue
                
            # Detect ball position
            x, y, area, center, processed_frame = camera.find_ball(frame)
            
            # Display tracking visualization
            cv2.imshow("Ball Tracking", processed_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            
            if center is not None:
                
                theta, phi = pid.compute(goal, (x, y))
                
               
                platform_pose = [theta, phi, Pz]
                
                
                robot.control_t_posture(platform_pose, control_interval)
                t2=time.time()
                print(t2-t1)
                # print(f"Ball: ({x}, {y}) | Platform: θ={theta:.1f}°, φ={phi:.1f}°")
    
    except KeyboardInterrupt:
        print("Control interrupted")
    
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        # robot.Initialize_posture()
        port.send_angles([90,90,90])
        # sport.close()  
        print("Robot shutdown complete")

if __name__ == "__main__":
    main()

