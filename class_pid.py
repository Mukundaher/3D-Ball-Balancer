import math
import time
    
class PID:
    def __init__(self, K_PID, k, alpha):
        self.set_gains(K_PID)  
        self.k = k
        self.alpha = alpha
        self.reset_state()

    def set_gains(self, K_PID):
        self.kp = K_PID[0]
        self.ki = K_PID[1]
        self.kd = K_PID[2]
        
    def reset_state(self):
        self.last_output_x = 0
        self.last_output_y = 0
        self.last_error_x = 0
        self.integral_x = 0
        self.last_error_y = 0
        self.integral_y = 0
        self.last_time = None

    def compute(self, Goal, Current_value):
        current_time = time.perf_counter()
        if self.last_time is None:
             self.last_time = current_time
             return 0, 0    
        dt = current_time - self.last_time
        if dt <= 0:
            return 100,100

        error_x = (Goal[0] - Current_value[0])
        error_y = (Goal[1] - Current_value[1])
        #ki set to zero after ball is within specified circle
        if(error_x**2+error_y**2<=25):
            self.ki=0
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        
        derivative_x = (error_x - self.last_error_x) / dt
        derivative_y = (error_y - self.last_error_y) / dt
        #PID Controller
        output_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        output_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y
        #this was low pass filter designed but it wasn't needed 
        output_x = self.alpha * output_x + (1 - self.alpha) * self.last_output_x
        output_y = self.alpha * output_y + (1 - self.alpha) * self.last_output_y
        
        theta = math.degrees(math.atan2(output_y, output_x))
        if theta < 0:
            theta += 360
        phi = self.k * math.sqrt(output_x**2 + output_y**2)

        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_output_x = output_x
        self.last_output_y = output_y
        self.last_time = current_time
        #for simulation prints error and calculated phi and theta
        print(f"errors: ={(error_x**2+error_y**2):.2f}")
        # print(f"outputs: ox={output_x:.5f}, oy={output_y:.5f}")
        print(f"phi={phi:.5f}, theta={theta:.2f}")
        return theta, phi
