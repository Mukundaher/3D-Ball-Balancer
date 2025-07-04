

from class_audrino import SPort
import math
import time

class BBrobot:
    
    def __init__(self, ids):

        # physical constraing defined specific to system
        self.ids  = ids
        self.port = SPort()
        self.L = [0.058, 0.071, 0.122, 0.130]
        self.ini_pos = [0, 0.0, 0.141]
        self.pz_max  = 0.150
        self.pz_min  = 0.075 #max & min height for edge Cases
        self.phi_max = 20.0 # max platform tilt defined 

    def safe_sqrt(self, x, label=""):
        if x < 0:
            print(f"[WARNING] sqrt domain error on {label}: {x:.6f}. Resetting posture.")
            self.Initialize_posture()
            raise ValueError(f"sqrt domain error on {label}")
        return math.sqrt(x)
    
    def kinema_inv(self, n, Pz):
        L = self.L
        #inverse kinematics as derived
        #gives motor angles from normal vector from plane
        try:
            A = (L[0]+L[1])/Pz
            B = (Pz**2+L[2]**2-(L[0]+L[1])**2-L[3]**2)/(2*Pz)
            C = A**2+1
            D = 2*(A*B-(L[0]+L[1]))
            E = B**2+(L[0]+L[1])**2-L[2]**2
            Pmx = (-D + self.safe_sqrt(D**2 - 4*C*E, "Pmx discriminant")) / (2*C)
            Pmz = self.safe_sqrt(L[2]**2 - Pmx**2 + 2*(L[0]+L[1])*Pmx - (L[0]+L[1])**2, "Pmz")
        
            den_a = self.safe_sqrt(n[0]**2 + n[2]**2, "denominator a")
            a_m_x = (L[3]/den_a)*(n[2])
            a_m_y = 0
            a_m_z = Pz + (L[3]/den_a)*(-n[0])
            A_m = [a_m_x, a_m_y, a_m_z]

            A = (L[0]-A_m[0])/A_m[2]
            B = (A_m[0]**2+A_m[1]**2+A_m[2]**2 - L[2]**2 - L[0]**2 + L[1]**2)/(2*A_m[2])
            C = A**2 + 1
            D = 2*(A*B - L[0])
            E = B**2 + L[0]**2 - L[1]**2
            ax = (-D + self.safe_sqrt(D**2 - 4*C*E, "a axis discriminant")) / (2*C)
            az = self.safe_sqrt(L[1]**2 - ax**2 + 2*L[0]*ax - L[0]**2, "az")
            if (a_m_z < Pmz): az = -az
            A_2 = [ax, 0, az]
            theta_a = 90 - math.degrees(math.atan2(A_2[0]-L[0], A_2[2]))

            # Joint B
            den_b = self.safe_sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*math.sqrt(3)*n[0]*n[1], "denominator b")
            b_m_x = (L[3]/den_b)*(-n[2])
            b_m_y = (L[3]/den_b)*(-math.sqrt(3)*n[2])
            b_m_z = Pz + (L[3]/den_b)*(math.sqrt(3)*n[1]+n[0])
            B_m = [b_m_x, b_m_y, b_m_z]

            A = -(B_m[0]+math.sqrt(3)*B_m[1]+2*L[0])/B_m[2]
            B = (B_m[0]**2 + B_m[1]**2 + B_m[2]**2 + L[1]**2 - L[2]**2 - L[0]**2)/(2*B_m[2])
            C = A**2 + 4
            D = 2*A*B + 4*L[0]
            E = B**2 + L[0]**2 - L[1]**2
            x = (-D - self.safe_sqrt(D**2 - 4*C*E, "b axis discriminant")) / (2*C)
            y = math.sqrt(3)*x
            z = self.safe_sqrt(L[1]**2 - 4*x**2 - 4*L[0]*x - L[0]**2, "bz")
            if (b_m_z < Pmz): z = -z
            B_2 = [x, y, z]
            theta_b = 90 - math.degrees(math.atan2(math.sqrt(B_2[0]**2 + B_2[1]**2) - L[0], B_2[2]))

            # Joint C
            den_c = self.safe_sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*math.sqrt(3)*n[0]*n[1], "denominator c")
            c_m_x = (L[3]/den_c)*(-n[2])
            c_m_y = (L[3]/den_c)*(math.sqrt(3)*n[2])
            c_m_z = Pz + (L[3]/den_c)*(-math.sqrt(3)*n[1]+n[0])
            C_m = [c_m_x, c_m_y, c_m_z]

            A = -(C_m[0]-math.sqrt(3)*C_m[1]+2*L[0])/C_m[2]
            B = (C_m[0]**2 + C_m[1]**2 + C_m[2]**2 + L[1]**2 - L[2]**2 - L[0]**2)/(2*C_m[2])
            C = A**2 + 4
            D = 2*A*B + 4*L[0]
            E = B**2 + L[0]**2 - L[1]**2
            x = (-D - self.safe_sqrt(D**2 - 4*C*E, "c axis discriminant")) / (2*C)
            y = -math.sqrt(3)*x
            z = self.safe_sqrt(L[1]**2 - 4*x**2 - 4*L[0]*x - L[0]**2, "cz")
            if (c_m_z < Pmz): z = -z
            C_2 = [x, y, z]
            theta_c = 90 - math.degrees(math.atan2(math.sqrt(C_2[0]**2 + C_2[1]**2) - L[0], C_2[2]))
            #this angle vectors changes according to resting position of motor to get perfectly flat platform
            return [106.6-theta_a,105.6- theta_b,113.6-theta_c]
            # return [90, 90, 90]

        except ValueError as e:
            print(f"[ERROR] {str(e)}")
            # return self.kinema_inv([0, 0, 1], self.ini_pos[2])
            return [91,90,98]   # fallback to initial posture

    def control_t_posture(self, pos, t):
        theta, phi, Pz = pos
        #this lines of code makes sure that system is within constraints to work safely
        if phi > self.phi_max:
            phi = self.phi_max
        Pz = pos[2]
        if Pz > self.pz_max:
            Pz = self.pz_max
        elif Pz < self.pz_min:
            Pz = self.pz_min

        #for testing 
        # if((theta==100) & (phi==100)):
        #      return [180, 180, 180]
        # calculating planes normal vectors from plane vector returned by PID controller
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))
        n = [x, y, z]
        prev_angles=[91,90,98]
        angles = self.kinema_inv(n, Pz)
        
        # time.sleep(t)
        print(f"[IK Input] n={n}, Pz={Pz}")
        print(f"[IK Output] angles={angles}")
        
        #this was  makes sure platform noise doesnt disturb ball after its at center but it works fine without it due to good controller
        # if((angles[0]-prev_angles[0]>=3)|(angles[1]-prev_angles[1]>=3)|(angles[2]-prev_angles[2]>=3)|(angles[0]-prev_angles[0]<=-3)|(angles[1]-prev_angles[1]<=-3)|(angles[2]-prev_angles[2]<=-3)):
        #     prev_angles=angles
        #     self.port.send_angles(angles)
        self.port.send_angles(angles)
        
        time.sleep(.01)
        # delay(0.001)
        #intial position defined
        if (pos==self.ini_pos):
            return [91, 90, 98] 
         
        return angles

    def Initialize_posture(self):
        pos = self.ini_pos
        t = 0.02
        self.control_t_posture(pos, t)