import serial
import time


class SPort:
    #links to arduino R4 minima micro controller
    def __init__(self,
                 port:str = '/dev/cu.usbmodem1101',  
                 baud:int = 9600):
        self.ser = serial.Serial(port, baud,timeout=1)
        time.sleep(2)         
    def send_angles(self, angles):
        #integer angles send via serial communication
        # line = f"{angles[0]:.1f} {angles[1]:.1f} {angles[2]:.1f}\n"
        line = f"{int(angles[0])} {int(angles[1])} {int(angles[2])}\n"

        self.ser.write(line.encode('utf-8'))
        print(f"Sent: {line.strip()}")

    def close(self):
        self.ser.close()
        self.ser.reset_input_buffer()   
        self.ser.reset_output_buffer()  
