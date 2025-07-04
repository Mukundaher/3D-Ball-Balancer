#program to test serial connection + servo motors

import serial
import time
import random

# Change this to your correct port, like 'COM3' on Windows or '/dev/ttyUSB0' on Linux/Mac users
arduino = serial.Serial('/dev/cu.usbmodem101', 9600)
time.sleep(2)  # Give time for Arduino to reset

def send_random_angles():
    while True:
        # Random angles between 45 and 135 sent to arduino
        a1 = random.randint(45, 135)
        a2 = random.randint(45, 135)
        a3 = random.randint(45, 135)

        command = f"{a1} {a2} {a3}\n"
        arduino.write(command.encode('utf-8'))

        print(f"Sent: {command.strip()}")

        time.sleep(0.1)  # Send every 0.5 seconds

try:
    send_random_angles()

except KeyboardInterrupt:
    print("\nStopped by user.")
    arduino.close()


