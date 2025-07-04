// connection pins for servos
const int servo1Pin = 3;
const int servo2Pin = 5;
const int servo3Pin = 6;
//intial angles
int angle1 = 91, angle2 = 90, angle3 = 98;

String inputString = "";
boolean stringComplete = false;

void setup() {
  pinMode(servo1Pin, OUTPUT);
  pinMode(servo2Pin, OUTPUT);
  pinMode(servo3Pin, OUTPUT);

  Serial.begin(9600);
  
}
//scaning serial string
void loop() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
//writing those to motors by PMW signal
  if (stringComplete) {
    int parsed = sscanf(inputString.c_str(), "%d %d %d", &angle1, &angle2, &angle3);
    if (parsed == 3) {
      angle1 = constrain(angle1, 45, 135);
      angle2 = constrain(angle2, 45, 135);
      angle3 = constrain(angle3, 45, 135);

      // Serial.print("Received: ");
      // Serial.print(angle1); Serial.print(" ");
      // Serial.print(angle2); Serial.print(" ");
      // Serial.println(angle3);
    } 
    // else {
    //   Serial.println("Invalid input");
    // }

    inputString = "";
    stringComplete = false;
  }

  sendPulse(servo1Pin, map(angle1, 0, 180, 550, 2400));
  sendPulse(servo2Pin, map(angle2, 0, 180, 550, 2400));
  sendPulse(servo3Pin, map(angle3, 0, 180, 550, 2400));

  delay(20);  // Maintain 50 Hz update rate
}


// Send PWM pulse to a pin
void sendPulse(int pin, int pulseWidth) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pin, LOW);
  delayMicroseconds(5000-pulseWidth);
}

// Read complete serial line