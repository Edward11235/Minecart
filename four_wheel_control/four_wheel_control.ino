String inputString = "";         // A String to hold incoming data
boolean stringComplete = false;  // Whether the string is complete

// Motor 1
#define ENA1 2
#define DIR1A 3
#define DIR1B 4
// Motor 2
#define ENA2 5
#define DIR2A 6
#define DIR2B 7
// Motor 3
#define ENA3 8
#define DIR3A 9
#define DIR3B 10  // Using A0 as a digital output
// Motor 4
#define ENA4 11
#define DIR4A 12  // Using A1 as a digital output
#define DIR4B 13  // Using A2 as a digital output

void setup() {
  Serial.begin(9600);
  // Set motor control pins as outputs
  pinMode(ENA1, OUTPUT);
  pinMode(DIR1A, OUTPUT);
  pinMode(DIR1B, OUTPUT);
  pinMode(ENA2, OUTPUT);
  pinMode(DIR2A, OUTPUT);
  pinMode(DIR2B, OUTPUT);
  pinMode(ENA3, OUTPUT);
  pinMode(DIR3A, OUTPUT);
  pinMode(DIR3B, OUTPUT);
  pinMode(ENA4, OUTPUT);
  pinMode(DIR4A, OUTPUT);
  pinMode(DIR4B, OUTPUT);
}

void loop() {
  if (stringComplete) {
    // Extract and parse the integers
    int nums[4]; // Array to hold the parsed integers
    int startIdx = 2; // Start after "<["
    for (int i = 0; i < 4; i++) {
      int endIdx = inputString.indexOf("]", startIdx);
      if (endIdx != -1) {
        nums[i] = inputString.substring(startIdx, endIdx).toInt();
        startIdx = endIdx + 2; // Move past the "]" and to the start of the next number
      }
    }

    // Control each wheel based on the parsed integers
    controlWheel(ENA1, DIR1A, DIR1B, nums[0]);
    controlWheel(ENA2, DIR2A, DIR2B, nums[1]);
    controlWheel(ENA3, DIR3A, DIR3B, nums[2]);
    controlWheel(ENA4, DIR4A, DIR4B, nums[3]);

    // Reset the input string and the complete flag
    inputString = "";
    stringComplete = false;
  }
}

void controlWheel(int enaPin, int dirPin1, int dirPin2, int speed) {
  bool direction = speed >= 0;
  speed = abs(speed);
  if (speed > 255) speed = 255; // Limit speed to PWM range

  digitalWrite(dirPin1, direction ? HIGH : LOW);
  digitalWrite(dirPin2, !direction ? HIGH : LOW);
  analogWrite(enaPin, speed);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '>') {
      stringComplete = true;
    }
  }
}
