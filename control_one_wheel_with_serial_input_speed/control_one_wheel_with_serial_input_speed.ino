#include <util/atomic.h>

// Pins
#define ENCA 2
#define ENCB 3
#define PWM 4
#define IN1 5
#define IN2 6

// Globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

// Target velocity (vt) as a global variable
volatile float vt = 20; // Default value

// Serial communication
String inputString = "";      // A String to hold incoming data
boolean stringComplete = false;  // Whether the string is complete

void setup() {
  Serial.begin(115200);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  // Check for a complete serial message
  if (stringComplete) {
    if (inputString.startsWith("<[") && inputString.endsWith("]>")) {
      int startIdx = 2; // Start after "<["
      int endIdx = inputString.indexOf("]", startIdx);
      if (endIdx != -1) {
        // Parse the first integer and use it as vt
        vt = inputString.substring(startIdx, endIdx).toFloat();
      }
      // Reset the input string and the complete flag
      inputString = "";
      stringComplete = false;
    }
  }

  // Continue with your original loop code, using the updated vt
  // The rest of your loop code remains unchanged
  
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
  }
  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/784.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;

  // Compute the control signal u
  float kp = 5;
  float ki = 10;
  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  setMotor(dir,pwr,PWM,IN1,IN2);
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

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal); // Motor speed
  if(dir == 1) { 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if(dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);    
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  int increment = b > 0 ? 1 : -1;
  pos_i = pos_i + increment;
  
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}
