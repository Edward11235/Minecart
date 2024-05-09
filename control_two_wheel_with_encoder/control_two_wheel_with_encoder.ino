#include <util/atomic.h>

// Pins for Wheel 1
#define ENCA1 2
#define ENCB1 4
#define PWM1 7
#define IN11 8
#define IN12 9

// Pins for Wheel 2
#define ENCA2 3
#define ENCB2 5
#define PWM2 10
#define IN21 11
#define IN22 12

// Globals
long prevT1 = 0, prevT2 = 0;
int posPrev1 = 0, posPrev2 = 0;

// Use the "volatile" directive for variables used in interrupts
volatile int pos_i1 = 0, pos_i2 = 0;
volatile float velocity_i1 = 0, velocity_i2 = 0;
volatile long prevT_i1 = 0, prevT_i2 = 0;

float v1Filt1 = 0, v1Prev1 = 0;
float v1Filt2 = 0, v1Prev2 = 0;

float eintegral1 = 0, eintegral2 = 0;

// Target velocity (vt) as a global variable
volatile float vt = 0; // Default value

// Serial communication
String inputString = "";      // A String to hold incoming data
boolean stringComplete = false;  // Whether the string is complete

void setup() {
  Serial.begin(115200);
  
  // Setup for Wheel 1
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(IN11, OUTPUT);
  pinMode(IN12, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);
  
  // Setup for Wheel 2
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(IN21, OUTPUT);
  pinMode(IN22, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);
}

void loop() {
  if (stringComplete) {
    // Process incoming serial command for both wheels
    processSerialCommand();
  }

  // Control logic for Wheel 1
  controlMotor(1, &pos_i1, &prevT1, &posPrev1, &v1Filt1, &v1Prev1, &eintegral1, PWM1, IN11, IN12);
  
  // Control logic for Wheel 2
  controlMotor(2, &pos_i2, &prevT2, &posPrev2, &v1Filt2, &v1Prev2, &eintegral2, PWM2, IN21, IN22);
}

void processSerialCommand() {
  if (inputString.startsWith("<[") && inputString.endsWith("]>")) {
    int startIdx = 2; // Start after "<["
    int endIdx = inputString.indexOf("]", startIdx);
    if (endIdx != -1) {
      vt = inputString.substring(startIdx, endIdx).toFloat();
    }
    inputString = "";
    stringComplete = false;
  }
}

void controlMotor(int motorId, volatile int* pos_i, long* prevT, int* posPrev, float* v1Filt, float* v1Prev, float* eintegral, int pwm, int in1, int in2) {
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = *pos_i;
  }
  
  long currT = micros();
  float deltaT = ((float) (currT-*prevT))/1.0e6;
  float velocity1 = (pos - *posPrev)/deltaT;
  *posPrev = pos;
  *prevT = currT;

  float v1 = velocity1 / 784.0 * 60.0;

  *v1Filt = 0.854 * *v1Filt + 0.0728 * v1 + 0.0728 * *v1Prev;
  *v1Prev = v1;

  float kp = 5;
  float ki = 10;
  float e = vt - *v1Filt;
  *eintegral = *eintegral + e * deltaT;

  float u = kp * e + ki * *eintegral;

  setMotor(u, pwm, in1, in2);
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

void setMotor(float u, int pwm, int in1, int in2) {
  int dir = u < 0 ? -1 : 1;
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  
  Serial.println(pwr); // Debugging output
  
  analogWrite(pwm, pwr); // Motor speed
  digitalWrite(in1, dir > 0 ? HIGH : LOW);
  digitalWrite(in2, dir > 0 ? LOW : HIGH);
}

void readEncoder1() {
  readEncoder(&pos_i1, &prevT_i1, ENCB1);
}

void readEncoder2() {
  readEncoder(&pos_i2, &prevT_i2, ENCB2);
}

void readEncoder(volatile int* pos_i, volatile long* prevT_i, int encBPin) {
  int b = digitalRead(encBPin);
  int increment = b > 0 ? 1 : -1;
  *pos_i += increment;
  
  long currT = micros();
  float deltaT = ((float) (currT - *prevT_i))/1.0e6;
  velocity_i1 = increment/deltaT;  // This should be handled per wheel if separate velocity is needed
  *prevT_i = currT;
}
