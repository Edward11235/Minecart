#include <util/atomic.h>

// Pins for each motor
#define ENCA1 2
#define ENCB1 6
#define PWM1 24
#define IN11 22
#define IN21 26

#define ENCA2 7
#define ENCB2 3
#define PWM2 32
#define IN12 28
#define IN22 30

#define ENCA3 18
#define ENCB3 15
#define PWM3 36
#define IN13 34
#define IN23 32

#define ENCA4 19
#define ENCB4 16
#define PWM4 38
#define IN14 40
#define IN24 42

// Global variables for each motor
struct Motor {
    volatile int pos = 0;
    volatile float velocity = 0;
    long prevT = 0;
    int posPrev = 0;
    float vFilt = 0;
    float vPrev = 0;
    float eintegral = 0;
    volatile float vt = 20; // Target velocity
};

Motor motors[4];

// Serial communication
String inputString = "";      
boolean stringComplete = false;

void setup() {
    Serial.begin(115200);

    pinMode(ENCA1, INPUT);
    pinMode(ENCB1, INPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(IN11, OUTPUT);
    pinMode(IN21, OUTPUT);

    pinMode(ENCA2, INPUT);
    pinMode(ENCB2, INPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(IN12, OUTPUT);
    pinMode(IN22, OUTPUT);

    pinMode(ENCA3, INPUT);
    pinMode(ENCB3, INPUT);
    pinMode(PWM3, OUTPUT);
    pinMode(IN13, OUTPUT);
    pinMode(IN23, OUTPUT);

    pinMode(ENCA4, INPUT);
    pinMode(ENCB4, INPUT);
    pinMode(PWM4, OUTPUT);
    pinMode(IN14, OUTPUT);
    pinMode(IN24, OUTPUT);

    // Attach interrupts for all encoders
    attachInterrupt(digitalPinToInterrupt(ENCA1), []{ readEncoder(0, ENCB1); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA2), []{ readEncoder(1, ENCB2); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA3), []{ readEncoder(2, ENCB3); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA4), []{ readEncoder(3, ENCB4); }, RISING);
}

void loop() {
    if (stringComplete) {
        if (inputString.startsWith("<[") && inputString.endsWith("]>")) {
            int startIdx = 2;
            for (int i = 0; i < 4; i++) {
                int endIdx = inputString.indexOf(",", startIdx);
                if (endIdx == -1 && i < 3) {
                    endIdx = inputString.indexOf("]", startIdx);
                }
                motors[i].vt = inputString.substring(startIdx, endIdx).toFloat();
                startIdx = endIdx + 1;
            }
            inputString = "";
            stringComplete = false;
        }
    }

    processMotor(motors[0], PWM1, IN11, IN21);
    processMotor(motors[1], PWM2, IN12, IN22);
    processMotor(motors[2], PWM3, IN13, IN23);
    processMotor(motors[3], PWM4, IN14, IN24);
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

void setMotor(Motor& motor, int pwmVal, int pwm, int in1, int in2) {
    analogWrite(pwm, pwmVal); 
    if (motor.vt - motor.vFilt < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
}

void readEncoder(int index, int encBPin) {
    int b = digitalRead(encBPin);
    int increment = b > 0 ? 1 : -1;
    motors[index].pos += increment;
}

void processMotor(Motor& motor, int pwmPin, int in1, int in2) {
    int pos;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        pos = motor.pos;
    }
    long currT = micros();
    float deltaT = ((float)(currT - motor.prevT)) / 1.0e6;
    float velocity = (pos - motor.posPrev) / deltaT;
    motor.posPrev = pos;
    motor.prevT = currT;

    float v = velocity / 784.0 * 60.0;
    motor.vFilt = 0.854 * motor.vFilt + 0.0728 * v + 0.0728 * motor.vPrev;
    motor.vPrev = v;

    float kp = 5;
    float ki = 10;
    float e = motor.vt - motor.vFilt;
    motor.eintegral += e * deltaT;

    float u = kp * e + ki * motor.eintegral;
    int dir = u < 0 ? -1 : 1;
    int pwr = (int) fabs(u);
    if (pwr > 255) {
        pwr = 255;
    }

    setMotor(motor, pwr, pwmPin, in1, in2);
}
