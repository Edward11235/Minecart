volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder
volatile int dir = 0;  // dir is -1, 0, or 1
volatile float degree = 0.0;


void setup() {
  Serial.begin (9600);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, ai0, RISING);  //AttachInterrupt 0 is d2 on most Arduino.
  attachInterrupt(1, ai1, RISING);  //AttachInterrupt 1 is d3 on most Arduino.
  }
   
void loop() {
  if( counter != temp ){  // if counter is updated, send it to serial monitor
    if(counter >= 32) {counter = counter % 32;}
    if(abs(degree) >= 360.0) {degree = fmod(degree, 360);}
    Serial.println (degree);
    temp = counter;
  }
}

   
void ai0() {  // ai0 is activated if d2 is going from LOW to HIGH
  if(digitalRead(3)==LOW) {  // if d3 is low, the motor is rotating clockwise
    counter++;
    dir = 1;
    degree += 11.25;
  }else{
    counter--;
    dir = -1;
    degree -= 11.25;
  }
}

 
void ai1() {  // ai1 is activated if d3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {  // if d3 is low, the motor is rotating counterclockwise
    counter--;
    dir = -1;
    degree -= 11.25;
  }else{
    counter++;
    dir = 1;
    degree += 11.25;
  }
}
