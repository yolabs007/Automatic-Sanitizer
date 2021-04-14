
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

boolean item;

uint8_t servonum = 0; // the number which is servo is connected

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  pwm.begin();
  
  pwm.setPWMFreq(60); 

  pinMode(7, INPUT);
}


void loop() {

item = digitalRead(7);   // Read the data from IR sensor
  if (item == 0) {
     pwm.setPWM(0, 0, 150 ); // change the last number for vary the position of servo
  delay(500);
  }
  if (item == 1) {
        pwm.setPWM(0, 0, 255 ); // change the last number for vary the position of servo
  delay(500);
  }

 
}
