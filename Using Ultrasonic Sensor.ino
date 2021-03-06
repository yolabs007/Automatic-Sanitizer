#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


#define echopin A1 // echo pin
#define trigpin A2 // Trigger pin


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

uint8_t servonum = 0; // the number which is servo is connected (PCA 9685 - 16 channel)


int distance;
long duration;

int set = 5;

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  pinMode (trigpin, OUTPUT);
  pinMode (echopin, INPUT );
  
  pwm.begin();
  
  pwm.setPWMFreq(60); 
}


void loop() {

distance = data();
 Serial.print("S=");
 Serial.println(distance);
  if (distance < set){
      pwm.setPWM(0, 0, 150 );
  delay(500);
    }
  else{
         pwm.setPWM(0, 0, 255 );
  delay(500);
   }
}


long data(){
  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  duration = pulseIn (echopin, HIGH);
  return duration / 29 / 2;
}
 
