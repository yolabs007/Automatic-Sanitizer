# Automatic Sanitizer

#### `Testing of Arduino`

* Test the Arduino Board by using blinking an In-Built LED (Below code) 

```C++
/*
  This Code is written by Rahul Sharma for Yolabs. 
This is the  simplest code possible to blink in build LED  
Turns inbuild LED on and off at diff frequency to chk your arduino IDE, Arduino and cable is working
Note: please check the port in case you have error while uploadig 
 www.yolabs.in - 2020
  
*/

// the setup function runs once when you press reset or power the board

void setup()
{
  pinMode(13,OUTPUT);
  Serial.begin(9600);
}

void loop()
{
    digitalWrite(13, HIGH);
    
    Serial.println("I am High");
    delay(3000); // Wait for 1000 millisecond(s)
    digitalWrite(13, LOW);
    Serial.println("I am Low");
    delay(3000);
 
}


```


###  `use ultrasonic sensor or IR Sensor based on availability.`


### ` Connections`

Ultrasonic Sensor | Arduino
------------      | -------------
Trigg             | A2
Echo              | A1
VCC               | 5V
GND               | GND


### `Based on Ultrasonic sensor`


![Ultrasonic Sensor](https://microcontrollerslab.com/wp-content/uploads/2014/12/HC-SR04-Ultrasonic-Sensor-Pinout-diagram.jpg)

```C++

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

```

### `Based on IR Sensor`

![IR Sensor](https://5.imimg.com/data5/WA/GS/MY-5726208/delta-plc-repair-service-500x500.jpg)

### ` Connections`


IR Sensor         | Arduino
------------      | -------------
OUT               | Pin Number 7
VCC               | 5V
GND               | GND



```C++


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

boolean item;

uint8_t servonum = 0; // the number which is servo is connected (PCA 9685 - 16 channel)

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

```
