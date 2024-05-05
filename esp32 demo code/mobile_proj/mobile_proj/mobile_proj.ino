#include "DHTesp.h"
#include  <Robojax_L298N_DC_motor.h>
// #include <Arduino.h>
// #include <analogWrite.h>

//set DHT pin 2
#define DHTPIN 19

#define DHTTYPE DHT11


// Pin configurations
// #define motionSensorPin 26 // Motion sensor output pin
#define redPin 27        // Red pin of RGB LED
#define greenPin  25       // Green pin of RGB LED

#define infraredPin  14 //digital output
#define streetLampLedPin  12

#define  trigPin 26  
#define echoPin 35
float duration, distance;


#define smokeDetectorPin 32 //digital output
#define buzzerPin 33

// motor  settings
#define IN3 18
#define IN4 5
#define ENB 4// this pin must be PWM enabled pin if Arduino board is used
#define CHB 1

 const int CCW = 2; // do not change
 const int CW  = 1; // do not change
 #define motor2 2 // do not change

Robojax_L298N_DC_motor robot(IN3, IN4, ENB, CHB, true);
DHTesp dht;

// #define motorpin1 18
// #define motorpin2 5
// #define ENA 4
// Variables to track the previous state of the sensor
int prevMotionState = LOW;

void setup() {
  Serial.begin(115200); // Set baud rate to 115200
  Serial.println("Parking Sensor Test");
  /// ultrasonic sensor
  pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT);  

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);

  ///end motion sensor
  /////////////////////////////
  //infraredPin sensor
  pinMode(infraredPin, INPUT);
  pinMode(streetLampLedPin,OUTPUT);
  // //end infraredPin sensor
  // /////////////////////////////
  // smoke sensor
  pinMode(smokeDetectorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  //end smoke sensor
  // ////////////////////////////
  // //temparature sensor
  //  // Initialize temperature and motor sensor
	// dht.setup(DHTPIN, DHTesp::DHTTYPE);
  // pinMode(motorpin1,OUTPUT);
  // pinMode(motorpin2,OUTPUT);
  // pinMode(ENA,OUTPUT);
  // analogWrite(ENA,100);
  // digitalWrite(motorpin1,HIGH);
  // digitalWrite(motorpin2,LOW);
   robot.begin();
  Serial.println("Parking Sensor Test");
}

void loop() {
  
  // digitalWrite(trigPin, LOW);  
	// delayMicroseconds(2);  
	// digitalWrite(trigPin, HIGH);  
	// delayMicroseconds(10);  
	// digitalWrite(trigPin, LOW);

  // duration = pulseIn(echoPin, HIGH); 
  // distance = (duration*.0343)/2;
  // Serial.print("Distance: ");  
	// Serial.println(distance); 

 
  // if (distance <= 7) {
  //   // Car enters the parking spot
  //   digitalWrite(redPin, HIGH);   // Turn on red color
  //   digitalWrite(greenPin, LOW);  // Turn off green color
  //   Serial.println("Car parked!");
  // } else {
  //   // Car leaves or parking slot is empty
  //   digitalWrite(redPin, LOW);    // Turn off red color
  //   digitalWrite(greenPin, HIGH); // Turn on green color
  //   Serial.println("Parking spot empty!");
  // }

  // Serial.println(motionState);
  int gasValue = digitalRead(smokeDetectorPin);
  // int irValue = digitalRead(infraredPin);
  Serial.println(gasValue);
  //float temperature = dht.getTemperature();
  // motion sensor
  // if (motionState != prevMotionState) {
  //   // If the current state is different from the previous state,
  //   // it indicates a change in the sensor state
  //   if (motionState == HIGH) {
  //     // Car enters the parking spot
  //     digitalWrite(redPin, HIGH);   // Turn on red color
  //     digitalWrite(greenPin, LOW);  // Turn off green color
  //     Serial.println("Car parked!");
  //   } else {
  //     // Car leaves or parking slot is empty
  //     digitalWrite(redPin, LOW);    // Turn off red color
  //     digitalWrite(greenPin, HIGH); // Turn on green color
  //     Serial.println("Parking spot empty!");
  //   }
  //   // Update previous state to the current state
  //   prevMotionState = motionState;
  // }

   
  //smoke sensor
  if (gasValue == LOW) {

    // Gas detected, turn on the buzzer
    digitalWrite(buzzerPin, HIGH);
  }
  else{
    digitalWrite(buzzerPin, LOW);
  }

  
  // // // infrared sensor
  // if (irValue == HIGH) {
  //   digitalWrite(streetLampLedPin, LOW); // Turn on the LED
  // } else if(irValue == LOW) {
  //   digitalWrite(streetLampLedPin, HIGH); // Turn off the LED
  // }
  Serial.println(9);
 robot.rotate(motor2, 70, CCW);
 Serial.println(10);
 delay(10000);
    robot.brake(2);
  //temperature
  // if(temperature>=25){
  //   //turn on motor
  //   robot.rotate(motor2, 70, CCW);//run motor2 at 70% speed in CW direction
  //   delay(10000);
  //   robot.brake(2);
  // }
  // if(temperature<=15){ //change temperature according to classroom temperature
  //   //turn off motor
  //   robot.brake(2);
  // }
  
  delay(500);



}





