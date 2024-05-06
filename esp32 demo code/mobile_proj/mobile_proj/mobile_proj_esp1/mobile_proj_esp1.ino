
// Pin configurations
// #define motionSensorPin 26 // Motion sensor output pin
#define redPin 27        // Red pin of RGB LED
#define greenPin  25       // Green pin of RGB LED

#define infraredPin  14 //digital output
#define streetLampLedPin  12

#define  trigPin 26  
#define echoPin 35
float duration, distance;


void setup() {
  Serial.begin(115200); // Set baud rate to 115200
  Serial.println("Parking Sensor Test");
  /// ultrasonic sensor
  pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT);  

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);

  /////////////////////////////
  //infraredPin sensor
  pinMode(infraredPin, INPUT);
  pinMode(streetLampLedPin,OUTPUT);
  // //end infraredPin sensor
  // /////////////////////////////
  Serial.println("Parking Sensor Test");
}

void loop() {
  
  digitalWrite(trigPin, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trigPin, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH); 
  distance = (duration*.0343)/2;
  Serial.print("Distance: ");  
	Serial.println(distance); 
  int irValue = digitalRead(infraredPin);
 
  if (distance <= 7) {
    // Car enters the parking spot
    digitalWrite(redPin, HIGH);   // Turn on red color
    digitalWrite(greenPin, LOW);  // Turn off green color
    Serial.println("Car parked!");
  } else {
    // Car leaves or parking slot is empty
    digitalWrite(redPin, LOW);    // Turn off red color
    digitalWrite(greenPin, HIGH); // Turn on green color
    Serial.println("Parking spot empty!");
  }

  // // // infrared sensor
  if (irValue == HIGH) {
    digitalWrite(streetLampLedPin, LOW); // Turn on the LED
  } else if(irValue == LOW) {
    digitalWrite(streetLampLedPin, HIGH); // Turn off the LED
  }

  delay(500);



}