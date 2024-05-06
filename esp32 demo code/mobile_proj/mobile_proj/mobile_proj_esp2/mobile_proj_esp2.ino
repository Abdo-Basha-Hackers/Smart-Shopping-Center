



// Pin configurations

#define smokeDetectorPin 32 //digital output
#define buzzerPin 33
#define temperaturePin  14 //digital output


// motor  settings
#define motorPin 25


void setup() {
  Serial.begin(115200); // Set baud rate to 115200
  // smoke sensor
  pinMode(smokeDetectorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(temperaturePin, INPUT);
  Serial.println("Mall Sensors Test");
}

void loop() {
  int gasValue = digitalRead(smokeDetectorPin);
  
  int tempValue = digitalRead(temperaturePin);
  Serial.println(tempValue);
  //smoke sensor
  if (gasValue == LOW) {

    // Gas detected, turn on the buzzer
    digitalWrite(buzzerPin, HIGH);
  }
  else{
    digitalWrite(buzzerPin, LOW);
  }

  //temperature
  if(tempValue == HIGH){
    //turn on motor
    digitalWrite(motorPin, HIGH);
    delay(3000);
  } else if(tempValue == LOW) {
    //turn off motor
    digitalWrite(motorPin, LOW);
  }
  delay(500);

}