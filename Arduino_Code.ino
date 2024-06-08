#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servoMotor;

const int trigPin = 12;
const int echoPin = 11;
const int trigPin2 = 5;
const int echoPin2 = 6;
long duration;
int distance = 0;
int soil = 0;
int fsoil = 0;
int potPin = A0;

const int gasPin = A1; // Analog pin for gas sensor
const int buzzerPin = 7; // Digital pin for buzzer
int gasThreshold = 200; // Adjust this threshold based on sensor calibration
int distanceThreshold = 20; // Adjust this threshold for dustbin fullness

long ultrasonicRead(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) / 58.2;
}

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  pinMode(trigPin2, OUTPUT); 
  pinMode(echoPin2, INPUT);
  servoMotor.attach(8);
  pinMode(buzzerPin, OUTPUT);
  delay(2000); // Delay to stabilize LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Dustbin");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000); // Delay for initialization
}

void loop() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dry Wet Waste");
  lcd.setCursor(0, 1);
  lcd.print("Segregator");

  // Ultrasonic Sensor 1 and Moisture Sensor
  distance = ultrasonicRead(trigPin, echoPin);
  Serial.println(distance);
  
  if (distance < 15 && distance > 1) {
    soil = readMoisture(potPin);
    Serial.println(soil);
    if (soil > 500) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Garbage Detected!");
      lcd.setCursor(6, 1);
      lcd.print("DRY");
      servoMotor.write(180);
      delay(1000);
    } 
    else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Garbage Detected!");
      lcd.setCursor(6, 1);
      lcd.print("WET");
      servoMotor.write(0);
      delay(1000);
    }
    servoMotor.write(96);
  }
  
  // Ultrasonic Sensor 2 for Dustbin Fullness
  distance = getUltrasonicDistance();
  Serial.println(distance);
  
  if (distance < distanceThreshold) {
    digitalWrite(buzzerPin, HIGH); // Turn on the buzzer
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dustbin is Full");
  } else {
    digitalWrite(buzzerPin, LOW); // Turn off the buzzer
  }

  delay(1000);
}

int getUltrasonicDistance() {
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  long duration = pulseIn(echoPin2, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

int readMoisture(int pin) {
  int moistureValue = analogRead(pin);
  return moistureValue;
}
void rotateServo(int angle) {
  servoMotor.write(angle);
  delay(1000); // Wait for servo to reach position
  servoMotor.write(90); // Return servo to original position
  delay(1000); // Wait for servo to return to original position
}