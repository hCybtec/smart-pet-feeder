#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pins
const int trigPin = 7;    // Ultrasonic trigger pin for proximity
const int echoPin = 6;    // Ultrasonic echo pin for proximity
const int foodTrigPin = 4; // Ultrasonic trigger pin for food level
const int foodEchoPin = 3; // Ultrasonic echo pin for food level
const int pirPin = 8;     // PIR sensor pin
const int servoPin = 10;  // Servo pin
const int buzzerPin = 11; // Buzzer pin

// LCD Initialization
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Constants
const float lowFoodThreshold = 17.0; // Food level threshold in cm
const float proximityThreshold = 5.0; // Proximity threshold in cm
const unsigned long sensorDelay = 10000; // 10 seconds delay for disabling sensors
const int waitTime = 15000;  // Wait time for second motion detection in ms

// Objects
Servo myServo;

// Variables
bool sensorsEnabled = true;      // Tracks if sensors are enabled
unsigned long sensorDisableTime; // Time when sensors were disabled
bool isServoOpen = false;        // Track if the servo is open

void setup() {
  Serial.begin(9600);

  // Initialize pins
  pinMode(foodTrigPin, OUTPUT);
  pinMode(foodEchoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(pirPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach servo
  myServo.attach(servoPin);
  myServo.write(0); // Start with servo in closed position

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void loop() {
  // Re-enable sensors after delay
  if (!sensorsEnabled && millis() - sensorDisableTime >= sensorDelay) {
    sensorsEnabled = true;
    Serial.println("Sensors re-enabled. Ready for new input.");
  }

  // Handle motion and servo control
  if (sensorsEnabled) {
    handleMotionAndServo();
  }

  // Check food level
  checkFoodLevel();

  delay(500); // Small delay for stability
}

// Function to check food level and alert if low
void checkFoodLevel() {
  float foodDistance = getUltrasonicDistance(foodTrigPin, foodEchoPin);

  Serial.print("Food Level Distance: ");
  Serial.print(foodDistance);
  Serial.println(" cm");

  lcd.setCursor(0, 0);
  if (foodDistance > 0 && foodDistance >= lowFoodThreshold) {
    lcd.print("Food Level Low! ");
    lcd.setCursor(0, 1);
    lcd.print("Time to Refill");

    for (int i = 0; i < 2; i++) {
      digitalWrite(buzzerPin, HIGH);
      delay(500);
      digitalWrite(buzzerPin, LOW);
      delay(500);
    }
  } else {
    lcd.print("Food Level OK :) ");
    lcd.setCursor(0, 1);
    lcd.print("                "); // Clear second line

    digitalWrite(buzzerPin, LOW);
  }
}

// Function to handle motion detection and servo control
void handleMotionAndServo() {
  bool motionDetected = digitalRead(pirPin);
  float proximityDistance = getUltrasonicDistance(trigPin, echoPin);

  Serial.print("Motion Detected: ");
  Serial.print(motionDetected);
  Serial.print(", Proximity Distance: ");
  Serial.print(proximityDistance);
  Serial.println(" cm");

  if (!isServoOpen && motionDetected && proximityDistance > 0 && proximityDistance <= proximityThreshold) {
    Serial.println("Motion and proximity conditions met. Opening servo.");

    myServo.write(90);
    isServoOpen = true;
    sensorsEnabled = false;
    sensorDisableTime = millis();

    delay(waitTime);
    waitForSecondMotion();
  }
}

// Function to wait for the second motion to reset the servo
void waitForSecondMotion() {
  while (true) {
    bool motionDetected = digitalRead(pirPin);
    float proximityDistance = getUltrasonicDistance(trigPin, echoPin);

    Serial.print("Second Motion Waiting... Motion: ");
    Serial.print(motionDetected);
    Serial.print(", Distance: ");
    Serial.print(proximityDistance);
    Serial.println(" cm");

    if (isServoOpen && motionDetected && proximityDistance > 0 && proximityDistance <= proximityThreshold) {
      Serial.println("Second motion detected. Resetting servo.");

      myServo.write(0);
      isServoOpen = false;
      delay(1000);
      break;
    }
    delay(100);
  }
}

// Function to get distance from ultrasonic sensor
float getUltrasonicDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000); // Timeout after 30ms

  if (duration == 0) {
    return -1; // No echo received
  }

  return duration * 0.034 / 2; // Convert duration to distance in cm
}
