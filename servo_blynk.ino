#define BLYNK_TEMPLATE_ID "TMPL6e9ctqALh"
#define BLYNK_TEMPLATE_NAME "ServoTest"
#define BLYNK_AUTH_TOKEN "aPUbevsk3xGs4XMuBj685QO04ThN1rw3"  

#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

// WiFi credentials
char ssid[] = "MDX welcomes you";  
char pass[] = "MdxL0vesyou";  

// Hardware setup
Servo myServo;
int servoPin = 2;          // GPIO pin for servo signal

// Default settings
int defaultPosition = 0;   // Servo's default position (in degrees)
int blynkServoAngle = 90;  // Default servo angle for manual or sensor-triggered activation

void setup() {
  Serial.begin(115200);

  // Initialize servo
  myServo.attach(servoPin);
  myServo.write(defaultPosition);  // Initialize servo to default position

  // Connect to WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected!");

  // Connect to Blynk
  Serial.println("Connecting to Blynk...");
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();
  Serial.println("Blynk connected!");
}

// Function to activate the servo
void activateServo(int angle) {
  Serial.print("Activating servo at angle: ");
  Serial.println(angle);

  myServo.write(angle);  // Move servo to specified angle
  delay(600);            // Wait for 0.4 seconds
  myServo.write(defaultPosition);  // Reset to default position
  Serial.println("Servo reset to default position.");

  // Send an event notification to Blynk
  Blynk.logEvent("servo_activated", String("Servo moved to ") + angle + " degrees");
  Serial.println("Blynk event notification sent.");
}

// Blynk slider to control servo angle
BLYNK_WRITE(V0) {
  blynkServoAngle = param.asInt();  // Read the value sent to V0 (0-180 degrees)
  Serial.print("Received servo position from Blynk: ");
  Serial.println(blynkServoAngle);

  if (blynkServoAngle >= 0 && blynkServoAngle <= 180) {
    activateServo(blynkServoAngle);  // Move servo to specified position
    Blynk.virtualWrite(V0, 0);       // Reset slider in Blynk app
    Serial.println("Blynk slider reset to 0.");
  } else {
    Serial.println("Invalid servo position received from Blynk.");
  }
}

void loop() {
  Blynk.run();       // Keep Blynk connected
  delay(100);        // Small delay for stability
}
