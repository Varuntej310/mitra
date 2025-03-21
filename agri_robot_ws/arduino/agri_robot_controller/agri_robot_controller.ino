#include <ArduinoJson.h>

// Motor pins
const int leftMotorPWM = 5;
const int leftMotorDir1 = 4;
const int leftMotorDir2 = 3;
const int rightMotorPWM = 6;
const int rightMotorDir1 = 7;
const int rightMotorDir2 = 8;

// Sprayer pins
const int sprayerPin = 9;

// Sensors
const int soilMoisturePin = A0;
const int batteryVoltagePin = A1;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Set motor pins as outputs
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);
  
  // Set sprayer pin as output
  pinMode(sprayerPin, OUTPUT);
  
  // Initialize motors to stopped
  stopMotors();
  
  // Send status message
  sendStatus("Robot initialized");
}

void loop() {
  // Check for commands from ROS
  if (Serial.available() > 0) {
    // Read and process command
    processCommand();
  }
  
  // Send sensor data periodically
  static unsigned long lastSensorTime = 0;
  if (millis() - lastSensorTime > 500) { // Every 500ms
    sendSensorData();
    lastSensorTime = millis();
  }
}

void processCommand() {
  // Read JSON command
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, Serial);
  
  if (error) {
    sendStatus("Error parsing command");
    return;
  }
  
  // Process different command types
  const char* type = doc["type"];
  
  if (strcmp(type, "motor") == 0) {
    // Motor control command
    float leftSpeed = doc["left"];
    float rightSpeed = doc["right"];
    setMotorSpeed(leftSpeed, rightSpeed);
  }
  else if (strcmp(type, "sprayer") == 0) {
    // Sprayer control command
    bool state = doc["state"];
    setSprayer(state);
  }
}

void setMotorSpeed(float leftSpeed, float rightSpeed) {
  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, -1.0, 1.0);
  rightSpeed = constrain(rightSpeed, -1.0, 1.0);
  
  // Set left motor direction and speed
  if (leftSpeed > 0) {
    digitalWrite(leftMotorDir1, HIGH);
    digitalWrite(leftMotorDir2, LOW);
    analogWrite(leftMotorPWM, leftSpeed * 255);
  }
  else if (leftSpeed < 0) {
    digitalWrite(leftMotorDir1, LOW);
    digitalWrite(leftMotorDir2, HIGH);
    analogWrite(leftMotorPWM, abs(leftSpeed) * 255);
  }
  else {
    digitalWrite(leftMotorDir1, LOW);
    digitalWrite(leftMotorDir2, LOW);
    analogWrite(leftMotorPWM, 0);
  }
  
  // Set right motor direction and speed
  if (rightSpeed > 0) {
    digitalWrite(rightMotorDir1, HIGH);
    digitalWrite(rightMotorDir2, LOW);
    analogWrite(rightMotorPWM, rightSpeed * 255);
  }
  else if (rightSpeed < 0) {
    digitalWrite(rightMotorDir1, LOW);
    digitalWrite(rightMotorDir2, HIGH);
    analogWrite(rightMotorPWM, abs(rightSpeed) * 255);
  }
  else {
    digitalWrite(rightMotorDir1, LOW);
    digitalWrite(rightMotorDir2, LOW);
    analogWrite(rightMotorPWM, 0);
  }
  
  // Send back confirmation
  StaticJsonDocument<100> doc;
  doc["type"] = "status";
  doc["message"] = "Motors updated";
  serializeJson(doc, Serial);
  Serial.println();
}

void stopMotors() {
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, 0);
  
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, 0);
}

void setSprayer(bool state) {
  digitalWrite(sprayerPin, state ? HIGH : LOW);
  
  // Send back confirmation
  StaticJsonDocument<100> doc;
  doc["type"] = "status";
  doc["message"] = state ? "Sprayer ON" : "Sprayer OFF";
  serializeJson(doc, Serial);
  Serial.println();
}

void sendSensorData() {
  // Read sensor values
  int soilMoisture = analogRead(soilMoisturePin);
  int batteryRaw = analogRead(batteryVoltagePin);
  
  // Convert battery reading to voltage (example conversion)
  float batteryVoltage = batteryRaw * (5.0 / 1023.0) * 3.0; // Assuming voltage divider
  
  // Create JSON message
  StaticJsonDocument<100> doc;
  doc["type"] = "sensors";
  JsonArray values = doc.createNestedArray("values");
  values.add(soilMoisture);
  values.add(batteryVoltage);
  
  // Send data
  serializeJson(doc, Serial);
  Serial.println();
}

void sendStatus(const char* message) {
  StaticJsonDocument<100> doc;
  doc["type"] = "status";
  doc["message"] = message;
  serializeJson(doc, Serial);
  Serial.println();
}