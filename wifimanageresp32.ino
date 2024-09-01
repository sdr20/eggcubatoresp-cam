#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <Stepper.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h> // Updated LCD library for ESP32
#include <DHT.h>
#include <ESP32Servo.h>
#include <WiFiManager.h>

// Firebase configuration
#define FIREBASE_HOST "itlog-database-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_API_KEY "AIzaSyBlob6OB0gXhG7JTno7zY_mTFhHQkzVI3g"

// Define the number of steps per revolution for your motor
#define STEPS_PER_REVOLUTION 2048
#define STEPS_TO_MOVE 1024 // 180 degrees

// Initialize the stepper library on pins 26, 25, 33, and 32
Stepper myStepper(STEPS_PER_REVOLUTION, 26, 25, 33, 32);

// Servo pins
#define SERVO_PAN_PIN 18
#define SERVO_TILT_PIN 19

Servo panServo;
Servo tiltServo;

// Initialize the DHT sensor
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Initialize the LCD
LiquidCrystal_PCF8574 lcd(0x27); // Updated for ESP32

// Relay pin definitions
#define RELAY1_PIN 27
#define RELAY2_PIN 14
#define RELAY3_PIN 12

// LED light relay pin
#define LED_LIGHT_PIN 33

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
unsigned long motorStartMillis = 0;
unsigned long moveBackTime = 0;
int count = 0; // Declare count variable here
bool signupOK = false;
unsigned long motorOperationTimeMillis = 3600000; // Default to 1 hour in milliseconds
bool motorMoved = false; // Track motor movement state

int lastPanAngle = -1; // Initialize with an invalid value
int lastTiltAngle = -1; // Initialize with an invalid value

void displayConnectingToWiFi() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting to");
  lcd.setCursor(0, 1);
  lcd.print("WiFi...");
}

void displayWiFiConnected() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connected");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);

  // Initialize the LCD
  lcd.begin(16, 2);
  lcd.setBacklight(255);

  // Display "Connecting to WiFi" on the LCD while connecting
  displayConnectingToWiFi();

  // Initialize WiFiManager
  WiFiManager wm;
  bool res;
  
  // res = wm.autoConnect(); // auto generated AP name from chipid
  res = wm.autoConnect("Eggcubator-AP", "admin123"); // custom AP name

  if (!res) {
    Serial.println("Failed to connect");
    // Reset and try again, or you could put it to deep sleep
    ESP.restart();
  } else {
    // Display "WiFi Connected" on the LCD after connecting
    displayWiFiConnected();
    Serial.println("Connected to Wi-Fi successfully");
  }

  // Firebase configuration
  config.api_key = FIREBASE_API_KEY;
  config.database_url = FIREBASE_HOST;

  // Sign up
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  // Assign the callback function for the long-running token generation task
  config.token_status_callback = tokenStatusCallback;

  // Initialize Firebase
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Set the speed of the motor (RPM)
  myStepper.setSpeed(15);

  // Initialize the DHT sensor
  dht.begin();

  // Initialize the relay pins
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);

  // Initialize the LED light pin
  pinMode(LED_LIGHT_PIN, OUTPUT);

  // Ensure the relays and LED light are off at startup
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(LED_LIGHT_PIN, LOW);

  // Initialize the servos
  panServo.attach(SERVO_PAN_PIN);
  tiltServo.attach(SERVO_TILT_PIN);
}

void loop() {
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    // Write an Int number on the database path test/int
    if (Firebase.RTDB.setInt(&fbdo, "test/int", count)) {
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    } else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    count++;

    // Write a Float number on the database path test/float
    if (Firebase.RTDB.setFloat(&fbdo, "test/float", 0.01 + random(0, 10))) {
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    } else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  }

  // Retrieve motor operation time from Firebase
  if (Firebase.RTDB.getInt(&fbdo, "/control/motorOperationTime")) {
    motorOperationTimeMillis = fbdo.intData() * 3600000UL; // Convert hours to milliseconds
    Serial.print("Motor Operation Time (ms): ");
    Serial.println(motorOperationTimeMillis);
  } else {
    Serial.print("Failed to get motorOperationTime: ");
    Serial.println(fbdo.errorReason());
  }

  // If motorOperationTime is 0, turn off the motor
  if (motorOperationTimeMillis == 0) {
    if (motorMoved) {
      // Return the motor to the original position if it has moved
      Serial.println("Returning to 0 degrees (Motor Operation Time is 0)");
      myStepper.step(-STEPS_TO_MOVE); // Move back to 0 degrees
      motorMoved = false; // Reset motor state
    }
  } else {
    // Rotate motor to 180 degrees and then back to 0 degrees based on the operation time
    if (!motorMoved) {
      Serial.println("Rotating 180 degrees");
      myStepper.step(STEPS_TO_MOVE); // Move 180 degrees
      motorStartMillis = millis(); // Record the time when the motor started moving
      moveBackTime = motorStartMillis + motorOperationTimeMillis; // Set time to move back to 0 degrees
      motorMoved = true; // Mark motor as moved
    }

    // Move the motor back to 0 degrees after the set interval
    if (motorMoved && millis() >= moveBackTime) {
      Serial.println("Returning to 0 degrees");
      myStepper.step(-STEPS_TO_MOVE); // Move back to 0 degrees
      motorMoved = false; // Reset motor state
    }
  }

  // Read humidity and temperature from DHT sensor
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Turn off relay 1 if temperature exceeds 36.5°C
  if (t >= 36.5) {
    digitalWrite(RELAY1_PIN, LOW);
    Serial.println("Temperature exceeds 36.5°C. Light turned off.");
  }

  // Update Firebase with sensor data
  if (Firebase.RTDB.setFloat(&fbdo, "/sensor_data/humidity", h)) {
    Serial.println("Humidity updated successfully");
  } else {
    Serial.print("Failed to update humidity: ");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.setFloat(&fbdo, "/sensor_data/temperature", t)) {
    Serial.println("Temperature updated successfully");
  } else {
    Serial.print("Failed to update temperature: ");
    Serial.println(fbdo.errorReason());
  }

  // Display temperature and humidity on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Hum: ");
  lcd.print(h);
  lcd.print(" %");
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(t);
  lcd.print(" C");

  // Read control settings from Firebase
  if (Firebase.RTDB.getFloat(&fbdo, "/settings/maxHumidity")) {
    float maxHumidity = fbdo.floatData();
    Serial.print("Max Humidity: ");
    Serial.println(maxHumidity);
  } else {
    Serial.print("Failed to get maxHumidity: ");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.getFloat(&fbdo, "/settings/maxTemperature")) {
    float minTemperature = fbdo.floatData();
    Serial.print("Min Temperature: ");
    Serial.println(minTemperature);
  } else {
    Serial.print("Failed to get minTemperature: ");
    Serial.println(fbdo.errorReason());
  }

  // Control servos based on Firebase data
  if (Firebase.RTDB.getInt(&fbdo, "/servos/pan")) {
    int panAngle = fbdo.intData();
    if (panAngle != lastPanAngle) {
      panServo.write(panAngle);
      lastPanAngle = panAngle;
      Serial.print("Pan Angle: ");
      Serial.println(panAngle);
    }
  } else {
    Serial.print("Failed to get panAngle: ");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.getInt(&fbdo, "/servos/tilt")) {
    int tiltAngle = fbdo.intData();
    if (tiltAngle != lastTiltAngle) {
      tiltServo.write(tiltAngle);
      lastTiltAngle = tiltAngle;
      Serial.print("Tilt Angle: ");
      Serial.println(tiltAngle);
    }
  } else {
    Serial.print("Failed to get tiltAngle: ");
    Serial.println(fbdo.errorReason());
  }

  // Control LED light based on Firebase data
  if (Firebase.RTDB.getBool(&fbdo, "/light")) {
    bool ledLight = fbdo.boolData();
    digitalWrite(LED_LIGHT_PIN, ledLight ? HIGH : LOW);
    Serial.print("LED Light: ");
    Serial.println(ledLight ? "ON" : "OFF");
  } else {
    Serial.print("Failed to get ledLight: ");
    Serial.println(fbdo.errorReason());
  }

  delay(2000); // Delay between sensor readings
}
