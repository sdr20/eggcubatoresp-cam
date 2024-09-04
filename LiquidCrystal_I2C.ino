#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <Stepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <WiFiManager.h>

// Firebase configuration
#define FIREBASE_HOST "itlog-database-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_API_KEY "AIzaSyBlob6OB0gXhG7JTno7zY_mTFhHQkzVI3g"

// Define constants
#define STEPS_PER_REVOLUTION 2048 // Full revolution steps for the stepper motor
#define STEPS_FOR_180_DEGREES (STEPS_PER_REVOLUTION / 2) // Steps for 180 degrees

#define SERVO_PAN_PIN 18
#define SERVO_TILT_PIN 19
#define DHTPIN 4
#define DHTTYPE DHT11
#define RELAY1_PIN 27
#define RELAY2_PIN 14
#define RELAY3_PIN 12
#define LED_LIGHT_PIN 33

// Initialize peripherals
Stepper myStepper(STEPS_PER_REVOLUTION, 26, 25, 33, 32);
Servo panServo, tiltServo;
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Firebase and WiFi objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
WiFiManager wm;

// State variables
int lastPanAngle = -1;
int lastTiltAngle = -1;
bool signupOK = false;
int lastOperationTime = -1;

// Function prototypes
void connectToWiFi();
void initializeFirebase();
void updateSensorData();
void controlRelays(float temperature);
void controlServos();
void controlStepper();
void updateFirebase(const String &path, float value);
void handleFirebaseError();

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.backlight();

  connectToWiFi();
  initializeFirebase();

  myStepper.setSpeed(15); // Speed of the stepper motor
  dht.begin();

  // Initialize relays and LED light
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(LED_LIGHT_PIN, OUTPUT);

  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(LED_LIGHT_PIN, LOW);

  panServo.attach(SERVO_PAN_PIN);
  tiltServo.attach(SERVO_TILT_PIN);
}

void loop() {
  updateSensorData();
  controlRelays(dht.readTemperature());
  controlServos();
  controlStepper();
  delay(2000);
}

void connectToWiFi() {
  lcd.clear();
  lcd.print("Connecting to WiFi...");
  if (wm.autoConnect("Eggcubator-AP", "admin123")) {
    lcd.clear();
    lcd.print("WiFi Connected");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(3000);
    lcd.clear();
    lcd.print("Welcome to");
    lcd.setCursor(0, 1);
    lcd.print("Eggcubator");
    delay(3000);
  } else {
    Serial.println("Failed to connect to WiFi");
    ESP.restart();
  }
}

void initializeFirebase() {
  config.api_key = FIREBASE_API_KEY;
  config.database_url = FIREBASE_HOST;
  config.token_status_callback = tokenStatusCallback;

  if (Firebase.signUp(&config, &auth, "", "")) {
    signupOK = true;
  } else {
    Serial.printf("Firebase Sign-Up Error: %s\n", config.signer.signupError.message.c_str());
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void updateSensorData() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(t);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(h);
  lcd.print("%");

  updateFirebase("/sensor_data/humidity", h);
  updateFirebase("/sensor_data/temperature", t);
}

void controlRelays(float temperature) {
  digitalWrite(RELAY1_PIN, temperature < 30 ? HIGH : LOW);
  digitalWrite(RELAY2_PIN, temperature < 35 ? HIGH : LOW);
  digitalWrite(RELAY3_PIN, temperature >= 37 ? HIGH : LOW);
  Serial.printf("Relay Status: %s %s %s\n", 
                 temperature < 30 ? "RELAY1 ON" : "RELAY1 OFF", 
                 temperature < 35 ? "RELAY2 ON" : "RELAY2 OFF",
                 temperature >= 37 ? "RELAY3 ON" : "RELAY3 OFF");
}

void controlServos() {
  int panAngle, tiltAngle;

  if (Firebase.RTDB.getInt(&fbdo, "/servos/pan")) {
    panAngle = fbdo.intData();
    if (panAngle != lastPanAngle) {
      panServo.write(panAngle);
      lastPanAngle = panAngle;
      Serial.printf("Pan Servo Angle: %d\n", panAngle);
    }
  } else {
    handleFirebaseError();
  }

  if (Firebase.RTDB.getInt(&fbdo, "/servos/tilt")) {
    tiltAngle = fbdo.intData();
    if (tiltAngle != lastTiltAngle) {
      tiltServo.write(tiltAngle);
      lastTiltAngle = tiltAngle;
      Serial.printf("Tilt Servo Angle: %d\n", tiltAngle);
    }
  } else {
    handleFirebaseError();
  }

  if (Firebase.RTDB.getBool(&fbdo, "/light")) {
    digitalWrite(LED_LIGHT_PIN, fbdo.boolData() ? HIGH : LOW);
    Serial.printf("LED Light: %s\n", fbdo.boolData() ? "ON" : "OFF");
  } else {
    handleFirebaseError();
  }
}

void controlStepper() {
  int operationTime;
  
  // Check for motorOperationTime value in Firebase
  if (Firebase.RTDB.getInt(&fbdo, "/control/motorOperationTime")) {
    operationTime = fbdo.intData();
    if (operationTime != lastOperationTime) {
      int steps = STEPS_FOR_180_DEGREES; // Fixed 180-degree movement
      myStepper.step(steps); // Move the stepper motor 180 degrees
      lastOperationTime = operationTime;
      Serial.printf("Stepper moved 180 degrees for operation time %d hours\n", operationTime);
    }
  } else {
    handleFirebaseError();
  }
}

void updateFirebase(const String &path, float value) {
  if (Firebase.RTDB.setFloat(&fbdo, path.c_str(), value)) {
    Serial.printf("%s updated successfully\n", path.c_str());
  } else {
    handleFirebaseError();
  }
}

void handleFirebaseError() {
  Serial.printf("Firebase Error: %s\n", fbdo.errorReason().c_str());
}
