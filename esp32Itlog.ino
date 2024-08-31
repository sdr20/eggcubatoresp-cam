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

// Firebase configuration
#define FIREBASE_HOST "itlog-database-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_API_KEY "AIzaSyBlob6OB0gXhG7JTno7zY_mTFhHQkzVI3g"

// WiFi credentials
#define WIFI_SSID "simonrhain"
#define WIFI_PASSWORD "Sheila*082574"

// Define the number of steps per revolution for your motor
#define STEPS_PER_REVOLUTION 2048
#define STEPS_TO_MOVE 1024

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
LiquidCrystal_I2C lcd(0x27, 16, 2);

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
int count = 0;
bool signupOK = false;
int motorOperationTime = 60000; // Default 60 seconds if not set

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
  lcd.init();
  lcd.backlight();

  // Display "Connecting to WiFi" on the LCD while connecting
  displayConnectingToWiFi();

  // Initialize WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(3000);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Display "Eggcubator" on the LCD after connecting to WiFi
  displayWiFiConnected();
  delay(3000); // Wait for 3 seconds

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Eggcubator");
  delay(1000); // Wait for 3 seconds

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
  if (Firebase.RTDB.getFloat(&fbdo, "/control/motorOperationTime")) {
    motorOperationTime = fbdo.floatData() * 1000; // Convert seconds to milliseconds
    Serial.print("Motor Operation Time: ");
    Serial.println(motorOperationTime);
  } else {
    Serial.print("Failed to get motorOperationTime: ");
    Serial.println(fbdo.errorReason());
  }

  // Control stepper motor based on the retrieved motor operation time
  if (millis() - motorStartMillis > motorOperationTime) {
    Serial.println("Clockwise");
    myStepper.step(STEPS_TO_MOVE);
    motorStartMillis = millis(); // Reset the motor start time
  }

  // Read humidity and temperature from DHT sensor
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
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
    float maxTemperature = fbdo.floatData();
    Serial.print("Max Temperature: ");
    Serial.println(maxTemperature);
  } else {
    Serial.print("Failed to get maxTemperature: ");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.getFloat(&fbdo, "/control/motorOperationTime")) {
    float motorOperationTime = fbdo.floatData();
    Serial.print("Motor Operation Time: ");
    Serial.println(motorOperationTime);
  } else {
    Serial.print("Failed to get motorOperationTime: ");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.getBool(&fbdo, "/control/fanOn")) {
    bool fanOn = fbdo.boolData();
    Serial.print("Fan On: ");
    Serial.println(fanOn);
  } else {
    Serial.print("Failed to get fanOn status: ");
    Serial.println(fbdo.errorReason());
  }

  // Control relays based on temperature
  digitalWrite(RELAY1_PIN, t > 30.0 ? HIGH : LOW);
  digitalWrite(RELAY2_PIN, t > 40.0 ? HIGH : LOW);
  digitalWrite(RELAY3_PIN, t > 50.0 ? HIGH : LOW);

  // Control the LED light based on time of day or other conditions
  // (You can customize this based on your requirements)
  if (t > 25.0) {
    digitalWrite(LED_LIGHT_PIN, HIGH);
  } else {
    digitalWrite(LED_LIGHT_PIN, LOW);
  }

  // Retrieve pan servo angle from Firebase
  if (Firebase.RTDB.getInt(&fbdo, "/servos/pan")) {
    int panAngle = fbdo.intData();
    if (panAngle != lastPanAngle && panAngle >= 0 && panAngle <= 180) { // Only move if different and within bounds
      panServo.write(panAngle);
      lastPanAngle = panAngle;
      Serial.print("Pan Servo Angle: ");
      Serial.println(panAngle);
    }
  } else {
    Serial.print("Failed to get pan_servo_angle: ");
    Serial.println(fbdo.errorReason());
  }

  // Retrieve tilt servo angle from Firebase
  if (Firebase.RTDB.getInt(&fbdo, "/servos/tilt")) {
    int tiltAngle = fbdo.intData();
    if (tiltAngle != lastTiltAngle && tiltAngle >= 0 && tiltAngle <= 180) { // Only move if different and within bounds
      tiltServo.write(tiltAngle);
      lastTiltAngle = tiltAngle;
      Serial.print("Tilt Servo Angle: ");
      Serial.println(tiltAngle);
    }
  } else {
    Serial.print("Failed to get tilt_servo_angle: ");
    Serial.println(fbdo.errorReason());
  }

  delay(2000);
}
