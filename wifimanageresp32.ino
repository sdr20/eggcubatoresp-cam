#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <Stepper.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h> // Updated LCD library for ESP32
#include <DHT.h>
#include <WiFiManager.h>

// Firebase configuration
#define FIREBASE_HOST "itlog-database-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_API_KEY "AIzaSyBlob6OB0gXhG7JTno7zY_mTFhHQkzVI3g"

// Define the number of steps per revolution for your motor
#define STEPS_PER_REVOLUTION 2048
#define STEPS_TO_MOVE 1024 // 180 degrees

// Initialize the stepper library on pins 26, 25, 33, and 32
Stepper myStepper(STEPS_PER_REVOLUTION, 26, 25, 33, 32);

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
#define RELAY4_PIN 13 // Added RELAY4_PIN

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
unsigned long motorOperationTimeMillis = 3600000; // Default to 1 hour in milliseconds for motor
bool motorMoved = false; // Track motor movement state

unsigned long relay4OperationTime = 0; // Time for how long RELAY4 will stay on
unsigned long relay4StartTime = 0; // To track when RELAY4 was turned on
bool relay4Active = false; // Track state of RELAY4

void displayConnectToWiFi() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Please connect");
  lcd.setCursor(0, 1);
  lcd.print("to WiFi...");
}

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

void displayWelcomeMessage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Welcome to");
  lcd.setCursor(0, 1);
  lcd.print("Eggcubator");
  delay(3000); // Display welcome message for 3 seconds
}

void setup() {
  Serial.begin(115200);

  // Initialize the LCD
  lcd.begin(16, 2);
  lcd.setBacklight(255);

  // Display "Please connect to WiFi" on the LCD
  displayConnectToWiFi();

  // Initialize WiFiManager
  WiFiManager wm;
  bool res;
  
  // Start the WiFi connection process
  res = wm.autoConnect("Eggcubator-WM", "admin123"); // custom AP name

  if (!res) {
    Serial.println("Failed to connect to WiFi");
    ESP.restart();
  } else {
    displayWiFiConnected();
    Serial.println("Connected to WiFi successfully");
    displayWelcomeMessage();
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

  config.token_status_callback = tokenStatusCallback;
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
  pinMode(RELAY4_PIN, OUTPUT); // Initialize RELAY4

  // Initialize the LED light pin
  pinMode(LED_LIGHT_PIN, OUTPUT);

  // Ensure all relays and LED light are off at startup
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW); // RELAY4 off initially
  digitalWrite(LED_LIGHT_PIN, LOW);
}

void loop() {
  // Read humidity and temperature from DHT sensor
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Check if readings are valid
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error");
    lcd.setCursor(0, 1);
    lcd.print("Check Wiring");
    delay(2000); // Delay for 2 seconds before retrying
    return;
  }

  // Display temperature and humidity on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(t);
  lcd.print((char)223); // Degree symbol
  lcd.print("C");

  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(h);
  lcd.print("%");

  // Firebase: Upload temperature and humidity data
  if (Firebase.RTDB.setFloat(&fbdo, "/sensor_data/temperature", t)) {
    Serial.print("Temperature uploaded: ");
    Serial.println(t);
  } else {
    Serial.print("Failed to upload temperature: ");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.setFloat(&fbdo, "/sensor_data/humidity", h)) {
    Serial.print("Humidity uploaded: ");
    Serial.println(h);
  } else {
    Serial.print("Failed to upload humidity: ");
    Serial.println(fbdo.errorReason());
  }

  // Relay 1 and 2 always ON
  digitalWrite(RELAY1_PIN, HIGH);
  Serial.println("RELAY1 is always activated.");

  digitalWrite(RELAY2_PIN, HIGH);
  Serial.println("RELAY2 is always activated.");

  // Relay 3 turns OFF at 38°C and ON at 37°C
  if (t >= 38) {
    digitalWrite(RELAY3_PIN, LOW);
    Serial.println("Temperature >= 38°C. RELAY3 deactivated.");
  } else if (t <= 37) {
    digitalWrite(RELAY3_PIN, HIGH);
    Serial.println("Temperature <= 37°C. RELAY3 activated.");
  }

  // Firebase: Retrieve motor operation time and control RELAY4
  if (Firebase.RTDB.getInt(&fbdo, "/control/motorOperationTime")) {
    relay4OperationTime = fbdo.intData() * 1000UL; // Convert seconds to milliseconds
    Serial.print("Relay 4 Operation Time: ");
    Serial.println(relay4OperationTime);

    // If not already active, start the relay
    if (!relay4Active && relay4OperationTime > 0) {
      relay4StartTime = millis();
      digitalWrite(RELAY4_PIN, HIGH); // Turn on RELAY4
      relay4Active = true;
      Serial.println("RELAY4 activated.");
    }
  } else {
    Serial.print("Failed to retrieve motor operation time: ");
    Serial.println(fbdo.errorReason());
  }

  // Turn off RELAY4 after the set duration
  if (relay4Active && (millis() - relay4StartTime >= relay4OperationTime)) {
    digitalWrite(RELAY4_PIN, LOW); // Turn off RELAY4
    relay4Active = false;
    Serial.println("RELAY4 deactivated.");
  }

  delay(2000); // Delay for 2 seconds before the next loop iteration
}
