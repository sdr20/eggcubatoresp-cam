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
    // Optionally reset and try again, or you could put it to deep sleep
    ESP.restart();
  } else {
    // Display "WiFi Connected" on the LCD after connecting
    displayWiFiConnected();
    Serial.println("Connected to WiFi successfully");

    // Display the welcome message
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
}

void loop() {
  // Read humidity and temperature from DHT sensor
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
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

  // Control the relays based on the temperature
  if (t < 30) {
    digitalWrite(RELAY1_PIN, HIGH);
    Serial.println("Temperature below 30°C. RELAY1 activated.");
  } else {
    digitalWrite(RELAY1_PIN, LOW);
    Serial.println("Temperature above 30°C. RELAY1 deactivated.");
  }

  if (t < 35) {
    digitalWrite(RELAY2_PIN, HIGH);
    Serial.println("Temperature below 35°C. RELAY2 activated.");
  } else {
    digitalWrite(RELAY2_PIN, LOW);
    Serial.println("Temperature above 35°C. RELAY2 deactivated.");
  }

  if (t < 37) {
    digitalWrite(RELAY3_PIN, LOW);
    Serial.println("Temperature below 37°C. RELAY3 deactivated.");
  } else {
    digitalWrite(RELAY3_PIN, HIGH);
    Serial.println("Temperature above 37°C. RELAY3 activated.");
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

  // Retrieve LED light control from Firebase
  if (Firebase.RTDB.getBool(&fbdo, "/light")) {
    digitalWrite(LED_LIGHT_PIN, fbdo.boolData() ? HIGH : LOW);
    Serial.print("LED Light state: ");
    Serial.println(fbdo.boolData() ? "ON" : "OFF");
  } else {
    Serial.print("Failed to get LED light state: ");
    Serial.println(fbdo.errorReason());
  }

  delay(2000); // Delay for 2 seconds before the next loop iteration
}
