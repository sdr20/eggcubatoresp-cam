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
#define RELAY4_PIN 13 // Relay 4

// LED light relay pin
#define LED_LIGHT_PIN 33

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long lastTimeChecked = 0; // Keeps track of last time we checked Firebase
unsigned long relay4StartMillis = 0; // Start time when Relay 4 turns on
unsigned long hoursToWait = 0; // Time (in hours) fetched from Firebase
unsigned long relayCycleStartMillis = 0; // Start time for waiting period
bool relay4Active = false; // State of Relay 4

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
  pinMode(RELAY4_PIN, OUTPUT); // Initialize Relay 4

  // Initialize the LED light pin
  pinMode(LED_LIGHT_PIN, OUTPUT);

  // Ensure all relays and LED light are off at startup
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW); // Relay 4 off initially
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

  // Relay 1 and Relay 2 are always ON
  digitalWrite(RELAY1_PIN, HIGH);
  Serial.println("Relay 1 is always ON.");

  digitalWrite(RELAY2_PIN, HIGH);
  Serial.println("Relay 2 is always ON.");

  // Relay 3 turns ON when temperature <= 37°C and OFF when >= 38°C
  if (t >= 38) {
    digitalWrite(RELAY3_PIN, LOW);
    Serial.println("Temperature >= 38°C. Relay 3 OFF.");
  } else if (t <= 37) {
    digitalWrite(RELAY3_PIN, HIGH);
    Serial.println("Temperature <= 37°C. Relay 3 ON.");
  }

  // Get the current time in milliseconds
  unsigned long currentMillis = millis();

  // Every 10 seconds, check for a new value from Firebase
  if (currentMillis - lastTimeChecked >= 10000) {
    if (Firebase.RTDB.getInt(&fbdo, "/motorOperationTime")) {
      hoursToWait = fbdo.intData(); // Get the hours set in Firebase
      Serial.print("Fetched hours from Firebase: ");
      Serial.println(hoursToWait);
      relayCycleStartMillis = currentMillis; // Reset the cycle timer
    } else {
      Serial.print("Failed to retrieve hours from Firebase: ");
      Serial.println(fbdo.errorReason());
    }
    lastTimeChecked = currentMillis; // Reset check time
  }

  // Check if it's time to turn on Relay 4
  if (!relay4Active && currentMillis - relayCycleStartMillis >= hoursToWait * 3600000UL) {
    // It's time to activate Relay 4
    relay4StartMillis = currentMillis;
    digitalWrite(RELAY4_PIN, HIGH); // Turn on Relay 4
    relay4Active = true;
    Serial.println("Relay 4 activated for 4 seconds.");
  }

  // Turn off Relay 4 after 4 seconds
  if (relay4Active && currentMillis - relay4StartMillis >= 4000) {
    digitalWrite(RELAY4_PIN, LOW); // Turn off Relay 4
    relay4Active = false;
    relayCycleStartMillis = currentMillis; // Reset the cycle timer
    Serial.println("Relay 4 deactivated, waiting for the next cycle.");
  }

  delay(1000); // Short delay to avoid spamming the loop
}
