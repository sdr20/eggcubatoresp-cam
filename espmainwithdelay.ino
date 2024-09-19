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

// Variables for Relay 4 Timer logic
unsigned long relay4TimerDuration = 0; // Store the timer duration from Firebase
unsigned long relay4StartMillis = 0;   // Store when the relay 4 timer started
bool relay4Active = false;             // State tracking if relay 4 is active
bool relay4OnState = false;            // Track if relay 4 is in ON state for 8 seconds

// State variables to manage sequential activation
bool lcdInitialized = false;
bool dhtInitialized = false;
bool relaysInitialized = false;

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
  delay(2000); // Display welcome message for 3 seconds
}

void setup() {
  Serial.begin(115200);

  // Initialize the LCD, but delay before proceeding to next steps
  lcd.begin(16, 2);
  lcd.setBacklight(255);
  lcdInitialized = true;  // Mark LCD as initialized
  displayConnectToWiFi();
  delay(3000); // Delay 3 seconds to ensure sequential operation

  // Initialize WiFiManager
  WiFiManager wm;
  bool res;

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

  delay(2000); // Delay before initializing the DHT sensor

  // Initialize the DHT sensor
  dht.begin();
  dhtInitialized = true;  // Mark DHT as initialized

  delay(2000); // Delay before initializing the relays

  // Initialize the relay pins
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT); // Initialize RELAY4
  pinMode(LED_LIGHT_PIN, OUTPUT); // Initialize LED light relay

  // Ensure all relays and LED light are off at startup
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, HIGH); // Relay 3 ON by default
  digitalWrite(RELAY4_PIN, LOW); // RELAY4 off initially
  digitalWrite(LED_LIGHT_PIN, LOW);
  relaysInitialized = true;  // Mark relays as initialized
}

void loop() {
  if (lcdInitialized) {
    Serial.println("LCD Initialized");
  }
  
  if (dhtInitialized) {
    Serial.println("DHT Initialized");
  }
  
  if (relaysInitialized) {
    Serial.println("Relays Initialized");
  }

  // Existing logic for DHT sensor reading, relay control, etc.

  delay(2000); // Delay to prevent flooding the loop
}
