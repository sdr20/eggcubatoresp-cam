#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h> // Updated LCD library for ESP32
#include <DHT.h>
#include <WiFiManager.h>
#include <esp_task_wdt.h> // Include watchdog library

// Firebase configuration
#define FIREBASE_HOST "itlog-database-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_API_KEY "AIzaSyBlob6OB0gXhG7JTno7zY_mTFhHQkzVI3g"

// Define relay pin definitions
#define RELAY1_PIN 27
#define RELAY2_PIN 14
#define RELAY3_PIN 12
#define RELAY4_PIN 13 // Added RELAY4_PIN

// Initialize the DHT sensor
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Initialize the LCD
LiquidCrystal_PCF8574 lcd(0x27); // Updated for ESP32

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variables for Relay 4 Timer logic
unsigned long relay4TimerDuration = 0; // Store the timer duration from Firebase
unsigned long relay4StartMillis = 0;   // Store when the relay 4 timer started
bool relay4Active = false;             // State tracking if relay 4 is active
bool relay4OnState = false;            // Track if relay 4 is in ON state for 8 seconds

unsigned long previousMillis = 0;
const long updateInterval = 10000; // Data upload interval (10 seconds)

// WiFi reconnection checker
unsigned long lastWiFiCheckMillis = 0;
const unsigned long wifiCheckInterval = 30000; // Check WiFi every 30 seconds

void i2cScanner() {
  Serial.println("\nI2C Scanner");
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
}

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

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.reconnect();
    displayConnectingToWiFi();
  } else {
    displayWiFiConnected();
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin();

  // Run I2C scanner to detect devices
  i2cScanner();

  // Initialize the LCD
  lcd.begin(16, 2);  // 16 columns, 2 rows
  lcd.setBacklight(255);  // Ensure the backlight is ON
  displayConnectToWiFi();

  // Initialize WiFiManager
  WiFiManager wm;
  bool res;

  // Start the WiFi connection process
  res = wm.autoConnect("Eggcubator-WM", "admin123");

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

  // Initialize the DHT sensor
  dht.begin();

  // Initialize the relay pins
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT); // Initialize RELAY4

  // Ensure relays are off at startup except RELAY1 which is always ON
  digitalWrite(RELAY1_PIN, HIGH);  // Relay 1 always ON
  digitalWrite(RELAY2_PIN, LOW);   // Relay 2 starts OFF
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);   // RELAY4 off initially

  // Watchdog timer configuration
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = 60000, // Timeout in milliseconds (60 seconds)
    .trigger_panic = true // Panic on timeout
  };
  esp_task_wdt_init(&wdtConfig);   // Initialize watchdog timer with the configuration
  esp_task_wdt_add(NULL);       // Add the current task (loop) to the WDT
}

void loop() {
  unsigned long currentMillis = millis();

  // Check WiFi connection every wifiCheckInterval
  if (currentMillis - lastWiFiCheckMillis >= wifiCheckInterval) {
    lastWiFiCheckMillis = currentMillis;
    checkWiFiConnection();
  }

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

  // Upload temperature and humidity data to Firebase every updateInterval
  if (currentMillis - previousMillis >= updateInterval) {
    previousMillis = currentMillis;

    if (Firebase.RTDB.setFloat(&fbdo, "/sensor_data/temperature", t)) {
      Serial.println("Temperature data uploaded");
    } else {
      Serial.println("Failed to upload temperature data, retrying...");
      delay(1000);
    }

    if (Firebase.RTDB.setFloat(&fbdo, "/sensor_data/humidity", h)) {
      Serial.println("Humidity data uploaded");
    } else {
      Serial.println("Failed to upload humidity data, retrying...");
      delay(1000);
    }
  }

  // Relay logic
  digitalWrite(RELAY1_PIN, HIGH); // Relay 1 is always ON

  // Control Relay 2 based on Firebase "light" data (true = ON, false = OFF)
  if (Firebase.RTDB.getBool(&fbdo, "/light")) {
    digitalWrite(RELAY2_PIN, fbdo.boolData() ? HIGH : LOW);
  }

  // Relay 3 Logic (fan control based on temperature)
  digitalWrite(RELAY3_PIN, (t >= 38) ? LOW : HIGH);

  // Relay 4 Timer Logic
  if (Firebase.RTDB.getInt(&fbdo, "/control/motorOperationTime")) {
    relay4TimerDuration = fbdo.intData() * 3600000UL; // Convert hours to milliseconds
  }
  
  if (!relay4Active && relay4TimerDuration > 0) {
    relay4Active = true;
    relay4StartMillis = currentMillis;
    digitalWrite(RELAY4_PIN, HIGH); // Turn Relay 4 ON
    relay4OnState = true;
  }

  if (relay4Active && (currentMillis - relay4StartMillis >= relay4TimerDuration)) {
    digitalWrite(RELAY4_PIN, LOW); // Turn Relay 4 OFF
    relay4Active = false; // Reset the active state
  }

  // Reset the watchdog timer to prevent ESP32 from resetting
  esp_task_wdt_reset();
}
