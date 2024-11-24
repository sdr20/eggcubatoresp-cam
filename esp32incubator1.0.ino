#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <DHT.h>
#include <WiFiManager.h>

// Firebase configuration
#define FIREBASE_HOST "itlog-database-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_API_KEY "AIzaSyBlob6OB0gXhG7JTno7zY_mTFhHQkzVI3g"

// Define relay pin definitions
#define RELAY1_PIN 27
#define RELAY2_PIN 14
#define RELAY3_PIN 12
#define RELAY4_PIN 13

// Initialize the DHT sensor
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Initialize the LCD
LiquidCrystal_PCF8574 lcd(0x27);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variables for Relay 4 Timer logic
unsigned long relay4TimerDuration = 0; 
unsigned long relay4StartMillis = 0;   
bool relay4Active = false;               
bool relay4OnState = false;              

unsigned long previousMillis = 0;
const long updateInterval = 10000; // Data upload interval (10 seconds)

// WiFi reconnection checker
unsigned long lastWiFiCheckMillis = 0;
const unsigned long wifiCheckInterval = 30000; // Check WiFi every 30 seconds
const unsigned long wifiConnectTimeout = 60000; // 60 seconds timeout for WiFi connection

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

bool connectToWiFi(WiFiManager &wm) {
  unsigned long startAttemptTime = millis();
  bool res = wm.autoConnect("Eggcubator-WM", "admin123");

  while (!res && millis() - startAttemptTime < wifiConnectTimeout) {
    Serial.println("Attempting to connect to WiFi...");
    delay(1000);
    res = wm.autoConnect("Eggcubator-WM", "admin123");
  }

  return res;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  i2cScanner();

  lcd.begin(16, 2);  
  lcd.setBacklight(255);  
  displayConnectToWiFi();

  WiFiManager wm;
  if (!connectToWiFi(wm)) {
    Serial.println("Failed to connect to WiFi, restarting...");
    ESP.restart();
  } else {
    displayWiFiConnected();
    Serial.println("Connected to WiFi successfully");
    displayWelcomeMessage();
  }

  config.api_key = FIREBASE_API_KEY;
  config.database_url = FIREBASE_HOST;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  dht.begin();

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);

  digitalWrite(RELAY1_PIN, HIGH);  
  digitalWrite(RELAY2_PIN, LOW);   
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);   
}

void loop() {
  unsigned long currentMillis = millis();

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
    delay(2000);
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

  // Control Relay 2 based on Firebase "light" data
  if (Firebase.RTDB.getBool(&fbdo, "/light")) {
    digitalWrite(RELAY2_PIN, fbdo.boolData() ? HIGH : LOW);
  }

  if (t <= 37) {
    digitalWrite(RELAY3_PIN, LOW); // Turn ON relay 3
  } else if (t > 38) {
    digitalWrite(RELAY3_PIN, HIGH); // Turn OFF relay 3
  }

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
}