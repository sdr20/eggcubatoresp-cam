#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ESP32Servo.h>
#include <WiFiManager.h>

// Firebase configuration
#define FIREBASE_HOST "itlog-database-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_API_KEY "AIzaSyBlob6OB0gXhG7JTno7zY_mTFhHQkzVI3g"

// Define constants
#define SERVO_PAN_PIN 18
#define SERVO_TILT_PIN 19

// Initialize peripherals
Servo panServo, tiltServo;

// Firebase and WiFi objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
WiFiManager wm;

// State variables
int lastPanAngle = -1;
int lastTiltAngle = -1;
bool signupOK = false;

// Function prototypes
void connectToWiFi();
void initializeFirebase();
void controlServos();
void handleFirebaseError();

void setup() {
  Serial.begin(115200);
  connectToWiFi();
  initializeFirebase();

  panServo.attach(SERVO_PAN_PIN);
  tiltServo.attach(SERVO_TILT_PIN);
}

void loop() {
  controlServos();
  delay(2000);
}

void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  if (wm.autoConnect("Servo_AP", "admin123")) {
    Serial.println("WiFi Connected");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to WiFi");
    ESP.restart();
  }
}

void initializeFirebase() {
  config.api_key = FIREBASE_API_KEY;
  config.database_url = FIREBASE_HOST;

  if (Firebase.signUp(&config, &auth, "", "")) {
    signupOK = true;
  } else {
    Serial.printf("Firebase Sign-Up Error: %s\n", config.signer.signupError.message.c_str());
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
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
}

void handleFirebaseError() {
  Serial.printf("Firebase Error: %s\n", fbdo.errorReason().c_str());
}
