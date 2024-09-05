#include "esp_camera.h"
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <FirebaseFS.h>
#include "mbedtls/base64.h"

// Your network credentials
#define WIFI_SSID "simonrhain"
#define WIFI_PASSWORD "Sheila*082574"

// Your Firebase project API key and URL
#define FIREBASE_HOST "https://itlog-database-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_API_KEY "AIzaSyBlob6OB0gXhG7JTno7zY_mTFhHQkzVI3g"

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Timer for capturing image every 4 hours (14400000 milliseconds)
unsigned long lastCaptureMillis = 0;
const unsigned long captureInterval = 14400000;

// Camera setup for AI Thinker model
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

void setup() {
  Serial.begin(115200);

  // Initialize Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Firebase configuration
  config.api_key = FIREBASE_API_KEY;
  config.database_url = FIREBASE_HOST;

  // Initialize Firebase with config and auth
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Ensure Firebase setup is successful
  if (!Firebase.ready()) {
    Serial.println("Failed to connect to Firebase");
    return;
  }

  // Initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Capture the first image immediately
  captureAndUploadImage();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check if 4 hours have passed since the last capture
  if (currentMillis - lastCaptureMillis >= captureInterval) {
    captureAndUploadImage();
    lastCaptureMillis = currentMillis; // Update the capture time
  }

  delay(1000);
}

String base64_encode(uint8_t *data, size_t length) {
  // Calculate the output buffer size needed for base64 encoding
  size_t encoded_length = 4 * ((length + 2) / 3);
  char *encoded_data = (char *)malloc(encoded_length + 1);  // +1 for null terminator

  size_t actual_length;
  mbedtls_base64_encode((unsigned char *)encoded_data, encoded_length + 1, &actual_length, data, length);

  String result = String(encoded_data);
  free(encoded_data);  // Free the buffer memory
  return result;
}

void captureAndUploadImage() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Convert the image data to a base64 string
  String base64Image = "data:image/jpeg;base64," + base64_encode(fb->buf, fb->len);

  // Get the current timestamp
  String timestamp = String(millis());

  // Upload the image to Firebase Realtime Database
  if (Firebase.RTDB.setString(&fbdo, "/camera/images/" + timestamp, base64Image)) {
    Serial.println("Image uploaded successfully");
  } else {
    Serial.println("Image upload failed: " + fbdo.errorReason());
  }

  // Return the frame buffer to the driver for reuse
  esp_camera_fb_return(fb);
}
