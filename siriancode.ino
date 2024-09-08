#include <Stepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// Define the number of steps per revolution for your motor
const int stepsPerRevolution = 2048; // Typically for 28BYJ-48
const int stepsToMove = 1024; // Number of steps to move

// Initialize the stepper library on pins 8 through 11 (adjust for ESP32)
Stepper myStepper(stepsPerRevolution, 26, 25, 33, 32);

// Initialize the DHT sensor
#define DHTPIN 4     // Pin connected to the data pin of the DHT sensor (adjust for ESP32)
#define DHTTYPE DHT11   // DHT 22 (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

// Initialize the LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // Change the I2C address if needed

// Relay pin definitions
#define RELAY1_PIN 27  // Adjust for ESP32
#define RELAY2_PIN 14  // Adjust for ESP32
#define RELAY3_PIN 12

void setup() {
  // Set the speed of the motor (RPM)
  myStepper.setSpeed(15); // Adjust the speed as needed
  // Initialize the serial communication for debugging
  Serial.begin(115200);
  Serial.println("Stepper motor control");

  // Initialize the DHT sensor
  dht.begin();

  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.print("ITlog Incubator");

  // Initialize the relay pins
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  

  // Ensure the relays are off at startup
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
}

void loop() {
  // Rotate the motor clockwise for 1024 steps
  Serial.println("Clockwise");
  myStepper.step(stepsToMove);
  delay(1000); // Wait for a second

  // Read temperature and humidity from DHT22
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Print the temperature and humidity to the Serial Monitor
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" *C");

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

  // Control the relays based on the temperature
  if (t < 30) {
    digitalWrite(RELAY1_PIN, HIGH);
  } else {
    digitalWrite(RELAY1_PIN, LOW);
  }

  if (t < 35) {
    digitalWrite(RELAY2_PIN, HIGH);
  } else {
    digitalWrite(RELAY2_PIN, LOW);
  }

  if (t <37) {
    digitalWrite(RELAY3_PIN, LOW);
  } else {
    digitalWrite(RELAY3_PIN, HIGH);
  }


  // Rotate the motor counterclockwise for 1024 steps
  Serial.println("Counterclockwise");
  myStepper.step(-stepsToMove);
  delay(10000); // Wait for a second
}