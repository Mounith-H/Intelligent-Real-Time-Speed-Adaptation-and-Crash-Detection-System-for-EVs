#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

const int crashPin = PB15;   // Output pin to ESP32
const int ledPin = PC13;    // Onboard LED for crash indication
const float crashThreshold =4 ;  // Lowered threshold for high sensitivity
const int numSamples = 5;  // Number of readings for averaging
float accelHistory[numSamples] = {0};  // Store previous acceleration values
int historyIndex = 0;
const int externalLedPin = PC15;  // External LED pin

void setup() {
  pinMode(crashPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(externalLedPin, OUTPUT);
  digitalWrite(crashPin, LOW);  // Initialize pin to LOW
  digitalWrite(ledPin, HIGH);   // Turn LED off (Active LOW on STM32)
  digitalWrite(externalLedPin, LOW);  // Initialize external LED to OFF

  Serial.begin(9600);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050.");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);  // Set max sensitivity
  Serial.println("MPU6050 initialized with high sensitivity.");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Compute acceleration magnitude
  float accelMagnitude = sqrt(a.acceleration.x * a.acceleration.x + 
                              a.acceleration.y * a.acceleration.y + 
                              a.acceleration.z * a.acceleration.z);

  // Store in history for smoothing
  accelHistory[historyIndex] = accelMagnitude;
  historyIndex = (historyIndex + 1) % numSamples;

  // Compute the moving average
  float avgAccel = 0;
  for (int i = 0; i < numSamples; i++) {
    avgAccel += accelHistory[i];
  }
  avgAccel /= numSamples;

  // Detect crash: sudden spike in acceleration
  if (accelMagnitude - avgAccel > crashThreshold) {
    digitalWrite(crashPin, HIGH);
    digitalWrite(ledPin, LOW);         // Turn onboard LED ON (Active LOW)
    digitalWrite(externalLedPin, HIGH); // Turn external LED ON
    Serial.println("🚨 Crash detected! Sending alert to ESP32...");
    delay(4000);  // Keep signals HIGH for 4 seconds
    digitalWrite(crashPin, LOW);
    digitalWrite(ledPin, HIGH);        // Turn onboard LED OFF
    digitalWrite(externalLedPin, LOW);  // Turn external LED OFF
  }

  delay(50);  // Faster sampling for better response
}
