  #include "BluetoothSerial.h"

BluetoothSerial SerialBT;  // Create a Bluetooth Serial object

const int pinTrigger = 2;    // Pin to check for voltage
const int potPin = 4;        // Potentiometer input pin (D4)
const int motorPin = 5;      // Motor control output pin (D5)

// Variables for speed control
int currentSpeedLimit = 150;  // Default max speed limit (150 kmph)
int throttleValue = 0;        // To store potentiometer reading
int mappedMotorSpeed = 0;     // Final motor speed after applying limit

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT");  // Bluetooth device name
  
  pinMode(pinTrigger, INPUT);   // Set the pin to input mode for detecting voltage
  pinMode(potPin, INPUT);       // Set potentiometer pin as input
  pinMode(motorPin, OUTPUT);    // Set motor control pin as output
  
  Serial.println("Bluetooth Started! Ready to receive speed limit and control motor.");
}

void loop() {
  // Read throttle value from potentiometer (0-4095)
  throttleValue = analogRead(potPin);
  
  // Map throttle value to speed range (0-150 kmph)
  int requestedSpeed = map(throttleValue, 0, 4095, 0, 150);
  
  // Limit the speed based on received speed limit
  int actualSpeed = min(requestedSpeed, currentSpeedLimit);
  
  // Map the limited speed back to PWM range (0-255)
  mappedMotorSpeed = map(actualSpeed, 0, 150, 0, 255);
  
  // Output the PWM signal to motor driver
  analogWrite(motorPin, mappedMotorSpeed);

  // Print debug information
  Serial.print("Throttle: ");
  Serial.print(requestedSpeed);
  Serial.print(" kmph, Limited to: ");
  Serial.print(actualSpeed);
  Serial.print(" kmph, PWM: ");
  Serial.println(mappedMotorSpeed);

  // Check if pinTrigger gets voltage
  if (digitalRead(pinTrigger) == HIGH) {
    SerialBT.println("1");
    Serial.println("Trigger sent to app.");
  }

  // Check if the app sends speed limit data
  if (SerialBT.available()) {
    String receivedData = SerialBT.readStringUntil('\n');
    
    // Convert received string to integer
    currentSpeedLimit = receivedData.toInt();
    
    Serial.print("Received Speed Limit: ");
    Serial.println(currentSpeedLimit);

    // Send acknowledgment back to Android app
    SerialBT.println("Speed Limit Received: " + receivedData + " km/h");
  }
  
  // Small delay to prevent overwhelming the serial output
  delay(100);
}
