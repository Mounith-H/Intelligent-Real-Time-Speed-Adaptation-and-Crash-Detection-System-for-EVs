#include <Arduino.h>
#include "BluetoothSerial.h"
#include <LiquidCrystal_I2C.h>

// Pin definitions
#define HALL_PIN GPIO_NUM_34    // Pin of Hall effect sensor to track speed
#define POT_PIN GPIO_NUM_4      // Potentiometer input pin (D4)
#define MOTOR_PIN GPIO_NUM_15   // Motor control output pin (D5)
#define CRASH_PIN GPIO_NUM_13    // Pin to check for voltage (trigger pin)

// Motor and speed calculation constants
#define WHEEL_DIAMETER_CM 7.0   // Wheel diameter in cm
#define WHEEL_CIRCUMFERENCE_M (WHEEL_DIAMETER_CM * 3.14159 / 100.0)  // Wheel circumference in meters
#define MAGNETS_COUNT 2         // Number of magnets on the wheel
#define PWM_FREQUENCY 5000      // PWM frequency in Hz
#define PWM_RESOLUTION 8        // PWM resolution (8-bit = 0-255)
#define PWM_CHANNEL 0           // PWM channel
#define MAX_SPEED_LIMIT 150     // Maximum speed limit in km/h for trottle control

// Variables for speed calculation
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long timeBetweenPulses = 0;
volatile bool newPulseReceived = false;
unsigned long lastSpeedCalcTime = 0;
volatile float currentRPM = 0.0;
volatile float currentKMPH = 0.0;
bool lastHallState = false;
int motorPWM = 0;  // Variable to store motor PWM value
bool updatedisplay = false;  // Flag to indicate if display needs update

bool crashDetected = false;  // Flag for crash detection
int currentSpeedLimit = 150;  // Default max speed limit (150 kmph)

int oldTrottle;
int oldcurrentRPM;
int oldcurrentKMPH;
int oldcurrentSpeedLimit;

BluetoothSerial SerialBT;  // Create a Bluetooth Serial object
LiquidCrystal_I2C lcd(0x27, 16, 2); // Initialize LCD with I2C address 0x27, 16 columns and 2 rows

// Interrupt Service Routine for Hall effect sensor
void IRAM_ATTR hallSensorISR() {
    unsigned long currentTime = micros();
    
    // Only store the time difference, don't do calculations in ISR
    if (lastPulseTime > 0) {
        timeBetweenPulses = currentTime - lastPulseTime;
        newPulseReceived = true;
    }
    
    lastPulseTime = currentTime;
    pulseCount++;
}

// Interrupt Service Routine for Crash detection sensor
void IRAM_ATTR crashDetectionISR() {
    crashDetected = true;  // Set crash detected flag
}

void lcdinit() {
    lcd.init();
    lcd.backlight();  // Turn on the backlight
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");
    delay(1000);  // Wait for LCD to initialize
    lcd.clear();

    lcd.setCursor(0, 0);    lcd.print("T:    "); // Clear previous value
    lcd.setCursor(2, 0);    lcd.print("0");    lcd.print("%");
    lcd.setCursor(8, 0);    lcd.print("R:       ");  //Clear previous value
    lcd.setCursor(10, 0);   lcd.print("0");
    lcd.setCursor(8, 1);    lcd.print("C:       "); // Clear previous value
    lcd.setCursor(10, 1);   lcd.print("0");    lcd.print("km");
    lcd.setCursor(0, 1);    lcd.print("L:      "); // Clear previous value
    lcd.setCursor(2, 1);    lcd.print("0");   lcd.print("km");
}

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_BT");  // Bluetooth device name

    // LCD initialization
    lcdinit();

    // Configure PWM for motor control
    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PIN, PWM_CHANNEL);

    pinMode(HALL_PIN, INPUT_PULLUP);  // Set Hall effect sensor pin as input with pullup
    pinMode(POT_PIN, INPUT);          // Set potentiometer pin as input
    pinMode(CRASH_PIN, INPUT_PULLUP); // Set crash detection pin as input with pullup

    // Attach interrupt to Hall effect sensor
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallSensorISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CRASH_PIN), crashDetectionISR, FALLING);

    lastSpeedCalcTime = millis();
    
    Serial.println("System initialized - Motor control with speed monitoring active");
}

void calculateSpeed() {
    // Check if we have a new pulse to process
    if (newPulseReceived) {
        newPulseReceived = false;
        
        // Only calculate if time difference is reasonable (avoid noise)
        if (timeBetweenPulses > 1000) { // Minimum 1ms between pulses
            // Calculate RPM from time between pulses
            // Time for one revolution = timeBetweenPulses * MAGNETS_COUNT
            float timePerRevolution = (timeBetweenPulses * MAGNETS_COUNT) / 1000000.0; // Convert to seconds
            
            if (timePerRevolution > 0) {
                currentRPM = 60.0 / timePerRevolution; // RPM = 60 / time_per_revolution_in_seconds
                
                // Calculate speed in km/h
                currentKMPH = ((currentRPM * WHEEL_CIRCUMFERENCE_M * 60.0) / 1000.0) * 2;
            }
        }
    }
    
    // Check if no pulses for more than 2 seconds, assume stopped
    unsigned long currentTime = millis();
    if (currentTime - (lastPulseTime / 1000) > 2000) {
        currentRPM = 0.0;
        currentKMPH = 0.0;
    }
}

void controlMotor() {
    // Read potentiometer value (0-4095 for 12-bit ADC)
    int potValue = analogRead(POT_PIN);
    
    // Map potentiometer value to desired speed (0-currentSpeedLimit km/h)
    float desiredSpeed = map(potValue, 0, 4095, 0, MAX_SPEED_LIMIT);
    
    // Ensure desired speed doesn't exceed speed limit
    if (desiredSpeed > currentSpeedLimit) {
        desiredSpeed = currentSpeedLimit;
    }
    

    
    // Apply PID output to motor (pidOutput is already limited to 0-255)
    motorPWM = (int)((desiredSpeed/MAX_SPEED_LIMIT) * 255); // Scale desired speed to PWM range (0-255)
    
    // Ensure motor speed doesn't go below 0 or above 255
    if (motorPWM < 0) motorPWM = 0;
    if (motorPWM > 255) motorPWM = 255;
    
    // Set motor speed using PWM
    ledcWrite(PWM_CHANNEL, motorPWM);
}

void sendDataToSerial() {
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
    
    // Send data every 1 second
    if (currentTime - lastPrintTime >= 1000) {
        int potValue = analogRead(POT_PIN);
        float desiredSpeed = map(potValue, 0, 4095, 0, MAX_SPEED_LIMIT);
        
        float motorSpeedPercent = (motorPWM / 255.0) * 100.0;
        
        // Print to Serial Monitor
        Serial.println("=== Vehicle Status ===");
        Serial.printf("Potentiometer: %d (%.1f%%)\n", potValue, (potValue/4095.0)*100);
        Serial.printf("Desired Speed: %.2f km/h\n", desiredSpeed);
        Serial.printf("Current Speed: %.2f km/h\n", currentKMPH);
        Serial.printf("Motor PWM: %d/255 (%.1f%%)\n", motorPWM, motorSpeedPercent);
        Serial.printf("RPM: %.2f\n", currentRPM);
        Serial.printf("Speed Limit: %d km/h\n", currentSpeedLimit);
        Serial.printf("Pulse Count: %lu\n", pulseCount);
        Serial.println("=====================");
        
        lastPrintTime = currentTime;
    }
}

void bluetooth(){
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
}

void crashSend(){
    if(crashDetected){
        SerialBT.println("1");
        Serial.println("Trigger sent to app.");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("CRASH DETECTED!");
        lcd.clear();
        updatedisplay = true; // Set flag to update display
        
        // Reset crash detection flag
        crashDetected = false;
        
        // Optionally, stop the motor
        ledcWrite(PWM_CHANNEL, 0);  // Stop the motor
    }
}

void lcdUpdate() {
    // This function can be used to update an LCD display

    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
    int potValue = analogRead(POT_PIN);
    float Trottle = map(potValue, 0, 4095, 0, 100);

    if(Trottle != oldTrottle){
        lcd.setCursor(0, 0);    lcd.print("T:    "); // Clear previous value
        lcd.setCursor(2, 0);    lcd.print((int)Trottle);    lcd.print("%");
        oldTrottle = Trottle;
    }

    if(updatedisplay)
    {
        lcd.setCursor(0, 0);    lcd.print("T:    "); // Clear previous value
        lcd.setCursor(2, 0);    lcd.print((int)Trottle);    lcd.print("%");
        lcd.setCursor(8, 0);    lcd.print("R:       ");  //Clear previous value
        lcd.setCursor(10, 0);   lcd.print((int)currentRPM);
        lcd.setCursor(8, 1);    lcd.print("C:       "); // Clear previous value
        lcd.setCursor(10, 1);   lcd.print((int)currentKMPH);    lcd.print("km");
        lcd.setCursor(0, 1);    lcd.print("L:      "); // Clear previous value
        lcd.setCursor(2, 1);    lcd.print((int)currentSpeedLimit);   lcd.print("km");
        updatedisplay = false;
    }
    // update LCD every 1 second
    if (currentTime - lastPrintTime >= 1000) {
    
        if(currentRPM != oldcurrentRPM){
            lcd.setCursor(8, 0);    lcd.print("R:       ");  //Clear previous value
            lcd.setCursor(10, 0);    lcd.print((int)currentRPM);
            oldcurrentRPM = currentRPM;
        }
        if(currentKMPH != oldcurrentKMPH){
            lcd.setCursor(8, 1);    lcd.print("C:       "); // Clear previous value
            lcd.setCursor(10, 1);   lcd.print((int)currentKMPH);    lcd.print("km");
            oldcurrentKMPH = currentKMPH;
        }
        if(currentSpeedLimit != oldcurrentSpeedLimit){
            lcd.setCursor(0, 1);    lcd.print("L:      "); // Clear previous value
            lcd.setCursor(2, 1);    lcd.print((int)currentSpeedLimit);   lcd.print("km");
            oldcurrentSpeedLimit = currentSpeedLimit;
        }
        lastPrintTime = currentTime;
    }
}

void loop() {
    bluetooth();
    calculateSpeed();
    controlMotor();
    sendDataToSerial();
    crashSend();
    lcdUpdate();
    
    // Small delay to prevent overwhelming the system
    delay(10);
}