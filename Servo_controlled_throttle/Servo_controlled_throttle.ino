#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <EEPROM.h>

// ----- Pin Definitions -----
#define ENCODER_CLK 13      // Rotary Encoder Clock Pin (A)
#define ENCODER_DT 26       // Rotary Encoder Data Pin (B)
#define ENCODER_SW 14       // Rotary Encoder Button Pin (SW)
#define SERVO_PIN 18        // Servo Motor Pin
#define BEEP_PIN 16   // Pin for the beeper

#define BUTTON_1 2          // Additional Button 1 (GPIO 2)
#define BUTTON_2 15         // Additional Button 2 (GPIO 15)

// ----- Servo Setup -----
Servo myServo;            // Servo object
int servoAngle = 90;      // Start angle for the servo (default 90°)
const int minAngle = 0;   // Minimum servo angle (0°)
const int maxAngle = 180; // Maximum servo angle (180°)
const int stepSize = 1;   // Precision movement (1° per step)

// ----- LCD Setup -----
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address (0x27), LCD size (16x2)

// ----- Encoder State -----
volatile int encoderSteps = 0; // To store the number of steps from the encoder
int lastClkState = HIGH;       // To track the last state of the encoder clock pin
bool encoderButtonPressed = false; // Flag to track the encoder button state

// ----- Persistent Zero Value in EEPROM -----
int zeroValue = 0;             // Default zero value for the servo
const int EEPROM_SIZE = 512;   // EEPROM size for ESP32

// ----- Invert Axis Flag -----
bool invertAxis = false; // Flag to track if servo direction is inverted

// EEPROM addresses for storing data
const int ZERO_VALUE_ADDR = 0;
const int INVERT_AXIS_ADDR = 1; // Store the invertAxis flag (1 byte)

// Forward declaration for the interrupt function
void IRAM_ATTR handleEncoder();

void setup() {
  // Begin serial communication for debugging
  Serial.begin(115200);
  Serial.println("Starting ESP32 Servo Controller...");

  // --- Initialize EEPROM ---
  EEPROM.begin(EEPROM_SIZE);   // Initialize EEPROM storage
  zeroValue = EEPROM.read(ZERO_VALUE_ADDR);  // Read the stored zero value
  if (zeroValue == 255) {      // If no value is saved (empty EEPROM)
    zeroValue = 0;             // Set default zero to 0°
  }
  invertAxis = EEPROM.read(INVERT_AXIS_ADDR);  // Read the invert axis setting from EEPROM

  // --- Pin Setup ---
  pinMode(ENCODER_CLK, INPUT_PULLUP); // Encoder clock pin as input with pull-up
  pinMode(ENCODER_DT, INPUT_PULLUP);  // Encoder data pin as input with pull-up
  pinMode(ENCODER_SW, INPUT_PULLUP);  // Encoder button pin as input with pull-up
  pinMode(BUTTON_1, INPUT_PULLUP);    // Button 1 pin as input with pull-up
  pinMode(BUTTON_2, INPUT_PULLUP);    // Button 2 pin as input with pull-up
  pinMode(BEEP_PIN, OUTPUT);           // Beeper pin setup

  // --- Servo Initialization ---
  myServo.setPeriodHertz(50);        // Set servo frequency to 50Hz (standard)
  myServo.attach(SERVO_PIN, 500, 2400); // Attach the servo to pin with pulse width range
  servoAngle = zeroValue;           // Set initial servo position to the stored zero value
  myServo.write(servoAngle);        // Move servo to the initial position

  // --- LCD Initialization ---
  Wire.begin(21, 22); // Initialize I2C with SDA = GPIO21, SCL = GPIO22
  lcd.init();          // Initialize the LCD
  lcd.backlight();     // Turn on the backlight
  lcd.setCursor(0, 0); // Set the cursor to the first row
  lcd.print("Servo Angle:"); // Display message on LCD
  updateLCD();         // Update LCD with the current angle

  // --- Set up Encoder Interrupt ---
  lastClkState = digitalRead(ENCODER_CLK);  // Get the initial state of the clock pin
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), handleEncoder, CHANGE); // Attach interrupt to encoder clock

  // --- Debug Output ---
  Serial.print("Initial angle set to: ");
  Serial.println(servoAngle);
  Serial.print("Invert Axis set to: ");
  Serial.println(invertAxis ? "Yes" : "No");
}

void loop() {
  // --- Process Encoder Movement ---
  if (encoderSteps != 0) { // If the encoder has moved
    int newAngle = servoAngle + (encoderSteps * stepSize); // Calculate the new angle

    // If the axis is inverted, reverse the direction of the movement
    if (invertAxis) {
      newAngle = servoAngle - (encoderSteps * stepSize);
    }

    encoderSteps = 0; // Reset encoder steps after applying

    // Constrain the new angle within the allowable range (0° to 180°)
    newAngle = constrain(newAngle, minAngle, maxAngle);
    if (newAngle != servoAngle) { // If the angle has changed
      servoAngle = newAngle;      // Update servo angle
      myServo.write(servoAngle);  // Move the servo to the new angle
      updateLCD();                // Update the LCD display
      Serial.println("Updated Angle → " + String(servoAngle)); // Print to Serial Monitor
    }
  }

  // --- Handle Encoder Button Press (Reset Servo to "Zero" Value) ---
  if (digitalRead(ENCODER_SW) == LOW && !encoderButtonPressed) { // Button pressed (LOW)
    encoderButtonPressed = true;
    Serial.println("Encoder button pressed - Resetting Servo to stored zero value");
    servoAngle = zeroValue; // Reset to stored zero position
    myServo.write(servoAngle); // Move servo to stored zero position
    updateLCD(); // Update the LCD with the new angle
    
  }

  // --- Handle Encoder Button Release ---
  if (digitalRead(ENCODER_SW) == HIGH && encoderButtonPressed) { // Button released (HIGH)
    encoderButtonPressed = false; // Reset the button press state
  }

  // --- Handle Button 1 Press (Set New "Zero" Value if held for 3 seconds) ---
  static unsigned long button1PressTime = 0;  // Track the time when Button 1 is pressed
  static bool button1Held = false;             // Flag to track if Button 1 is held

  if (digitalRead(BUTTON_1) == LOW) { // Button 1 pressed (LOW)
    if (!button1Held) {  // If Button 1 is just pressed (not held)
      button1PressTime = millis();  // Record the time when pressed
      button1Held = true;           // Mark button as held
    }

    // If Button 1 is held for 3 seconds, set the current angle as the new zero
    if (millis() - button1PressTime >= 3000 && button1Held) {
      zeroValue = servoAngle; // Set the current angle as the new zero
      EEPROM.write(ZERO_VALUE_ADDR, zeroValue); // Save the new zero value to EEPROM
      EEPROM.commit(); // Ensure the value is written to EEPROM
      Serial.println("Button 1 held - New zero value set");
      button1PressTime = millis(); // Reset the button press time after action
      updateLCD(); // Update the LCD with the new "zero" value
      digitalWrite(BEEP_PIN, HIGH);   // Turn on beeper
      delay(500);                     // Wait for 500 milliseconds
      digitalWrite(BEEP_PIN, LOW);    // Turn off beeper
    }
  } else {
    button1Held = false; // Reset button held state when released
  }

  // --- Handle Button 2 Press (Invert Axis if held for 10 seconds) ---
  static unsigned long button2PressTime = 0;  // Track the time when Button 2 is pressed
  static bool button2Held = false;             // Flag to track if Button 2 is held

  if (digitalRead(BUTTON_2) == LOW) { // Button 2 pressed (LOW)
    if (!button2Held) {  // If Button 2 is just pressed (not held)
      button2PressTime = millis();  // Record the time when pressed
      button2Held = true;           // Mark button as held
    }

    // If Button 2 is held for 10 seconds, invert the axis direction
    if (millis() - button2PressTime >= 10000 && button2Held) {
      invertAxis = !invertAxis; // Toggle the invertAxis flag
      EEPROM.write(INVERT_AXIS_ADDR, invertAxis); // Save the invertAxis flag to EEPROM
      EEPROM.commit(); // Ensure the value is written to EEPROM
      Serial.println("Button 2 held - Inverting servo axis direction");
      button2PressTime = millis(); // Reset the button press time after action
      updateLCD(); // Update the LCD with the new axis direction
      digitalWrite(BEEP_PIN, HIGH);   // Turn on beeper
      delay(500);                     // Wait for 500 milliseconds
      digitalWrite(BEEP_PIN, LOW);    // Turn off beeper
    }
  } else {
    button2Held = false; // Reset button held state when released
  }

  delay(50); // Small delay to reduce CPU load and debounce buttons
}

// ----- Encoder ISR (Interrupt Service Routine) -----
void IRAM_ATTR handleEncoder() {
  int clkState = digitalRead(ENCODER_CLK); // Read current clock pin state
  int dtState = digitalRead(ENCODER_DT);   // Read current data pin state

  // Determine the direction of rotation based on the clock and data states
  if (clkState != lastClkState) {
    if (dtState != clkState) {
      encoderSteps++;  // Clockwise rotation
    } else {
      encoderSteps--;  // Counter-clockwise rotation
    }
  }

  lastClkState = clkState; // Update the last clock state for next interrupt
}

// ----- Update LCD Function -----
// ----- Update LCD Function -----
void updateLCD() {
  lcd.setCursor(0, 1); // Set cursor to second row
  lcd.print("                "); // Clear the second row
  lcd.setCursor(0, 1); // Reset cursor to the start of second row
  lcd.print("Angle: "); // Display the "Angle" label
  lcd.print(servoAngle); // Display the current servo angle

  // Display Axis Inversion Status
  lcd.setCursor(0, 0);  // Set cursor to first row
  lcd.print("Invert: ");
  if (invertAxis) {
    lcd.print("Yes "); // Ensure extra space to clear any previous letters
  } else {
    lcd.print("No ");  // Ensure extra space to clear any previous letters
  }
}

