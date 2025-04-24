#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// ----- Pin Definitions -----
#define ENCODER_CLK 13      // Rotary Encoder Clock Pin (A)
#define ENCODER_DT 26       // Rotary Encoder Data Pin (B)
#define ENCODER_SW 14       // Rotary Encoder Button Pin (SW)
#define SERVO_PIN 18        // Servo Motor Pin
#define BEEP_PIN 16         // Pin for the beeper
#define OTA_BUTTON_PIN 17    // or another unused GPIO pin
#define BUTTON_1 4          // Additional Button 1 (GPIO 2)
#define BUTTON_2 15         // Additional Button 2 (GPIO 15)
//#define SCREENCLOCK D22
//#define SCREENSDA D21
//purple = all clocks
//grey = all SDA


// ----- Servo Setup -----
Servo myServo;            // Servo object
int servoAngle = 90;      // Start angle for the servo (default 90°)
const int minAngle = 0;   // Minimum servo angle (0°)
const int maxAngle = 180; // Maximum servo angle (180°)
const int stepSize = 1;   // Precision movement (1° per step)
int currentAngle = 0;

// ----- Wifi Setup -----
const char* ssid = "------";
const char* password = "--------";

// ----- LCD Setup -----
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address (0x27), LCD size (16x2)

// ----- Encoder State -----
volatile int encoderSteps = 0; // To store the number of steps from the encoder
int lastClkState = HIGH;       // To track the last state of the encoder clock pin
bool encoderButtonPressed = false; // Flag to track the encoder button state

// ----- Persistent Zero Value in EEPROM ----- test
int zeroValue = 0;             // Default zero value for the servo
const int EEPROM_SIZE = 512;   // EEPROM size for ESP32

// ----- Invert Axis Flag -----
bool invertAxis = false; // Flag to track if servo direction is inverted
bool flipIdle = true; // Flag to flip flop between idle and last position

// EEPROM addresses for storing data
const int ZERO_VALUE_ADDR = 0;
const int INVERT_AXIS_ADDR = 1; // Store the invertAxis flag (1 byte)

// ----- Random Mode -----
bool randomMode = false;  // Flag to track if random mode is active
unsigned long randomModeStartTime = 0; // To track when random mode started

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
  pinMode(BEEP_PIN, OUTPUT);          // Beeper pin setup
  pinMode(OTA_BUTTON_PIN, INPUT_PULLUP); // assumes button pulls pin LOW when pressed

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


//enterOTAUpdateMode();

void loop() {
 //Check if OTA button was pressed 
  static unsigned long buttonOTAPressTime = 0;  // Track the time when Button 1 is pressed
  static bool buttonOTAHeld = false;             // Flag to track if Button 1 is held

  if (digitalRead(OTA_BUTTON_PIN) == LOW) {
    if (!buttonOTAHeld) {  // If Button 1 is just pressed (not held)
      buttonOTAPressTime = millis();  // Record the time when pressed
      buttonOTAHeld = true;           // Mark button as held
    }
  // If Button 1 is held for 3 seconds, set the current angle as the new zero
    if (millis() - buttonOTAPressTime >= 2000 && buttonOTAHeld) {
      enterOTAUpdateMode();
    }
  } else {
    buttonOTAHeld = false; // Reset button held state when released
  }
    


//<------------ Process Encoder Movement ------------>
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




//<-------Handle Encoder Button Press AKA Flip Flop Idle Position---------  

  if (digitalRead(ENCODER_SW) == LOW && !encoderButtonPressed) { // Button pressed (LOW)
    encoderButtonPressed = true;
    Serial.println("Encoder button pressed - Flipping throttle position");
    if (flipIdle){
      //save current position and set throttle to idle
      currentAngle = servoAngle;
      servoAngle = zeroValue; // Reset to stored zero position
      myServo.write(servoAngle); // Move servo to stored zero position
      flipIdle = !flipIdle;
    }

    else {
      //Set throttle to previous position before idle
      servoAngle = currentAngle; // Reset to stored zero position
      myServo.write(servoAngle); // Move servo to stored zero position
      flipIdle = !flipIdle;
    }
    //servoAngle = zeroValue; // Reset to stored zero position
    //randomMode = false;     // Stop random mode if it's active
    updateLCD(); // Update the LCD with the new angle
  }

  // --- Handle Encoder Button Release ---
  if (digitalRead(ENCODER_SW) == HIGH && encoderButtonPressed) { // Button released (HIGH)
    encoderButtonPressed = false; // Reset the button press state
  }






//<--------Handle Button 1 AKA Reset Zero Position----------------->
  // --- Handle Button 1 Press (Set New "Zero" Value if held for 3 seconds) ---
  static unsigned long button1PressTime = 0;  // Track the time when Button 1 is pressed
  static bool button1Held = false;             // Flag to track if Button 1 is held

  if (digitalRead(BUTTON_1) == LOW && digitalRead(BUTTON_2) == HIGH) { // Button 1 pressed (LOW)
    if (!button1Held) {  // If Button 1 is just pressed (not held)
      button1PressTime = millis();  // Record the time when pressed
      button1Held = true;           // Mark button as held
    }

    // If Button 1 is held for 3 seconds, set the current angle as the new zero
    if (millis() - button1PressTime >= 2000 && button1Held) {
      zeroValue = servoAngle; // Set the current angle as the new zero
      EEPROM.write(ZERO_VALUE_ADDR, zeroValue); // Save the new zero value to EEPROM
      EEPROM.commit(); // Ensure the value is written to EEPROM
      Serial.println("Button 1 held - New zero value set");
      button1PressTime = millis(); // Reset the button press time after action
      updateLCD(); // Update the LCD with the new "zero" value
    }
  } else {
    button1Held = false; // Reset button held state when released
  }







//<---------Button 2 Logic AKA Random Mode --------------->
  static unsigned long button2PressTime = 0;  // Track the time when Button 2 is pressed
  static bool button2Held = false;             // Flag to track if Button 2 is held

  if (digitalRead(BUTTON_2) == LOW && digitalRead(BUTTON_1) == HIGH) { // Button 2 pressed (LOW)
    if (!button2Held) {  // If Button 2 is just pressed (not held)
      button2PressTime = millis();  // Record the time when pressed
      button2Held = true;           // Mark button as held
    }

    // If Button 2 is held for 2 seconds, toggle random mode
    if (millis() - button2PressTime >= 2000 && button2Held) {
      randomMode = !randomMode; // Toggle the randomMode flag
      Serial.println(randomMode ? "Random mode ON" : "Random mode OFF");
      button2PressTime = millis(); // Reset the button press time after action
      updateLCD(); // Update the LCD with the new mode status
    }
  } else {
    button2Held = false; // Reset button held state when released
  }






//<---------Invert Axis Logic--------------->
  static unsigned long button1and2PressTime = 0;  // Track the time when Button 2 is pressed
  static bool button1and2Held = false;             // Flag to track if Button 2 is held
  if (digitalRead(BUTTON_2) == LOW && digitalRead(BUTTON_1) == LOW) { // Button 1 and 2 pressed at the same time
    if (!button1and2Held) {  // If Button 2 is just pressed (not held)
      button1and2PressTime = millis();  // Record the time when pressed
      button1and2Held = true;           // Mark button as held
    }

    // If Button 2 is held for 10 seconds, invert the axis direction
    if (millis() - button1and2PressTime >= 5000 && button1and2Held) {
      invertAxis = !invertAxis; // Toggle the invertAxis flag
      EEPROM.write(INVERT_AXIS_ADDR, invertAxis); // Save the invertAxis flag to EEPROM
      EEPROM.commit(); // Ensure the value is written to EEPROM
      Serial.println("Button 2 held - Inverting servo axis direction");
      button1and2PressTime = millis(); // Reset the button press time after action
      updateLCD(); // Update the LCD with the new axis direction
    }
  } else {
    button1and2Held = false; // Reset button held state when released
  }

  delay(50); // Small delay to reduce CPU load and debounce buttons
  

}

// ----- Encoder Interrupt Function -----
void IRAM_ATTR handleEncoder() {
  int clkState = digitalRead(ENCODER_CLK); // Read the current state of the clock pin
  if (clkState != lastClkState) { // Only process if the state has changed
    if (digitalRead(ENCODER_DT) != clkState) { // Clockwise rotation
      encoderSteps++;
    } else { // Counterclockwise rotation
      encoderSteps--;
    }
  }
  lastClkState = clkState; // Update the last state
}



// ----- Update LCD Display -----
void updateLCD() {
  lcd.clear();  // Clear the LCD screen
  //Print Throttle
  lcd.setCursor(0, 0); // Set cursor to first row
  lcd.print("Speed:");
  lcd.print(servoAngle);

  //Print Random on or off
  lcd.setCursor(9, 0); // Set cursor to first row, 13th position
  lcd.print(!randomMode ? "" : "Random");

  //Print Idle Value
  lcd.setCursor(0, 1); // Set cursor to second row
  lcd.print("Idle:");
  lcd.print(zeroValue);

  //Print Inverted or not
  lcd.setCursor(8, 1); // Set cursor to the second row, after "Zero: "
  lcd.print(invertAxis ? "Invert" : "Normal");
}






//------------- ----- ENTER OTA MODE ----------------------------------------------
void enterOTAUpdateMode() {
  Serial.println("Entering OTA update mode...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Entering OTA...");

  // Connect to WiFi
  WiFi.begin(ssid, password);
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("IP:");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP()); // Show IP address

    ArduinoOTA.setHostname("YourDeviceName"); // Optional
    ArduinoOTA.begin();

    // Flash OTA indicator
    bool blink = false;
    while (true) {
      ArduinoOTA.handle();

      // Optional: blink "Waiting..." on top row
      if (millis() % 1000 < 500) {
        if (!blink) {
          lcd.setCursor(4, 0);
          lcd.print("OTA...");
          blink = true;
        }
      } else {
        if (blink) {
          lcd.setCursor(4, 0);
          lcd.print("       ");
          blink = false;
        }
      }

      delay(10);
    }
  } else {
    Serial.println("\nWiFi connection failed.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed");
    delay(2000);
    updateLCD();
  }
}



//to be added later:
  // --- Random Mode Logic ---
  // if (randomMode) {
  //   if (millis() - randomModeStartTime > 500) { // 500 ms for random movements
  //     randomModeStartTime = millis(); // Reset the start time for the next cycle
  //     int randomMovement = random(0, 10); // Random movement within a small range
  //     int randomDelay = random(200, 500); // Random delay between movements
  //     int targetAngle = servoAngle + randomMovement;
  //     targetAngle = constrain(targetAngle, minAngle, maxAngle); // Ensure it stays within bounds
  //     myServo.write(targetAngle);  // Move servo to random target position
  //     Serial.print("Random Target Angle: ");
  //     Serial.println(targetAngle);  // Output random target angle to Serial Monitor
  //     delay(randomDelay); // Wait for random delay before next movement
  //     updateLCD(); // Update the LCD with the random movement angle
  //   }
  // }