#include <HCSR04.h>
#include <LiquidCrystal_I2C.h> 
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include "MQ135.h"
#include <ZMPT101B.h>
#include <RCSwitch.h>
#include <Adafruit_INA219.h>
#include <EEPROM.h> // For non-volatile memory

// Define the MQ135 sensor pin
#define MQ135_PIN 34
#define RLOAD 10000 // 10k ohms

// Instantiate the MQ135 object
MQ135 mq135(MQ135_PIN);

// Instantiate the AHT20 sensor object
Adafruit_AHTX0 aht;

//Rf switches MENU
RCSwitch rfTransmitter = RCSwitch();
#define RF_PIN 13 // Define the pin connected to the RF transmitter data pin
// RF codes
#define CODE_A 467170
#define CODE_B 467176
#define CODE_C 467169
#define CODE_D 467172
#define CODE_ALL_ON 467180
#define CODE_ALL_OFF 467171

// ZMPT101 Voltage sensor
ZMPT101B voltageSensor(39, 50.0);   // Initialize ZMPT101B sensor (pin A0, 50Hz)
float currentVoltage = 0.0f;  // Global variable to store the voltage
#define SENSITIVITY 510.0f // Calibrated sensitivity value from your sensor

// create ina219 object
Adafruit_INA219 ina219;

// Battery parameters
float chargeAccumulated = 0.0;   // Total charge accumulated (mAh)
float dischargeAccumulated = 0.0; // Total discharge accumulated (mAh)
float batteryCapacity = 130000.0;  // Nominal battery capacity (mAh)
float busVoltage;
float current_mA;
float shuntvolt;
float power_mW;
float shuntResistance; 
// global variables for current in Amp and power in watts etc
float current = current_mA /1000;
float power = power_mW /1000;
float Charge = chargeAccumulated / 1000;
float Discharge = dischargeAccumulated / 1000;
float shuntR = shuntResistance / 1000;

// Efficiency factors (typical values for lithium-ion)
const float chargeEfficiency = 0.90;  // 90% charging efficiency
const float dischargeEfficiency = 0.95; // 95% discharging efficiency

float soc = 0;                   // State of Charge (%)
float lastSavedSOC = 100.0;      // For EEPROM save logic
float lastSavedCharge = 0.0;  
unsigned long previousTime = 0;
unsigned long long prevTime = 0; // For time-based integration
unsigned long lastSave = 0;       // For EEPROM save

void saveToEEPROM() {
  EEPROM.put(0, chargeAccumulated);
  EEPROM.put(10, dischargeAccumulated);
  EEPROM.put(20, soc);
  EEPROM.commit();
}

void loadFromEEPROM() {;
  EEPROM.get(0, chargeAccumulated);
  EEPROM.get(10, dischargeAccumulated);
  EEPROM.put(20, soc);
}

void resetEEPROM() {
  chargeAccumulated = 0;
  dischargeAccumulated = 0;
  soc = 100;
  saveToEEPROM();
}

// Instantiate the BMP280 sensor object
Adafruit_BMP280 bmp;

// Pin definitions
const int PIN_TRIG = 27;
const int PIN_ECHO = 26;
const int MOTOR = 18;
const int PIN_BUZZER = 15; // Buzzer pin
#define DFPLAYER_TX 1
#define DFPLAYER_RX 3

// Define pins for the TTP223 touch sensors
#define TOUCH_PIN_MODE 32    // Pin for toggling mode
#define TOUCH_PIN_MOTOR 33   // Pin for toggling motor manually
#define TOUCH_PIN_UV 35      // Pin for toggling UV bulb
#define UV_BULB_PIN 19       // Pin for UV bulb
#define RCWL_PIN 12           // Pin connected to RCWL-0516

// UV bulb states
bool uvBulbOn = false;
unsigned long uvBulbStopTime = 0;

// Lights status
bool LIGHT1ON = false;
bool LIGHT2ON = false;
bool LIGHT3ON = false;

// Function prototypes
void toggleMode();
void toggleMotorManual();
void toggleUvBulb();
void toggleLIGHT1();
void toggleLIGHT2();
void toggleLIGHT3();
void handleMotorControl(unsigned int waterLevelPercent);

// Define modes 
#define MODE_AUTOMATIC 0
#define MODE_MANUAL 1
bool currentMode = MODE_AUTOMATIC;


// Pin definitions for UV bulb relay
#define UV_BULB_PIN 19 // UV bulb relay pin

// Create DFPlayer Mini object
SoftwareSerial mySoftwareSerial(DFPLAYER_RX, DFPLAYER_TX); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

// Create an LCD object
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Constants
const int maxDistanceCm = 121; // Maximum distance in cm
const int minDistanceCm = 14;  // Minimum distance in cm for 100% water level
const int tankCapacityLiters = 970; // Tank capacity in liters
const float distanceChangeThreshold = 0.4; // Minimum change in distance to update the display (in cm)
const unsigned long measurementInterval = 500; // Measurement interval in milliseconds
const unsigned long displayInterval = 5000; // Display update interval in milliseconds
const unsigned long buzzerDuration = 20000; // Buzzer alert duration in milliseconds (15 seconds)
const unsigned int buzzerLevelChangeThreshold = 10; // 10% change in water level to retrigger buzzer
const unsigned long dryRunCheckInterval = 10 * 60 * 1000; // 10 minute in milliseconds
const unsigned int dryRunThreshold = 5; // 5% water level change threshold
const unsigned long dryRunRecheckInterval = 20 * 60 * 1000; // 20 minutes in milliseconds
const unsigned long dryRunDisplayDuration = 6000; // Duration to display dry run status (6 seconds)
const unsigned long mainDisplayDuration = 6000; // Duration to display main water level (5 seconds)
// Constants for timing
const unsigned long BACKLIGHT_DURATION = 15000; // Backlight ON duration in milliseconds

// Baseline resistance value (adjust this value after calibration)
float baselineResistance = 44000.0; // Example value, replace with your measured value


// Default values for sensor failure
const float defaultPressure = 900; // Default pressure in hPa (standard atmospheric pressure)
const float defaultAltitude = 1600; // Default altitude in meters
const float defaulttemp = 25.5; // Default temperature
const float defaultHumi = 60.5; // Default humidity

// Variables
unsigned int previousWaterLevel = 0; // Previous water level percentage
bool buzzerActive = false; // Flag to track if buzzer is active
bool motorRunning = false; // Flag to track if motor is running
float previousDistanceCm = 0.0; // Previous measured distance
bool overflowWarningActive = false; // Flag to track if overflow warning is active
unsigned long lastMeasurementTime = 0; // Time of last measurement
unsigned long lastDisplayUpdateTime = 0; // Time of last display update
unsigned long buzzerStartTime = 0; // Start time for buzzer alert
unsigned int lastPlayedWaterLevel = 0; // Last water level percentage that triggered audio
unsigned int lastBuzzerLevel = 0; // Last water level percentage that triggered the buzzer
unsigned long lastDryRunCheckTime = 0; // Last dry run check time
unsigned int dryRunCheckCount = 0; // Dry run check count
bool dryRunActive = false; // Dry run active flag
bool manualIntervention = false;   // Manual intervention flag
bool waitingForRecheck = false;    // Recheck waiting flag
unsigned int dryRunFailedCount = 0; // Counter for consecutive failed dry run checks
unsigned int initialWaterLevel = 0; // Initial water level for dry run check
unsigned long dryRunIntervalStart = 0; // Start time of the current dry run interval
unsigned long dryRunRecheckStart = 0; // start time recheck
unsigned long displaySwitchTime = 0; // Time of last display switch
bool displayDryRunStatus = false; // Flag to indicate whether to display dry run status
unsigned long lastAudioPlayTime = 0; // Last time an audio file was played
unsigned int audioQueue[10]; // Queue for audio tracks
int queueStart = 0; // Start index of the queue
int queueEnd = 0; // End index of the queue
bool playing = false; // Flag to check if a track is currently playing
unsigned long previousMillis = 0;  // For timing control
const long interval = 50;          // Interval for performing the task
// Global variable to track if mode was automatically switched
bool autoSwitchedMode = false;
const int MODE_SWITCH_TEMP = 10;
int aqi = 0;
int co2 = 0;
// Variables to track mode and motion detection for LIGHT1
bool motionDetectionForLight1 = false;  // Default: Motion detection for LIGHT1 is ON
// Variables to track time
unsigned long lastMotionDetectedTime = 0;
bool backlightOn = false;

/* appliances pins */
 #define LIGHT1 2
 #define LIGHT2 4
 #define LIGHT3 5

const byte rxPin = 17;  // RX2 pin for ESP32
const byte txPin = 16;  // TX2 pin for ESP32
HardwareSerial dwin(1);

/* Adresses of all sensors */
unsigned char Buffer[9];
#define temperature_add   0x50
#define humidity_add      0x51
#define previousWaterLevel_add     0x52
#define uvBulbOn_add     0x53
#define motorRunning_add 0x54
#define LIGHT1ON_add     0x55
#define LIGHT2ON_add     0x56
#define LIGHT3ON_add     0x58
#define currentMode_add  0x57
#define pressure_add     0x59
#define altitude_add     0x60
#define dewpoint_add     0x61
#define dryRunActive_add 0x62
#define aqi_add          0x63
#define co2_add          0x64
#define capacity_add     0x68
#define soc_add          0x80
#define busVoltage_add   0x81
#define current_add      0x82
#define power_add        0x83
#define currentVoltage_add    0x84
#define charge_add       0x85
#define Discharge_add    0x86
#define shuntR_add       0x87
#define shuntvolt_add    0x88
#define motionDetectionForLight1_add     0x89
#define manualIntervention_add    0x90

unsigned char   Temperature[8] = {0x5a, 0xa5, 0x05, 0x82, temperature_add , 0x00, 0x00, 0x00};
unsigned char      Humidity[8] = {0x5a, 0xa5, 0x05, 0x82, humidity_add, 0x00, 0x00, 0x00};
unsigned char      Pressure[8] = {0x5a, 0xa5, 0x05, 0x82, pressure_add , 0x00, 0x00, 0x00};
unsigned char      Altitude[8] = {0x5a, 0xa5, 0x05, 0x82, altitude_add, 0x00, 0x00, 0x00};
unsigned char      DewPoint[8] = {0x5a, 0xa5, 0x05, 0x82, dewpoint_add, 0x00, 0x00, 0x00};
unsigned char     waterLevel[8] = {0x5a, 0xa5, 0x05, 0x82, previousWaterLevel_add, 0x00, 0x00, 0x00};
unsigned char     Capacity[8] = {0x5a, 0xa5, 0x05, 0x82, capacity_add, 0x00, 0x00, 0x00};
unsigned char     motor[8] = {0x5a, 0xa5, 0x05, 0x82, motorRunning_add, 0x00, 0x00, 0x00};
unsigned char     uv[8] = {0x5a, 0xa5, 0x05, 0x82, uvBulbOn_add, 0x00, 0x00, 0x00};
unsigned char     L1[8] = {0x5a, 0xa5, 0x05, 0x82, LIGHT1ON_add, 0x00, 0x00, 0x00};
unsigned char     L2[8] = {0x5a, 0xa5, 0x05, 0x82, LIGHT2ON_add, 0x00, 0x00, 0x00};
unsigned char     L3[8] = {0x5a, 0xa5, 0x05, 0x82, LIGHT3ON_add, 0x00, 0x00, 0x00};
unsigned char     mod[8] = {0x5a, 0xa5, 0x05, 0x82, currentMode_add, 0x00, 0x00, 0x00};
unsigned char     dry[8] = {0x5a, 0xa5, 0x05, 0x82, dryRunActive_add, 0x00, 0x00, 0x00};
unsigned char     air[8] = {0x5a, 0xa5, 0x05, 0x82, aqi_add, 0x00, 0x00, 0x00};
unsigned char     co[8] = {0x5a, 0xa5, 0x05, 0x82, co2_add, 0x00, 0x00, 0x00};
unsigned char     SOC[8] = {0x5a, 0xa5, 0x05, 0x82, soc_add, 0x00, 0x00, 0x00};
unsigned char     BUS[8] = {0x5a, 0xa5, 0x05, 0x82, busVoltage_add, 0x00, 0x00, 0x00};
unsigned char     CURRENT[8] = {0x5a, 0xa5, 0x05, 0x82, current_add, 0x00, 0x00, 0x00};
unsigned char     POWER[8] = {0x5a, 0xa5, 0x05, 0x82, power_add, 0x00, 0x00, 0x00};
unsigned char     MAINS[8] = {0x5a, 0xa5, 0x05, 0x82, currentVoltage_add, 0x00, 0x00, 0x00};
unsigned char     CHARGE[8] = {0x5a, 0xa5, 0x05, 0x82, charge_add, 0x00, 0x00, 0x00};
unsigned char     DISCHARGE[8] = {0x5a, 0xa5, 0x05, 0x82, Discharge_add, 0x00, 0x00, 0x00};
unsigned char     SHUNTR[8] = {0x5a, 0xa5, 0x05, 0x82, shuntR_add, 0x00, 0x00, 0x00};
unsigned char     SHUNTVOLT[8] = {0x5a, 0xa5, 0x05, 0x82, shuntvolt_add, 0x00, 0x00, 0x00};
unsigned char     MOTION[8] = {0x5a, 0xa5, 0x05, 0x82, motionDetectionForLight1_add, 0x00, 0x00, 0x00};
unsigned char     MANUAL[8] = {0x5a, 0xa5, 0x05, 0x82, manualIntervention_add, 0x00, 0x00, 0x00};

void setup() {
  
  // Initialize the LCD
  lcd.init();
  lcd.backlight();

  // Initialize DWIN HMI UART
      dwin.begin(512000,SERIAL_8N1, rxPin, txPin);

  // Set RF_PIN as the transmitter pin
      rfTransmitter.enableTransmit(RF_PIN); 

  // Initialize the INA219
       ina219.begin();
       EEPROM.begin(512);
       loadFromEEPROM();
       ina219.setCalibration_16V_60A(); 

  // Set the sensitivity for the voltage sensor
      voltageSensor.setSensitivity(SENSITIVITY);
  
  // Set up the pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(MOTOR, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT); // Set buzzer pin as output
  pinMode(TOUCH_PIN_MODE, INPUT);
  pinMode(TOUCH_PIN_MOTOR, INPUT);
  pinMode(TOUCH_PIN_UV, INPUT);
  pinMode(UV_BULB_PIN, OUTPUT);
  pinMode(LIGHT1, OUTPUT);
  pinMode(LIGHT2, OUTPUT);
  pinMode(LIGHT3, OUTPUT);
  pinMode(RCWL_PIN, INPUT);
  
  // Initialize UV bulb relay
  pinMode(UV_BULB_PIN, OUTPUT);
  digitalWrite(UV_BULB_PIN, LOW); // UV bulb OFF by default
  
  // Initialize the motor relay to OFF
  digitalWrite(MOTOR, LOW); // Set motor relay to OFF
  digitalWrite(LIGHT1, LOW);
  digitalWrite(LIGHT2, LOW);
  digitalWrite(LIGHT3, LOW);


  // Initialize the sensors
    if (!bmp.begin(0x76)) {
    lcd.setCursor(0, 0);
    lcd.print("BMP280 Failed");
    return;
  }
  
  if (!aht.begin()) {
    lcd.setCursor(0, 1);
    lcd.print("AHT20 Failed");
    return;
  }
  
  // Initialize DFPlayer Mini
  mySoftwareSerial.begin(9600);
  lcd.setCursor(0, 0);
  lcd.print("Starting DFPlayer");

  if (!myDFPlayer.begin(mySoftwareSerial)) {
    lcd.setCursor(0, 1);
    lcd.print("DFPlayer Failed");
    return;
  }
  
  myDFPlayer.volume(30);  // Set volume to 30 out of 30
  lcd.setCursor(0, 1);
  lcd.print("DFPlayer Done");

  delay(1000);
  lcd.clear();
  
  // Print initial message on LCD
  lcd.setCursor(0, 0);
  lcd.print("Home Automation");
  lcd.setCursor(0, 1);
  lcd.print("Starting....");
}

// Function to read temperature from AHT20
float readTemperature() {
  if (aht.begin()) {
    sensors_event_t humidity, temperature;
    aht.getEvent(&humidity, &temperature);
    return temperature.temperature;
  } else {
    return defaulttemp; // Return default temperature if sensor fails
  }
}

// Function to read humidity from AHT20
float readHumidity() {
  if (aht.begin()) {
    sensors_event_t humidity, temperature;
    aht.getEvent(&humidity, &temperature);
    return humidity.relative_humidity;
  } else {
    return defaultHumi; // Return default humidity if sensor fails
  }
}

// Function to read pressure from BMP280
int readPressure() {
  if (bmp.begin(0x76)) {
    return (int)(bmp.readPressure() / 100.0F); // Convert Pa to hPa and cast to int
  } else {
    return (int)defaultPressure; // Return default pressure if sensor fails
  }
}

// Function to read altitude from BMP280
int readAltitude() {
  if (bmp.begin(0x76)) {
    return (int)bmp.readAltitude(1013.25); // Convert to meters and cast to int
  } else {
    return (int)defaultAltitude; // Return default altitude if sensor fails
  }
}

void loop() {

    // Get current time
  unsigned long currentMillis = millis();
  unsigned long currentTime = millis();
  
   // Check sensor control from HMI
  sensor_data();
  delay(5);
 
   // Check relay control from HMI
   if (currentMillis - previousMillis >= interval) {
       previousMillis = currentMillis;  // Update the last performed time
       realy_control();
       delay(5); }

   // call  Handle lights
      toggleLIGHT1();
      toggleLIGHT2();
      
   //  call motion sensor and toggle
      checkMotionSensor();
      toggleMotionDetectionForLight1();

   // Call the voltage function to calculate and update the voltage
      voltage();
      
   // Call the consolidated battery function
      updateBatteryMonitor();  

  // Read temperature and humidity from AHT20 sensor
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  float temperature = temp.temperature;
  float humidityValue = humidity.relative_humidity;
  // Read pressure from BMP280 sensor
  float pressure = bmp.readPressure() / 100.0F; // Pressure in hPa
  int altitude = bmp.readAltitude(1013.25); // Altitude in meters

  // Get the corrected CO2 concentration in ppm
   co2 = mq135.getCorrectedPPM(temperature, humidityValue);

  // Get the AQI based on the overall air quality
   aqi = mq135.getAQI(temperature, humidityValue);
   

  // Measure distance at specified intervals
  if (currentMillis - lastMeasurementTime >= measurementInterval) {
    lastMeasurementTime = currentMillis;
    long duration = getDistance();
    float distanceCm = (duration * 0.0343) / 2;

  // Check if the mode toggle touch sensor is pressed
  if (digitalRead(TOUCH_PIN_MODE) == HIGH) {
    toggleMode();
    delay(500); // Debounce delay to prevent multiple toggles from one touch
  }

  // Check if the manual motor toggle touch sensor is pressed
  if (digitalRead(TOUCH_PIN_MOTOR) == HIGH) {
    toggleMotorManual();
    delay(500); // Debounce delay
  }

  // Check if the UV bulb toggle touch sensor is pressed
  if (digitalRead(TOUCH_PIN_UV) == HIGH) {
    toggleUvBulb();
    delay(500); // Debounce delay
  }

    // Check if the distance is within a valid range
    if (distanceCm >= 0 && distanceCm <= maxDistanceCm) {
      // Convert distance to percentage
      unsigned int waterLevelPercent = calculateWaterLevelPercentage(distanceCm);

      // Calculate the current water volume in liters
      unsigned int waterVolumeLiters = waterLevelPercent * tankCapacityLiters / 100;

      // Handle motor control based on water level
      handleMotorControl(waterLevelPercent);
      
      // Handle mode control based on temperature
      handleTemperatureModeSwitch();

      // Handle buzzer activation for specific levels
      handleBuzzer(waterLevelPercent);
      
  // Check if UV bulb should turn off 30 minutes after the motor stops
  if (uvBulbOn && !motorRunning && millis() - uvBulbStopTime >= 10 * 60 * 1000) {
    digitalWrite(UV_BULB_PIN, LOW); // Turn off UV bulb
    uvBulbOn = false;
  }
  
      // Check for overflow condition
      if (waterLevelPercent > 100) {
        overflowWarningActive = true;
      } else {
        overflowWarningActive = false;
      }

      // Play audio based on water level
      playAudioForWaterLevel(waterLevelPercent);

      // Handle dry run protection and manual intervention
      handleDryRunProtection(waterLevelPercent);
      resetManualIntervention();

      // Update previous water level and distance
      previousWaterLevel = waterLevelPercent;
      previousDistanceCm = distanceCm;
    } else {
      Serial.println("Error: Invalid distance measurement");
    }
  }

  // Update display at specified intervals
  if (currentMillis - lastDisplayUpdateTime >= displayInterval) {
    lastDisplayUpdateTime = currentMillis;
    updateDisplay();
  }

  // Switch between dry run status and main water level display
  if (dryRunActive && currentMillis - displaySwitchTime >= (displayDryRunStatus ? dryRunDisplayDuration : mainDisplayDuration)) {
    displaySwitchTime = currentMillis;
    displayDryRunStatus = !displayDryRunStatus;
  }

  // Handle audio track queue
  handleAudioQueue();
  // Added call to manage voltage-specific audio
  handleVoltageAudio();
  // handle inverter audio
  handleInverterAudio();
  // Handle temperature-related audio playback
  handleTemperatureAudio();  
}

// Function to measure the distance using HC-SR04 with averaging
long getDistance() {
  const int numReadings = 10; // Number of readings to average
  long sum = 0;
  
  for (int i = 0; i < numReadings; i++) {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    
    // Calculate the duration
    long duration = pulseIn(PIN_ECHO, HIGH);
    
    // Add to sum
    sum += duration;
    
    // Small delay to prevent rapid measurement
    delay(50); 
  }

  // Calculate average duration
  long averageDuration = sum / numReadings;

  // Convert duration to distance
  return averageDuration;
}

// Function to calculate the water level percentage
unsigned int calculateWaterLevelPercentage(float distanceCm) {
  // Calculate the water level percentage
  unsigned int percentage = 0;
  if (distanceCm <= maxDistanceCm && distanceCm >= minDistanceCm) {
    percentage = 100 - ((distanceCm - minDistanceCm) / (maxDistanceCm - minDistanceCm) * 100);
  }

  // Allow percentage to exceed 100 for overflow detection
  if (distanceCm < minDistanceCm) {
    percentage = 100 + ((minDistanceCm - distanceCm) / minDistanceCm) * 100;
  }

  return percentage;
}

  // Function to handle temperature-based mode switching
void handleTemperatureModeSwitch() {
  int currentTemperature = readTemperature();

  if (currentTemperature < MODE_SWITCH_TEMP) {
    // If temperature is below 10 degrees, switch to manual mode if not already in it
    if (currentMode != MODE_MANUAL) {
      currentMode = MODE_MANUAL;
      autoSwitchedMode = true;  // Track that mode was automatically switched to manual
    }
  } else if (currentTemperature >= MODE_SWITCH_TEMP && autoSwitchedMode) {
    // If temperature is above or equal to 10 degrees and mode was auto-switched, return to automatic mode
    currentMode = MODE_AUTOMATIC;
    autoSwitchedMode = false;  // Reset the auto-switch flag
  }
}

// Centralized motor control function
void handleMotorControl(unsigned int waterLevelPercent) {

  // Ensure motor is off if water level is above 100%, overriding all conditions
  if (waterLevelPercent > 100) {
    digitalWrite(MOTOR, LOW); // Turn off motor
    motorRunning = false;
    uvBulbStopTime = millis(); // Set UV bulb stop time
    return; // Exit function to ensure motor remains off
  }
     
  if (currentVoltage < 150 || currentVoltage > 220) {
  if (digitalRead(MOTOR) == HIGH) { 
    digitalWrite(MOTOR, LOW);  // Turn off motor if it was running
    motorRunning = false;
  }
    return; // Skip the rest of the motor control logic
  }
    
  if (!dryRunActive && currentMode == MODE_AUTOMATIC) {
    if (waterLevelPercent >= 100) {
      digitalWrite(MOTOR, LOW); // Turn off motor
      motorRunning = false;
      uvBulbStopTime = millis(); // Set UV bulb stop time
    } else if (waterLevelPercent <= 50 && !motorRunning) {
      digitalWrite(MOTOR, HIGH); // Turn on motor
      motorRunning = true;
      if (!uvBulbOn) {
        toggleUvBulb(); // Turn on UV bulb
      }
    }
  }
}

// Function to handle buzzer activation for specific water levels
void handleBuzzer(unsigned int waterLevelPercent) {
  unsigned long currentMillis = millis();

  // Check if the buzzer needs to be activated based on the level
  if ((waterLevelPercent == 99 || waterLevelPercent == 55) && !buzzerActive && 
      abs(int(waterLevelPercent) - int(lastBuzzerLevel)) >= buzzerLevelChangeThreshold) {
    buzzerStartTime = currentMillis; // Record the start time
    digitalWrite(PIN_BUZZER, HIGH); // Turn on buzzer
    buzzerActive = true; // Set buzzer active flag
    lastBuzzerLevel = waterLevelPercent; // Update the last buzzer level
  }

  // Check if the buzzer duration has passed
  if (buzzerActive && currentMillis - buzzerStartTime >= buzzerDuration) {
    digitalWrite(PIN_BUZZER, LOW); // Turn off buzzer
    buzzerActive = false; // Reset buzzer active flag
  }
}

// Function to handle dry run protection
void handleDryRunProtection(unsigned int waterLevelPercent) {
  unsigned long currentMillis = millis(); // No longer static
  unsigned long dryRunIntervalStart = 0; // No longer static
  unsigned long dryRunRecheckStart = 0; // No longer static
  unsigned int initialWaterLevel = 0;   // No longer static
  int dryRunFailedCount = 0;            // No longer static
  bool waitingForRecheck = false;       // No longer static
  bool manualIntervention = false;      // No longer static

  // Reset logic if manual intervention is cleared
  if (!manualIntervention && dryRunFailedCount == 0 && !dryRunActive) {
    return; // System is idle
  }

  // Check for manual intervention
  if (manualIntervention) {
    digitalWrite(MOTOR, LOW);
    motorRunning = false;
    return;
  }

  // Dry Run Activation
  if (!dryRunActive) {
    if (motorRunning) {
      // Start dry run check
      dryRunActive = true;
      dryRunIntervalStart = currentMillis;
      initialWaterLevel = waterLevelPercent;

      // Play voice prompts 17 and 18
      queueAudioTrack(17);
      queueAudioTrack(18);

      // Turn off motor
      digitalWrite(MOTOR, LOW);
      motorRunning = false;
    }
  } else if (!waitingForRecheck) {
    // Check for 10-minute interval completion
    if (currentMillis - dryRunIntervalStart >= 10 * 60 * 1000) {
      if (waterLevelPercent >= initialWaterLevel + dryRunThreshold) {
        // Threshold met: Reset interval and start next check
        dryRunIntervalStart = currentMillis;
        initialWaterLevel = waterLevelPercent;
      } else {
        // Threshold not met: Increment failed count
        dryRunFailedCount++;

        // Stop motor and wait for 20-minute recheck interval
        waitingForRecheck = true;
        dryRunRecheckStart = currentMillis;

        // Play voice prompts 19 and 20
        queueAudioTrack(19);
        queueAudioTrack(20);

        digitalWrite(MOTOR, LOW);
        motorRunning = false;
      }
    }
  } else {
    // During 20-minute recheck interval
    if (currentMillis - dryRunRecheckStart >= 20 * 60 * 1000) {
      // Recheck interval complete: Restart 10-minute dry run
      waitingForRecheck = false;
      dryRunIntervalStart = currentMillis;
      initialWaterLevel = waterLevelPercent;

      // Turn on motor briefly to check water level
      digitalWrite(MOTOR, HIGH);
      motorRunning = true;
    }
  }

  // Check for maximum failed dry run checks
  if (dryRunFailedCount >= 5) {
    dryRunActive = false;
    manualIntervention = true; // Flag manual intervention
    digitalWrite(MOTOR, LOW);  // Turn off motor completely
    motorRunning = false;
  }
}

// Reset function to clear manual intervention
void resetManualIntervention() {
  manualIntervention = false;
  dryRunFailedCount = 0;
  dryRunActive = false;
  waitingForRecheck = false;
}

// Function to queue audio tracks for playback
void queueAudioTrack(unsigned int track) {
  // Add the track to the queue if there is space
  if ((queueEnd + 1) % 16 != queueStart) {
    audioQueue[queueEnd] = track;
    queueEnd = (queueEnd + 1) % 16;
  }
}

// Function to handle the playback queue
void handleAudioQueue() {
  unsigned long currentMillis = millis();

  if (queueStart != queueEnd && !playing && currentMillis - lastAudioPlayTime >= 100) {
    // Play the next track in the queue
    myDFPlayer.play(audioQueue[queueStart]);
    lastAudioPlayTime = currentMillis;
    playing = true;
  }

  // Check if the current track has finished playing
  if (playing && currentMillis - lastAudioPlayTime >= 32000 + 1000) {
    queueStart = (queueStart + 1) % 16;
    playing = false;
  }
}

// Function to play audio based on water level
void playAudioForWaterLevel(unsigned int waterLevelPercent) {
  if (waterLevelPercent == 20 && lastPlayedWaterLevel != 20) {
    queueAudioTrack(1);
    queueAudioTrack(2);
    lastPlayedWaterLevel = 20;
  } else if (waterLevelPercent == 30 && lastPlayedWaterLevel != 30) {
    queueAudioTrack(3);
    queueAudioTrack(4);
    lastPlayedWaterLevel = 30;
  } else if (waterLevelPercent == 50 && lastPlayedWaterLevel != 50) {
    queueAudioTrack(5);
    queueAudioTrack(6);
    lastPlayedWaterLevel = 50;
  } else if (waterLevelPercent == 95 && lastPlayedWaterLevel != 95) {
    queueAudioTrack(7);
    queueAudioTrack(8);
    lastPlayedWaterLevel = 95;
  } else if (waterLevelPercent > 100 && lastPlayedWaterLevel != 100) {
    queueAudioTrack(9);
    queueAudioTrack(10);
    lastPlayedWaterLevel = 100;
  }
}

// Function to update the display
void updateDisplay() {
  unsigned long currentMillis = millis();
  
  lcd.clear(); // Clear the display before updating
  
  if (dryRunActive && displayDryRunStatus) {
    lcd.setCursor(0, 0);
    lcd.print("Motor is Runing Dry");
    lcd.setCursor(0, 1);
    lcd.print("water Not Available");
    lcd.setCursor(0, 2);
    lcd.print("DryRun Checks ");
    lcd.print(dryRunFailedCount);
    lcd.setCursor(0, 3);
    lcd.print("Next Check in ");
    lcd.print((dryRunRecheckInterval - (currentMillis - dryRunIntervalStart)) / 60000);
    lcd.print(" Min");
  } else {
    unsigned long displayCycleTime = 5000; // 5 seconds for each display
    unsigned long elapsed = currentMillis % (5 * displayCycleTime);

    if (elapsed < displayCycleTime) {
      lcd.clear();
      // Display Water Level
      lcd.setCursor(0, 0);
      lcd.print("Water Level : ");
      lcd.setCursor(15, 0);
      lcd.print(previousWaterLevel);
      lcd.print(" %"); // Adding spaces to clear any previous longer text

      // Display motor status on LCD
      lcd.setCursor(0, 1);
      lcd.print("motor status : ");
      lcd.print(motorRunning ? "ON " : "OFF");
      lcd.setCursor(0, 3);
      lcd.print("capacity: ");
      lcd.print(previousWaterLevel * tankCapacityLiters / 100);
      lcd.print(" Liter");  
      lcd.setCursor(13, 2);
      lcd.print("UV: ");
      lcd.print(uvBulbOn ? "ON " : "OFF"); 
      lcd.setCursor(0, 2);
      lcd.print("Mode: ");
      lcd.print(currentMode == MODE_AUTOMATIC ? "AUTO " : "MANUAL");  
      
    } else if (elapsed < 2 * displayCycleTime) {
      lcd.clear();
      // Display Water Level
      lcd.setCursor(0, 0);
      lcd.print("Water Level :");
      lcd.setCursor(15, 0);
      lcd.print(previousWaterLevel);
      lcd.print(" %"); // Adding spaces to clear any previous longer text

      // Display Temperature and Humidity
      lcd.setCursor(0, 1);
      lcd.print("temperature: ");
      lcd.print(readTemperature());
      lcd.print(" C");
      lcd.setCursor(0, 2);
      lcd.print("humidity : ");
      lcd.print(readHumidity());
      lcd.print(" %");
      
      // Display AQI CO2
      lcd.setCursor(0, 3);
      lcd.print("co2 : ");
      lcd.print(co2);
      lcd.setCursor(12, 3);
      lcd.print("AQI: ");
      lcd.print(aqi);
   
   } else if (elapsed < 3 * displayCycleTime) {
      lcd.clear();
      // Display inverter battery level
      lcd.setCursor(0, 0);
      lcd.print("Inverter SOC : ");
      lcd.print(soc, 0); 
      lcd.print(" %");
      // Real-time values
      lcd.setCursor(0, 1);
      lcd.print("Voltage : ");
      lcd.print(busVoltage, 2); // Voltage with 2 decimal places
      lcd.setCursor(0, 2);
      lcd.print("current : ");
      lcd.print(current_mA /1000, 2); // Current in mA (integer)
      lcd.setCursor(0, 3);
      lcd.print("Power : ");
      lcd.print(power_mW /1000, 0); // Power in watts with 1 decimal place
      lcd.print("  watts");
      
   } else if (elapsed < 4 * displayCycleTime) {
      lcd.clear();
      // voltages
      lcd.setCursor(0, 0);
      lcd.print("mains voltage : ");
      lcd.print(currentVoltage, 0); // Display with 2 decimal places
      // Display Charge and Discharge with units (mAh or Ah)
      lcd.setCursor(0, 1);
      lcd.print("Charge: ");
      if (chargeAccumulated >= 1000) {
      lcd.print(chargeAccumulated / 1000.0, 2); // Convert mAh to Ah, display with 2 decimal places
      lcd.print(" Ah");
      } else {
      lcd.print((int)chargeAccumulated); // Display in mAh
      lcd.print(" mAh");
      }
      lcd.setCursor(0, 2);
      lcd.print("Discharge: ");
      lcd.print(dischargeAccumulated / 1000.0, 2); // Convert mAh to Ah, display with 2 decimal places
      lcd.print(" Ah"); 
      lcd.setCursor(0, 3);
      lcd.print("shuntVolt: ");
      lcd.print(shuntvolt, 2); 
      lcd.print(" mv");
      
    } else if (elapsed < 5  * displayCycleTime) {
      lcd.clear();
       // Display Water Level
      lcd.setCursor(0, 0);
      lcd.print("Water Level : ");
      lcd.setCursor(15, 0);
      lcd.print(previousWaterLevel);
      lcd.print(" %"); // Adding spaces to clear any previous longer text

      // Display Pressure and Altitude
      lcd.setCursor(0, 1);
      lcd.print("pressure : ");
      lcd.print(readPressure());
      lcd.print(" hPa");
      lcd.setCursor(0, 2);
      lcd.print("altitude :");
      lcd.print(readAltitude());
      lcd.print(" meter");
      // Air quality 
      lcd.setCursor(0, 3);
      lcd.print("Air Quality : ");
      if (aqi <= 50) {
        lcd.print("Best");
      } else if (aqi <= 100) {
        lcd.print("Good");
      } else if (aqi <= 150) {
        lcd.print("Bad");
      } else if (aqi <= 200) {
        lcd.print("Worst");
      } else {
        lcd.print("Hazard");
      }
    }
  }
}

void checkMotionSensor() {
  if (motionDetectionForLight1) { // Motion detection for LIGHT1 is enabled
    if (digitalRead(RCWL_PIN) == HIGH) { // Motion detected
      lastMotionDetectedTime = millis(); // Update motion detection time

      // Turn on backlight if not already on
      if (!backlightOn) {
        lcd.backlight(); // Turn on LCD backlight
        backlightOn = true;
      }

      // Turn on Light1 if not already on
      if (!LIGHT1ON) {
        digitalWrite(LIGHT1, HIGH); // Turn on Light1
        LIGHT1ON = true;
      }
    } else {
      // If no motion detected, manage backlight and Light1
      if (backlightOn && (millis() - lastMotionDetectedTime > 15000)) { // No motion for 10 seconds
        lcd.noBacklight(); // Turn off LCD backlight after 10 seconds
        backlightOn = false;
      }

      // If no motion for 5 seconds and motion detection for LIGHT1 is enabled, turn off Light1
      if (LIGHT1ON && (millis() - lastMotionDetectedTime > 5000)) {
        digitalWrite(LIGHT1, LOW); // Turn off Light1
        LIGHT1ON = false;
      }
    }
  } else {
    // Motion detection for Light1 is disabled, only manage backlight
    if (digitalRead(RCWL_PIN) == HIGH) { // Motion detected
      lastMotionDetectedTime = millis(); // Update motion time

      // Turn on backlight if not already on
      if (!backlightOn) {
        lcd.backlight(); // Turn on LCD backlight
        backlightOn = true;
      }
    } else {
      // If no motion detected for 10 seconds, turn off backlight
      if (backlightOn && (millis() - lastMotionDetectedTime > 15000)) {
        lcd.noBacklight(); // Turn off LCD backlight
        backlightOn = false;
      }
    }
  }
}

void toggleMotionDetectionForLight1() {
  motionDetectionForLight1 = !motionDetectionForLight1; // Toggle the state
}

// Void function to calculate and store the voltage
void voltage() {
  static float voltageSum = 0;        // Sum of voltage readings
  static int readingCount = 0;        // Number of readings taken
  const int numReadings = 10;         // Total number of readings to take
  const int interval = 100;           // Delay between readings (milliseconds)
  static unsigned long previousMillis = 0; // Last reading time
  static bool isMeasuring = false;    // State flag for measurement

  // Check if it's time to take a new reading
  if (millis() - previousMillis >= interval) {
    previousMillis = millis(); // Update the last reading time

    // Start or continue taking readings
    if (readingCount < numReadings) {
      voltageSum += voltageSensor.getRmsVoltage();
      readingCount++;
      isMeasuring = true; // Measurement is in progress
    }

    // Once all readings are taken, compute the average
    if (readingCount >= numReadings && isMeasuring) {
        currentVoltage = voltageSum / numReadings; // Compute average
      // Reset for the next set of readings
      voltageSum = 0;
      readingCount = 0;
      isMeasuring = false; // Measurement complete
    }
  }
}

// function to monitor battery parameters
void updateBatteryMonitor() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 3600000.0; // Convert elapsed time to hours
    previousTime = currentTime;

    // Read voltage and current from INA219
          busVoltage = ina219.getBusVoltage_V();
          current_mA = ina219.getCurrent_mA();
          shuntvolt = ina219.getShuntVoltage_mV();
          power_mW = ina219.getPower_mW();
          shuntResistance = (shuntvolt/current_mA);

    // Update accumulated charge/discharge
    if (current_mA > 0) {
        chargeAccumulated += current_mA * deltaTime;
    } else {
        dischargeAccumulated += abs(current_mA) * deltaTime;
    }

    // Calculate State of Charge (SOC)
    soc = ((batteryCapacity - dischargeAccumulated * dischargeEfficiency + chargeAccumulated * chargeEfficiency) / batteryCapacity) * 100.0;
    soc = constrain(soc, 0, 100); // Limit SOC between 0% and 100%

    // Reset dischargeAccumulated to 0 when SOC reaches 100%
    if (soc == 100) {
        dischargeAccumulated = 0;
        saveToEEPROM();  // Save reset values to EEPROM
    }

    // Reset chargeAccumulated to 0 when SOC reaches 0%
    if (soc == 0) {
        chargeAccumulated = 0;
        saveToEEPROM();  // Save reset values to EEPROM
    }

    // Save data to EEPROM periodically if significant changes
    if (millis() - lastSave > 20000) { // Save every 20 seconds
        if (abs(soc - lastSavedSOC) > 0.5 || abs(chargeAccumulated - lastSavedCharge) > 1.0) {
            saveToEEPROM();
            lastSavedSOC = soc;
            lastSavedCharge = chargeAccumulated;
        }
        lastSave = millis();
    }
}

  // Function to send RF code
void sendCode(unsigned long code) {
  rfTransmitter.send(code, 24); // Send the RF code (24-bit)
}

 // Function to measure the baseline resistance
float getCleanAirResistance() {
    int sampleCount = 100; // Number of samples to average
    float totalResistance = 0.0;
    
    for (int i = 0; i < sampleCount; i++) {
        totalResistance += getResistance(); // Read resistance
        delay(100); // Short delay between readings
    }
    
    return totalResistance / sampleCount; // Average resistance
}

//function to handle voltage based audio
void handleVoltageAudio() {
  static unsigned long lastTrackPlayTime = 0; // Last time tracks were played
  unsigned long currentMillis = millis();

  // Check if voltage is in the desired range and water level is below 50%
  if (currentVoltage > 0 && currentVoltage < 150 && previousWaterLevel < 50) {
    if (currentMillis - lastTrackPlayTime >= 300000) { // 5 minutes = 300,000 ms
      queueAudioTrack(11);
      queueAudioTrack(12);
      lastTrackPlayTime = currentMillis;
    }
  } else if (currentVoltage >= 150 || currentVoltage <= 0) {
    lastTrackPlayTime = currentMillis; // Reset timer to avoid immediate playback
  }
}

//function to handle inverter level based audio
void handleInverterAudio() {
  static unsigned long lastTrackPlayTime = 0; // Last time tracks were played
  static int playCount = 0;                  // Count of how many times tracks played
  static bool canPlay = true;                // Flag to enable/disable playback
  unsigned long currentMillis = millis();

  // Check if SOC is in the desired range
  if (soc > 0 && soc < 35) {
    if (canPlay && playCount < 3) {
      if (currentMillis - lastTrackPlayTime >= 600000) { // 10 minutes = 600,000 ms
        queueAudioTrack(13);
        queueAudioTrack(14);
        playCount++;                  // Increment the play count
        lastTrackPlayTime = currentMillis; // Update last play time
      }
    } else if (playCount >= 3) {
      canPlay = false;                // Disable further playback until SOC resets
    }
  } else if (soc > 30) {
    // Reset playCount and re-enable playback when SOC goes above 30%
    playCount = 0;
    canPlay = true;
  }
}

//function to handle temperature based audio
void handleTemperatureAudio() {
  static unsigned long lastTrackPlayTime = 0; // Last time tracks were played
  static int playCount = 0;                    // Count of how many times tracks played
  static bool canPlay = true;                  // Flag to enable/disable playback
  unsigned long currentMillis = millis();
  float temperature = readTemperature();

  // Check if temperature is below 10°C and water level is below 50%
  if (temperature < 10 && previousWaterLevel < 50) { 
    if (canPlay && playCount < 4) {
      if (currentMillis - lastTrackPlayTime >= 600000) { // 10 minutes = 600,000 ms
        queueAudioTrack(15);
        queueAudioTrack(16);
        playCount++;                     // Increment the play count
        lastTrackPlayTime = currentMillis; // Update last play time
      }
    } else if (playCount >= 4) {
      canPlay = false;                    // Disable further playback after 5 plays
    }
  } else if (temperature >= 10 || previousWaterLevel >= 50) {
    // Reset playCount and re-enable playback when conditions are no longer met
    playCount = 0;
    canPlay = true;
  }
}

// Function to get the current resistance value from the sensor
float getResistance() {
    int rawValue = analogRead(MQ135_PIN); // Read the analog value
    float voltage = (rawValue / 4095.0) * 5.0; // Convert to voltage (5.0V reference)
    float resistance = ((5.0 / voltage) - 1.0) * RLOAD; // Calculate resistance
    return resistance;
}

// Function to get the smoke value
float getSmoke(float temperature, float humidity) {
    float resistance = getResistance();
    float ratio = resistance / baselineResistance;
    // Adjust the constants based on empirical data or sensor datasheet
    float ppm = pow(10, ((log10(ratio) - 0.43) / -1.44)); // Adjusted formula
    return ppm;
}
  // Function to toggle between automatic and manual modes
void toggleMode() {
  currentMode = (currentMode == MODE_AUTOMATIC) ? MODE_MANUAL : MODE_AUTOMATIC;
}
 
// Function to manually control the motor in manual mode
void toggleMotorManual() {
  if (currentMode == MODE_MANUAL ) {
    motorRunning = !motorRunning;
    digitalWrite(MOTOR, motorRunning ? HIGH : LOW);
  }
}

// Function to toggle the UV bulb on/off
void toggleUvBulb() {
  uvBulbOn = !uvBulbOn;
  digitalWrite(UV_BULB_PIN, uvBulbOn ? HIGH : LOW);

  // If motor is running, update UV bulb stop time
  if (motorRunning) {
    uvBulbStopTime = millis(); // Adjust this according to your needs
  }
}

// Function to toggle Light1 on/off
void toggleLIGHT1() {
  digitalWrite(LIGHT1, LIGHT1ON ? HIGH : LOW);
}

// Function to toggle Light1 on/off
void toggleLIGHT2() {
  digitalWrite(LIGHT2, LIGHT2ON ? HIGH : LOW);
  }
  
  // Function to toggle Light1 on/off
void toggleLIGHT3() {
  digitalWrite(LIGHT3, LIGHT3ON ? HIGH : LOW);
  }

//Dwin receives data from sensors
void sensor_data()
{
  int t = readTemperature();
  int h = readHumidity();
  int p = bmp.readPressure() / 100.0F;
  int d = dewPointFast(t, h);
  int a = bmp.readAltitude();
  int w = previousWaterLevel;
  int tc = previousWaterLevel * tankCapacityLiters / 100;
  int m = motorRunning;
  int u = uvBulbOn;
  int o = currentMode;
  int r = dryRunActive;
  int i = aqi;
  int c = co2;
  int Lt1 = LIGHT1ON;
  int Lt2 = LIGHT2ON;
  int Lt3 = LIGHT3ON;
  int so = soc;
  int bu = busVoltage;
  int cu = current;
  int po = power;
  int ma = currentVoltage;
  int ch = Charge;
  int ds = Discharge;
  int sr = shuntR;
  int sv = shuntvolt;
  int mo = motionDetectionForLight1;
  int mn = manualIntervention;
  
  Temperature[6] = highByte(t);
  Temperature[7] = lowByte(t);
  dwin.write(Temperature, 8);
 
  Humidity[6] = highByte(h);
  Humidity[7] = lowByte(h);
  dwin.write(Humidity, 8);
 
  Pressure[6] = highByte(p);
  Pressure[7] = lowByte(p);
  dwin.write(Pressure, 8);

  Altitude[6] = highByte(a);
  Altitude[7] = lowByte(a);
  dwin.write(Altitude, 8);
 
  DewPoint[6] = highByte(d);
  DewPoint[7] = lowByte(d);
  dwin.write(DewPoint, 8);
  
  waterLevel[6] = highByte(w);
  waterLevel[7] = lowByte(w);
  dwin.write(waterLevel, 8);

  Capacity[6] = highByte(tc);
  Capacity[7] = lowByte(tc);
  dwin.write(Capacity, 8);

  motor[6] = highByte(m);
  motor[7] = lowByte(m);
  dwin.write(motor, 8);

  uv[6] = highByte(u);
  uv[7] = lowByte(u);
  dwin.write(uv, 8);

  mod[6] = highByte(o);
  mod[7] = lowByte(o);
  dwin.write(mod, 8);

  dry[6] = highByte(r);
  dry[7] = lowByte(r);
  dwin.write(dry, 8);

  air[6] = highByte(i);
  air[7] = lowByte(i);
  dwin.write(air, 8);

  co[6] = highByte(c);
  co[7] = lowByte(c);
  dwin.write(co, 8);
  
  L1[6] = highByte(Lt1);
  L1[7] = lowByte(Lt1);
  dwin.write(L1, 8);

  L2[6] = highByte(Lt2);
  L2[7] = lowByte(Lt2);
  dwin.write(L2, 8);

  L3[6] = highByte(Lt3);
  L3[7] = lowByte(Lt3);
  dwin.write(L3, 8);

  SOC[6] = highByte(so);
  SOC[7] = lowByte(so);
  dwin.write(SOC, 8);

  BUS[6] = highByte(bu);
  BUS[7] = lowByte(bu);
  dwin.write(BUS, 8);

  CURRENT[6] = highByte(cu);
  CURRENT[7] = lowByte(cu);
  dwin.write(CURRENT, 8);

  POWER[6] = highByte(po);
  POWER[7] = lowByte(po);
  dwin.write(POWER, 8);

  MAINS[6] = highByte(ma);
  MAINS[7] = lowByte(ma);
  dwin.write(MAINS, 8);

  CHARGE[6] = highByte(ch);
  CHARGE[7] = lowByte(ch);
  dwin.write(CHARGE, 8);

  DISCHARGE[6] = highByte(ds);
  DISCHARGE[7] = lowByte(ds);
  dwin.write(DISCHARGE, 8);

  SHUNTR[6] = highByte(sr);
  SHUNTR[7] = lowByte(sr);
  dwin.write(SHUNTR, 8);

  SHUNTVOLT[6] = highByte(sv);
  SHUNTVOLT[7] = lowByte(sv);
  dwin.write(SHUNTVOLT, 8);

  MOTION[6] = highByte(mo);
  MOTION[7] = lowByte(mo);
  dwin.write(MOTION, 8);

  MANUAL[6] = highByte(mn);
  MANUAL[7] = lowByte(mn);
  dwin.write(MANUAL, 8);
}
/*----------DewPoint Calculation--------*/
double dewPointFast(double celsius, double humidity)
{
  double a = 17.271;
  double b = 237.7;
  double temp = (a * celsius) / (b + celsius) + log(humidity * 0.01);
  double Td = (b * temp) / (a - temp);
  return Td;
}
 
// relay control from dwim hmi
void realy_control()
{
  unsigned long startMillis = millis();
  unsigned long timeout = 500;
  
  while (dwin.available())
  {
    for (int i = 0; i <= 8; i++) //this loop will store whole frame in buffer array.
    {
      Buffer[i] = dwin.read();
      delay(5);
    }
 
    if (Buffer[0] == 0X5A)
    {
      switch (Buffer[4])
      {
        case 0x54:   //for MOTOR
          if (Buffer[8] == 1)
          {
            digitalWrite(MOTOR, HIGH);
            motorRunning = true;
            Serial.println("MOTOR ON");
          }
          else
          {
            digitalWrite(MOTOR, LOW);
            motorRunning = false;
          }
          updateDisplay(); 
          break;
 
        case 0x53:   //for UV_BULB_PIN
          if (Buffer[8] == 1)
          {
            digitalWrite(UV_BULB_PIN, HIGH);
            uvBulbOn = true;
          }
          else
          {
            digitalWrite(UV_BULB_PIN, LOW);
            uvBulbOn = false;
          }
          updateDisplay(); 
          break;
 
        case 0x55:   //for LIGHT1
          if (Buffer[8] == 1)
          {
            digitalWrite(LIGHT1, HIGH);
            LIGHT1ON = true;
          }
          else
          {
            digitalWrite(LIGHT1, LOW);
            LIGHT1ON = false;
          }
          updateDisplay();
          break;
 
        case 0x56:   //for LIGHT2
          if (Buffer[8] == 1)
          {
            digitalWrite(LIGHT2, HIGH);
            LIGHT2ON = true;
          }
          else
          {
            digitalWrite(LIGHT2, LOW);
            LIGHT2ON = false;
          }
          updateDisplay(); 
          break;

          case 0x58:   //for LIGHT3
          if (Buffer[8] == 1)
          {
            digitalWrite(LIGHT3, HIGH);
            LIGHT3ON = true;  
          }
          else
          {
            digitalWrite(LIGHT3, LOW);
            LIGHT3ON = false;
          }
          updateDisplay(); 
          break;

       case 0x57:   //for mode change
          if (Buffer[8] == 1)
          {
            
            currentMode = MODE_MANUAL;
          }
          else
          {
           
           currentMode = MODE_AUTOMATIC;
          }
          
          updateDisplay(); 
          break;

          case 0x65:   //for reset
          if (Buffer[8] == 1)
          {
           esp_restart();
          }
          else
          {
            esp_restart();
          }
          updateDisplay(); 
          break;
          
          case 0x69:   //for reset EEPROM
          if (Buffer[8] == 1)
          {
           resetEEPROM();
          }
          else
          {
            resetEEPROM();
          }
          updateDisplay(); 
          break;
          
          case 0x70:   //for light A
          if (Buffer[8] == 1)
          {
            sendCode(467170); // RF code for LIGHT A ON/OFF
          }
          else
          {
            sendCode(467170); // RF code for LIGHT A ON/OFF
          }
          updateDisplay(); 
          break;

          case 0x71:   //for light B
          if (Buffer[8] == 1)
          {
           sendCode(467176); // RF code for LIGHT B ON/OFF
          }
          else
          {
            sendCode(467176); // RF code for LIGHT BON/OFF
          }
          updateDisplay(); 
          break;

          case 0x72:   //for light C
          if (Buffer[8] == 1)
          {
           sendCode(467169); // RF code for LIGHT C ON/OFF
          }
          else
          {
            sendCode(467169); // RF code for LIGHT C ON/OFF
          }
          updateDisplay(); 
          break;

          case 0x73:   //for light D
          if (Buffer[8] == 1)
          {
            sendCode(467172); // RF code for LIGHT D ON/OFF
          }
          else
          {
            sendCode(467172); // RF code for LIGHT D ON/OFF
          }
          updateDisplay(); 
          break;

          case 0x74:   //for ALL ON 
          if (Buffer[8] == 1)
          {
           sendCode(467180); // RF code for ALL ON
          }
          else
          {
            sendCode(467180); // RF code for ALL ON
          }
          updateDisplay(); 
          break;

          case 0x75:   //for ALL OFF 
          if (Buffer[8] == 1)
          {
           sendCode(467171); // RF code for ALL OFF
          }
          else
          {
            sendCode(467171); // RF code for ALL OFF
          }
          updateDisplay(); 
          break;
          
          case 0x76:   //for motion sensor
          if (Buffer[8] == 1)
          {
           toggleMotionDetectionForLight1();
           motionDetectionForLight1 = false;
          }
          else
          {
           toggleMotionDetectionForLight1();
           motionDetectionForLight1 = true;  
          }
          updateDisplay(); 
          break;

          case 0x77:   //for motion sensor
          if (Buffer[8] == 1)
          {
           resetManualIntervention(); // reset manual intervention
          }
          else
          {
           resetManualIntervention(); // reset manual intervention  
          }
          updateDisplay(); 
          break;
 
        default:
          Serial.println("No data..");
      }
    }
  }
}
