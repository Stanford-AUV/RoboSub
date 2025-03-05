#include <Servo.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include "MS5837.h"
#include "Adafruit_MCP9808.h"
#include <Adafruit_BME280.h>
#include <ILI9341_t3.h>
////////////////////////////////////////////////////////////////////////////
////////////////Pin Definitions/////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// Display configuration
#define TFT_CS 10
#define TFT_DC 31
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

// Global display update timing
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 500; // update display every 500 ms

// SD Card
int sd_loop_counter = 0;
const int chipSelect = BUILTIN_SDCARD;
unsigned long loopIterationCounter = 0; // for perioidic SD logging
// int sdLoggingFrequency = 10000;
int sdLoggingFrequency = 40000; 

// Depth Sensor
MS5837 sensor;
bool sensorInitialized = false;

// Tempsensors (0x18, 0x19, 0x1A, I2C_Bus_1)
Adafruit_MCP9808 tempsensor1 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsensor2 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsensor3 = Adafruit_MCP9808();
String tempSensorNames[] = {"IMU_Temp", "ESC_Temp", "Orin_Temp"};

// BME280 Sensors (addresses 0x76 and 0x77)
Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
float bmeValues[2][3];
String bmeSensorNames[] = {"Bulkhead_BME", "MID_BME"};

// LED Indicators
int redIndicatorLedPin = 37;
int greenIndicatorLedPin = 36;
int lightPWM;

// Kill Switch
int killSwitchPin = 26;

// Battery sensing
int currentPin = 41;
int voltagePin = 40;
int currentVal = 0;
int voltageVal = 0;

// This flag determines operational status
// 0. Teensy and/or power off. LED OFF
// 1. All systems nominal. LED Solid ON
// 2. Kill switch on. LED slowly blinking.
// 3. Sensor issue detected. LED blinking 
const int OFF = 0;
const int NOMINAL = 1;
const int KILLED = 2;
unsigned long previousKillMillis = 0;
const unsigned long blinkKillInterval = 500; // Blink every 500 ms (2 seconds)
const int SENSOR_ISSUE = 3;
const unsigned long sensorIssueLongDuration  = 1000; // long duration (ms)
const unsigned long sensorIssueShortDuration = 250;  // short duration (ms)
const unsigned long sensorIssueOffDuration   = 1000; // off duration (ms)
const int sensorIssueStepsCount = 6;
unsigned long previousSensorIssueMillis = 0;
int sensorIssueStep = 0;
const unsigned long sensorIssueDurations[sensorIssueStepsCount] = {
    sensorIssueLongDuration,  // step 0: long (LED on)
    sensorIssueLongDuration,  // step 1: long (LED on)
    sensorIssueShortDuration, // step 2: short (LED on)
    sensorIssueShortDuration, // step 3: short (LED on)
    sensorIssueLongDuration,  // step 4: long (LED on)
    sensorIssueOffDuration    // step 5: off (LED off)
};
const bool sensorIssueStates[sensorIssueStepsCount] = {
    true,  // LED on during step 0
    true,  // LED on during step 1
    true,  // LED on during step 2
    true,  // LED on during step 3
    true,  // LED on during step 4
    false  // LED off during step 5
};
int operational = NOMINAL;

// Servo control
Servo servo[8];
byte servoPins[] = {0, 1, 2, 3, 4, 5, 6, 7};
int lastThrusterPWM[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Serial Input Buffer
char inputBuffer[64]; // Buffer to store incoming serial data
int bufferPosition = 0; // Position in the buffer

// Digits for sensor output formatting
int DIGITS = 8;

// Light Control
Servo lightServo;
int LumenPin = 8;
int lightVal = 1100;

////////////////////////////////////////////////////////////////////////////
///////////////////////Setup////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 8; i++) {
    servo[i].attach(servoPins[i]);
  }
  config_servo();
  config_depth_sensor();
  config_sd_card();

  tft.begin();
  tft.fillScreen(ILI9341_BLUE);
  tft.setRotation(1); 

  pinMode(redIndicatorLedPin, OUTPUT);
  pinMode(greenIndicatorLedPin, OUTPUT);
  // digitalWrite(greenIndicatorLedPin, HIGH);
  pinMode(killSwitchPin, INPUT_PULLDOWN);

  tempsensor1.begin(0x18);
  tempsensor1.setResolution(3);
  tempsensor1.wake();
  tempsensor2.begin(0x19);
  tempsensor2.setResolution(3);
  tempsensor2.wake();
  tempsensor3.begin(0x1A);
  tempsensor3.setResolution(3);
  tempsensor3.wake();

  // Initialize the two BME280 sensors
  if (!bme1.begin(0x76)) {
    Serial.println("BME280 sensor not detected at 0x76!");
    operational = SENSOR_ISSUE;
  }
  if (!bme2.begin(0x77)) {
    Serial.println("BME280 sensor not detected at 0x77!");
    operational = SENSOR_ISSUE;
  }

  // Attach the light servo
  lightServo.attach(LumenPin);
  lightServo.writeMicroseconds(1100); // turn off the light
}

void config_servo() {
  for (int i = 0; i < 8; i++) {
    servo[i].writeMicroseconds(1500); // Default neutral position
  }
}

void config_depth_sensor() {
  Wire.begin();
  // sensor.setModel(MS5837::MS5837_02BA); // model number for Bar02
  sensorInitialized = sensor.init();   // Check if the sensor is detected
  if (!sensorInitialized) {
    // Serial.println("MS5837 sensor not detected!");
    operational = SENSOR_ISSUE; // flag this
  } else {
    // Serial.println("MS5837 sensor detected");
    sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
  }
}

void config_sd_card() {
  // see if the card is present and can be initialized
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
  }
}

////////////////////////////////////////////////////////////////////////////
///////////////////////Loop and Serial Processing///////////////////////////
////////////////////////////////////////////////////////////////////////////
void loop() {
  // Increment the loop counter and log to SD every 200 iterations
  loopIterationCounter++;
  if (loopIterationCounter % sdLoggingFrequency == 0) {
    logPeriodicData();
  }

  // Update operational status based on kill switch
  int killSwitch = !digitalRead(killSwitchPin); // Read the value from the pin (HIGH or LOW)
  operational = (killSwitch == 1) ? KILLED : NOMINAL;

  if (operational == NOMINAL) {
    digitalWrite(greenIndicatorLedPin, HIGH);
    digitalWrite(redIndicatorLedPin, LOW);
  } else if (operational == KILLED) { // If KILLED, reset motors and blink red LED
    config_servo();
    digitalWrite(greenIndicatorLedPin, LOW);
    unsigned long currentMillis = millis();
    if (currentMillis - previousKillMillis >= blinkKillInterval) {
      previousKillMillis = currentMillis;
      lightPWM = !lightPWM;
      digitalWrite(redIndicatorLedPin, lightPWM ? HIGH : LOW);
    }
    return; // Skip further processing when killed
  } else if (operational == SENSOR_ISSUE) {  // In SENSOR_ISSUE state, blink with the custom pattern.
    unsigned long currentMillis = millis();
    if (currentMillis - previousSensorIssueMillis >= sensorIssueDurations[sensorIssueStep]) {
      previousSensorIssueMillis = currentMillis;
      digitalWrite(redIndicatorLedPin, sensorIssueStates[sensorIssueStep] ? HIGH : LOW);  // Set LED according to the current step in the pattern.
      sensorIssueStep = (sensorIssueStep + 1) % sensorIssueStepsCount;  // Move to the next step (wrap around at the end of the pattern)
    }
    return; // End current loop execution
  }

  // Process incoming serial data
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') { // End of one command
      inputBuffer[bufferPosition] = '\0'; // Null-terminate the string
      process_input(inputBuffer);
      bufferPosition = 0;
      sd_loop_counter++;
    } else {
      if (bufferPosition < (int)sizeof(inputBuffer) - 1) { // Prevent buffer overflow
        inputBuffer[bufferPosition++] = inChar;
      }
    }
  }
}

void process_input(char *input) {
  if (strcmp(input, "transfer") == 0) { // SD Card Transfer
    transfer_sd_log();
  }
  else if (strcmp(input, "delete") == 0) { // SD Card Delete
    delete_sd_log();
  }
  // Handle usual command
  int s0, s1, s2, s3, s4, s5, s6, s7;
  int servoNum, val;
  if (sscanf(input, "%d %d %d %d %d %d %d %d", &s0, &s1, &s2, &s3, &s4, &s5, &s6, &s7) == 8) {
    set_servo(0, s0);
    set_servo(1, s1);
    set_servo(2, s2);
    set_servo(3, s3);
    set_servo(4, s4);
    set_servo(5, s5);
    set_servo(6, s6);
    set_servo(7, s7);

    lastThrusterPWM[0] = s0;
    lastThrusterPWM[1] = s1;
    lastThrusterPWM[2] = s2;
    lastThrusterPWM[3] = s3;
    lastThrusterPWM[4] = s4;
    lastThrusterPWM[5] = s5;
    lastThrusterPWM[6] = s6;
    lastThrusterPWM[7] = s7;

    float pressure, temperature, depth;
    handle_depth_command(pressure, temperature, depth);
    float current, voltage;
    handle_voltage_command(current, voltage);

    String dataString = "> pressure:" + String(pressure, DIGITS) +
    " temperature:" + String(temperature, DIGITS) +
    " depth:" + String(depth, DIGITS) +
    " current:" + String(current, DIGITS) +
    " voltage:" + String(voltage, DIGITS) +
    " servo:" + String(s0) + "," + String(s1) + "," + String(s2) + "," +
    String(s3) + "," + String(s4) + "," + String(s5) + "," +
    String(s6) + "," + String(s7);
    Serial.println(dataString);

    logPeriodicData();
  } else if (strcmp(input, "batt") == 0) { // for debugging
    float current, voltage;
    handle_voltage_command(current, voltage);
    Serial.println("Current:" + String(current, DIGITS) + " voltage:" + String(voltage, DIGITS));
  } else if (strcmp(input, "bar") == 0) {
    float pressure, temperature, depth;
    handle_depth_command(pressure, temperature, depth);
    Serial.println("> pressure:" + String(pressure, DIGITS) + " temperature:" + String(temperature, DIGITS) + " depth:" + String(depth, DIGITS));
  } else if (strcmp(input, "temp") == 0) {
    float temps[3];
    readTempSensors(temps);
    for (int i = 0; i < 3; i++) {
      Serial.print(tempSensorNames[i] + ": ");
      Serial.println(temps[i]);
    }
    // Read BME sensor values into a 2D array
    float bmeVals[2][3];
    readBmeSensorsArray(bmeVals);
    // Loop through the two BME sensors and print their values
    for (int i = 0; i < 2; i++) {
      Serial.print(bmeSensorNames[i] + ": ");
      Serial.print("Temperature: " + String(bmeVals[i][0]) + " °C, ");
      Serial.print("Humidity: " + String(bmeVals[i][1]) + " %, ");
      Serial.println("Pressure: " + String(bmeVals[i][2]) + " hPa");
    }
  } else if (strcmp(input, "test") == 0) {
    test_servos();
  } else if (sscanf(input, "%d %d", &servoNum, &val) == 2) { //for debugging without the VM
    if (val >= 1100 && val <= 1900 && servoNum >= 0 && servoNum <= 7) {
      set_servo(servoNum, val);
      lastThrusterPWM[servoNum] = val;
      Serial.println("Servo " + String(servoNum) + " set to " + String(val));
    } else {
      Serial.println("Invalid command");
    }
  } else if (sscanf(input, "light %d", &lightVal) == 1) {
    if (lightVal >= 1100 && lightVal <= 1900) {
      lightServo.writeMicroseconds(lightVal);
      Serial.println("Light set to " + String(lightVal));
    } else {
      Serial.println("Invalid light value. Please enter a value between 1100 and 1900.");
    }
    logPeriodicData();
  } else {
    Serial.println(input);
  }
}

////////////////////////////////////////////////////////////////////////////
///////////////// SD Card Functions/////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void logPeriodicData() {
  float current, voltage;
  handle_voltage_command(current, voltage);

  float pressure, temperature, depth;
  handle_depth_command(pressure, temperature, depth);

  float temps[3];
  readTempSensors(temps);

  float bmeVals[2][3];
  readBmeSensorsArray(bmeVals);
  
  String dataString = "> current:" + String(current, DIGITS) +
                  " voltage:" + String(voltage, DIGITS) +
                  " servo:"; 
  for (int i = 0; i < 8; i++) {
    dataString += "PWM:" + String(lastThrusterPWM[i]);
    if (i < 7)
      dataString += ",";
  }

  dataString += " internalStates:" + 
                String("Operational") + ":" + String(operational) +
                "," + String("KillSwitch") + ":" + String(digitalRead(killSwitchPin)) + ",";

  dataString += " temps:" + 
                String(tempSensorNames[0]) + ":" + String(temps[0], DIGITS) +
                "," + String(tempSensorNames[1]) + ":" + String(temps[1], DIGITS) +
                "," + String(tempSensorNames[2]) + ":" + String(temps[2], DIGITS) +
                "," + String(bmeSensorNames[0]) + ":" + String(bmeVals[0][0], DIGITS) +
                "," + String(bmeSensorNames[1]) + ":" + String(bmeVals[1][0], DIGITS) + ",";
  
  dataString += " humidity:" +  
                String(bmeSensorNames[0]) + ":" + String(bmeVals[0][1], DIGITS) +
                "," + String(bmeSensorNames[1]) + ":" + String(bmeVals[1][1], DIGITS) + ",";

  dataString += " pressure:" +
                String(bmeSensorNames[0]) + ":" + String(bmeVals[0][2], DIGITS) +
                "," + String(bmeSensorNames[1]) + ":" + String(bmeVals[1][2], DIGITS) + ",";

  dataString += "EXTpressure:" + String(pressure, DIGITS) + "," +
                "EXTtemperature:" + String(temperature, DIGITS) + "," +
                "EXTdepth:" + String(depth, DIGITS) + ",";

  write_data_sd(dataString);
}

void transfer_sd_log(){
  if (SD.exists("datalog.txt")) {
    Serial.println("Transfering datalog.txt ...");
    // Open the file for reading
    File dataFile = SD.open("datalog.txt", FILE_READ);
    if (dataFile) {
      // Read the file and send its content to the Serial port
      while (dataFile.available()) {
        Serial.write(dataFile.read());
      }
      dataFile.close();
      Serial.println("\nFile transfer complete. Save the output on your local device as datalog.txt.");
    } else {
      Serial.println("Error opening datalog.txt for reading.");
    }
  } else {
    Serial.println("datalog.txt does not exist.");
  }
}
void delete_sd_log() {
  if (SD.exists("datalog.txt")) {
      // Remove the file
      if (SD.remove("datalog.txt")) {
        Serial.println("datalog.txt deleted.");
      } else {
        Serial.println("Error deleting datalog.txt.");
      }
      // Recreate the file
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.close();
        Serial.println("datalog.txt recreated.");
      } else {
        Serial.println("Error recreating datalog.txt.");
      }
    } else {
      Serial.println("datalog.txt does not exist.");
    }
}
void write_data_sd(String dataString) {
  // Get the timestamp
  String timestamp = getTimestamp();
  // open the file.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it
  if (dataFile) {
    dataFile.println(timestamp + " -" + dataString);
    dataFile.close();
    //Serial.println("logged data!!");
    //Serial.println(timestamp + " -" + dataString);
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
}
String getTimestamp() {
  unsigned long milliseconds = millis();
  unsigned long seconds = milliseconds / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;

  milliseconds %= 1000;
  seconds %= 60;
  minutes %= 60;

  // Format the timestamp
  String timestamp = "";
  if (hours < 10) timestamp += "0"; // Add leading zero for hours
  timestamp += String(hours) + ":";
  if (minutes < 10) timestamp += "0"; // Add leading zero for minutes
  timestamp += String(minutes) + ":";
  if (seconds < 10) timestamp += "0"; // Add leading zero for seconds
  timestamp += String(seconds) + ".";
  if (milliseconds < 100) timestamp += "0"; // Add leading zeros for milliseconds
  if (milliseconds < 10) timestamp += "0";
  timestamp += String(milliseconds);

  return timestamp;
}
//////////////////////////////////////////////////////////////////////////////
/// Sensor Reading Functions
//////////////////////////////////////////////////////////////////////////////
void handle_depth_command(float& pressure, float& temperature, float& depth) {
  sensor.read();
  pressure = sensor.pressure(); // mbar
  temperature = sensor.temperature(); // C
  depth = sensor.depth(); // m
}
void handle_voltage_command(float& current, float& voltage) {
  currentVal = analogRead(currentPin);
  voltageVal = analogRead(voltagePin);
  current = (currentVal * 120.0) / 1024; // A
  voltage = (voltageVal * 60.0) / 1024; // V
}
void readTempSensors(float temps[3]) {
    temps[0] = tempsensor1.readTempC();
    temps[1] = tempsensor2.readTempC();
    temps[2] = tempsensor3.readTempC();
}
// Read both BME280 sensors and store their values in a 2D array.
void readBmeSensorsArray(float bmeVals[2][3]) {
  // Sensor 1 (address 0x76)
  bmeVals[0][0] = bme1.readTemperature();
  bmeVals[0][1] = bme1.readHumidity();
  bmeVals[0][2] = bme1.readPressure() / 100.0F; // Convert from Pa to hPa
  
  // Sensor 2 (address 0x77)
  bmeVals[1][0] = bme2.readTemperature();
  bmeVals[1][1] = bme2.readHumidity();
  bmeVals[1][2] = bme2.readPressure() / 100.0F;
}

////////////////////////////////////////////////////////////////////////////
// Servo Control
////////////////////////////////////////////////////////////////////////////
void test_servos() {
  for (int i = 0; i < 8; i++) {
    // Set the current servo to 1550 microseconds
    set_servo(i, 1550);
    delay(2000);  // wait 500 ms for the servo to move
    // Return the servo to its neutral position (1500 microseconds)
    set_servo(i, 1500);
    delay(500);  // wait 500 ms before moving to the next servo
  }
}
void set_servo(int servoNum, int val) {
    int remappedServo;
    switch (servoNum) {
        case 0: remappedServo = 7; break;
        case 1: remappedServo = 1; break;
        case 2: remappedServo = 6; break;
        case 3: remappedServo = 5; break;
        case 4: remappedServo = 0; break;
        case 5: remappedServo = 4; break;
        case 6: remappedServo = 2; break;
        case 7: remappedServo = 3; break;
        default: return;
    }
    if (remappedServo >= 0 && remappedServo <= 7 && val >= 1100 && val <= 1900) {
        servo[remappedServo].writeMicroseconds(val);
    }
}

/// DISPLAY
#define FRAME_X 210
#define FRAME_Y 180
#define FRAME_W 100
#define FRAME_H 50

#define REDBUTTON_X FRAME_X
#define REDBUTTON_Y FRAME_Y
#define REDBUTTON_W (FRAME_W/2)
#define REDBUTTON_H FRAME_H

#define GREENBUTTON_X (REDBUTTON_X + REDBUTTON_W)
#define GREENBUTTON_Y FRAME_Y
#define GREENBUTTON_W (FRAME_W/2)
#define GREENBUTTON_H FRAME_H

void drawFrame()
{
  tft.drawRect(FRAME_X, FRAME_Y, FRAME_W, FRAME_H, ILI9341_BLACK);
}
