#include <Servo.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include "MS5837.h"
#include "Adafruit_MCP9808.h"

// SD Card
int sd_loop_counter = 0;
const int chipSelect = BUILTIN_SDCARD;

// Depth Sensor
MS5837 sensor;
bool sensorInitialized = false;

// Tempsensors (0x18, 0x19, 0x1A, I2C_Bus_1)
Adafruit_MCP9808 tempsensor1 = Adafruit_MCP9808();
Adafruit_MCP9808 ESCtempsensor = Adafruit_MCP9808();
Adafruit_MCP9808 tempsensor3 = Adafruit_MCP9808();

// LED Indicators
int indicatorLedPin = 37;
int lightPWM;

// Kill Switch
int killSwitchPin = 26;

// Voltage sensing (note current sensing non-functional as of 2/23/25)
int currentPin = 41;
int voltagePin = 40;
int currentVal = 0;
int voltageVal = 0;

// This flag determines operational status
// 0. Teensy and/or power off. LED OFF
// 1. All systems nominal. LED Solid ON
// 2. Kill switch on. LED slowly blinking.
// 3. Leak detected. LED blinking very quickly
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
char inputBuffer[64]; // Buffer to store incoming serial data
int bufferPosition = 0; // Position in the buffer
int DIGITS = 8;

void setup() {
  Serial.begin(9600);
  config_servo();
  config_depth_sensor();
  config_sd_card();
  pinMode(indicatorLedPin, OUTPUT);
  pinMode(killSwitchPin, INPUT_PULLDOWN);

  tempsensor1.begin(0x18);
  tempsensor1.setResolution(3);
  tempsensor1.wake();

  ESCtempsensor.begin(0x19);
  ESCtempsensor.setResolution(3);
  ESCtempsensor.wake();

  tempsensor3.begin(0x1A);
  tempsensor3.setResolution(3);
  tempsensor3.wake();
}

String getTimestamp() {
  unsigned long milliseconds = millis();
  unsigned long seconds = milliseconds / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  milliseconds %= 1000;
  seconds %= 60;
  minutes %= 60;
  char timestamp[13]; // HH:MM:SS.mmm
  sprintf(timestamp, "%02lu:%02lu:%02lu.%03lu", hours, minutes, seconds, milliseconds);
  return String(timestamp);
}

void config_servo() {
  for (int i = 0; i < 8; i++) {
    servo[i].attach(servoPins[i]);
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

void loop() {
  // Operational considerations
  int killSwitch = !digitalRead(killSwitchPin); // Read the value from the pin (HIGH or LOW)
  if (killSwitch == 1) {
    operational = KILLED;
  } else {
    operational = NOMINAL;
  }

  // Serial.print("Operational status: "); // for debugging
  // Serial.println(operational);    
  if (operational == NOMINAL) {
    digitalWrite(indicatorLedPin, HIGH);
  } else if (operational == KILLED) {
    config_servo();
    unsigned long currentMillis = millis();
    if (currentMillis - previousKillMillis >= blinkKillInterval) {
      previousKillMillis = currentMillis;
      if (lightPWM == 1) {
          digitalWrite(indicatorLedPin, LOW);
          lightPWM = 0;
      } else {
          digitalWrite(indicatorLedPin, HIGH);
          lightPWM = 1;
      }
    }
    return; // End current loop execution
  } else if (operational == SENSOR_ISSUE) {
    // In SENSOR_ISSUE state, blink with the custom pattern.
    unsigned long currentMillis = millis();
    if (currentMillis - previousSensorIssueMillis >= sensorIssueDurations[sensorIssueStep]) {
      previousSensorIssueMillis = currentMillis;
      // Set LED according to the current step in the pattern.
      digitalWrite(indicatorLedPin, sensorIssueStates[sensorIssueStep] ? HIGH : LOW);
      // Move to the next step (wrap around at the end of the pattern)
      sensorIssueStep = (sensorIssueStep + 1) % sensorIssueStepsCount;
    }
    return; // End current loop execution
  }

  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') { // End of one command
      inputBuffer[bufferPosition] = '\0'; // Null-terminate the string
      // log to sd card every 150 inputs
      if (sd_loop_counter % 150 == 0) {
        process_input(inputBuffer, true);
      } else {
        process_input(inputBuffer, false);
      }
      bufferPosition = 0; // Reset buffer for the next command
      sd_loop_counter++;
    } else {
      if (bufferPosition < (int)sizeof(inputBuffer) - 1) { // Prevent buffer overflow
        inputBuffer[bufferPosition++] = inChar;
      }
    }
  }
}

void process_input(char *input, bool log_sd) {
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
    float pressure, temperature, depth;
    handle_depth_command(pressure, temperature, depth);
    float current, voltage;
    handle_voltage_command(current, voltage);
    String dataString = "> pressure:" + String(pressure, DIGITS) + " temperature:" + String(temperature, DIGITS) + " depth:" + String(depth, DIGITS) + " current:" + String(current, DIGITS) + " voltage:" + String(voltage, DIGITS);
    Serial.println(dataString);
    if (log_sd) {  // log to sd card
      float temps[3];
      readTempSensors(temps);
      write_data_sd(dataString + " servo:" + s0 + "," + s1 + "," + s2 + "," + s3 + "," + s4 + "," + s5 + "," + s6 + "," + s7 + "temps:" + temps[0] + "," + temps[1] + "," + temps[2]);
    }
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
        Serial.print("Temperature ");
        Serial.print(i+1) ;
        Serial.print(": ");
        Serial.println(temps[i]);
    }
  } else if (strcmp(input, "test") == 0) {
    test_servos();
  } else if (sscanf(input, "%d %d", &servoNum, &val) == 2) { //for debugging without the VM
    if (val >= 1100 && val <= 1900 && servoNum >= 0 && servoNum <= 7) {
      set_servo(servoNum, val);
    } else {
      Serial.println("Invalid command");
    }
  } else {
    Serial.println(input);
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
    temps[1] = ESCtempsensor.readTempC();
    temps[2] = tempsensor3.readTempC();
}

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
