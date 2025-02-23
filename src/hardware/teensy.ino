#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;
int currentPin = 41;
int voltagePin = 40;
int currentVal = 0;
int voltageVal = 0;

Servo servo[8];
byte servoPins[] = {0, 1, 2, 3, 4, 5, 6, 7};
char inputBuffer[64]; // Buffer to store incoming serial data
int bufferPosition = 0; // Position in the buffer

int DIGITS = 8;

void setup() {
  Serial.begin(9600);
  config_servo();
  config_depth_sensor();
}

void config_servo() {
  for (int i = 0; i < 8; i++) {
    servo[i].attach(servoPins[i]);
    servo[i].writeMicroseconds(1500); // Default neutral position
  }
}

void config_depth_sensor() {
  Wire.begin();
  sensor.setModel(MS5837::MS5837_02BA); // model number for Bar02
  sensor.init();
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
}

void loop() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') { // End of one command
      inputBuffer[bufferPosition] = '\0'; // Null-terminate the string
      process_input(inputBuffer);
      bufferPosition = 0; // Reset buffer for the next command
    } else {
      if (bufferPosition < (int)sizeof(inputBuffer) - 1) { // Prevent buffer overflow
        inputBuffer[bufferPosition++] = inChar;
      }
    }
  }
}

void process_input(char *input) {
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
    Serial.println("> pressure:" + String(pressure, DIGITS) + " temperature:" + String(temperature, DIGITS) + " depth:" + String(depth, DIGITS) + " current:" + String(current, DIGITS) + " voltage:" + String(voltage, DIGITS));
  } else if (strcmp(input, "batt") == 0) { // for debugging
    float current, voltage;
    handle_voltage_command(current, voltage);
    Serial.println("Current:" + String(current, DIGITS) + " voltage:" + String(voltage, DIGITS));
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
