#include <Wire.h>
#include "Door.h"
#include "Light.h"

// Arbitrary I2C address for this device
const int I2C_ADDRESS = 0x10;

const int I2C_COMMAND_DOOR_MASK = 0x10;
const int I2C_COMMAND_LIGHT_MASK = 0x20;
const int I2C_COMMAND_MASK = 0x0F;

// Pins connected to buttons to open and close the door
// Switches should be momentary normally open with the switch input wired to ground.
// Pullups should be enabled on these pins.
const int OPEN_BUTTON_PIN = A0;
const int CLOSE_BUTTON_PIN = A1;

// Pins connected to switches to open and close the door
// Switches should be momentary normally open with the switch input wired to ground.
// Pullups should be enabled on these pins.
const int LIGHT_ON_BUTTON_PIN = A2;
const int LIGHT_OFF_BUTTON_PIN = A3;

// Pin to output a heartbeat signal, on Arduino, 13 is usually connected to an LED.
const int HEARTBEAT_PIN = LED_BUILTIN;

// Arbitrary number of polling cycles to change the heartbeat output
//  This makes it toggle roughly 4 times a second.
const int HEARTBEAT_COUNTER_RESET = 50;

int heartbeatCounter = 0;

byte millisHighByte = 0;
boolean millisHigh = false;

void setup() {
  setupDoor();
  setupLight();

  pinMode(OPEN_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CLOSE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LIGHT_ON_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LIGHT_OFF_BUTTON_PIN, INPUT_PULLUP);

  pinMode(HEARTBEAT_PIN, OUTPUT);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(processI2cWrite);
  Wire.onRequest(processI2cRead);
}


void loop() {
  updateDoor();
  updateLight();
  pollDoorButtons();
  pollLightButtons();
  updateHeartbeat();
  updateUptime();
  delay(5);
}


/**
 * Updates counters and state for a visible heartbeat.
 */
void updateHeartbeat() {
  heartbeatCounter++;
  if (heartbeatCounter >= HEARTBEAT_COUNTER_RESET) {
    heartbeatCounter = 0;
    noInterrupts();
    digitalWrite(HEARTBEAT_PIN, !digitalRead(HEARTBEAT_PIN));
    interrupts();
  }  
}


/**
 * Updates the variables associated with tracking the uptime.  Since millis() will overflow
 * every roughly 50 days, we add an extra byte which is incremented on every overflow.  This
 * stretches the overflow to onece every approximately 35 years.
 */
void updateUptime() {
  unsigned long currentTime = millis();
  if (!millisHigh && (currentTime > 0x7FFFFFFF)) {
    millisHigh = true;
  }
  else if (millisHigh && (currentTime <= 0x7FFFFFFF)) {
    millisHighByte++;
    millisHigh = false;
  }
}


/**
 * Poll the buttons for manually opening and closing the door.  Since these are not interrupt driven, the button
 * will need to be held down at least one polling cycle.
 */
void pollDoorButtons() {
  noInterrupts();

  if (digitalRead(OPEN_BUTTON_PIN) == LOW) {
    requestDoorCommand(DOOR_COMMAND_OPEN);
  }
  else if (digitalRead(CLOSE_BUTTON_PIN) == LOW) {
    requestDoorCommand(DOOR_COMMAND_CLOSE);
  }

  interrupts();  
}


/**
 * Poll the buttons for manually turning the light on and off.  Since these are not interrupt driven, the button
 * will need to be held down at least one polling cycle.
 */
void pollLightButtons() {
  noInterrupts();

  if (digitalRead(LIGHT_ON_BUTTON_PIN) == LOW) {
    requestLightCommand(LIGHT_COMMAND_ON);
  }
  else if (digitalRead(LIGHT_OFF_BUTTON_PIN) == LOW) {
    requestLightCommand(LIGHT_COMMAND_OFF);
  }

  interrupts();  
}


/**
 * Processes and I2C write request.
 * 
 * Data structure for write:
 * Byte 0: Top 4 bits: command type 0x1 for door, 0x2 for light
 * Byte 0: Bottom 4 bits: command appropriate for type (DOOR_COMMAND_* or LIGHT_COMMAND_*
 */
void processI2cWrite(int bytes) {
  int data = Wire.read();
  if ((data & I2C_COMMAND_DOOR_MASK) != 0) {
    requestDoorCommand(data & I2C_COMMAND_MASK);
  }
  else if ((data & I2C_COMMAND_LIGHT_MASK) != 0) {
    requestLightCommand(data & I2C_COMMAND_MASK);
  }
}


/**
 * Processes an I2C read request.
 * 
 * Data structure for read:
 * Byte 0: Version: 0x01
 * Byte 1: Top 4 bits: door state, Bottom 4 bits: light state
 * Bytes 2-6: approximate uptime in millis (big endian), note: overflows approximately every 35 years
 */
void processI2cRead() {
  byte data[7];
  data[0] = 0x01;
  data[1] = (getDoorState() << 4) | getLightState();

  unsigned long currentUptime = millis();
  data[2] = millisHighByte;
  data[3] = (currentUptime >> 24) & 0xFF;
  data[4] = (currentUptime >> 16) & 0xFF;
  data[5] = (currentUptime >>  8) & 0xFF;
  data[6] = (currentUptime      ) & 0xFF;

  Wire.write(data, 7);
}

