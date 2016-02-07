#include <Wire.h>
#include "Door.h"
#include "Light.h"

// Arbitrary I2C address for this device
const int I2C_ADDRESS = 0x10;

// I2C command bytes
const byte I2C_COMMAND_DOOR = 0x01;
const byte I2C_COMMAND_LIGHT = 0x02;
const byte I2C_COMMAND_PAN = 0x03;
const byte I2C_COMMAND_TILT = 0x04;

// Pins connected to buttons to open and close the door
// Switches should be momentary normally open with the switch input wired to ground.
// Pullups should be enabled on these pins.
const int DOOR_OPEN_BUTTON_PIN = A0;
const int DOOR_CLOSE_BUTTON_PIN = A1;

// Pins connected to switches to open and close the door
// Switches should be momentary normally open with the switch input wired to ground.
// Pullups should be enabled on these pins.
const int LIGHT_ON_BUTTON_PIN = A2;
const int LIGHT_OFF_BUTTON_PIN = A3;

// Minimum amount of time between button polling intervals, in microseconds
const unsigned long BUTTON_POLL_INTERVAL = 1000;

// This multiplied by the button poll interval above give the minimum response time
const byte BUTTON_DEBOUNCE = 20;

// Pin to output a heartbeat signal, on Arduino, 13 is usually connected to an LED.
const int HEARTBEAT_PIN = LED_BUILTIN;

// Amount of time between toggling the hearbeat
const unsigned long HEARTBEAT_MILLIS = 250;

unsigned long lastHeartbeat = 0;

byte millisHighByte = 0;
boolean millisHigh = false;

byte doorOpenButtonDebounceCounter;
byte doorCloseButtonDebounceCounter;
byte lightOnButtonDebounceCounter;
byte lightOffButtonDebounceCounter;
unsigned long buttonPollTime;

void setup() {
  setupDoor();
  setupLight();
  setupPanTilt();

  pinMode(DOOR_OPEN_BUTTON_PIN, INPUT_PULLUP);
  pinMode(DOOR_CLOSE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LIGHT_ON_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LIGHT_OFF_BUTTON_PIN, INPUT_PULLUP);

  pinMode(HEARTBEAT_PIN, OUTPUT);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(processI2cWrite);
  Wire.onRequest(processI2cRead);
}


void loop() {
  updateDoor();
  pollDoorLimitSwitches();
  updateLight();
  pollDoorLimitSwitches();
  updatePanTilt();
  pollDoorLimitSwitches();
  pollButtons();
  pollDoorLimitSwitches();
  updateHeartbeat();
  pollDoorLimitSwitches();
  updateUptime();
  pollDoorLimitSwitches();
}


/**
 * Updates counters and state for a visible heartbeat.
 */
void updateHeartbeat() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastHeartbeat > HEARTBEAT_MILLIS) {
    lastHeartbeat = currentMillis;
    noInterrupts();
    digitalWrite(HEARTBEAT_PIN, !digitalRead(HEARTBEAT_PIN));
    interrupts();
  }
}


/**
 * Updates the variables associated with tracking the uptime.  Since millis() will overflow
 * every roughly 50 days, we add an extra byte which is incremented on every overflow.  This
 * stretches the overflow to once every approximately 35 years.
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
 * Polls the buttons from the pendant.  Buttons will need to be pressed for at least
 * BUTTON_DEBOUNCE * BUTTON_POLL_INTERVAL microseconds (longer if bouncing) to be
 * registered as pressed.
 */
void pollButtons() {
  unsigned long currentTime = micros();
  if (currentTime - buttonPollTime < BUTTON_POLL_INTERVAL) {
    return;
  }
  buttonPollTime = currentTime;

  if (isSwitchClosed(DOOR_OPEN_BUTTON_PIN, &doorOpenButtonDebounceCounter, BUTTON_DEBOUNCE)) {
    requestDoorCommand(DOOR_COMMAND_OPEN);
  }
  if (isSwitchClosed(DOOR_CLOSE_BUTTON_PIN, &doorCloseButtonDebounceCounter, BUTTON_DEBOUNCE)) {
    requestDoorCommand(DOOR_COMMAND_CLOSE);
  }
  
  if (isSwitchClosed(LIGHT_ON_BUTTON_PIN, &lightOnButtonDebounceCounter, BUTTON_DEBOUNCE)) {
    requestLightCommand(LIGHT_COMMAND_ON);
  }
  if (isSwitchClosed(LIGHT_OFF_BUTTON_PIN, &lightOffButtonDebounceCounter, BUTTON_DEBOUNCE)) {
    requestLightCommand(LIGHT_COMMAND_OFF);
  }
}


/**
 * Checks if an active-low switch is closed. Performs debounsing on the signal.
 * Should be called based on time rather than every loop cycle.
 */
boolean isSwitchClosed(byte pin, byte *debounceCounter, byte debounceCounterLimit) {
  if (!digitalRead(pin)) {
    if (*debounceCounter < debounceCounterLimit) {
      ++(*debounceCounter);
    }
    else {
      return true;
    }
  }
  else {
    debounceCounter = 0;
  }
  return false;
}


/**
 * Processes an I2C write request.
 * 
 * Data structure for write:
 * Byte 0: command type 0x1 for door, 0x2 for light, 0x03 for pan servo, 0x04 for tilt servo
 * Byte 1: data appropriate for type (DOOR_COMMAND_* or LIGHT_COMMAND_* or servo angle)
 */
void processI2cWrite(int bytes) {
  byte command = Wire.read();
  switch (command) {
    case I2C_COMMAND_DOOR:
      if (Wire.available() > 0) {
        requestDoorCommand(Wire.read());
      }
      break;

    case I2C_COMMAND_LIGHT:
      if (Wire.available() > 0) {
        requestLightCommand(Wire.read());
      }
      break;

    case I2C_COMMAND_PAN:
      if (Wire.available() > 0) {
        requestPanAngle(Wire.read());
      }
      break;

    case I2C_COMMAND_TILT:
      if (Wire.available() > 0) {
        requestTiltAngle(Wire.read());
      }
      break;

    default:
      // Unknown command - do nothing
      break;
  }

  // Clear out any remaiing bytes
  while (Wire.read() > 0) {
    Wire.read();
  }
}


/**
 * Processes an I2C read request.
 * 
 * Data structure for read:
 * Byte 0: Version: 0x02
 * Byte 1: Top 4 bits: door state, Bottom 4 bits: light state
 * Bytes 2-6: approximate uptime in millis (big endian), note: overflows approximately every 35 years (unsigned)
 * Byte 7: current Pan angle (unsigned)
 * Byte 8: current Tilt angle (unsigned)
 */
void processI2cRead() {
  byte data[9];
  data[0] = 0x02;
  data[1] = (getDoorState() << 4) | getLightState();

  unsigned long currentUptime = millis();
  data[2] = millisHighByte;
  data[3] = (currentUptime >> 24) & 0xFF;
  data[4] = (currentUptime >> 16) & 0xFF;
  data[5] = (currentUptime >>  8) & 0xFF;
  data[6] = (currentUptime      ) & 0xFF;

  data[7] = getCurrentPanAngle();
  data[8] = getCurrentTiltAngle();

  Wire.write(data, 9);
}

