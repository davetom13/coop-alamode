#include "Light.h"

// Physical configuration

// Connected to IN1 of relay module.
// Active low
const int LIGHT_PIN = 8;


// Globals

// Current light state, should always be referenced/set with interrupts disabled to ensure consistency
// and atomicity
volatile byte lightState = LIGHT_STATE_OFF;

// Current command, should always be referenced/set with interrupts disabled to ensure consistency
// and atomicity
volatile byte lightCommand = LIGHT_COMMAND_NONE;

/**
 * Sets up the pins for the light and defaults it to off.
 */
void setupLight() {
  digitalWrite(LIGHT_PIN, HIGH);
  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, HIGH);
}


/**
 * Updates the light based on external and internal inputs.
 * This should be called from the main loop as it needs to be
 * run relatively frequently to ensure a quick response to
 * incoming commands.
 */
void updateLight() {
  processLightCommand();
}


/**
 * External request to run a light command.
 * 
 * @param command requested command, one of LIGHT_COMMAND_*
 */
void requestLightCommand(byte command) {
  lightCommand = command;
}


/**
 * Gets the current state of the light.  Should be called when interrupts are disabled.
 * 
 * @return current light state, one of LIGHT_STATE_*
 */
byte getLightState() {
  return lightState;
}


/**
 * Methods below here are for internal use of this module only.  They should not be called by external code.
 */


void processLightCommand() {
  noInterrupts();

  if (lightCommand == LIGHT_COMMAND_ON) {
    lightCommand = LIGHT_COMMAND_NONE;
    processLightOnCommand();
  }
  else if (lightCommand == LIGHT_COMMAND_OFF) {
    lightCommand = LIGHT_COMMAND_NONE;
    processLightOffCommand();
  }
  else {
    lightCommand = LIGHT_COMMAND_NONE;
  }

  interrupts();
}


/**
 * Turns the light on.
 */
void processLightOnCommand() {
  digitalWrite(LIGHT_PIN, LOW);
  lightState = LIGHT_STATE_ON;
}


/**
 * Turns the light off.
 */
void processLightOffCommand() {
  digitalWrite(LIGHT_PIN, HIGH);
  lightState = LIGHT_STATE_OFF;
}

