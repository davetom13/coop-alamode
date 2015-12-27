#include "Door.h"

// Physical configuration

// PWM pin for controlling motor speed, output, must be one of the valid PWM pins
// Connected to ENA and ENB on L298N module
const int PWM_PIN = 11;

// Pins connected to IN1-IN4 on L298N module
// To parallel, IN1 and IN4 should be tied together and IN2 and IN3
// For more info see Figure 7 in the datasheet
// If this setup causes the door to move the wrong way, swap the pin numbers here, the motor leads, or the pin connections
const int OPENING_PIN = 10;
const int CLOSING_PIN = 12;

// Pins connected to limit switches for full open and full close.
// Switches should be normally open with the switch input wired to ground.
// Pullups should be enabled on these pins.
const int OPEN_LIMIT_PIN = 2;
const int CLOSE_LIMIT_PIN = 3;

// Interrupt numbers for the open and close interrupts
const int OPENED_INTERRUPT = 0;
const int CLOSED_INTERRUPT = 1;

// Interrupt register bits for the open and close interrupts
// These are used to clear any pending interrupts before re-attaching
const int OPEN_INTERRUPT_REGISTER_BIT = bit (INTF0);
const int CLOSE_INTERRUPT_REGISTER_BIT = bit (INTF1);

// Software configuration

// Maximum PWM setting.  Valid values are 0 (off) to 255 (full on)
const int MAX_PWM = 255;

const int MAX_DOOR_SPEED = 0x3FFF;

// Door low speed when approaching an end stop.
// Valid values are 0 (off) to MAX_DOOR_SPEED (full on)
const int DOOR_LOW_SPEED_CLOSE = 60 << 6;
const int DOOR_LOW_SPEED_OPEN = 95 << 6;


// This value controls the slope of the soft start / stop
const int DOOR_SPEED_RAMP = 32; // amount to change speed per cycle

// Initial speed for the motor when reversing. If negative, motor will not be started until the ramp up has
// brought the value above 0.
const int INITIAL_REVERSING_MOTOR_SPEED = -4096 / DOOR_SPEED_RAMP;

// Arbitrary value defining when to shut down the motor if it has been on for this many update cycles.
const unsigned int DOOR_MOTOR_WATCHDOG_CLOSE_MAX = 2600;
const unsigned int DOOR_MOTOR_WATCHDOG_OPEN_MAX = 2700;

// Arbitrary value defining when to slow motor down 
const unsigned int DOOR_MOTOR_WATCHDOG_CLOSE_LOW_SPEED = 1700;
const unsigned int DOOR_MOTOR_WATCHDOG_OPEN_LOW_SPEED = 1900;

// Globals

// Current door state, should always be referenced/set with interrupts disabled to ensure consistency
// and atomicity
volatile byte doorState = DOOR_STATE_UNKNOWN;

// Current command, should always be referenced/set with interrupts disabled to ensure consistency
// and atomicity
volatile byte doorCommand = DOOR_COMMAND_NONE;

// Door speed tracker. Negative values indicates no motor output and are used for a motor start delay.
// Note this is not directly used as a PWM value but rather the PWM is derived from this.
int doorSpeed = 0;

// Door watchdog counter.
unsigned int doorWatchdog = 0;

// Max allowed for for door watchdog, variable as it can differ between opening and closing
unsigned int doorWatchdogMax;

// door watchdog value at which it transitions to low speed, variable as it can differ between opening and closing
unsigned int doorWatchdogLowSpeedCutover;

// door speed setting for low speed, variable as it can differ between opening and closing
int doorLowSpeed;


/**
 * Sets up the pins / interrupts to control / monitor the door
 */
void setupDoor() {
  pinMode(PWM_PIN, OUTPUT);  
  digitalWrite(PWM_PIN, LOW);
  
  pinMode(OPENING_PIN, OUTPUT);
  digitalWrite(OPENING_PIN, LOW);
  
  pinMode(CLOSING_PIN, OUTPUT);
  digitalWrite(CLOSING_PIN, LOW);
  
  pinMode(OPEN_LIMIT_PIN, INPUT_PULLUP);
//  attachInterrupt(OPENED_INTERRUPT, processDoorOpenLimit, FALLING);
  
  pinMode(CLOSE_LIMIT_PIN, INPUT_PULLUP);
//  attachInterrupt(CLOSED_INTERRUPT, processDoorCloseLimit, FALLING);

  if (digitalRead(OPEN_LIMIT_PIN) == LOW) {
    doorState = DOOR_STATE_OPEN;
  }
  else if (digitalRead(CLOSE_LIMIT_PIN) == LOW) {
    doorState = DOOR_STATE_CLOSE;
  }
}


/**
 * Updates the door based on external and internal inputs.
 * This should be called from the main loop as it needs to be
 * run relatively frequently to register button presses and
 * ramp up motor speed.
 */
void updateDoor() {
  processDoorCommand();
  updateDoorMotorSpeed();
}


/**
 * Gets the current door state.  Should be called when interrupts are disabled.
 * 
 * @return current door state, one of DOOR_STATE_*
 */
byte getDoorState() {
  return doorState;
}


/**
 * External request to run a door command.
 * 
 * @param command requested command, one of DOOR_COMMAND_*
 */
void requestDoorCommand(byte command) {
  doorCommand = command;
}


/**
 * Methods below here are for internal use of this module only.  They should not be called by external code.
 */

/**
 * Processes door command.
 */
void processDoorCommand() {
  noInterrupts();

  if (doorCommand == DOOR_COMMAND_OPEN) {
    doorCommand = DOOR_COMMAND_NONE;
    processDoorOpenCommand();
  }
  else if (doorCommand == DOOR_COMMAND_CLOSE) {
    doorCommand = DOOR_COMMAND_NONE;
    processDoorCloseCommand();
  }
  else if (doorCommand != DOOR_COMMAND_NONE) {
    // invalid command received
    doorCommand = DOOR_COMMAND_NONE;
  }


  interrupts();
}

/**
 * Process the command to open the door.
 * Assumes interrupts are disabled.  Will not change interrupt state.
 */
void processDoorOpenCommand() {
  if ((doorState == DOOR_STATE_OPEN) || (doorState == DOOR_STATE_OPENING)) {
    return;
  }
  
  digitalWrite(PWM_PIN, LOW);
  digitalWrite(OPENING_PIN, HIGH);
  digitalWrite(CLOSING_PIN, LOW);

  EIFR = OPEN_INTERRUPT_REGISTER_BIT;
  attachInterrupt(OPENED_INTERRUPT, processDoorOpenLimit, FALLING);

  // Check for motor reversal
  if (doorState == DOOR_STATE_CLOSING) {
    doorSpeed = INITIAL_REVERSING_MOTOR_SPEED;
    detachInterrupt(CLOSED_INTERRUPT);
  }
  else {
    doorSpeed = 0;
  }

  doorWatchdog = 0;
  doorWatchdogMax = DOOR_MOTOR_WATCHDOG_OPEN_MAX;
  doorWatchdogLowSpeedCutover = DOOR_MOTOR_WATCHDOG_OPEN_LOW_SPEED;
  doorLowSpeed = DOOR_LOW_SPEED_OPEN;
  doorState = DOOR_STATE_OPENING;
}


/**
 * Process the command to close the door.
 * Assumes interrupts are disabled.  Will not change interrupt state.
 */
void processDoorCloseCommand() {
  if ((doorState == DOOR_STATE_CLOSE) || (doorState == DOOR_STATE_CLOSING)) {
    return;
  }

  digitalWrite(PWM_PIN, LOW);
  digitalWrite(OPENING_PIN, LOW);
  digitalWrite(CLOSING_PIN, HIGH);

  EIFR = CLOSE_INTERRUPT_REGISTER_BIT;
  attachInterrupt(CLOSED_INTERRUPT, processDoorCloseLimit, FALLING);
  
  // Check for motor reversal
  if (doorState == DOOR_STATE_OPENING) {
    doorSpeed = INITIAL_REVERSING_MOTOR_SPEED;
    detachInterrupt(OPENED_INTERRUPT);
  }
  else {
    doorSpeed = 0;
  }

  doorWatchdog = 0;
  doorWatchdogMax = DOOR_MOTOR_WATCHDOG_CLOSE_MAX;
  doorWatchdogLowSpeedCutover = DOOR_MOTOR_WATCHDOG_CLOSE_LOW_SPEED;
  doorLowSpeed = DOOR_LOW_SPEED_CLOSE;
  doorState = DOOR_STATE_CLOSING;
}


/**
 * Updates the speed of the motor.  The PWM field will be incremented by DOOR_SPEED_RAMP
 * each time this method is called until the MAX_PWM is reached.  If the door is no longer opening
 * or closing, no speed change will occur. If the speed is negative, no speed change will occur.
 * 
 * In order to not bang into the limit switches, after a pre-determined amount of time, the motor
 * speed will be ramped down to a low speed value appropriate for the door direction.
 * 
 * Finally, this method increments and checks a watchdog counter in case the limit switches
 * are never tripped.  If the motor runs for too long, it is stopped and the door state becomes
 * unknown.
 */
void updateDoorMotorSpeed() {
  noInterrupts();
  if (!((doorState == DOOR_STATE_OPENING) || (doorState == DOOR_STATE_CLOSING))) {
    interrupts();
    return;
  }

  int pwm;
  
  doorWatchdog++;
  if (doorWatchdog > doorWatchdogMax) {
    digitalWrite(PWM_PIN, LOW);
    digitalWrite(OPENING_PIN, LOW);
    digitalWrite(CLOSING_PIN, LOW);
    doorSpeed = 0;
    doorState = DOOR_STATE_UNKNOWN;
    interrupts();
    return;
  }
  
  if (doorWatchdog >= doorWatchdogLowSpeedCutover) {
    if (doorSpeed > doorLowSpeed) {
      doorSpeed -= DOOR_SPEED_RAMP;
      if (doorSpeed < doorLowSpeed) {
        doorSpeed = doorLowSpeed;
      }
      pwm = (doorSpeed >> 6) & 0xFF;
      analogWrite(PWM_PIN, pwm);
    }
    interrupts();
    return;
  }

  if (doorSpeed >= MAX_DOOR_SPEED) {
    interrupts();
    return;
  }

  doorSpeed += DOOR_SPEED_RAMP;
  if (doorSpeed >= MAX_DOOR_SPEED) {
    doorSpeed = MAX_DOOR_SPEED;
  }

  pwm = (doorSpeed >> 6) & 0xFF;
  
  if (doorSpeed > 0) {
    analogWrite(PWM_PIN, pwm);
  }
  else {
    // Ensure motor is not turned on
    digitalWrite(PWM_PIN, LOW);
  }

  interrupts();
}


/**
 * Interrupt Service Routine (ISR) to handle the fully opened limit switch triggering.
 * Puts the L298N into free run stop mode and updates the door state.
 */
void processDoorOpenLimit() {
  digitalWrite(PWM_PIN, LOW);
  digitalWrite(OPENING_PIN, LOW);
  digitalWrite(CLOSING_PIN, LOW);
  doorState = DOOR_STATE_OPEN;
  detachInterrupt(OPENED_INTERRUPT);
}


/**
 * Interrupt Service Routine (ISR) to handle the fully closed limit switch triggering.
 * Puts the L298N into free run stop mode and updates the door state.
 */
void processDoorCloseLimit() {
  digitalWrite(PWM_PIN, LOW);
  digitalWrite(OPENING_PIN, LOW);
  digitalWrite(CLOSING_PIN, LOW);
  doorState = DOOR_STATE_CLOSE;
  detachInterrupt(CLOSED_INTERRUPT);
}

