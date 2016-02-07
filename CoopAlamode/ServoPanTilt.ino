#include <Servo.h>

// Physical configuration

// Pins to be connected to the signal line of RC type servos.
const byte PAN_SERVO_PIN = 6;
const byte TILT_SERVO_PIN = 7;

const byte SERVO_POWER_PIN = 0;

// Software configuration

// Min and max request angles to be allowed.
// Any request outside of this range will be rejected
const byte PAN_SERVO_MIN_ANGLE = 10;
const byte PAN_SERVO_MAX_ANGLE = 170;
const byte TILT_SERVO_MIN_ANGLE = 30;
const byte TILT_SERVO_MAX_ANGLE = 150;

const byte ANGLE_NONE = 0xFF;

// Initial servo angles to be set upon boot
const byte PAN_SERVO_INITIAL_ANGLE = 90;
const byte TILT_SERVO_INITIAL_ANGLE = 90;

// Amount of time after last servo movement to shutoff servos in millis.
const unsigned long SERVO_POWER_SHUTOFF_DELAY=5000;

Servo panServo;
Servo tiltServo;

// Variables for tracing the requested servo angles
volatile byte commandedPanServoAngle = ANGLE_NONE;
volatile byte commandedTiltServoAngle = ANGLE_NONE;

volatile byte currentPanServoAngle = PAN_SERVO_INITIAL_ANGLE;
volatile byte currentTiltServoAngle = TILT_SERVO_INITIAL_ANGLE;

// time of last servo update in millis
unsigned long lastServoUpdateTime = 0;


/**
 * Sets up the pan and tilt servos by initializing the Servo library and making sure
 * the servos are in a known position. Initializes the pin to control power to the
 * servos and defaults it to off (high).
 */
void setupPanTilt() {
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);

  panServo.write(PAN_SERVO_INITIAL_ANGLE);
  tiltServo.write(TILT_SERVO_INITIAL_ANGLE);
  
  digitalWrite(SERVO_POWER_PIN, HIGH);
  pinMode(SERVO_POWER_PIN, OUTPUT);
  digitalWrite(SERVO_POWER_PIN, HIGH);
}


/**
 * Updates the pan and tilt servos to move to the requested positions.
 */
void updatePanTilt() {
  boolean updated = false;
  updated |= updatePanServo();
  updated |= updateTiltServo();
  
  unsigned long currentTime = millis();
  if (updated) {
    digitalWrite(SERVO_POWER_PIN, LOW);
    lastServoUpdateTime = currentTime;
  }
  else {
    if (currentTime - lastServoUpdateTime > SERVO_POWER_SHUTOFF_DELAY) {
      if (digitalRead(SERVO_POWER_PIN) == LOW) {
        digitalWrite(SERVO_POWER_PIN, HIGH);
      }
    }
  }
}


/**
 * Requests a move of the pan servo to the requested angle. Can be called from
 * within interrupt handler. If the angle is outside of the valid range, the
 * request wil be siltently dropped.
 * 
 * @param desired desired angle
 */
void requestPanAngle(byte desired) {
  if ((desired <= PAN_SERVO_MAX_ANGLE) && (desired >= PAN_SERVO_MIN_ANGLE)) {
    commandedPanServoAngle = desired;
  }
}


/**
 * Requests a move of the tilt servo to the requested angle. Can be called from
 * within interrupt handler. If the angle is outside of the valid range, the
 * request wil be siltently dropped.
 * 
 * @param desired desired angle
 */
void requestTiltAngle(byte desired) {
  if ((desired <= TILT_SERVO_MAX_ANGLE) && (desired >= TILT_SERVO_MIN_ANGLE)) {
    commandedTiltServoAngle = desired;
  }
}


/**
 * Gets the current angle of the pan servo.
 * 
 * @return current pan servo angle
 */
byte getCurrentPanAngle() {
  return currentPanServoAngle;
}


/**
 * Gets the current angle of the tilt servo.
 * 
 * @return current pan servo angle
 */
byte getCurrentTiltAngle() {
  return currentTiltServoAngle;
}


/**
 * Methods below here are for internal use of this module only.  They should not be called by external code.
 */

/**
 * Moves the pan servo to the requested angle if it is not already there.
 */
boolean updatePanServo() {
  byte desired = commandedPanServoAngle;
  if (desired == ANGLE_NONE) {
    return false;
  }
  
  commandedPanServoAngle = ANGLE_NONE;
  panServo.write(180 - desired);
  currentPanServoAngle = desired;
  return true;
}


/**
 * Moves the tilt servo towards the requested angle if it is not already there.
 */
boolean updateTiltServo() {
  byte desired = commandedTiltServoAngle;
  if (desired == ANGLE_NONE) {
    return false;
  }
  
  commandedTiltServoAngle = ANGLE_NONE;
  tiltServo.write(180 - desired);
  currentTiltServoAngle = desired;
  return true;
}
