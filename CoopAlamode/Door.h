#ifndef Door_h
#define Door_h

#include <Arduino.h>

const byte DOOR_STATE_UNKNOWN = 0;
const byte DOOR_STATE_CLOSE = 1;
const byte DOOR_STATE_CLOSING = 2;
const byte DOOR_STATE_OPEN = 3;
const byte DOOR_STATE_OPENING = 4;

const byte DOOR_COMMAND_NONE = 0;
const byte DOOR_COMMAND_OPEN = 1;
const byte DOOR_COMMAND_CLOSE = 2;

#endif
