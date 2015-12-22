# coop-alamode
This is a project for chicken coop automation firmware intended to run on an [Alamode for Raspberry Pi](http://wyolum.com/projects/alamode/).  It will likely run on other Arduino Uno like platforms.  I chose an Alamode because I am controlling this with a Raspberry
Pi so the stackable design along with the built-in RTC and level converters made this simple.

## Functionality
This firmware provides the following functionality:

* Control of one pop door
  * Open and close door using DC motor
  * Limit switches for monitoring opened and closed position
  * Software motor watchdog in case limit switches are not hit in reasonable timeframe
  * Motor soft start
  * Reduced speed at ends of travel to avoid banging into the limits
* Control of one AC relay (light)
  * On and Off

Control can be accomplished manually (active low pins) and/or by software (I2C commands).

## Pin configuration

* Inputs
  * Pin A0 - Open door - active low with internal pullup enabled
  * Pin A1 - Close door - active low with internal pullup enabled
  * Pin A2 - Turn on light - active low with internal pullup enabled
  * Pin A3 - Turn off light - active low with internal pullup enabled
  * Pin D2 - Door opened limit switch - active low with internal pullup enabled
  * Pin D3 - Door closed limit switch - active low with internal pullup enabled
* Outputs
  * Pin D11 - PWM pin for controlling motor speed.  Connected to ENA and ENB on L298N module
  * Pin D10 - Motor direction control 1 - connected to IN1 and IN4 on L298N module
  * Pin D12 - Motor direction control 2 - connected to IN2 and IN3 on L298N module
  * Pin D8 - Light control - connected to IN1 of relay module
  * Pin LED_BUILTIN - heartbeat indicator - pulses roughly 2 times a second
* Special
  * Pin AD4 - I2C SDA
  * Pin AD5 - I2C SCL

## Software Control
In order to facilitate higher level control, this device is an I2C slave at address 0x10.  I2C writes can open and close the door as well as turn the light on and off.  An I2C read can get the current state of the door and light as well as a simple uptime counter good for around 35 years.  See the source for the specifics.

## Hardware needed
* Alamode for Raspberry Pi - for running this firmware.  I purchased mine from Makershed.
 * Raspberry Pi - for higher level control.  This isn't strictly necessary as this firmware is freestanding and can run by itself.  However, you'd be better off with a Uno or similar hardware rather than the Alamode if not planning on using a Pi.
* DC motor - to raise and lower the door.  I'm using a small servo motor (rated 24V 5 amp) with no encoder.
* L298N module - to control the DC motor.  I'm using a module purchased from eBay.
* 1 or more channel AC relay module - to turn the light on and off.  I'm using a 4 channel relay purchased from Amazon.  Make sure the relay board is rated for the voltage you are planning to use.  Some relay modules are only rated for 24VAC and could be dangerous if used at 120VAC or higher.  You don't need a relay module as you could just wire in relays directly, but as inexpensive as these things are, they are worth it to simplify the mounting and wiring.
* 4 Manual control buttons.  I'm using cheap momentary-on push buttons purchased from Amazon.  They are mounted in a 3D printed box attached to the front of the chicken coop.
* Microswitches - door limit switches.  I'm using cheap KW12 microswitches with a roller arm purchased from Amazon.
* Motor power supply.  I'm using a surplus 120VAC-12VAC 240 Watt transformer along with a full wave bridge rectifier and capacitor filter for the motor voltage.  The wattage is overkill but I had it lying around from a previous project.  Output voltage is around 16VDC.
* Electronics power supply - to power the Alamode / Raspberry Pi and all the other components.  To get the necessary 5VDC for the electronics I am using a cheap 12VDC to 5VDC switching regulator (3 Amp) purchased from Amazon.  The input is hooked up to the motor power supply.
