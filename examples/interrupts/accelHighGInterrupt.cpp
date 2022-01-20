//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#include <Arduino.h>
#include "BNO055.h"

const byte I2C_ADDRESS = 0x28;

BNO055 sensor(I2C_ADDRESS, &Wire);

// define our callback (known as an ISR, or interrupt service routine) to be called when the interrupt is
// detected. Note that ISRs have various limitations.
void myCallback() {
    digitalWrite(3, !digitalRead(3));
}

void setup() {
    // begin I2C communication
    Wire.begin();

    // reset the sensor to remove any existing configuration
    sensor.resetSystem();

    // we need to be in configmode to set up the interrupt.
    // this is not necessary as we just called sensor#resetSystem however it is here for completeness
    sensor.setOperationMode(CONFIGMODE);

    // define the configuration of our interrupt
    BNO055::AccelHighGInterrupt interrupt(
            sensor,
            (new BNO055::Interrupt::EnabledAxes)->enableX().enableY().enableZ(),
            myCallback,
            2);

    // write the configuration to the BNO055
    interrupt.setup();

    // enable this interrupt
    interrupt.enable();

    // route this interrupt to the INT pin on the sensor
    interrupt.mask();

    // enter the ACCONLY operation mode (note that the above code must run in COMFIGMODE)
    sensor.setOperationMode(ACCONLY);
    pinMode(3, OUTPUT);
}

void loop() {
    // we constantly clear the interrupt on the sensor. Not an ideal solution and not efficient, but
    // works for this example
    sensor.clearInterrupt();
}
