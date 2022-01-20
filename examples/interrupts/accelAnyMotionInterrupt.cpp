//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#include <Arduino.h>
#include "BNO055.h"

const byte I2C_ADDRESS = 0x28;

BNO055 sensor(I2C_ADDRESS, &Wire);

int n = 0;

void myCallback() {
    digitalWrite(3, !digitalRead(3));
    n++;
}

void setup() {
    n = 0;
    Serial.begin(9600);
    Wire.begin();

    sensor.begin();

    sensor.resetSystem();

    BNO055::AccelAnyMotionInterrupt anyMotionInterrupt(
            sensor,
            (new BNO055::Interrupt::EnabledAxes)->enableX().enableY().enableZ(),
            myCallback,
            2);
    anyMotionInterrupt.setup();
    anyMotionInterrupt.enable();
    anyMotionInterrupt.mask();
    sensor.setOperationMode(ACCONLY);
    pinMode(3, OUTPUT);
}

void loop() {
    Serial.println(n);
    sensor.clearInterrupt();
}
