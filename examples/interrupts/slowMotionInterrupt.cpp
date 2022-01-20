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

    BNO055::AccelSlowNoMotionInterrupt slowNoMotionInterrupt(
            sensor,
            (new BNO055::Interrupt::AxesSetting)->enableX().enableY().enableZ(),
            myCallback,
            2, BNO055::AccelSlowNoMotionInterrupt::SLOW_MOTION);
    slowNoMotionInterrupt.setup();
    slowNoMotionInterrupt.enable();
    slowNoMotionInterrupt.mask();
    sensor.setOperationMode(ACCONLY);
    pinMode(3, OUTPUT);
}

void loop() {
    Serial.println(n);
    sensor.clearInterrupt();
}
