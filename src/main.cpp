//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#include <Arduino.h>
#include "BNO055.h"

const byte I2C_ADDRESS = 0x28;

BNO055 sensor(I2C_ADDRESS, &Wire);

void myCallback() {
    digitalWrite(3, !digitalRead(3));
}

void setup() {
    sensor.resetSystem();
    Serial.begin(9600);
    Wire.begin();
    BNO055::AccelHighGInterrupt interrupt(
            sensor,
            (new BNO055::Interrupt::AxesSetting)->enableX().enableY().enableZ(),
            myCallback,
            2);
    interrupt.setup();
    interrupt.enable();
    interrupt.mask();
    sensor.setOperationMode(ACCONLY);
    pinMode(3, OUTPUT);
}

void loop() {
    sensor.writeRegister(BNO055_SYS_TRIGGER, 0b01000000);
}
