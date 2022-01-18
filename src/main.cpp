//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#include <Arduino.h>
#include "BNO055.h"

const byte I2C_ADDRESS = 0x28;

BNO055 sensor(I2C_ADDRESS, &Wire);

volatile int interrupts_recv;

void myCallback() {
    interrupts_recv++;
    digitalWrite(3, !digitalRead(3));
}

void setup() {
    sensor.resetSystem();
    interrupts_recv = 0;
    Serial.begin(9600);
    Wire.begin();
    sensor.begin();
    sensor.setOperationMode(CONFIGMODE);
    BNO055::Interrupt::AxisSetting axisSetting =
            (new BNO055::Interrupt::AxisSetting)->enableX().enableY().enableZ();
    Serial.println(axisSetting.x);
    BNO055::AccelHighGInterrupt interrupt(
            sensor,
            axisSetting,
            myCallback,
            2);
    interrupt.setup();
    interrupt.enable();
    interrupt.mask();
    sensor.setOperationMode(ACCONLY);
    pinMode(3, OUTPUT);
}

void loop() {
    Serial.println("Got interrupt " + (String) interrupts_recv);
    Serial.println(sensor.readRegister(BNO055_INT_STA), BIN);
    Serial.println(digitalRead(2));
    sensor.writeRegister(BNO055_SYS_TRIGGER, 0b01000000);
    delay(10);
}
