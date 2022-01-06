//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#include <Arduino.h>
#include "BNO055.h"

const byte I2C_ADDRESS = 0x28;

BNO055 sensor(I2C_ADDRESS, &Wire);

void setup() {
    Wire.begin();
    sensor.begin();
    sensor.setOperationMode(CONFIGMODE);
    sensor.setPageID(1);
    sensor.writeRegister(BNO055_ACC_CONFIG, 0b00011100);
    sensor.setPageID(0);
    sensor.setOperationMode(ACCONLY);
}

void loop() {
    bno055_burst_t data = sensor.getAllData();
    Serial.println((String)data.accel.getX() + "," + (String)data.accel.getY() + ","
                   + (String)data.accel.getZ() + ",");
}
