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
    sensor.setOperationMode(ACCGYRO);
}

void loop() {
    bno055_burst_t data = sensor.getAllData();
    Serial.print((String)data.accel.getX() + "," + (String)data.accel.getY() + ","
                   + (String)data.accel.getZ() + ",");
    Serial.println((String)data.gyro.getX() + "," + (String)data.gyro.getY() + ","
                   + (String)data.gyro.getZ() + ",");
}
