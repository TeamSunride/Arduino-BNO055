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
    sensor.setOperationMode(GYROONLY);
}

void loop() {
    Vector<double> gyroVector = sensor.getRawGyro();
    Serial.println((String)gyroVector.getX() + "," + (String)gyroVector.getY() + ","
                   + (String)gyroVector.getZ() + ",");
}
