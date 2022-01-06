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
    sensor.setOperationMode(MAGONLY);
}

void loop() {
    Vector<double> magVector = sensor.getRawMagnetometer();
    Serial.println((String)magVector.getX() + "," + (String)magVector.getY() + ","
                   + (String)magVector.getZ() + ",");
}
