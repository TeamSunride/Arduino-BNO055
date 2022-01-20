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
    sensor.setOperationMode(IMU);
}

void loop() {
    Vector<double> euler = sensor.getEuler();
    // using "pyteaplot"
    Serial.println("y" + (String) euler.getX() + "yp" + (String) euler.getY() + "pr"
                   + (String) euler.getZ() + "r");
}
