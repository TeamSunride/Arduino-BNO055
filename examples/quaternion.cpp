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
    Quaternion quat = sensor.getQuaternion();
    // using "pyteaplot"
    Serial.println("w"+(String)quat.getW() + "wa" + (String)quat.getX() + "ab"
                   + (String)quat.getY() + "bc" + quat.getZ() + "c");

}
