//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#ifndef BNO055_H
#define BNO055_H

#include "Arduino.h"
#include "I2CDevice.h"
#include "BNO055_constants.h"

class BNO055 : public I2CDevice {
public:
    using I2CDevice::I2CDevice;  // inherit the constructor from I2CDevice
    bool begin();
};


#endif //BNO055_H
