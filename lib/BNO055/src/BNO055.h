//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#ifndef BNO055_H
#define BNO055_H

#include "Arduino.h"
#include "I2CDevice.h"
#include "BNO055_constants.h"

// DATASHEET: https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

class BNO055 : public I2CDevice {
public:
    using I2CDevice::I2CDevice;  // inherit the constructor from I2CDevice
    bool begin();

    /**
     * Set the power mode for the BNO055. Refer to section 3.2 of the datasheet
     * @param mode The BNO055_POWER_MODE to be set
     * @return Whether writing the new mode to the PWR_MODE register was successful
     */
    bool setPowerMode(BNO055_POWER_MODE mode);

    /**
     * @return The current BNO055_POWER_MODE that the sensor is in
     */
    BNO055_POWER_MODE getPowerMode();

    /**
     * Set the operation mode for the BNO055. Refer to section 3.3 of the datasheet
     * @param mode The BNO055_OPERATION_MODE to be set
     * @return Whether writing the new mode to the OPR_MODE register was successful
     */
    bool setOperationMode(BNO055_OPERATION_MODE mode);
    /**
     * @return The current BNO055_OPERATION_MODE that the sensor is in
     */
    BNO055_OPERATION_MODE getOperationMode();

    bool resetSystem();
};


#endif //BNO055_H
