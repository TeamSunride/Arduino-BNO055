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

    /**
     * Trigger a power-on reset (POR), wait for the reset to happen (650ms) and then run a test
     * to see if the restarted properly.
     * @return Whether the reset was successful.
     */
    bool resetSystem();

    bool performSelfTest();

    /**
     * Burst read data corresponding to the current operation mode
     */
    bno055_burst_t getAllData();

    /**
     * Sets the page ID of the register map
     * @param pageID The page ID to select. Should be 0 or 1
     * @return Whether the write was successful
     */
    bool setPageID(int pageID);

    /**
     * @return A Vector containing unprocessed acceleration data in m/s
     */
    Vector<double> getRawAcceleration();

    /**
     * @return A Vector containing unprocessed magnetometer data in microTesla
     */
    Vector<double> getRawMagnetometer();

    /**
     * @return A Vector containing unprocessed gyroscope data in microTesla
     */
    Vector<double> getRawGyro();

    byte getTemp();
    bool setTempSource(BNO055_TEMP_SOURCE_TYPE source);

private:
    int BNO055_ACCEL_CONVERSION_FACTOR = 100;
    int BNO055_MAG_CONVERSION_FACTOR = 16;
    int BNO055_GYRO_CONVERSION_FACTOR = 16;
    Vector<int16_t> getVector(byte offset);
    Vector<int16_t> getVector(byte buffer[], int startIndex);
    BNO055_OPERATION_MODE currentOperationMode = CONFIGMODE;
};


#endif //BNO055_H
