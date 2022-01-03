//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#include "BNO055.h"

bool BNO055::begin() {
    delay(400); // wait 400ms for the device to start (Page 13)

    /**
     * During the device startup, a power on self test is executed. This feature checks that the
     * connected sensors and microcontroller are responding / functioning correctly. Following
     * tests are executed
     *
     * | Components |                                     | Test type |
     * =========================================================================
     * Accelerometer                                    Verify chip ID
     * Magnetometer                                     Verify chip ID
     * Gyroscope                                        Verify chip ID
     * Microcontroller                                  Memory Build In Self Test
     *
     * The results of the POST are stored at register ST_RESULT, where a bit set indicates test
     * passed and cleared indicates self test failed.
     *
     * (Section 3.8)
     */

    byte selfTestResult = readRegister(BNO055_ST_RESULT);

    // A successful self test would return a value of 0b1111

    return selfTestResult == 0b1111;
}

bool BNO055::setPowerMode(BNO055_POWER_MODE mode) {
    /**
     * We cannot change the power mode when in a mode other than CONFIGMODE.
     * Therefore we must switch into CONFIGMODE to change the power mode
     *
     * So we store the current operation mode, switch to the new power mode, then
     * restore the old operation mode
     */

    BNO055_OPERATION_MODE currentOperationMode = getOperationMode();
    setOperationMode(CONFIGMODE);


    // default to normal mode if an incorrect input is made
    if(mode > SUSPEND || mode < NORMAL)
        mode = NORMAL;

    bool writeResult = writeRegister(BNO055_PWR_MODE_REG, (int) mode);
    setOperationMode(currentOperationMode);
    return writeResult;
}

BNO055_POWER_MODE BNO055::getPowerMode() {
    byte value = readRegister(BNO055_PWR_MODE_REG);
    return static_cast<BNO055_POWER_MODE>(value);
}

bool BNO055::setOperationMode(BNO055_OPERATION_MODE mode) {
    // we first set the sensor to config mode, so that we can set a new mode afterwards
    bool writeResult1 = writeRegister(BNO055_OPR_MODE_REG, CONFIGMODE);
    delay(20); // any operation mode to CONFIGMODE takes 19 milliseconds

    // if the user wanted to set configmode then we stop here
    if (mode == CONFIGMODE) {
        return writeResult1;
    }

    bool writeResult2 = writeRegister(BNO055_OPR_MODE_REG, mode);
    delay(8); // CONFIGMODE to any operation mode takes 7 milliseconds
    return writeResult2;
}

BNO055_OPERATION_MODE BNO055::getOperationMode() {
    byte value = readRegister(BNO055_OPR_MODE_REG);
    // clear the first 4 bits as they are marked as reserved and there is
    // no guarantee that they will equal 0
    value &= 0b00001111;
    return static_cast<BNO055_OPERATION_MODE>(value);
}

bool BNO055::resetSystem() {
    byte currentSetting = readRegister(BNO055_SYS_TRIGGER);
    byte newSetting = currentSetting | 0b00100000;  // modify SYS_TRIGGER to set RST_SYS to 1
    bool writeResult = writeRegister(BNO055_SYS_TRIGGER, newSetting);
    delay(650); // delay for the POR time specified in Table 0-2
    return writeResult;
}
