//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#include "BNO055.h"

bool BNO055::begin() {
    delay(1000); // wait 400ms for the device to start (Page 13)

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

    Serial.println(selfTestResult, BIN);

    // A successful self test would return a value of 0b1111

    return selfTestResult == 0b1111;
}
