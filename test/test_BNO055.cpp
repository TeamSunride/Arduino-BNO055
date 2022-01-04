//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#include "unity.h"
#include "BNO055.h"

const byte I2C_ADDRESS = 0x28;

BNO055 sensor(I2C_ADDRESS, &Wire);

void test_BNO055_begin() {
    TEST_ASSERT_TRUE(sensor.begin());
}

void test_BNO055_OperationMode() {
    TEST_ASSERT_TRUE(sensor.resetSystem());

    sensor.setOperationMode(ACCONLY);
    TEST_ASSERT_EQUAL(ACCONLY, sensor.getOperationMode());

    sensor.setOperationMode(ACCGYRO);
    TEST_ASSERT_EQUAL(ACCGYRO, sensor.getOperationMode());

    sensor.setOperationMode(CONFIGMODE);
}


void test_BNO055_PowerMode() {
    // we reset the system
    TEST_ASSERT_TRUE(sensor.resetSystem());
    // this means that the power mode is NORMAL and the operation mode is CONFIGMODE

    // set the operation mode to something other than configmode to test that the method
    // properly sets the power mode (see setPowerMode comments for details)
    TEST_ASSERT_TRUE(sensor.setOperationMode(ACCONLY))

    TEST_ASSERT_TRUE(sensor.setPowerMode(SUSPEND));
    TEST_ASSERT_EQUAL(SUSPEND, sensor.getPowerMode());

    TEST_ASSERT_TRUE(sensor.setPowerMode(LOW_POWER));
    TEST_ASSERT_EQUAL(LOW_POWER, sensor.getPowerMode());

    TEST_ASSERT_TRUE(sensor.setPowerMode(NORMAL));
    TEST_ASSERT_EQUAL(NORMAL, sensor.getPowerMode());
}

void test_BNO055_performSelfTest() {
    TEST_ASSERT_TRUE(sensor.performSelfTest());
}

void setup() {
    delay(3000);
    UNITY_BEGIN();
    Wire.begin();

    RUN_TEST(test_BNO055_begin);
    RUN_TEST(test_BNO055_OperationMode);
    RUN_TEST(test_BNO055_PowerMode);
    RUN_TEST(test_BNO055_performSelfTest);

    UNITY_END();
}

void loop() {
    ;
}