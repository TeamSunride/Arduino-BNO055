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

void test_BNO055_setMode() {

    //TODO: Add some other test to verify that the modes are functioning as expected

    sensor.setMode(BNO055_POWER_MODE::SUSPEND);
    TEST_ASSERT_EQUAL(BNO055_POWER_MODE::SUSPEND, sensor.readRegister(BNO055_PWR_MODE));

    sensor.setMode(BNO055_POWER_MODE::LOW_POWER);
    TEST_ASSERT_EQUAL(BNO055_POWER_MODE::LOW_POWER, sensor.readRegister(BNO055_PWR_MODE));

    sensor.setMode(BNO055_POWER_MODE::NORMAL);
    TEST_ASSERT_EQUAL(BNO055_POWER_MODE::NORMAL, sensor.readRegister(BNO055_PWR_MODE));
}

void setup() {
    UNITY_BEGIN();
    Wire.begin();

    RUN_TEST(test_BNO055_begin);
    RUN_TEST(test_BNO055_setMode);

    UNITY_END();
}

void loop() {

}