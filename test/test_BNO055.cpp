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

void setup() {
    UNITY_BEGIN();
    Wire.begin();

    RUN_TEST(test_BNO055_begin);

    UNITY_END();
}

void loop() {

}