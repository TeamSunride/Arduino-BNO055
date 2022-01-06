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

void test_BNO055_ACCONLY() {
    sensor.resetSystem();

    // enter config mode
    sensor.setOperationMode(CONFIGMODE);

    // switch to page 1 of the register map
    sensor.setPageID(1);

    /**
     * Configure the sensor to operate in normal mode
     * with a bandwidth of 1000Hz
     * with a G range of 2G
     */
    sensor.writeRegister(BNO055_ACC_CONFIG, 0b00011100);
    sensor.setPageID(0);

    // enter ACCONLY operation mode
    sensor.setOperationMode(ACCONLY);

    delay(100);

    // assume that the sensor is placed on a flat table for the test

    bno055_burst_t result = sensor.getAllData();
    Vector<double> accel = result.accel;
    TEST_ASSERT_FLOAT_WITHIN(1, 9.81, accel.getZ());
    TEST_ASSERT_FLOAT_WITHIN(1, 0, accel.getX());
    TEST_ASSERT_FLOAT_WITHIN(1, 0, accel.getY());

}

void test_BNO055_temperature() {
    sensor.setOperationMode(AMG);
    delay(100);
    TEST_ASSERT_TRUE(sensor.setTempSource(ACCEL_TEMP));
    TEST_ASSERT_EQUAL(ACCEL_TEMP, sensor.readRegister(BNO055_TEMP_SOURCE));
    byte accelTemp = sensor.getTemp();
    Serial.println("Accelerometer temperature: " + String(accelTemp));
    TEST_ASSERT_INT_WITHIN(10, 20, accelTemp);

    sensor.setTempSource(GYRO_TEMP);
    byte gyroTemp = sensor.getTemp();
    Serial.println("Gyroscope temperature: " + String(gyroTemp));
    TEST_ASSERT_INT_WITHIN(10, 20, gyroTemp);
}

void test_BNO055_sensor_config() {
    sensor.resetSystem();
    // set accelerometer to normal mode, 1000Hz, 16G
    sensor.setAccelerometerConfig(0b00011111);

    // set gyro to 523Hz and 500dps
    sensor.setGyroscopeConfig(0b00000010);
    // set gyro to fast power up
    sensor.setGyroscopeOperationMode(0b1);

    // set magnetometer to normal mode, high accuracy, 30Hz
    sensor.setMagnetometerConfig(0b00011111);

    sensor.setPageID(1);

    TEST_ASSERT_EQUAL(0b00011111, sensor.readRegister(BNO055_ACC_CONFIG));
    TEST_ASSERT_EQUAL(0b00000010, sensor.readRegister(BNO055_GYR_CONFIG_0));
    TEST_ASSERT_EQUAL(0b1, sensor.readRegister(BNO055_GYR_CONFIG_1));
    TEST_ASSERT_EQUAL(0b00011111, sensor.readRegister(BNO055_MAG_CONFIG));

    sensor.setPageID(0);

}

void setup() {
    delay(3000);
    UNITY_BEGIN();
    Wire.begin();

    RUN_TEST(test_BNO055_begin);
    RUN_TEST(test_BNO055_OperationMode);
    RUN_TEST(test_BNO055_PowerMode);
    RUN_TEST(test_BNO055_performSelfTest);
    RUN_TEST(test_BNO055_temperature);
    RUN_TEST(test_BNO055_sensor_config);

    RUN_TEST(test_BNO055_ACCONLY);

    UNITY_END();
}

void loop() {
    ;
}