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

    BNO055_OPERATION_MODE operationMode = getOperationMode();
    setOperationMode(CONFIGMODE);


    // default to normal mode if an incorrect input is made
    if (mode > SUSPEND || mode < NORMAL)
        mode = NORMAL;

    bool writeResult = writeRegister(BNO055_PWR_MODE_REG, (int) mode);
    setOperationMode(operationMode);
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
    if (writeResult2) currentOperationMode = mode;
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
    // modify SYS_TRIGGER to set RST_SYS to 1
    bool writeResult = writeRegister(BNO055_SYS_TRIGGER, 0b00100000);
    delay(650); // delay for the POR time specified in Table 0-2

    // now perform a self test
    return performSelfTest() && writeResult;
}

bool BNO055::performSelfTest() {
    // modify SYS_TRIGGER to set Self_Test to 1
    bool writeSuccess = writeRegister(BNO055_SYS_TRIGGER, 0b1);

    // the datasheet does not say (?) how long the built-in self test takes so delay for the same time
    // as the power on self test
    delay(400);

    // filter the test result to eliminate the reserved bits and ST_MCU as
    // there is no test performed on ST_MCU for the self test
    byte selfTestResult = 0b00000111 & readRegister(BNO055_ST_RESULT);

    return (selfTestResult == 0b111) && writeSuccess;
}

bool BNO055::setPageID(int pageID) {
    if (!(pageID == 0 || pageID == 1)) pageID = 0;
    return writeRegister(BNO055_PAGE_ID, pageID);
}

Vector<double> BNO055::getRawAcceleration() {
    Vector<int16_t> accel = getVector(BNO055_ACC_DATA_X_LSB);
    Vector<double> convertedAccel = accel.divideScalar(BNO055_ACCEL_CONVERSION_FACTOR);
    return convertedAccel;
}

Vector<double> BNO055::getRawMagnetometer() {
    Vector<int16_t> mag = getVector(BNO055_MAG_DATA_X_LSB);
    Vector<double> convertedMag = mag.divideScalar(BNO055_MAG_CONVERSION_FACTOR);
    return convertedMag;
}

Vector<double> BNO055::getRawGyro() {
    Vector<int16_t> gyro = getVector(BNO055_GYR_DATA_X_LSB);
    Vector<double> convertedGyro = gyro.divideScalar(BNO055_GYRO_CONVERSION_FACTOR);
    return convertedGyro;
}

bno055_burst_t BNO055::getAllData() {
    bno055_burst_t returnData;
    byte buffer[42];
    switch (currentOperationMode) {
        case ACCONLY:
            returnData.accel = getRawAcceleration();
            break;
        case MAGONLY:
            returnData.mag = getRawMagnetometer();
            break;
        case GYROONLY:
            returnData.gyro = getRawGyro();
            break;
        case ACCMAG:
            memset(buffer, 0, 12);
            readMultipleRegisters(buffer, BNO055_ACC_DATA_X_LSB, 12);
            returnData.accel = getVector(buffer, 0).divideScalar(BNO055_ACCEL_CONVERSION_FACTOR);
            returnData.mag = getVector(buffer, 6).divideScalar(BNO055_MAG_CONVERSION_FACTOR);
            break;
        case ACCGYRO:
            memset(buffer, 0, 18);
            readMultipleRegisters(buffer, BNO055_ACC_DATA_X_LSB, 18);
            returnData.accel = getVector(buffer, 0).divideScalar(BNO055_ACCEL_CONVERSION_FACTOR);
            returnData.gyro = getVector(buffer, 12).divideScalar(BNO055_GYRO_CONVERSION_FACTOR);
            break;
        case MAGGYRO:
            memset(buffer, 0, 12);
            readMultipleRegisters(buffer, BNO055_MAG_DATA_X_LSB, 12);
            returnData.mag = getVector(buffer, 0).divideScalar(BNO055_MAG_CONVERSION_FACTOR);
            returnData.gyro = getVector(buffer, 6).divideScalar(BNO055_GYRO_CONVERSION_FACTOR);
            break;
        case AMG:
            memset(buffer, 0, 18);
            readMultipleRegisters(buffer, BNO055_ACC_DATA_X_LSB, 18);
            returnData.accel = getVector(buffer, 0).divideScalar(BNO055_ACCEL_CONVERSION_FACTOR);
            returnData.mag = getVector(buffer, 6).divideScalar(BNO055_MAG_CONVERSION_FACTOR);
            returnData.gyro = getVector(buffer, 12).divideScalar(BNO055_GYRO_CONVERSION_FACTOR);
            break;
        case IMU: case COMPASS: case M4G: case NDOF_FMC_OFF: case NDOF:
            memset(buffer, 0, 42);
            readMultipleRegisters(buffer, BNO055_ACC_DATA_X_LSB, 42);
            returnData.accel = getVector(buffer, 0).divideScalar(BNO055_ACCEL_CONVERSION_FACTOR);
            returnData.mag = getVector(buffer, 6).divideScalar(BNO055_MAG_CONVERSION_FACTOR);
            returnData.gyro = getVector(buffer, 12).divideScalar(BNO055_GYRO_CONVERSION_FACTOR);
            returnData.euler = getVector(buffer, 18).divideScalar(BNO055_GYRO_CONVERSION_FACTOR);
            returnData.quaternion = getQuaternion(buffer, 24);
            returnData.linearAccel = getVector(buffer, 32).divideScalar(BNO055_ACCEL_CONVERSION_FACTOR);
            returnData.gravityVector = getVector(buffer, 36).divideScalar(BNO055_ACCEL_CONVERSION_FACTOR);
            break;
        default:
            break;
    }
    return returnData;
}

Vector<int16_t> BNO055::getVector(byte offset) {
    Vector<int16_t> vector{};
    byte buffer[6];
    memset(buffer, 0, 6);
    readMultipleRegisters(buffer, offset, 6);

    vector.setX(((int16_t) buffer[0]) | (((int16_t) buffer[1]) << 8));
    vector.setY(((int16_t) buffer[2]) | (((int16_t) buffer[3]) << 8));
    vector.setZ(((int16_t) buffer[4]) | (((int16_t) buffer[5]) << 8));
    return vector;
}

Vector<int16_t> BNO055::getVector(byte *buffer, int startIndex) {
    Vector<int16_t> vector{};
    vector.setX(((int16_t) buffer[startIndex + 0]) | (((int16_t) buffer[startIndex + 1]) << 8));
    vector.setY(((int16_t) buffer[startIndex + 2]) | (((int16_t) buffer[startIndex + 3]) << 8));
    vector.setZ(((int16_t) buffer[startIndex + 4]) | (((int16_t) buffer[startIndex + 5]) << 8));
    return vector;
}

byte BNO055::getTemp() {
    return readRegister(BNO055_TEMP_REG);
}

bool BNO055::setTempSource(BNO055_TEMP_SOURCE_TYPE source) {
    return writeRegister(BNO055_TEMP_SOURCE, source);
}

bool BNO055::setAccelerometerConfig(byte value) {
    bool result1 = setPageID(1);
    bool result2 = writeRegister(BNO055_ACC_CONFIG, value);
    bool result3 = setPageID(0);
    return result1 && result2 && result3;
}

bool BNO055::setGyroscopeConfig(byte value) {
    bool result1 = setPageID(1);
    bool result2 = writeRegister(BNO055_GYR_CONFIG_0, value);
    bool result3 = setPageID(0);
    return result1 && result2 && result3;
}

bool BNO055::setGyroscopeOperationMode(byte value) {
    bool result1 = setPageID(1);
    bool result2 = writeRegister(BNO055_GYR_CONFIG_1, value);
    bool result3 = setPageID(0);
    return result1 && result2 && result3;
}

bool BNO055::setMagnetometerConfig(byte value) {
    bool result1 = setPageID(1);
    bool result2 = writeRegister(BNO055_MAG_CONFIG, value);
    bool result3 = setPageID(0);
    return result1 && result2 && result3;
}

Vector<double> BNO055::getLinearAcceleration() {
    Vector<int16_t> linearAccel = getVector(BNO055_LIA_DATA_X_LSB);
    return linearAccel.divideScalar(BNO055_ACCEL_CONVERSION_FACTOR);
}

Quaternion BNO055::getQuaternion() {
    Quaternion quat;
    byte buffer[8];
    memset(buffer, 0, 8);
    int16_t w, x, y, z;
    readMultipleRegisters(buffer, BNO055_QUA_DATA_W_LSB, 8);
    w = (((int16_t) buffer[0]) | (((int16_t) buffer[1]) << 8));
    x = (((int16_t) buffer[2]) | (((int16_t) buffer[3]) << 8));
    y = (((int16_t) buffer[4]) | (((int16_t) buffer[5]) << 8));
    z = (((int16_t) buffer[6]) | (((int16_t) buffer[7]) << 8));
    quat.setW(w / BNO055_QUAT_CONVERSION_FACTOR);
    quat.setX(x / BNO055_QUAT_CONVERSION_FACTOR);
    quat.setY(y / BNO055_QUAT_CONVERSION_FACTOR);
    quat.setZ(z / BNO055_QUAT_CONVERSION_FACTOR);
    return quat;
}

Quaternion BNO055::getQuaternion(const byte *buffer, int startIndex) const {
    Quaternion quat{};
    int16_t w, x, y, z;
    w = (((int16_t) buffer[startIndex + 0]) | (((int16_t) buffer[startIndex + 1]) << 8));
    x = (((int16_t) buffer[startIndex + 2]) | (((int16_t) buffer[startIndex + 3]) << 8));
    y = (((int16_t) buffer[startIndex + 4]) | (((int16_t) buffer[startIndex + 5]) << 8));
    z = (((int16_t) buffer[startIndex + 6]) | (((int16_t) buffer[startIndex + 7]) << 8));
    quat.setW(w / BNO055_QUAT_CONVERSION_FACTOR);
    quat.setX(x / BNO055_QUAT_CONVERSION_FACTOR);
    quat.setY(y / BNO055_QUAT_CONVERSION_FACTOR);
    quat.setZ(z / BNO055_QUAT_CONVERSION_FACTOR);
    return quat;
}

Vector<double> BNO055::getEuler() {
    Vector<int16_t> eulerAngles = getVector(BNO055_EUL_HEADING_LSB);
    return eulerAngles.divideScalar(BNO055_GYRO_CONVERSION_FACTOR);
}

Vector<double> BNO055::getGravity() {
    Vector<int16_t> gravity = getVector(BNO055_GRV_DATA_X_LSB);
    return gravity.divideScalar(BNO055_ACCEL_CONVERSION_FACTOR);
}

bno055_calib_stat_t BNO055::getCalibrationStatus() {
    byte result = readRegister(BNO055_CALIB_STAT);

    bno055_calib_stat_t status;
    status.mag = result & 0b00000011;
    status.accel = (result >> 2) & 0b11;
    status.gyro = (result >> 4) & 0b11;
    status.sys = (result >> 6) & 0b11;
    return status;
}

bool BNO055::clearInterrupt() {
    return writeRegister(BNO055_SYS_TRIGGER, 0b01000000);
}

bool BNO055::Interrupt::editInterruptState(byte offset, int bitLocation, bool bitValue) {
    sensor->setPageID(1);
    byte state = sensor->readRegister(offset);
    bitWrite(state, bitLocation, (int) bitValue);
    bool result = sensor->writeRegister(offset, state);
    sensor->setPageID(0);
    return result;
}

bool BNO055::Interrupt::enable() {
    return editInterruptState(BNO055_INT_EN, ENABLE_BIT, true);
}

bool BNO055::Interrupt::disable() {
    return editInterruptState(BNO055_INT_EN, ENABLE_BIT, false);
}

bool BNO055::Interrupt::mask() {
    pinMode(pin, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(pin), *callback, RISING);
    return editInterruptState(BNO055_INT_MSK, MASK_BIT, true);
}

bool BNO055::Interrupt::unmask() {
    return editInterruptState(BNO055_INT_MSK, MASK_BIT, false);
}

byte BNO055::Interrupt::EnabledAxes::getEnabledAxes() {
    byte result = 0b000;
    bitWrite(result, 2, z);
    bitWrite(result, 1, y);
    bitWrite(result, 0, x);
    return result;
}