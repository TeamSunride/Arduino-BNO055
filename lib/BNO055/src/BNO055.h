//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#ifndef BNO055_H
#define BNO055_H

#include "Arduino.h"
#include "I2CDevice.h"
#include "BNO055_constants.h"
#include "util/Quaternion.h"

// DATASHEET: https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

class BNO055 : public I2CDevice {
public:
    using I2CDevice::I2CDevice;
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

    /**
     * Get fused linear acceleration data in m/s
     * @return A Vector containing fused acceleration data in m/s
     */
    Vector<double> getLinearAcceleration();

    /**
     * Get fused absolute orientation data as a quaternion
     */
    Quaternion getQuaternion();

    /**
     * Get euler angles in degrees
     * @return Vector containing euler angles in degrees
     */
    Vector<double> getEuler();

    /**
     * @return Vector representing gravity
     */
    Vector<double> getGravity();


    /**
     * Get a temperature reading from the accelerometer or gyro, depending on which
     * BNO055_TEMP_SOURCE_TYPE is selected with #setTempSource.
     *
     * Keep in mind that the sensor will not return tenperature data if not turned on.
     * Therefore make sure the sensor is enabled with the correct operation mode
     * before trying to get its temperature
     * @return Temperature in degrees celsius
     */
    byte getTemp();

    /**
     * Set either the accelerometer or gyro to be used for temperature readings
     * @param source Either ACCEL_TEMP or GYRO_TEMP
     * @return Whether the write operation was successful
     */
    bool setTempSource(BNO055_TEMP_SOURCE_TYPE source);

    /**
     * Set the G range, bandwidth, and operation mode of the accelerometer.
     *
     * Note that you can't change the Bandwidth and Operation Mode when in a sensor fusion
     * mode
     * @param value The value to write to the ACC_Config register. Refer to section 3.5.2 in the
     * datasheet
     * @return Whether the writes were successful
     */
    bool setAccelerometerConfig(byte value);

    /**
     * Set the range and bandwidth of the gyroscope.
     *
     * Note that you can't control these when in a sensor fusion mode. Use
     * setGyroscopeOperationMode to set the operation mode of the gyro.
     *
     * @param value The value to write to GYR_Config_0. Refer to section 3.5.3 in the
     * datasheet.
     * @return Whether the writes were successful
     */
    bool setGyroscopeConfig(byte value);

    /**
     * Set the operation mode of the gyroscope.
     *
     * Note that you can't control this when in a sensor fusion mode. Use
     * setGyroscopeConfig to set the range and bandwidth of the gyro
     *
     * @param value The value to write to GYR_Config_1. Refer to section 3.5.3 in the
     * datasheet.
     * @return Whether the writes were successful
     */
    bool setGyroscopeOperationMode(byte value);

    /**
     * Set the data output date, power mode, and operation mode of the magnetometer.
     *
     * Note that you can't change any of these in sensor fusion mode
     * @param value The value to write to the MAG_Config register. Refer to section 3.5.4 in the
     * datasheet
     * @return Whether the writes were successful
     */
    bool setMagnetometerConfig(byte value);

    bno055_calib_stat_t getCalibrationStatus();

    /**
     * Clear the current interrupt, making the interrupt pin go low
     * @return Whether the write was successful
     */
    bool clearInterrupt();

    /**
     * Class to be inherited by implementations of the various interrupts offered by the
     * BNO055. Refer to section 3.7 of the datasheet.
     */
    class Interrupt {

    public:

        /**
         * Enable this Interrupt. This means that Interrupt occurences will be written to the
         * INT_STA register
         * @return Whether the operation was successful
         */
        bool enable();

        /**
         * Disable this Interrupt.
         * @return Whether the operation was successful
         */
        bool disable();

        /**
         * Interrupts can be routed to the INT pin by calling this function.
         * @return Whether the operation was successful
         */
        bool mask();

        /**
         * Stop routing this interrupt to the INT pin.
         * @return Whether the operation was successful
         */
        bool unmask();

        /**
         * An object to aid enabling/disabling the axes regarding Interrupts
         */
        class AxesSetting {

        public:
            bool x = false;
            bool y = false;
            bool z = false;;

            AxesSetting enableX() { x = true; return *this; }
            AxesSetting enableY() { y = true; return *this; };
            AxesSetting enableZ() { z = true; return *this; };

            AxesSetting disableX() { x = false; return *this; };
            AxesSetting disableY() { y = false; return *this; };
            AxesSetting disableZ() { z = false; return *this; };

            byte get();
        };

        /**
         * @param bno055 A reference to an instantiated BNO055 object. Used to communicate with
         * the various registers involved with interrupts.
         * @param enableBit The register INT_EN contains bits which enable/disable the different
         * available interrupts. This parameter specifies the location of the bit that describes
         * whether this Interrupt is enabled.
         * @param maskBit Interrupts can be routed to the INT pin by setting the corresponding
         * interrupt bit in the INT_MSK register. This parameter specified the location of this
         * interrupt bit.
         * @param callback0 The ISR (Interrupt Service Routine) to run when an interrupt is
         * detected.
         * @param pin0 The digital pin of the microcontroller to assign this interrupt to.
         */
        Interrupt(BNO055 &bno055, int enableBit, int maskBit, void (*callback0)(), int pin0) {
            ENABLE_BIT = enableBit;
            MASK_BIT = maskBit;
            sensor = &bno055;
            callback = callback0;
            pin = pin0;
        }

    protected:
        int ENABLE_BIT;
        int MASK_BIT;
        BNO055 *sensor;
        void (*callback)();
        int pin;
    private:
        /**
         * Helper function to aid configuration of Interrupts
         * @param offset The location of the register to edit
         * @param bit The bit to change
         * @param value The new value to set the bit to
         * @return Whether the write was successful
         */
        bool editInterruptState(byte offset, int bit, bool value);
    };

private:
    int BNO055_ACCEL_CONVERSION_FACTOR = 100;
    int BNO055_MAG_CONVERSION_FACTOR = 16;
    int BNO055_GYRO_CONVERSION_FACTOR = 16;
    double BNO055_QUAT_CONVERSION_FACTOR = 1 << 14;
    Vector<int16_t> getVector(byte offset);
    Vector<int16_t> getVector(byte buffer[], int startIndex);
    BNO055_OPERATION_MODE currentOperationMode = CONFIGMODE;

public:

    /**
     * Object that implements the High-G interrupt as specified in section 3.7.2.3
     * of the Datasheet.
     */
    class AccelHighGInterrupt : public BNO055::Interrupt {
    private:
        byte duration;
        byte threshold;
        AxesSetting axesSetting{};
    public:
        /**
         * Create a new interrupt object. This does not write the new configuration to the BNO055. In order
         * to write the new configuration to the sensor, use AccelHighGInterrupt#setup()
         * @param bno055 A reference to the instantiated sensor object. Used to communicate with the sensor.
         * @param axesSetting0 a BNO055::Interrupt::AxesSetting object
         * @param callback0 The ISR (Interrupt Service Routine) to run when an interrupt is
         * detected.
         * @param pin The digital pin of the microcontroller to assign this interrupt to.
         * @param duration0 The value to write to the ACC_HG_DURATION register.
         * @param threshold0 The value to write to the ACC_HG_THRES register.
         */
        AccelHighGInterrupt(
                BNO055 bno055,
                BNO055::Interrupt::AxesSetting axesSetting0,
                void (*callback0)(), int pin,
                byte duration0 = 0b1111, // default value from datasheet
                byte threshold0 = 0b11000000  // default value from datasheet
                            ) : Interrupt(bno055, 5, 5, callback0, pin)
        {
            duration = duration0;
            threshold = threshold0;
            axesSetting = axesSetting0;
        }

        /**
         * Write the new configuration to the sensor.
         * @return Whether the write was successful
         */
        bool setup() {
            sensor->setPageID(1);
            bool result = sensor->writeRegister(BNO055_ACC_HG_DURATION, duration) &&
                          sensor->writeRegister(BNO055_ACC_HG_THRES, threshold) &&
                          sensor->writeRegister(BNO055_ACC_INT_SETTINGS, axesSetting.get() << 5);
            sensor->setPageID(0);
            return result;
        }
    };
};




#endif //BNO055_H
