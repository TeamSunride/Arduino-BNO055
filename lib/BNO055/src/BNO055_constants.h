//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#ifndef BNO055_CONSTANTS_H
#define BNO055_CONSTANTS_H

enum BNO055_POWER_MODE {
    /**
     * In normal mode all sensors required for the selected operating mode (see section 3.3) are
     * always switched ON. The register map and the internal peripherals of the MCU are always
     * operative in this mode.
     */
    NORMAL = 0b00,

    /**
     * If no activity (i.e. no motion) is detected for a configurable duration (default 5 seconds), the
     * BNO055 enters the low power mode. In this mode only the accelerometer is active. Once
     * motion is detected (i.e. the accelerometer signals an any-motion interrupt), the system is
     * woken up and normal mode is entered.
     */
    LOW_POWER = 0b01,

    /**
     * In suspend mode the system is paused and all the sensors and the microcontroller are put
     * into sleep mode. No values in the register map will be updated in this mode. To exit from
     * suspend mode the mode should be changed by writing to the PWR_MODE register (see Table 3-1).
     */
    SUSPEND = 0b10
};

/**
 * Describes the possible values for the OPR_MODE register
 *
 *  ╔════════════════╦════════════════════╦═════════════════════════════════════════════╗
 *  ║                ║  Sensor Signals    ║        Available Fusion Data                ║
 *  ╠ Operating Mode ╬═══════╦═════╦══════║══════════════════════╦══════════════════════╣
 *  ║                ║ Accel ║ Mag ║ Gyro ║ Relative-orientation ║ Absolute-orientation ║
 *  ╠════════════════╬═══════╬═════╬══════╬══════════════════════╬══════════════════════╣
 *  ║ CONFIGMODE     ║ -     ║ -   ║ -    ║ -                    ║ -                    ║
 *  ║ ACCONLY        ║ X     ║ -   ║ -    ║ -                    ║ -                    ║
 *  ║ MAGONLY        ║ -     ║ X   ║ -    ║ -                    ║ -                    ║
 *  ║ GYROONLY       ║ -     ║ -   ║ X    ║ -                    ║ -                    ║
 *  ║ ACCMAG         ║ X     ║ X   ║ -    ║ -                    ║ -                    ║
 *  ║ ACCGYRO        ║ X     ║ -   ║ X    ║ -                    ║ -                    ║
 *  ║ MAGGYRO        ║ -     ║ X   ║ X    ║ -                    ║ -                    ║
 *  ║ AMG            ║ X     ║ X   ║ X    ║ -                    ║ -                    ║
 *  ║ IMU            ║ X     ║ -   ║ X    ║ X                    ║ -                    ║
 *  ║ COMPASS        ║ X     ║ X   ║ -    ║ -                    ║ X                    ║
 *  ║ M4G            ║ X     ║ X   ║ X    ║ -                    ║                      ║
 *  ║ NDOF_FMC_OFF   ║ X     ║ X   ║ X    ║ -                    ║ X                    ║
 *  ║ NDOF           ║ X     ║ X   ║ X    ║ -                    ║ X                    ║
 *  ╚════════════════╩═══════╩═════╩══════╩══════════════════════╩══════════════════════╝
 *
 */
enum BNO055_OPERATION_MODE {
    CONFIGMODE = 0b0000,
    ACCONLY = 0b0001,
    MAGONLY = 0b0010,
    GYROONLY = 0b0011,
    ACCMAG = 0b0100,
    ACCGYRO = 0b0101,
    MAGGYRO = 0b0110,
    AMG = 0b0111,
    IMU = 0b1000,
    COMPASS = 0b1001,
    M4G = 0b1010,
    NDOF_FMC_OFF = 0b1011,
    NDOF = 0b1100
};

/**
 * PAGE 0
 *
 * The following registers are on PAGE 0 of the register map
 */

#define BNO055_CHIP_ID              0x00
#define BNO055_ACC_ID               0x01
#define BNO055_MAG_ID               0x02
#define BNO055_GYR_ID               0x03
#define BNO055_SW_REV_ID_LSB        0x04
#define BNO055_SW_REV_ID_MSB        0x05
#define BNO055_BL_Rev_ID            0x06
#define BNO055_PAGE_ID              0x07

#define BNO055_ACC_DATA_X_LSB       0x08
#define BNO055_ACC_DATA_X_MSB       0x09
#define BNO055_ACC_DATA_Y_LSB       0xA
#define BNO055_ACC_DATA_Y_MSB       0xB
#define BNO055_ACC_DATA_Z_LSB       0xC
#define BNO055_ACC_DATA_Z_MSB       0xD

#define BNO055_MAG_DATA_X_LSB       0xE
#define BNO055_MAG_DATA_X_MSB       0xF
#define BNO055_MAG_DATA_Y_LSB       0x10
#define BNO055_MAG_DATA_Y_MSB       0x11
#define BNO055_MAG_DATA_Z_LSB       0x12
#define BNO055_MAG_DATA_Z_MSB       0x13

#define BNO055_GYR_DATA_X_LSB       0x14
#define BNO055_GYR_DATA_X_MSB       0x15
#define BNO055_GYR_DATA_Y_LSB       0x16
#define BNO055_GYR_DATA_Y_MSB       0x17
#define BNO055_GYR_DATA_Z_LSB       0x18
#define BNO055_GYR_DATA_Z_MSB       0x19

#define BNO055_EUL_ROLL_LSB         0x1C
#define BNO055_EUL_ROLL_MSB         0x1D
#define BNO055_EUL_PITCH_LSB        0x1E
#define BNO055_EUL_PITCH_MSB        0x1F

#define BNO055_QUA_DATA_W_LSB       0x20
#define BNO055_QUA_DATA_W_MSB       0x21
#define BNO055_QUA_DATA_X_LSB       0x22
#define BNO055_QUA_DATA_X_MSB       0x23
#define BNO055_QUA_DATA_Y_LSB       0x24
#define BNO055_QUA_DATA_Y_MSB       0x25
#define BNO055_QUA_DATA_Z_LSB       0x26
#define BNO055_QUA_DATA_Z_MSB       0x27

#define BNO055_LIA_DATA_X_LSB       0x28
#define BNO055_LIA_DATA_X_MSB       0x29
#define BNO055_LIA_DATA_Y_LSB       0x2A
#define BNO055_LIA_DATA_Y_MSB       0x2B
#define BNO055_LIA_DATA_Z_LSB       0x2C
#define BNO055_LIA_DATA_Z_MSB       0x2D

#define BNO055_GRV_DATA_X_LSB       0x2E
#define BNO055_GRV_DATA_X_MSB       0x2F
#define BNO055_GRV_DATA_Y_LSB       0x30
#define BNO055_GRV_DATA_Y_MSB       0x31
#define BNO055_GRV_DATA_Z_LSB       0x32
#define BNO055_GRV_DATA_Z_MSB       0x33

#define BNO055_TEMP_REG             0x34

#define BNO055_CALIB_STAT           0x35
#define BNO055_ST_RESULT            0x36
#define BNO055_INT_STA              0x37
#define BNO055_SYS_CLK_STATUS       0x38
#define BNO055_SYS_STATUS           0x39
#define BNO055_SYS_ERR              0x3A
#define BNO055_UNIT_SEL             0x3B
#define BNO055_OPR_MODE_REG             0x3D
#define BNO055_PWR_MODE_REG             0x3E
#define BNO055_SYS_TRIGGER          0x3F
#define BNO055_TEMP_SOURCE          0x40
#define BNO055_AXIS_MAP_CONFIG      0x41
#define BNO055_AXIS_MAP_SIGN        0x42

/**
 * PAGE 1
 *
 * The following registers are on PAGE 1 of the register map
 */

#define BNO055_ACC_CONFIG           0x08
#define BNO055_MAG_CONFIG           0x09
#define BNO055_GYR_CONFIG_0         0xA
#define BNO055_GYR_CONFIG_1         0xB
#define BNO055_ACC_SLEEP_CONFIG     0xC
#define BNO055_GYR_SLEEP_CONFIG     0xD
#define BNO055_INT_MSK              0xF
#define BNO055_INT_EN               0x10

#define BNO055_ACC_AM_THRES         0x11
#define BNO055_ACC_INT_SETTINGS     0x12
#define BNO055_ACC_HG_DURATION      0x13
#define BNO055_ACC_HG_THRES         0x14
#define BNO055_ACC_NM_THRES         0x15
#define BNO055_ACC_NM_SET           0x16

#define BNO055_GYR_INT_SETTING      0x17
#define BNO055_GYR_HR_X_SET         0x18
#define BNO055_GYR_DUR_X            0x19
#define BNO055_GYR_HR_Y_SET         0x1A
#define BNO055_GYR_DUR_Y            0x1B
#define BNO055_GYR_HR_Z_SET         0x1C
#define BNO055_GYR_DUR_Z            0x1D
#define BNO055_GYR_AM_THRES         0x1E
#define BNO055_GYR_AM_SET           0x1F


// copying all of this from the datasheet was a lot of fun

#endif //BNO055_CONSTANTS_H
