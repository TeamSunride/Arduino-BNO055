//
// Created by Tom Danvers on 02/01/2022.
// 2022 TeamSunride.
//

#ifndef BNO055_CONSTANTS_H
#define BNO055_CONSTANTS_H

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
#define BNO055_OPR_MODE             0x3D
#define BNO055_PWR_MODE             0x3E
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
