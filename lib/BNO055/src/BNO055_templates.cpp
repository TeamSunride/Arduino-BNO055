//
// Created by Tom Danvers on 04/01/2022.
// 2022 TeamSunride.
//

#ifndef BNO055_TEMPLATES_CPP
#define BNO055_TEMPLATES_CPP

#include "BNO055.h"

template<typename T>
Vector<T> BNO055::getVector(byte offset) {
    Vector<T> vector;
    byte buffer[6];
    memset(buffer, 0, 6);
    readMultipleRegisters(buffer, offset, 6);

    vector.setX(((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8));
    vector.setY(((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8));
    vector.setZ(((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8));
    return vector;
}


#endif