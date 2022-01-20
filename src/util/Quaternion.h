//
// Created by Tom Danvers on 06/01/2022.
// 2022 TeamSunride.
//

#ifndef QUATERNION_H
#define QUATERNION_H

class Quaternion {
public:
    double getX() { return dimensions[0]; };
    double getY() { return dimensions[1]; };
    double getZ() { return dimensions[2]; };
    double getW() { return dimensions[3]; };

    void setX(double value) { dimensions[0] = value; };
    void setY(double value) { dimensions[1] = value; };
    void setZ(double value) { dimensions[2] = value; };
    void setW(double value) { dimensions[3] = value; };

    Quaternion divideScalar(double factor) {
        Quaternion quat;
        for (int i=0; i<4; i++) {
            quat.dimensions[i] = dimensions[i] / factor;
        }
        return quat;
    }

private:
    double dimensions[4]{};
};

#endif //QUATERNION_H
