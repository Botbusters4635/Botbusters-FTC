//
// Created by alberto on 20/10/19.
//

#ifndef TEST_TWOJOINTARMKINEMATICS_H
#define TEST_TWOJOINTARMKINEMATICS_H
#include <math.h>

struct TwoJointArmValues {
    double lowerAngle = 0;
    double upperAngle = 0;
};

struct TwoJointArmState {
    double currentX = 0;
    double currentY = 0;
};

class TwoJointArmKinematics {
public:
    TwoJointArmKinematics(double lowerArmLength, double upperArmLength){
        this->lowerArmLength = lowerArmLength;
        this->upperArmLength = upperArmLength;
    }

    TwoJointArmState calcForwardKinematics(TwoJointArmValues values){
        TwoJointArmState state;
        state.currentX = std::cos(values.lowerAngle) * lowerArmLength + std::cos(values.lowerAngle + values.upperAngle) * upperArmLength;
        state.currentY =  std::sin(values.lowerAngle) * lowerArmLength + std::sin(values.lowerAngle + values.upperAngle) * upperArmLength;
        return state;
    }

    TwoJointArmValues calcInverseKinematics(double x, double y){

        TwoJointArmValues values;
        double cosAngle = (std::pow(x, 2.0) + std::pow(y, 2.0) - std::pow(lowerArmLength, 2.0) - std::pow(upperArmLength, 2.0)) / (2.0 * upperArmLength * lowerArmLength);

        if(cosAngle > 1.0){
            cosAngle = 1.0;
        }
        if(cosAngle < -1.0){
            cosAngle = -1.0;
        }

        if(x > 0){
            values.upperAngle = -std::acos(cosAngle);
            values.lowerAngle = std::atan2(y,x) + std::atan2((upperArmLength * std::sin(std::abs(values.upperAngle))),(lowerArmLength + upperArmLength * std::cos(std::abs(values.upperAngle))));
        }else{
            values.upperAngle = std::acos(cosAngle);
            values.lowerAngle = std::atan2(y,x) - std::atan2((upperArmLength * std::sin(std::abs(values.upperAngle))),(lowerArmLength + upperArmLength * std::cos(std::abs(values.upperAngle))));
        }

        return values;
    }

private:
    double lowerArmLength = 0, upperArmLength = 0;
};
#endif //TEST_TWOJOINTARMKINEMATICS_H
