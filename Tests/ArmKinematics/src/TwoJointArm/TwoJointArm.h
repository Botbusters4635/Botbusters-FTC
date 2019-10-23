//
// Created by alberto on 19/10/19.
//

#ifndef TEST_TWOJOINTARM_H
#define TEST_TWOJOINTARM_H
#include <SFML/Graphics.hpp>
#include <math.h>
#include <iostream>
#include "TwoJointArmKinematics.h"
#include "ScreenInfo.h"
#include <chrono>

class TwoJointArm : public sf::Drawable{
public:
    TwoJointArm(int lowerArmLength, int upperArmLength, sf::Vector2i initialPos);
    void setTargetCoordEnd(sf::Vector2i target);
protected:
    void draw(sf::RenderTarget &target, sf::RenderStates states) const override;
private:
    sf::RectangleShape lowerArm;
    sf::RectangleShape upperArm;
    sf::CircleShape targetPoint;
    TwoJointArmKinematics* kinematics;
    double maxAngleRate = 90; //Per second
    double targetLowerArmAngle = 0;
    double targetUpperArmAngle = 0;
    std::chrono::high_resolution_clock ::time_point lastTimeUpdate;
};

#endif //TEST_TWOJOINTARM_H