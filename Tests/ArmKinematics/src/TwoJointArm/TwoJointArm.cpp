//
// Created by alberto on 19/10/19.
//

#include "TwoJointArm.h"
#include <iostream>

TwoJointArm::TwoJointArm(int lowerArmLength, int upperArmLength, sf::Vector2i initialPos){
    sf::Vector2f lowerArmDimensions(lowerArmLength, 20);
    lowerArm.setSize(lowerArmDimensions);

    sf::Vector2f secondStageSizePx(upperArmLength, 20);
    upperArm.setSize(secondStageSizePx);

    kinematics = new TwoJointArmKinematics(lowerArmLength, upperArmLength);

    lowerArm.setPosition(initialPos.x, initialPos.y);

    int xEndArm = std::cos(lowerArm.getRotation() * M_PI / 180.0) * lowerArm.getSize().x;
    int yEndArm = std::sin(lowerArm.getRotation() * M_PI / 180.0) * lowerArm.getSize().x;

    upperArm.setPosition(lowerArm.getPosition().x + xEndArm, lowerArm.getPosition().y +  yEndArm);

    lowerArm.setOrigin(0, 10);
    upperArm.setOrigin(0, 10);

    targetPoint.setRadius(10);
    targetPoint.setPosition(0,0);
    targetPoint.setFillColor(sf::Color::Green);
    targetPoint.setOrigin(targetPoint.getRadius(), targetPoint.getRadius());
    lastTimeUpdate = std::chrono::high_resolution_clock::now();

}

void TwoJointArm::setTargetCoordEnd(sf::Vector2i target) {
    double timeStep = std::chrono::duration<double> (std::chrono::high_resolution_clock::now() - lastTimeUpdate).count();

    TwoJointArmValues values = kinematics->calcInverseKinematics(target.x - lowerArm.getPosition().x, target.y - lowerArm.getPosition().y);
    /**
     * On FTC Robot we would give the targets directly and not add "values.lowerAngle" to "values.upperAngle", I will
     * explain once we test with robot or someone asks. :D
     */
    targetLowerArmAngle = values.lowerAngle * 180.0 / M_PI;
    targetUpperArmAngle = targetLowerArmAngle + values.upperAngle * 180.0 / M_PI;

    double currentLowerArm =  -lowerArm.getRotation();
    if(currentLowerArm < -180){
        currentLowerArm += 360;
    }

    double currentUpperArm =  -upperArm.getRotation();

    if(currentUpperArm < -180){
        currentUpperArm += 360;
    }

    double errorLowerArm = targetLowerArmAngle - currentLowerArm;
    double errorUpperArm = targetUpperArmAngle - currentUpperArm;



    if(errorLowerArm > 180){
        errorLowerArm -= 360;
    }
    if(errorLowerArm < -180){
        errorLowerArm += 360;
    }

    if(errorUpperArm > 180){
        errorUpperArm -= 360;
    }

    if(errorUpperArm < -180){
        errorUpperArm += 360;
    }

    double lowerArmDelta = errorLowerArm * 3 * timeStep;

    if(std::abs(lowerArmDelta) > maxAngleRate * timeStep){
        lowerArmDelta = std::copysign(maxAngleRate * timeStep, lowerArmDelta);
    }

    double upperArmDelta = errorUpperArm * 3 * timeStep;

    if(std::abs(upperArmDelta) > maxAngleRate * timeStep){
        upperArmDelta = std::copysign(maxAngleRate * timeStep, upperArmDelta);
    }

    lowerArm.setRotation(currentLowerArm + lowerArmDelta);
    upperArm.setRotation(currentUpperArm + upperArmDelta);

    int xJointArm = std::cos(lowerArm.getRotation() * M_PI / 180.0) * lowerArm.getSize().x;
    int yJointArm = std::sin(lowerArm.getRotation() * M_PI / 180.0) * lowerArm.getSize().x;

    upperArm.setPosition(lowerArm.getPosition().x + xJointArm, lowerArm.getPosition().y + yJointArm);
    targetPoint.setPosition(lowerArm.getPosition().x + target.x - lowerArm.getPosition().x ,(lowerArm.getPosition().y  + target.y - lowerArm.getPosition().y));

    lowerArm.setRotation(-lowerArm.getRotation());
    upperArm.setRotation(-upperArm.getRotation());
    upperArm.setPosition(upperArm.getPosition().x, screenHeight - upperArm.getPosition().y);
    targetPoint.setPosition(targetPoint.getPosition().x, screenHeight - targetPoint.getPosition().y);

    lastTimeUpdate = std::chrono::high_resolution_clock::now();
}

void TwoJointArm::draw(sf::RenderTarget &target, sf::RenderStates states) const {

    target.draw(lowerArm);
    target.draw(upperArm);
    target.draw(targetPoint);
}


