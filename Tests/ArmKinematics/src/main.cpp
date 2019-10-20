#include <iostream>
#include <SFML/Graphics.hpp>
#include "TwoJointArm/TwoJointArm.h"
#include "ScreenInfo.h"

unsigned int screenHeight;
unsigned int screenWidth;

int main() {

    screenHeight = sf::VideoMode::getDesktopMode().height * 0.9;
    screenWidth = sf::VideoMode::getDesktopMode().width * 0.75;

    sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(screenWidth ),
                                          static_cast<unsigned int>(screenHeight )), "EctoSim", sf::Style::Close);

    window.setFramerateLimit(60);
    TwoJointArm twoJointArm(100,100, sf::Vector2i(screenWidth/2,screenHeight/2));


    while (window.isOpen()) {
        sf::Event event;

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }
        sf::Vector2<int> mousePos = sf::Mouse::getPosition(window);

        twoJointArm.setTargetCoordEnd(sf::Vector2i(mousePos.x, screenHeight - mousePos.y));

        window.clear(sf::Color(100,100,100));
        window.draw(twoJointArm);
        window.display();
    }

}