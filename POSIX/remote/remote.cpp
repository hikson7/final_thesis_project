// file: remote.cpp
// author: Hikari hashida
// brief: 
// date: 04/05/2021
// last visited: 04/05/2021

#include <iostream>

#include "SDL.h"

constexpr int AXIS_LEFT_X = 0;
constexpr int AXIS_LEFT_Y = 1;
constexpr int AXIS_RIGHT_X = 2;
constexpr int AXIS_RIGHT_Y = 3;

void printInfo(SDL_Joystick *joystick) {
    std::cout << "Name: " << SDL_JoystickName(joystick) << "\n";
    std::cout << "Axes: " << SDL_JoystickNumAxes(joystick) << "\n";
    std::cout << "Buttons: " << SDL_JoystickNumButtons(joystick) << "\n";
    std::cout << "Hats: " << SDL_JoystickNumHats(joystick) << "\n";
    std::cout << "Balls: " << SDL_JoystickNumBalls(joystick) << "\n";
}

int main(void) {
    if (SDL_Init(SDL_INIT_JOYSTICK)) {
        fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
        exit(1);
    }
    SDL_Joystick *joystick;                 // joystick struct pointer
    SDL_JoystickEventState(SDL_ENABLE);     // enable joystick event
    joystick = SDL_JoystickOpen(0);         // open 0th joystick
    if (joystick == NULL) {
        fprintf(stderr, "Couldn't find controller: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }

    printInfo(joystick);

    SDL_Event event;
    int axis, hat;
    bool exit;
    do {
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_JOYBUTTONDOWN:
                /* handle keyboard stuff here */
                    std::cout << "Button: ";
                    std::cout << static_cast<int> (event.jbutton.button) << "\n";
                    break;
                case SDL_JOYAXISMOTION:
                    // 16-bit signed integer value.
                    // -32768 ~ 0 ~ +32767
                    axis = static_cast<int>(event.jaxis.axis);
                    std::cout << "Axis: " << axis << ", ";
                    std::cout << "val="<< event.jaxis.value << "\n";
                    break;
                case SDL_JOYHATMOTION:
                    hat = static_cast<int>(event.jhat.hat);
                    std::cout << "Hat: ";
                    std::cout << hat << ": "<< static_cast<int>(event.jhat.value) << "\n";
                    break;
                case SDL_QUIT:
                    exit = true;
                    break;
            }
        }
    } while (!exit);
    // SDL_Quit();
    SDL_JoystickClose(joystick);
    return 0;
}
