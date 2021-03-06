/**
 * File: test_main.cpp
 * Author: Hikari Hashida
 * Brief: Basic serial communication between POSIX based computer and Arduino via USB.
 * Date: 2021-01-31
 * Last Revised: 2021-04-28
 */

#include <iostream>
#include "SDL.h"

#include "sercom.h"

constexpr char arduino_device_path[] = "/dev/tty.LunarSphere-ESP32SPP";
constexpr int BAUD_RATE = 115200;
constexpr size_t MESSAGE_LEN = 20;

constexpr int AXIS_LEFT_X = 0;
constexpr int AXIS_LEFT_Y = 1;
constexpr int AXIS_RIGHT_X = 2;
constexpr int AXIS_RIGHT_Y = 3;
constexpr int HAT_UP = 1;
constexpr int HAT_RIGHT = 2;
constexpr int HAT_DOWN = 4;
constexpr int HAT_LEFT = 8;

void printInfo(SDL_Joystick *joystick) {
    std::cout << "Name: " << SDL_JoystickName(joystick) << "\n";
    std::cout << "Axes: " << SDL_JoystickNumAxes(joystick) << "\n";
    std::cout << "Buttons: " << SDL_JoystickNumButtons(joystick) << "\n";
    std::cout << "Hats: " << SDL_JoystickNumHats(joystick) << "\n";
    std::cout << "Balls: " << SDL_JoystickNumBalls(joystick) << "\n";
}

int main(void) {
    /* Bluetooth Initialisation */
    SerialDescriptor sd(arduino_device_path, BAUD_RATE);
    sd.open();
    /* timeout < 1 [s] places no bytes to buffer */
    sd.setTimeoutSec(2);

    /* Game controller initialisation */
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
    int axis, hat_val;
    char cmd_ch;
    bool exit;
    do {
        // keep polling until event occurs
        // inside while loop in case events are received
        // while handling previous event.
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_JOYBUTTONDOWN:
                    std::cout << "Button: ";
                    std::cout << static_cast<int> (event.jbutton.button) << "\n";
                    if (static_cast<int> (event.jbutton.button) == 10)
                        exit = true;
                    break;
                case SDL_JOYAXISMOTION:
                    // 16-bit signed integer value.
                    // -32768 ~ 0 ~ +32767
                    axis = static_cast<int>(event.jaxis.axis);
                    std::cout << "Axis: " << axis << ", ";
                    std::cout << "val="<< event.jaxis.value << "\n";
                    break;
                case SDL_JOYHATMOTION:
                    hat_val = static_cast<int>(event.jhat.value);
                    cmd_ch = '\0';
                    switch (hat_val) {
                        case HAT_UP:
                            cmd_ch = 'F';
                        break;
                        case HAT_RIGHT:
                            cmd_ch = 'R';
                        break;
                        case HAT_DOWN:
                            cmd_ch = 'B';
                        break;
                        case HAT_LEFT:
                            cmd_ch = 'L';
                        break;
                    }
                    if (cmd_ch != '\0') {
                        sd.write(&cmd_ch, sizeof(unsigned char));
                        std::cout << "Sent command: " << cmd_ch << "\n";
                    }
                    break;
                case SDL_QUIT:
                    exit = true;
                    break;
            }
        }
    } while (!exit);

    SDL_JoystickClose(joystick);
    return 0;
}