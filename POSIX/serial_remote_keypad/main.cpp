/**
 * File: test_main.cpp
 * Author: Hikari Hashida
 * Brief: Basic serial communication between POSIX based computer and Arduino via USB.
 * Date: 2021-01-31
 * Last Revised: 2021-04-28
 */

#include <iostream>

#include "sercom.h"

constexpr char arduino_device_path[] = "/dev/tty.LunarSphere-ESP32SPP";
constexpr int BAUD_RATE = 115200;
constexpr size_t MESSAGE_LEN = 20;


int main(void) {
    /* Bluetooth Initialisation */
    SerialDescriptor sd(arduino_device_path, BAUD_RATE);
    sd.open();
    /* timeout < 1 [s] places no bytes to buffer */
    sd.setTimeoutSec(2);

    char cmd_ch;
    std::string val_str;
    bool exit = false;
    do {
        std::cout << "Enter command: ";
        std::cin >> cmd_ch;
        std::cout << "Enter value: ";
        for (std::string line; std::getline(std::cin, line);) {
            std::cout << line << std::endl;
            line = val_str;
            break;
        }

        std::getline(std::cin, val_str);
        std::cout << "read: " << val_str << "\n";
        uint8_t val = std::stoi(val_str);
        std::cout << val << std::endl;

        sd.write(&cmd_ch, sizeof(unsigned char));
        std::cout << "Sent command: " << cmd_ch << "\n";
        sd.write(&val, sizeof(unsigned char));
        std::cout << "Sent value: " << val << "\n";

        char reply[MESSAGE_LEN];
        sd.read(&reply, MESSAGE_LEN);
        std::cout << "Received: " << reply << "\n";
        cmd_ch = '\0';

    } while (!exit);

    return 0;
}