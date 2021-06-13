/**
 * File: test_main.cpp
 * Author: Hikari Hashida
 * Brief: Basic serial communication between POSIX based computer and Arduino via USB.
 * Date: 2021-01-31
 * Last Revised: 2021-04-28
 */

#include <cstdlib>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "sercom.h"

constexpr char arduino_device_path[] = "/dev/tty.LunarSphere-ESP32SPP";
constexpr int BAUD_RATE = 115200;
constexpr size_t MESSAGE_LEN = 20;

int main(int argc, char **argv) {
    SerialDescriptor sd(arduino_device_path, BAUD_RATE);
    sd.open();
    /* timeout < 1 [s] places no bytes to buffer */
    sd.setTimeoutSec(2);

    // return reply message buffer.
    const size_t BUFFER_SIZE = 50;
    char reply[BUFFER_SIZE];
    memset(reply, '\0', BUFFER_SIZE);

    // message buffer to be sent.
    const size_t MESSAGE_LEN = 20;
    char message[MESSAGE_LEN];

    while (1) {
        memset(message, '\0', MESSAGE_LEN);
        scanf("%s", message);

        sd.write(&message, sizeof(unsigned char)*MESSAGE_LEN);
        /*
        // uncomment this to read reply message.
        int nbytes = sd.read(&reply, BUFFER_SIZE);
        printf("%s\n", reply);
        memset(reply, '\0', BUFFER_SIZE);
        */
    }
    return 0;
}
