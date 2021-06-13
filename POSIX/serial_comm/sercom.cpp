/**
 * @file sercom.cpp
 * @author Hikari Hashida
 * @brief Serial communication class for POSIX based system and a microcontroller (tested on Arduino).
 * @version 0.1
 * @date 2021-01-31
 * 
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "sercom.h"

/**
 * @brief Construct a new Serial Descriptor:: Serial Descriptor object
 * 
 * @param device_path 
 * @param baudrate 
 */
SerialDescriptor::SerialDescriptor(
    const char* device_path, const int baudrate) {
    baudrate_ = baudrate;
    device_path_.append(device_path);
    timeout_.tv_sec = 0;
    timeout_.tv_usec = 0;
}

/**
 * @brief Destroy the Serial Descriptor:: Serial Descriptor object
 * 
 */
SerialDescriptor::~SerialDescriptor() {
    if (::close(fd_) < 0) {
        perror("port error");
    }
}

/**
 * @brief POSIX wrapper for opening serial connection with the Arduino.
 * 
 */
void SerialDescriptor::open() {
    int c_flag =    O_RDWR      // read/write operation
                |   O_NOCTTY    // not a controlling terminal
                |   O_NDELAY;   // don't delay it even if other
                                // end of the port is not up/running
    fd_ = ::open(device_path_.c_str(), c_flag);

    int old_flags;
    if (fd_ < 0) {
        perror("error opening port");
    } else {
        old_flags = fcntl(fd_, F_GETFL, 0);  // get current status
        old_flags = old_flags & ~O_NDELAY;   // turn off NDELAY
        fcntl(fd_, F_SETFL, old_flags);      // set blocking
    }
    struct termios port_option;
    memset(&port_option, 0, sizeof(port_option));

    // get current port options.
    tcgetattr(fd_, &port_option);

    /* set i/o baud rates */
    cfsetispeed(&port_option, baudrate_);
    cfsetospeed(&port_option, baudrate_);
    /* 8-bit data to be sent */
    port_option.c_cflag = (port_option.c_cflag & ~CSIZE) | CS8;
    /* prevents the port from becoming the "owner" */
    port_option.c_cflag |= (CLOCAL | CREAD);
    /* stop bit option */
    port_option.c_cflag &= ~CSTOPB;
    /* parity option */
    port_option.c_cflag &= ~(PARENB | PARODD);
    /* hardware based flow control */
    port_option.c_cflag &= ~CRTSCTS;
    /* software based flow control (outgoing/incoming) */
    port_option.c_iflag &= ~(IXON | IXOFF | IXANY);
    /* set cononical mode */
    port_option.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // set port options.
    tcsetattr(fd_, TCSANOW, &port_option);
}

/**
 * @brief POSIX serial read() wrapper.
 * 
 * @param reply buffer array to store received message
 * @param buff_len length of the storing buffer array
 * @return number of bytes received
 */
#include <iostream>

int SerialDescriptor::read(void* reply, int buff_len) {
    int num;
    if (timeout_.tv_sec > 0 || timeout_.tv_usec > 0) {
        FD_ZERO(&read_fds_);
        FD_SET(fd_, &read_fds_);
        /* wait til data is available with timeout */
        if (select(fd_+1, &read_fds_, NULL, NULL, &timeout_) > 0) {
            num = ::read(fd_, reply, buff_len);
        }
    } else {
        /* no timeout */
        num = ::read(fd_, reply, buff_len);
    }
    return num;
}

/**
 * @brief POSIX serial write() wrapper
 * 
 * @param msg message buffer array to send
 * @param nbyte number of bytes to be sent
 * @return number of successful bytes
 */
int SerialDescriptor::write(void* msg, int nbyte) {
    int num = ::write(fd_, msg, nbyte);
    tcdrain(fd_);
    return num;
}

void SerialDescriptor::setTimeout(int sec, int u_sec) {
    timeout_.tv_sec = sec;
    timeout_.tv_usec = u_sec;
}

void SerialDescriptor::setTimeoutSec(uint16_t sec) {
    timeout_.tv_sec = sec;
}

void SerialDescriptor::setTimeoutMicroSec(uint16_t usec) {
    timeout_.tv_usec = usec;
}