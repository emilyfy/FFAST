#ifndef TEENSY_SERIAL_COMMS_H_
#define TEENSY_SERIAL_COMMS_H_

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

int connect_to_serial_port(char device_filename[]) {
    struct termios options;

    int fd = open(device_filename, O_RDWR | O_SYNC | O_NOCTTY);

    while( fd < 0 ) {
        // printf("WARN: Failed to connect to %s. Retrying...\n", device_filename);
        fd = open(device_filename, O_RDWR | O_SYNC | O_NOCTTY);
        sleep(1);
    }

    tcgetattr(fd, &options);
    memset(&options, 0, sizeof(options));

    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR | ICRNL;
    options.c_oflag = 0;
    options.c_lflag = 0;

    options.c_cc[VINTR]  = 0;
    options.c_cc[VQUIT]  = 0;
    options.c_cc[VERASE] = 0;
    options.c_cc[VKILL]  = 0;
    options.c_cc[VEOF]   = 0;
    options.c_cc[VTIME]  = 0;
    options.c_cc[VMIN]   = 0;
    options.c_cc[VSWTC]  = 0;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

#endif // TEENSY_SERIAL_COMMS_H_

