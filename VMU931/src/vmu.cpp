#include "vmu.hpp"
#include "vmu_utils.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <cstring>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <string>


VMU931::VMU931(std::string filename) : _status_valid(false) {
    struct termios options;

    _fd = open(filename.c_str(), O_RDWR|O_SYNC|O_NOCTTY);

    while (_fd < 0) {
        std::cout << "ERROR: Failed to connect to VMU931. Retrying..." << std::endl;
        _fd = open(filename.c_str(), O_RDWR|O_SYNC|O_NOCTTY);
        sleep(1);
    }

    tcgetattr(_fd, &options);
    bzero(&options, sizeof(options));

    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR | ICRNL;
    options.c_oflag = 0;
    options.c_lflag = 0;

    options.c_cc[VINTR] = 0;
    options.c_cc[VQUIT] = 0;
    options.c_cc[VERASE] = 0;
    options.c_cc[VKILL] = 0;
    options.c_cc[VEOF] = 4;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 1;
    options.c_cc[VSWTC] = 0;

    tcflush(_fd, TCIFLUSH);
    tcsetattr(_fd, TCSANOW, &options);

    std::cout << "VMU931 Accelerometer Connected." << std::endl;
    update_status();
    std::cout << "VMU931 Accelerometer Status Received." << std::endl;
    set_stream_accel(false);
    set_stream_euler(false);
    set_stream_heading(false);
    set_stream_mag(true);
    set_stream_quat(false);
    sleep(5);
}

VMU931::~VMU931(void) {
    close(_fd);
}

int VMU931::read_msg(unsigned char *msg) {

    unsigned char byte;
    unsigned char msg_size;
    unsigned char data[VMU_RAW_MESS_SIZE];

    while (read(_fd, &byte, 1)) {
        if (byte == VMU_DATA_START) {

            if (!read(_fd, &msg_size, 1)) {
                std::cout << "ERROR: Failed to read data message length." << std::endl;
                return -1;
            }

            if(msg_size < 2) { return -1; }

            if (!read(_fd, &data, msg_size-2)) {
                std::cout << "ERROR: Failed to read data from message." << std::endl;
                return -1;
            }

            if (data[msg_size-3] != VMU_DATA_END) {
                continue;
            }

            msg[VMU_START_BYTE_OFFS] = VMU_DATA_START;
            msg[VMU_SIZE_OFFS] = msg_size;
            std::memcpy(&msg[VMU_SIZE_OFFS+1], data, msg_size-2);

            return msg_size;
        }

        else if (byte == VMU_STRING_START) {
            if (!read(_fd, &msg_size, 1)) {
                std::cout << "ERROR: Failed to read string message length." << std::endl;
                return -1;
            }

            if (!read(_fd, &data, msg_size-2)) {
                std::cout << "ERROR: Failed to read string from message." << std::endl;
                return -1;
            }

            if (data[msg_size-3] != VMU_STRING_END) {
                continue;
            }

            msg[VMU_START_BYTE_OFFS] = VMU_DATA_START;
            msg[VMU_SIZE_OFFS] = msg_size;
            std::memcpy(&msg[VMU_SIZE_OFFS+1], data, msg_size-2);

            return msg_size;
        }
    }
}

int VMU931::send_cmd(Cmd cmd) {
    char cmd_buffer[VMU_CMD_NUM_BYTES];

    if( vmutils_buildCmd(cmd, cmd_buffer) != VMU_ERR_SUCCESS) {
        std::cout << "ERROR: Failed to build command." << std::endl;
        return VMU_ERR_INV_CMD;
    }

    for (int i=0; i<VMU_CMD_NUM_BYTES; ++i) {
        int success = write(_fd, &cmd_buffer[i], 1);
        usleep(1000);
    }
    return VMU_ERR_SUCCESS;
}

void VMU931::update_status(void) {

    if (_status_valid) {return;}

    int msg_size, err_code;
    Data_status data_status_update;
    unsigned char msg_buff[VMU_RAW_MESS_SIZE];

    while (1) {
        err_code = send_cmd(cmd_req_status);

        double t_start = clock();
        while((clock() - t_start)/CLOCKS_PER_SEC < 1) {

            msg_size = read_msg(msg_buff);
            err_code = vmutils_loadMessage(msg_buff, msg_buff[VMU_SIZE_OFFS], (MessType*)&msg_buff[VMU_TYPE_OFFS]);

            if (msg_buff[VMU_TYPE_OFFS] == mt_data && vmutils_retrieveDataType() == dt_status) {
                std::cout << "GOT STATUS!" << std::endl;
                memcpy(&_data_status, &data_status_update, sizeof(_data_status));
                _status_valid = true;
                return;
            }
        }
    }
}

void VMU931::set_stream_accel(bool enable) {
    update_status();
    if (enable != (_data_status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_ACCEL)) {
        std::cout << "TOGGLE ACCEL" << std::endl;
        send_cmd(cmd_toggle_accel);
    }
    _data_status.data_streaming_array[3] ^= VMU_STATUS_MASK_STREAM_ACCEL;
}

void VMU931::set_stream_euler(bool enable) {
    update_status();
    if (enable != (_data_status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_EULER)) {
        std::cout << "TOGGLE EULER" << std::endl;
        send_cmd(cmd_toggle_euler);
    }
    _data_status.data_streaming_array[3] ^= VMU_STATUS_MASK_STREAM_EULER;
}

void VMU931::set_stream_heading(bool enable) {
    update_status();
    if (enable != (_data_status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_HEADING)) {
        std::cout << "TOGGLE HEADING" << std::endl;
        send_cmd(cmd_toggle_heading);
    }
    _data_status.data_streaming_array[3] ^= VMU_STATUS_MASK_STREAM_HEADING;
}

void VMU931::set_stream_mag(bool enable) {
    update_status();
    if (enable != (_data_status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_MAG)) {
        std::cout << "TOGGLE MAG" << std::endl;
        send_cmd(cmd_toggle_mag);
    }
    _data_status.data_streaming_array[3] ^= VMU_STATUS_MASK_STREAM_MAG;
}

void VMU931::set_stream_quat(bool enable) {
    update_status();
    if (enable != (_data_status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_QUAT)) {
        std::cout << "TOGGLE QUAT" << std::endl;
        send_cmd(cmd_toggle_quat);
    }
    _data_status.data_streaming_array[3] ^= VMU_STATUS_MASK_STREAM_QUAT;
}
