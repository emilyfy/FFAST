#include "vmu_utils.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <cstring>
#include <sys/ioctl.h>
#include <iostream>
#include <iomanip>
#include <string>

int open_port(std::string filename) {
    int fd;
    struct termios options;

    fd = open(filename.c_str(), O_RDWR|O_SYNC|O_NOCTTY);

    if (fd < 0) {
        //std::cout << "Failed to open " << filename << std::endl;
    } else {
        tcgetattr(fd, &options);
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

        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &options);
    }
    return fd;
}

int read_msg(int fd, unsigned char *msg) {

    unsigned char byte;
    unsigned char msg_size;
    unsigned char data[VMU_RAW_MESS_SIZE];

    while (read(fd, &byte, 1)) {
        if (byte == VMU_DATA_START) {

            if (!read(fd, &msg_size, 1)) {
                std::cout << "ERROR: Failed to read data message length." << std::endl;
                return -1;
            }

            if(msg_size < 2) { return -1; }

            if (!read(fd, &data, msg_size-2)) {
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
            if (!read(fd, &msg_size, 1)) {
                std::cout << "ERROR: Failed to read string message length." << std::endl;
                return -1;
            }

            if (!read(fd, &data, msg_size-2)) {
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

int send_cmd(int fd, Cmd cmd) {
    char cmd_buffer[VMU_CMD_NUM_BYTES];

    if( vmutils_buildCmd(cmd, cmd_buffer) != VMU_ERR_SUCCESS) {
        std::cout << "ERROR: Failed to build command." << std::endl;
        return VMU_ERR_INV_CMD;
    }

    for (int i=0; i<VMU_CMD_NUM_BYTES; ++i) {
        int success = write(fd, &cmd_buffer[i], 1);
        usleep(1000);
    }
    return VMU_ERR_SUCCESS;
}

int main(void) {

    std::string device_filename("/dev/VMU931");
    int VMU_fd = open_port(device_filename);

    while (VMU_fd < 0) {
        std::cout << "Failed to open " << device_filename << ". Trying again." << std::endl;
        VMU_fd = open_port(device_filename);
        usleep(5e5);
    }

    std::cout << "VMU931 Connected!" << std::endl;

    unsigned char msg[256];
    bool have_status = false;

    while (1) {

        if (!have_status) {
            send_cmd(VMU_fd, cmd_req_status);
            usleep(100);
        }

        int msg_size = read_msg(VMU_fd, msg);

        int err_code = vmutils_loadMessage(msg, msg[VMU_SIZE_OFFS], (MessType*)&msg[VMU_TYPE_OFFS]);

        if (msg[VMU_TYPE_OFFS] == mt_data) {
            int datatype = vmutils_retrieveDataType();

            Data_h      data_h;
            Data_xyz    data_xyz;
            Data_wxyz   data_wxyz;
            Data_status data_status;

            if (!have_status) {
                if (datatype == dt_status) {
                    have_status = true;

                    vmutils_retrieveStatus(&data_status);
                    std::cout << "Received Status" << std::endl;
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << (int)data_status.sensors_status << std::endl;
                    std::cout << (int)data_status.sensors_res << std::endl;
                    std::cout << (int)data_status.low_output_rate << std::endl;
                    for(int i=0; i<sizeof(uint32_t); ++i) {
                        std::cout << (int)data_status.data_streaming_array[i] << " ";
                    }
                    std::cout << std::endl;

                    if ((data_status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_ACCEL)) {
                        send_cmd(VMU_fd, cmd_toggle_accel);
                    }
                    if (!(data_status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_HEADING)) {
                        send_cmd(VMU_fd, cmd_toggle_heading);
                    }
                    if ((data_status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_QUAT)) {
                        send_cmd(VMU_fd, cmd_toggle_quat);
                    }
                    if ((data_status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_EULER)) {
                        send_cmd(VMU_fd, cmd_toggle_euler);
                    }
                    if ((data_status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_MAG)) {
                        send_cmd(VMU_fd, cmd_toggle_mag);
                    }
                }
            } else {
                switch (datatype) {
                    case dt_accel:
                        vmutils_retrieveXYZData(&data_xyz);
                        std::cout << "ACCEL DATA: " << std::fixed << std::setprecision(1)
                                  << data_xyz.x << " " << data_xyz.y << " " << data_xyz.z << std::endl;
                        break;
                    case dt_gyro:
                        vmutils_retrieveXYZData(&data_xyz);
                        std::cout << "GYRO DATA: "
                                  << data_xyz.x << " " << data_xyz.y << " " << data_xyz.z << std::endl;
                        break;
                    case dt_mag:
                        vmutils_retrieveXYZData(&data_xyz);
                        std::cout << "MAG DATA: "
                                  << data_xyz.x << " " << data_xyz.y << " " << data_xyz.z << std::endl;
                        break;
                    case dt_euler:
                        vmutils_retrieveXYZData(&data_xyz);
                        std::cout << "EULER DATA: "
                                  << data_xyz.x << " " << data_xyz.y << " " << data_xyz.z << std::endl;
                        break;
                    case dt_quat:
                        vmutils_retrieveWXYZData(&data_wxyz);
                        std::cout << "QUAT DATA: "
                                  << data_wxyz.x << " " << data_wxyz.y << " "
                                  << data_wxyz.z << " " << data_wxyz.w << std::endl;
                        break;
                    case dt_heading:
                        vmutils_retrieveHData(&data_h);
                        std::cout << "HEADING DATA: " << data_h.h << std::endl;
                        break;
                    case dt_status:
                        vmutils_retrieveStatus(&data_status);
                        std::cout << "Received Status" << std::endl;
                        std::cout << std::hex << std::setw(2) << std::setfill('0')
                                  << (int)data_status.sensors_status << std::endl;
                        std::cout << (int)data_status.sensors_res << std::endl;
                        std::cout << (int)data_status.low_output_rate << std::endl;
                        for(int i=0; i<sizeof(uint32_t); ++i) {
                            std::cout << (int)data_status.data_streaming_array[i] << " ";
                        }
                        std::cout << std::endl;
                        break;
                }
            }
        }
    }



    close(VMU_fd);

    return 0;
}





