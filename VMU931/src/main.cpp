#include "vmu.hpp"

#include <iostream>
#include <iomanip>
#include <string>


int main(void) {

    unsigned char msg[256];
    int err_code;

    std::string device_filename("/dev/VMU931");
    VMU931 accel(device_filename);

    while (1) {

        int msg_size = accel.read_msg(msg);

        err_code = vmutils_loadMessage(msg, msg[VMU_SIZE_OFFS], (MessType*)&msg[VMU_TYPE_OFFS]);

        if (msg[VMU_TYPE_OFFS] == mt_data) {
            int datatype = vmutils_retrieveDataType();

            Data_h      data_h;
            Data_xyz    data_xyz;
            Data_wxyz   data_wxyz;
            Data_status data_status;

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

    return 0;
}
