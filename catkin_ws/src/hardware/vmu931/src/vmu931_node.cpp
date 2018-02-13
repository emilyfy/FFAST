#include <ros/ros.h>
#include <vmu931/Vmu931.h>
#include <sensor_msgs/Imu.h>
#include <termios.h>
#include <sys/select.h>
#include <fcntl.h>
#include "vmu_utils.h"

#define G_MPS2 9.81

int connect_to_serial_port(const char device_filename[]) {
    struct termios options;

    int fd = open(device_filename, O_RDWR | O_SYNC | O_NOCTTY);

    while( fd < 0 ) {
        // printf("WARN: Failed to connect to %s. Retrying...\n", device_filename);
        ROS_WARN("Failed to connect to %s. Retrying...", device_filename);
        fd = open(device_filename, O_RDWR | O_SYNC | O_NOCTTY);
        sleep(1);
        if (!ros::ok()) { return -1; }
    }

    tcgetattr(fd, &options);
    memset(&options, 0, sizeof(options));

    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR | ICRNL;
    options.c_oflag = 0;
    options.c_lflag = 0;

    options.c_cc[VINTR] = 0;
    options.c_cc[VQUIT] = 0;
    options.c_cc[VERASE] = 0;
    options.c_cc[VKILL] = 0;
    options.c_cc[VEOF] = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    options.c_cc[VSWTC] = 0;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

// Return -1 on error. Otherwise, return the number of bytes received.
int load_msg(int vmu_fd) {
    unsigned char byte;
    unsigned char msg_size;
    unsigned char data[VMU_RAW_MESS_SIZE];
    unsigned char  msg[VMU_RAW_MESS_SIZE];
    MessType string_or_data_msg;

    while( read(vmu_fd, &byte, 1) ) {

        // Search for the start of a data message.
        if( byte == VMU_DATA_START ) {

            // Read in the length of the message.
            if( !read(vmu_fd, &msg_size, 1) ) {
            //    printf("WARN: Failed to read length of message.\n"); 
               return -1;
            }

            // If the message is too short to contain any data, return an error. 
            if( msg_size < 2 ) { return -1; }

            if( !read(vmu_fd, &data, msg_size-2) ) {
                // printf("WARN: Failed to read message data.\n");
                return -1;
            }

            // Check the message footer.
            if( data[msg_size-3] != VMU_DATA_END ) {
                continue;
            }

            msg[VMU_START_BYTE_OFFS] = VMU_DATA_START;
            msg[VMU_SIZE_OFFS] = msg_size;
            memcpy(&msg[VMU_SIZE_OFFS+1], data, msg_size-2);

            string_or_data_msg = mt_data;

            // Load the message for future use.
            if( VMU_ERR_SUCCESS != vmutils_loadMessage(msg, msg_size, &string_or_data_msg) ) {
                // printf("WARN: Failed to load message.\n");
                return -1;
            }
            return msg_size;

        } else if( byte == VMU_STRING_START) {

            // Read in the length of the message.
            if( !read(vmu_fd, &msg_size, 1) ) {
                // printf("WARN: Failed to read length of message.\n");
                return -1;
            }

            // If the message is too short to contain any data, return an error. 
            if( msg_size < 2 ) { return -1; }
            
            if( !read(vmu_fd, &data, msg_size-2) ) {
                // printf("WARN: Failed to read string from message.\n");
                return -1;
            }

            // Check the message footer.
            if( data[msg_size-3] != VMU_STRING_END ) {
               return -1; 
            }

            msg[VMU_START_BYTE_OFFS] = VMU_STRING_START;
            msg[VMU_SIZE_OFFS] = msg_size;
            memcpy(&msg[VMU_SIZE_OFFS+1], data, msg_size-2);

            string_or_data_msg = mt_string;

            // Load the message for future use.
            if( VMU_ERR_SUCCESS != vmutils_loadMessage(msg, msg_size, &string_or_data_msg) ) {
                // printf("WARN: Failed to load message. %d\n", vmutils_loadMessage(msg, msg_size, &string_or_data_msg));
                return -1;
            }
            return msg_size;

        }
    }
    return -1;
}

int send_cmd(int vmu_fd, Cmd cmd) {
    char cmd_buffer[VMU_CMD_NUM_BYTES];

    if( vmutils_buildCmd(cmd, cmd_buffer) != VMU_ERR_SUCCESS) {
        // printf("WARN: Failed to build command.\n");
        return -1;
    }


    int i;
    for( i=0; i<VMU_CMD_NUM_BYTES; ++i ) {
        int success = write(vmu_fd, &cmd_buffer[i], 1);
        usleep(100);
    }
    return 1;

}

int update_status(int vmu_fd) {

    int msg_size, err_code, retry_count;

    // Send status requests until you get a response.
    while(1) {
        err_code = send_cmd(vmu_fd, cmd_req_status);
        if( -1 == err_code ) { continue; }

        retry_count = 1000;
        while(retry_count-- > 0) {
            msg_size = load_msg(vmu_fd);
            if( msg_size == -1 ) { continue; } 
            
            if( singleMessage.messType == mt_data && vmutils_retrieveDataType() == dt_status ) {
                // Got a status message!
                return 0;
            }
            usleep(1000);
        }
    }
}

int set_sensors_status(int vmu_fd, char accel_en, char euler_en, char gyro_en,
                                   char heading_en, char mag_en, char quat_en) {
    Data_status status;
    update_status(vmu_fd);
    vmutils_retrieveStatus(&status);


    char accel_en_   = ( status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_ACCEL   ) != 0;
    char euler_en_   = ( status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_EULER   ) != 0;
    char gyro_en_    = ( status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_GYRO    ) != 0;
    char heading_en_ = ( status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_HEADING ) != 0;
    char mag_en_     = ( status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_MAG     ) != 0;
    char quat_en_    = ( status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_QUAT    ) != 0;

    char keep_going = 1;
    while( keep_going == 1 ) {

        keep_going = 0;

        if(   accel_en_ !=   accel_en ) { send_cmd(vmu_fd, cmd_toggle_accel); keep_going = 1; }
        if(   euler_en_ !=   euler_en ) { send_cmd(vmu_fd, cmd_toggle_euler); keep_going = 1; }
        if(    gyro_en_ !=    gyro_en ) { send_cmd(vmu_fd, cmd_toggle_gyro);  keep_going = 1; }
        if( heading_en_ != heading_en ) { send_cmd(vmu_fd, cmd_toggle_heading); keep_going = 1; }
        if(     mag_en_ !=     mag_en ) { send_cmd(vmu_fd, cmd_toggle_mag); keep_going = 1; }
        if(    quat_en_ !=    quat_en ) { send_cmd(vmu_fd, cmd_toggle_quat); keep_going = 1; }


        if( keep_going == 0 ) { break; }

        usleep(1000000);
        update_status(vmu_fd);
        vmutils_retrieveStatus(&status);

        accel_en_   = ( status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_ACCEL   ) != 0;
        euler_en_   = ( status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_EULER   ) != 0;
        gyro_en_    = ( status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_GYRO    ) != 0;
        heading_en_ = ( status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_HEADING ) != 0;
        mag_en_     = ( status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_MAG     ) != 0;
        quat_en_    = ( status.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_QUAT    ) != 0;

    }
    return 1;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "vmu931");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string serial_port;
    bool publish_vmu931;

    nh_private.param("serial_port", serial_port, std::string("/dev/VMU931"));
    nh_private.param("publish_vmu931", publish_vmu931, false);

    int vmu_fd = connect_to_serial_port(serial_port.c_str());
    if (vmu_fd == -1) { return 0; }
    ROS_INFO("IMU connected!\n");

    Data_xyz    data_accel;
    Data_xyz    data_euler;
    Data_xyz    data_gyro;
    Data_h      data_heading;
    Data_xyz    data_mag;
    Data_wxyz   data_quat;

    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("/imu", 10);
    sensor_msgs::Imu imu_data;
    imu_data.header.frame_id = "base_imu";
    
    ros::Publisher pub_vmu931;
    vmu931::Vmu931 vmu931_data;
    vmu931_data.header.frame_id = "base_imu";
    
    if (publish_vmu931) {
        pub_vmu931 = nh.advertise<vmu931::Vmu931>("/vmu931", 10);
        set_sensors_status(vmu_fd, 1, 1, 1, 1, 1, 1);
    } else set_sensors_status(vmu_fd, 1, 0, 1, 0, 0, 1);

    ros::Time curr_time, last_time;
    curr_time = ros::Time::now();
    last_time = curr_time;

    while (ros::ok()) {
        int msg_length = load_msg(vmu_fd);
        if( msg_length <= 0 ) { continue; }

        DataType data_type = vmutils_retrieveDataType();

        curr_time = ros::Time::now();
        if ( (curr_time - last_time).toSec() > 5 ) {
            ROS_WARN("Haven't received any IMU message in %f seconds.", (curr_time-last_time).toSec());
        }

        switch( data_type ) {
            case dt_accel:
                vmutils_retrieveXYZData(&data_accel);
                if (publish_vmu931) {
                    vmu931_data.accel.x = data_accel.x;
                    vmu931_data.accel.y = data_accel.y;
                    vmu931_data.accel.z = data_accel.z;
                    vmu931_data.header.stamp = curr_time;
                    pub_vmu931.publish(vmu931_data);
                }
                imu_data.linear_acceleration.x = data_accel.x * G_MPS2;
                imu_data.linear_acceleration.y = data_accel.y * G_MPS2;
                imu_data.linear_acceleration.z = data_accel.z * G_MPS2;
                imu_data.header.stamp = curr_time;
                pub_imu.publish(imu_data);
                last_time = curr_time;
                break;
            case dt_euler:
                vmutils_retrieveXYZData(&data_euler);
                if (publish_vmu931) {
                    vmu931_data.euler.x = data_euler.x;
                    vmu931_data.euler.y = data_euler.y;
                    vmu931_data.euler.z = data_euler.z;
                    vmu931_data.header.stamp = curr_time;
                    pub_vmu931.publish(vmu931_data);
                }
                last_time = curr_time;
                break;
            case dt_gyro:
                vmutils_retrieveXYZData(&data_gyro);
                if (publish_vmu931) {
                    vmu931_data.gyro.x = data_gyro.x;
                    vmu931_data.gyro.y = data_gyro.y;
                    vmu931_data.gyro.z = data_gyro.z;
                    vmu931_data.header.stamp = curr_time;
                    pub_vmu931.publish(vmu931_data);
                }
                imu_data.angular_velocity.x = data_gyro.x * M_PI / 180;
                imu_data.angular_velocity.y = data_gyro.y * M_PI / 180;
                imu_data.angular_velocity.z = data_gyro.z * M_PI / 180;
                imu_data.header.stamp = curr_time;
                pub_imu.publish(imu_data);
                last_time = curr_time;
                break;
            case dt_heading:
                vmutils_retrieveHData(&data_heading);
                if (publish_vmu931) {
                    vmu931_data.heading.data = data_heading.h;
                    vmu931_data.header.stamp = curr_time;
                    pub_vmu931.publish(vmu931_data);
                }
                last_time = curr_time;
                break;
            case dt_mag:
                vmutils_retrieveXYZData(&data_mag);
                if (publish_vmu931) {
                    vmu931_data.mag.x = data_mag.x;
                    vmu931_data.mag.y = data_mag.y;
                    vmu931_data.mag.z = data_mag.z;
                    vmu931_data.header.stamp = curr_time;
                    pub_vmu931.publish(vmu931_data);
                }
                last_time = curr_time;
                break;
            case dt_quat:
                vmutils_retrieveWXYZData(&data_quat);
                if (publish_vmu931) {
                    vmu931_data.quat.w = data_quat.w;
                    vmu931_data.quat.x = data_quat.x;
                    vmu931_data.quat.y = data_quat.y;
                    vmu931_data.quat.z = data_quat.z;
                    vmu931_data.header.stamp = curr_time;
                    pub_vmu931.publish(vmu931_data);
                }
                imu_data.orientation.w = data_quat.w;
                imu_data.orientation.x = data_quat.x;
                imu_data.orientation.y = data_quat.y;
                imu_data.orientation.z = data_quat.z;
                imu_data.header.stamp = curr_time;
                pub_imu.publish(imu_data);
                last_time = curr_time;
                break;
            default:
                break;
        }

    }

    close(vmu_fd);
    return 0;
}
