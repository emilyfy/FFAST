#include "vmu931/vmu931.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <vmu931/Vmu931.h>

Vmu931::Vmu931(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    // get params
    pnh.param("serial_port", serial_port_, std::string("/dev/ttyACM0"));
    pnh.param("publish_imu", publish_imu_, true);
    pnh.param("publish_mag", publish_mag_, false);
    pnh.param("publish_vmu931", publish_vmu_, false);

    // connect to vmu931
    connectSerialPort();
    ROS_INFO("VMU931: connected.");

    // set status according to what's to be published
    for (int i=0;i<6;i++) meas_enabled_[i] = false;
    if (publish_imu_) {
        meas_enabled_[VMU_ORDER_ACCEL] = true;
        meas_enabled_[VMU_ORDER_GYRO] = true;
        meas_enabled_[VMU_ORDER_QUAT] = true;
    }
    if (publish_mag_)
        meas_enabled_[VMU_ORDER_MAG] = true;
    if (publish_vmu_)
        for (int i=0;i<6;i++) meas_enabled_[i] = true;
    setSensorStatus();
    ROS_INFO("VMU931: sensor status set.");
    
    // set publishers where necessary
    if (publish_imu_)
    {
        pub_imu_ = nh.advertise<sensor_msgs::Imu>("/imu", 10);
        imu_msg_.header.frame_id = "base_imu";
    }
    if (publish_mag_)
    {
        pub_mag_ = nh.advertise<sensor_msgs::MagneticField>("/imu/mag", 10);
        mag_msg_.header.frame_id = "base_imu";
    }
    if (publish_vmu_)
    {
        pub_vmu_ = nh.advertise<vmu931::Vmu931>("/vmu931", 10);
        vmu_msg_.header.frame_id = "base_imu";
    }

    last_time_ = ros::Time::now();
}

Vmu931::~Vmu931()
{
    close(port_fd_);
}

void Vmu931::connectSerialPort() {
    struct termios options;

    port_fd_ = open(serial_port_.c_str(), O_RDWR | O_SYNC | O_NOCTTY);

    while( port_fd_ < 0 && ros::ok() ) {
        ROS_WARN("VMU931: Failed to connect to %s. Retrying...", serial_port_.c_str());
        port_fd_ = open(serial_port_.c_str(), O_RDWR | O_SYNC | O_NOCTTY);
        sleep(1);
    }

    tcgetattr(port_fd_, &options);
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

    tcflush(port_fd_, TCIFLUSH);
    tcsetattr(port_fd_, TCSANOW, &options);

    port_timeout_.tv_sec = 2;
    port_timeout_.tv_usec = 0;
}

void Vmu931::setSensorStatus() {
    updateStatus();
    vmutils_retrieveStatus(&data_status_);

    bool meas_active[6];

    meas_active[VMU_ORDER_ACCEL  ] = ( data_status_.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_ACCEL   ) != 0;
    meas_active[VMU_ORDER_EULER  ] = ( data_status_.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_EULER   ) != 0;
    meas_active[VMU_ORDER_GYRO   ] = ( data_status_.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_GYRO    ) != 0;
    meas_active[VMU_ORDER_HEADING] = ( data_status_.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_HEADING ) != 0;
    meas_active[VMU_ORDER_MAG    ] = ( data_status_.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_MAG     ) != 0;
    meas_active[VMU_ORDER_QUAT   ] = ( data_status_.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_QUAT    ) != 0;

    ros::Time start_time = ros::Time::now();
    bool keep_going = true;
    
    while( keep_going && ros::ok() ) {

        keep_going = false;

        if( meas_active[VMU_ORDER_ACCEL  ] != meas_enabled_[VMU_ORDER_ACCEL  ] ) { sendCommand(cmd_toggle_accel);   keep_going = true; }
        if( meas_active[VMU_ORDER_EULER  ] != meas_enabled_[VMU_ORDER_EULER  ] ) { sendCommand(cmd_toggle_euler);   keep_going = true; }
        if( meas_active[VMU_ORDER_GYRO   ] != meas_enabled_[VMU_ORDER_GYRO   ] ) { sendCommand(cmd_toggle_gyro);    keep_going = true; }
        if( meas_active[VMU_ORDER_HEADING] != meas_enabled_[VMU_ORDER_HEADING] ) { sendCommand(cmd_toggle_heading); keep_going = true; }
        if( meas_active[VMU_ORDER_MAG    ] != meas_enabled_[VMU_ORDER_MAG    ] ) { sendCommand(cmd_toggle_mag);     keep_going = true; }
        if( meas_active[VMU_ORDER_QUAT   ] != meas_enabled_[VMU_ORDER_QUAT   ] ) { sendCommand(cmd_toggle_quat);    keep_going = true; }


        if( keep_going == false ) { break; }
        ros::Time curr_time = ros::Time::now();
        if ( (curr_time - start_time).toSec() > 1.0 ) { ROS_WARN("Failed to set sensor status. Retrying..."); }

        // If it takes too long to set sensor status, IMU is not working
        if ( (curr_time - start_time).toSec() > 10 ) {
            ROS_ERROR("Cannot set sensor status. Please unplug and plug the VMU931 back in, wait for a few seconds and try again.");
            ros::shutdown();
        }

        ros::Duration(1).sleep();
        updateStatus();
        vmutils_retrieveStatus(&data_status_);

        meas_active[VMU_ORDER_ACCEL  ] = ( data_status_.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_ACCEL   ) != 0;
        meas_active[VMU_ORDER_EULER  ] = ( data_status_.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_EULER   ) != 0;
        meas_active[VMU_ORDER_GYRO   ] = ( data_status_.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_GYRO    ) != 0;
        meas_active[VMU_ORDER_HEADING] = ( data_status_.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_HEADING ) != 0;
        meas_active[VMU_ORDER_MAG    ] = ( data_status_.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_MAG     ) != 0;
        meas_active[VMU_ORDER_QUAT   ] = ( data_status_.data_streaming_array[3] & VMU_STATUS_MASK_STREAM_QUAT    ) != 0;

    }
}

void Vmu931::updateStatus() {

    // IMU often fails to send status
    // Stop trying to ask for status after 5 tries and determine status manually
    int retry_count = 0;

    // Send status requests until you get a response.
    while(retry_count <=5 && ros::ok()) {
        if( sendCommand(cmd_req_status) == -1 ) {
            continue;
        }

        for(int i=0; i<100 && ros::ok(); i++) {
            if( loadMessage() == -1 ) {
                continue;
            }

            if( singleMessage.messType == mt_data && vmutils_retrieveDataType() == dt_status ) {
                return;
            }

            ros::Duration(0.01).sleep();
        }
        
        ROS_WARN("VMU931: Failed to obtain status message. Retrying...");
        retry_count++;
        ros::Duration(1).sleep();
    }

    ROS_WARN("VMU931: Failed to obtain status in 5 tries. Deciding status manually.");
    bool active[6];
    for(int i=0; i<20 && ros::ok(); i++) {
        if( loadMessage() == -1 ) {i--; continue;}
        switch(vmutils_retrieveDataType()) {
            case dt_accel: active[VMU_ORDER_ACCEL] = true; break;
            case dt_euler: active[VMU_ORDER_EULER] = true; break;
            case dt_gyro: active[VMU_ORDER_GYRO] = true; break;
            case dt_heading: active[VMU_ORDER_HEADING] = true; break;
            case dt_mag: active[VMU_ORDER_MAG] = true; break;
            case dt_quat: active[VMU_ORDER_QUAT] = true; break;
            default: i--; break;
        }
    }
    ROS_WARN("status: %c %c %c %c %c %c", active[0]?'a':' ', active[1]?'e':' ', active[2]?'g':' ', active[3]?'h':' ', active[4]?'m':' ', active[5]?'q':' ');
    unsigned char status = 0xFF;
    if (active[VMU_ORDER_ACCEL  ]) status = status & (VMU_STATUS_MASK_STREAM_ACCEL ^ 0xFF);
    if (active[VMU_ORDER_EULER  ]) status = status & (VMU_STATUS_MASK_STREAM_GYRO ^ 0xFF);
    if (active[VMU_ORDER_GYRO   ]) status = status & (VMU_STATUS_MASK_STREAM_QUAT ^ 0xFF);
    if (active[VMU_ORDER_HEADING]) status = status & (VMU_STATUS_MASK_STREAM_MAG ^ 0xFF);
    if (active[VMU_ORDER_MAG    ]) status = status & (VMU_STATUS_MASK_STREAM_EULER ^ 0xFF);
    if (active[VMU_ORDER_QUAT   ]) status = status & (VMU_STATUS_MASK_STREAM_HEADING ^ 0xFF);

    unsigned char msg[VMU_DATA_STATUS_SIZE+4];
    MessType mt = mt_data;
    msg[VMU_START_BYTE_OFFS] = VMU_DATA_START;
    msg[VMU_SIZE_OFFS] = VMU_DATA_STATUS_SIZE+4;
    msg[VMU_TYPE_OFFS] = dt_status;
    msg[VMU_TYPE_OFFS+VMU_DATA_STATUS_SIZE] = status;
    msg[VMU_DATA_STATUS_SIZE+3] = VMU_DATA_END;
    vmutils_loadMessage(msg, VMU_DATA_STATUS_SIZE+4, &mt);
}

// Return -1 on error. Otherwise, return the number of bytes received.
int Vmu931::loadMessage() {
    unsigned char byte;
    unsigned char msg_size;
    unsigned char data[VMU_RAW_MESS_SIZE];
    unsigned char msg[VMU_RAW_MESS_SIZE];
    MessType string_or_data_msg;

    FD_ZERO(&read_fds_);
    FD_ZERO(&write_fds_);
    FD_ZERO(&except_fds_);
    FD_SET(port_fd_, &read_fds_);
    if (select(port_fd_ + 1, &read_fds_, &write_fds_, &except_fds_,  &port_timeout_) == 1) {
        while( read(port_fd_, &byte, 1) ) {

            // Search for the start of a data message.
            if( byte == VMU_DATA_START ) {

                // Read in the length of the message.
                if( !read(port_fd_, &msg_size, 1) ) {
                    ROS_DEBUG("VMU931: Failed to read length of message.");
                    return -1;
                }

                // If the message is too short to contain any data, return an error. 
                if( msg_size < 2 ) { 
                    ROS_DEBUG("VMU931: Message doesn't contain data.");
                    return -1;
                }

                if( !read(port_fd_, &data, msg_size-2) ) {
                    ROS_DEBUG("VMU931: Failed to read message data.");
                    return -1;
                }

                // Check the message footer.
                if( data[msg_size-3] != VMU_DATA_END ) {
                    ROS_DEBUG("VMU931: Message has wrong footer.");
                    return -1;
                }

                msg[VMU_START_BYTE_OFFS] = VMU_DATA_START;
                msg[VMU_SIZE_OFFS] = msg_size;
                memcpy(&msg[VMU_SIZE_OFFS+1], data, msg_size-2);
                string_or_data_msg = mt_data;

                // Load the message for future use.
                if( VMU_ERR_SUCCESS != vmutils_loadMessage(msg, msg_size, &string_or_data_msg) ) {
                    ROS_DEBUG("VMU931: Failed to load message.");
                    return -1;
                }
                return msg_size;

            } else if( byte == VMU_STRING_START) {

                // Read in the length of the message.
                if( !read(port_fd_, &msg_size, 1) ) {
                    ROS_DEBUG("VMU931: Failed to read length of message.");
                    return -1;
                }

                // If the message is too short to contain any data, return an error. 
                if( msg_size < 2 ) { 
                    ROS_DEBUG("VMU931: Message doesn't contain data.");
                    return -1;
                }
                
                if( !read(port_fd_, &data, msg_size-2) ) {
                    ROS_DEBUG("VMU931: Failed to read string from message.");
                    return -1;
                }

                // Check the message footer.
                if( data[msg_size-3] != VMU_STRING_END ) {
                    ROS_DEBUG("VMU931: Message has wrong footer.");
                    return -1; 
                }

                msg[VMU_START_BYTE_OFFS] = VMU_STRING_START;
                msg[VMU_SIZE_OFFS] = msg_size;
                memcpy(&msg[VMU_SIZE_OFFS+1], data, msg_size-2);
                string_or_data_msg = mt_string;

                // Load the message for future use.
                if( VMU_ERR_SUCCESS != vmutils_loadMessage(msg, msg_size, &string_or_data_msg) ) {
                    ROS_DEBUG("VMU931: Failed to load message. %d", vmutils_loadMessage(msg, msg_size, &string_or_data_msg));
                    return -1;
                }
                return msg_size;

            }
        }
    }
    else {
        ROS_DEBUG("VMU931: Failed to read from port.");
        return -1;
    }
}

// return -1 if build command fails
int Vmu931::sendCommand(Cmd cmd) {
    char cmd_buffer[VMU_CMD_NUM_BYTES];

    if( vmutils_buildCmd(cmd, cmd_buffer) != VMU_ERR_SUCCESS) {
        ROS_WARN("VMU931: Failed to build command.");
        return -1;
    }

    FD_ZERO(&read_fds_);
    FD_ZERO(&write_fds_);
    FD_ZERO(&except_fds_);
    FD_SET(port_fd_, &write_fds_);

    if (select(port_fd_ + 1, &read_fds_, &write_fds_, &except_fds_, &port_timeout_) == 1) {
        for( int i=0; i<VMU_CMD_NUM_BYTES; ++i ) {
            int success = write(port_fd_, &cmd_buffer[i], 1);
            usleep(100);
            if (success <= 0 ) ROS_DEBUG("VMU931: Failed to write to port");
        }
        return 1;
    } else {
        ROS_WARN("VMU931: Failed to write to port.");
    }
}

void Vmu931::spin() {
    while (ros::ok()) {

        curr_time_ = ros::Time::now();
        if ( (curr_time_ - last_time_).toSec() > 5 ) {
            ROS_WARN("Haven't received any IMU message in 5 seconds.");
            setSensorStatus();
            last_time_ = ros::Time::now();
        }

        int msg_length = loadMessage();
        if( msg_length <= 0 ) { continue; }

        DataType data_type = vmutils_retrieveDataType();

        switch( data_type ) {
            case dt_accel:
                vmutils_retrieveXYZData(&data_accel_);
                processAccelData(data_accel_);
                last_time_ = curr_time_;
                break;
            case dt_euler:
                vmutils_retrieveXYZData(&data_euler_);
                processEulerData(data_euler_);
                last_time_ = curr_time_;
                break;
            case dt_gyro:
                vmutils_retrieveXYZData(&data_gyro_);
                processGyroData(data_gyro_);
                last_time_ = curr_time_;
                break;
            case dt_heading:
                vmutils_retrieveHData(&data_heading_);
                processHeadingData(data_heading_);
                last_time_ = curr_time_;
                break;
            case dt_mag:
                vmutils_retrieveXYZData(&data_mag_);
                processMagData(data_mag_);
                last_time_ = curr_time_;
                break;
            case dt_quat:
                vmutils_retrieveWXYZData(&data_quat_);
                processQuatData(data_quat_);
                last_time_ = curr_time_;
                break;
            default:
                break;
        }
    }
}
