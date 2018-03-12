#ifndef VMU931_H
#define VMU931_H

#include "vmu931/vmu_utils.h"

#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <vmu931/Vmu931.h>

#define G_MPS2 9.81

#define VMU_ORDER_ACCEL   0
#define VMU_ORDER_EULER   1
#define VMU_ORDER_GYRO    2
#define VMU_ORDER_HEADING 3
#define VMU_ORDER_MAG     4
#define VMU_ORDER_QUAT    5

#define IMU_ORDER_ACCEL 0
#define IMU_ORDER_GYRO  1
#define IMU_ORDER_QUAT  2

class Vmu931
{
    public:
        Vmu931(ros::NodeHandle nh, ros::NodeHandle pnh);
        ~Vmu931();

        void spin();

    private:
        std::string serial_port_;
        bool publish_imu_, publish_mag_, publish_vmu_;
        int port_fd_;
        fd_set read_fds_, write_fds_, except_fds_;
        struct timeval port_timeout_;
        bool meas_enabled_[6];
        ros::Time curr_time_, last_time_;
        
        bool imu_msg_updated_[3];
        bool vmu_msg_updated_[6];

        Data_status data_status_;
        Data_xyz    data_accel_;
        Data_xyz    data_euler_;
        Data_xyz    data_gyro_;
        Data_h      data_heading_;
        Data_xyz    data_mag_;
        Data_wxyz   data_quat_;

        sensor_msgs::Imu imu_msg_;
        sensor_msgs::MagneticField mag_msg_;
        vmu931::Vmu931 vmu_msg_;

        ros::Publisher pub_imu_;
        ros::Publisher pub_mag_;
        ros::Publisher pub_vmu_;

        void connectSerialPort();
        void setSensorStatus();
        void updateStatus();
        int loadMessage();
        int sendCommand(Cmd cmd);

        // msg utils
        void processAccelData(Data_xyz data);
        void processEulerData(Data_xyz data);
        void processGyroData(Data_xyz data);
        void processHeadingData(Data_h data);
        void processMagData(Data_xyz data);
        void processQuatData(Data_wxyz data);
        void publishImu();
        void publishVmu();
};

#endif
