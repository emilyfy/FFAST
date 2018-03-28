#include "vmu931/vmu931.h"
#include "vmu931/vmu_utils.h"

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <vmu931/Vmu931.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>

void Vmu931::processAccelData(Data_xyz data)
{
    ROS_DEBUG("VMU931: acceleration data received.");
    if (publish_imu_) {
        imu_msg_.linear_acceleration.x = data.x * G_MPS2;
        imu_msg_.linear_acceleration.y = data.y * G_MPS2;
        imu_msg_.linear_acceleration.z = data.z * G_MPS2;
        imu_msg_updated_[IMU_ORDER_ACCEL] = true;
        publishImu();
    }

    if (publish_vmu_) {
        vmu_msg_.accel.x = data.x;
        vmu_msg_.accel.y = data.y;
        vmu_msg_.accel.z = data.z;
        vmu_msg_updated_[VMU_ORDER_ACCEL] = true;
        publishVmu();
    }
}

void Vmu931::processEulerData(Data_xyz data)
{
    ROS_DEBUG("VMU931: euler data received.");
    if (publish_vmu_) {
        vmu_msg_.euler.x = data.x;
        vmu_msg_.euler.y = data.y;
        vmu_msg_.euler.z = data.z;
        vmu_msg_updated_[VMU_ORDER_EULER] = true;
        publishVmu();
    }
}

void Vmu931::processGyroData(Data_xyz data)
{
    ROS_DEBUG("VMU931: gyro data received.");
    if (publish_imu_) {
        imu_msg_.angular_velocity.x = data.x * M_PI / 180;
        imu_msg_.angular_velocity.y = data.y * M_PI / 180;
        imu_msg_.angular_velocity.z = data.z * M_PI / 180;
        imu_msg_updated_[IMU_ORDER_GYRO] = true;
        publishImu();
    }

    if (publish_vmu_) {
        vmu_msg_.gyro.x = data.x * M_PI / 180;
        vmu_msg_.gyro.y = data.y * M_PI / 180;
        vmu_msg_.gyro.z = data.z * M_PI / 180;
        vmu_msg_updated_[VMU_ORDER_GYRO] = true;
        publishVmu();
    }
}

void Vmu931::processHeadingData(Data_h data)
{
    ROS_DEBUG("VMU931: heading data received.");
    if (publish_vmu_) {
        vmu_msg_.heading.data = data.h;
        vmu_msg_updated_[VMU_ORDER_HEADING] = true;
        publishVmu();
    }
}

void Vmu931::processMagData(Data_xyz data)
{
    ROS_DEBUG("VMU931: magnetic field data received.");
    if (publish_mag_) {
        mag_msg_.magnetic_field.x = data.x;
        mag_msg_.magnetic_field.y = data.y;
        mag_msg_.magnetic_field.z = data.z;
        mag_msg_.header.stamp = curr_time_;
        pub_mag_.publish(mag_msg_);
    }

    if (publish_vmu_) {
        vmu_msg_.mag.x = data.x;
        vmu_msg_.mag.y = data.y;
        vmu_msg_.mag.z = data.z;
        vmu_msg_updated_[VMU_ORDER_MAG] = true;
        publishVmu();
    }
}

void Vmu931::processQuatData(Data_wxyz data)
{
    ROS_DEBUG("VMU931: quaternion data received.");
    if (publish_imu_) {
        imu_msg_.orientation.w = data.w;
        imu_msg_.orientation.x = data.x;
        imu_msg_.orientation.y = data.y;
        imu_msg_.orientation.z = data.z;
        imu_msg_updated_[IMU_ORDER_QUAT] = true;
        publishImu();
    }

    if (publish_vmu_) {
        vmu_msg_.quat.w = data.w;
        vmu_msg_.quat.x = data.x;
        vmu_msg_.quat.y = data.y;
        vmu_msg_.quat.z = data.z;
        vmu_msg_updated_[VMU_ORDER_QUAT] = true;
        publishVmu();
    }
}

void Vmu931::publishImu()
{
    bool publish = true;
    for (int i=0;i<3;i++) if (!imu_msg_updated_[i]) publish = false;

    if (publish) {
        imu_msg_.header.stamp = curr_time_;
        pub_imu_.publish(imu_msg_);
        for (int i=0;i<3;i++) imu_msg_updated_[i] = false;
    }
}

void Vmu931::publishVmu()
{
    bool publish = true;
    for (int i=0;i<6;i++) if (!vmu_msg_updated_[i]) publish = false;

    if (publish) {
        vmu_msg_.header.stamp = curr_time_;
        pub_vmu_.publish(vmu_msg_);
        for (int i=0;i<6;i++) vmu_msg_updated_[i] = false;
    }
}
