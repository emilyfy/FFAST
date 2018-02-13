#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>

int main(int argc, char **argv)
{
    ros::init(argc,argv, "tf2_static_broadcaster");
    ros::NodeHandle nh;

    static tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

    geometry_msgs::TransformStamped lidar_tf_stamped;
    geometry_msgs::TransformStamped imu_tf_stamped;


    lidar_tf_stamped.header.stamp = ros::Time::now();
    lidar_tf_stamped.header.frame_id = "base_link";
    lidar_tf_stamped.child_frame_id = "base_lidar";
    lidar_tf_stamped.transform.translation.x = 0.06785;
    lidar_tf_stamped.transform.translation.y = 0.0;
    lidar_tf_stamped.transform.translation.z = 0.02072;
    lidar_tf_stamped.transform.rotation.x = 0;
    lidar_tf_stamped.transform.rotation.y = 0;
    lidar_tf_stamped.transform.rotation.z = 0;
    lidar_tf_stamped.transform.rotation.w = 1;

    imu_tf_stamped.header.stamp = ros::Time::now();
    imu_tf_stamped.header.frame_id = "base_link";
    imu_tf_stamped.child_frame_id = "base_imu";
    imu_tf_stamped.transform.translation.x = 0.0;
    imu_tf_stamped.transform.translation.y = 0.0;
    imu_tf_stamped.transform.translation.z = 0.0;
    imu_tf_stamped.transform.rotation.x = 0;
    imu_tf_stamped.transform.rotation.y = 0;
    imu_tf_stamped.transform.rotation.z = 1;
    imu_tf_stamped.transform.rotation.w = 0;

    std::vector<geometry_msgs::TransformStamped> static_tf_stamped;
    static_tf_stamped.push_back(lidar_tf_stamped);
    static_tf_stamped.push_back(imu_tf_stamped);

    static_tf_broadcaster.sendTransform(static_tf_stamped);
    ROS_INFO("Static tf broadcaster spinning until killed.");
    ros::spin();
    return 0;
}
