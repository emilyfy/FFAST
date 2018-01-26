#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster imu_broadcaster;
  tf::TransformBroadcaster lidar_broadcaster;

  while(n.ok()){
    imu_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 1, 0), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"base_link", "base_imu"));

    lidar_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.06785, 0.0, 0.02072)),
        ros::Time::now(),"base_link", "base_lidar"));
    
    r.sleep();
  }
}
